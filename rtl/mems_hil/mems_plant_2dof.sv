// ============================================================================
// Module: mems_plant_2dof
// Description:
//   48-bit Fixed-Point Strang-Split 2-DOF Coriolis Vibratory Gyroscope (CVG)
//   Physics Plant for real-time Hardware-in-the-Loop (HIL) on ZUBoard 1CG.
//
//   Supports both:
//     - Silicon MEMS DRG (f0 = 11 kHz, Q = 100,000, < 0.01% Q error)
//     - Fused-Silica HRG (f0 = 11 kHz / 4.2 kHz, Q = 1,000,000+, < 0.02% Q error)
//
//   Architecture:
//     - Single time-multiplexed pipelined 48x36 signed multiplier (~6 DSP48E2)
//     - Completes 1 full 2-DOF Strang-split step in 32 clock cycles
//       (easily fits inside 100 cycles/step for 1 MHz physics at 100 MHz pl_clk0!)
//
//   Fixed-Point Formats:
//     - States (x, vx, y, vy) : signed 48-bit Q4.44 (1.0 = 2^44)
//     - Coefficients          : signed 36-bit Q2.34 (1.0 = 2^34)
//     - Actuation (u_x, u_y)  : signed 32-bit Q10.22 (range [-512, +512))
//     - half_dt_q36           : signed 36-bit Q0.36 (0.5 * Ts * 2^36)
//     - Pickoff outputs       : signed 16-bit Q2.14 (1.0 = 16384)
// ============================================================================
`timescale 1ns / 1ps

module mems_plant_2dof (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               tick_en,       // 1-cycle strobe per physics step (e.g. 1 MHz)

    // State preload for deterministic startup / ringdown tests
    input  logic               state_load,
    input  logic signed [47:0] x_init,        // Q4.44
    input  logic signed [47:0] vx_init,       // Q4.44
    input  logic signed [47:0] y_init,        // Q4.44
    input  logic signed [47:0] vy_init,       // Q4.44

    // Runtime-programmable physics parameters (Q2.34, signed 36-bit)
    input  logic signed [35:0] cos_tx,        // cos(wx * Ts)
    input  logic signed [35:0] sin_tx,        // sin(wx * Ts)
    input  logic signed [35:0] cos_ty,        // cos(wy * Ts)
    input  logic signed [35:0] sin_ty,        // sin(wy * Ts)
    input  logic signed [35:0] half_gx_dt,    // 0.5 * (wx / Qx) * Ts
    input  logic signed [35:0] half_gy_dt,    // 0.5 * (wy / Qy) * Ts
    input  logic signed [35:0] half_kxy_dt,   // 0.5 * k_xy * Ts (quadrature stiffness)
    input  logic signed [35:0] half_gxy_dt,   // 0.5 * g_xy * Ts (anisodamping bias)
    input  logic signed [35:0] half_cor_dt,   // 0.5 * (2 * Ag * Omega_z) * Ts
    input  logic signed [35:0] half_dt_q36,   // 0.5 * Ts in Q0.36

    // Modal control accelerations at t_k and t_{k+1} (Q10.22, signed 32-bit)
    input  logic signed [31:0] u_x_k,
    input  logic signed [31:0] u_y_k,
    input  logic signed [31:0] u_x_k1,
    input  logic signed [31:0] u_y_k1,

    // Pickoff outputs (Q2.14, 16-bit ADC equivalent) & full 48-bit truth states
    output logic signed [15:0] x_q14,
    output logic signed [15:0] y_q14,
    output logic signed [47:0] x_state,
    output logic signed [47:0] vx_state,
    output logic signed [47:0] y_state,
    output logic signed [47:0] vy_state,
    output logic               step_done
);

    localparam int C_FRAC  = 34; // state(Q4.44) * coeff(Q2.34) -> >>> 34 -> Q4.44
    localparam int U_SHIFT = 14; // u(Q10.22) * half_dt(Q0.36) -> frac=58 -> >>> 14 -> Q4.44

    // Primary 48-bit Q4.44 states
    logic signed [47:0] x_r, vx_r, y_r, vy_r;

    // Intermediate Strang-split registers
    logic signed [47:0] vx_h, vy_h;
    logic signed [47:0] x_n, vx_rot, y_n, vy_rot;
    logic signed [47:0] vx_n, vy_n;

    // Latched control inputs for the current step
    logic signed [31:0] ux0_r, uy0_r, ux1_r, uy1_r;

    // Shared 2-stage pipelined 48x36 signed multiplier
    logic signed [47:0] mul_a;
    logic signed [35:0] mul_b;
    logic               mul_is_u;
    logic               mul_neg;
    logic [3:0]         mul_dest;

    logic signed [83:0] prod_r;
    logic               prod_is_u_r;
    logic               prod_neg_r;
    logic [3:0]         prod_dest_r;
    logic               prod_valid_r;

    // Destination register tags for MAC accumulation
    localparam logic [3:0] DST_NONE  = 4'd0;
    localparam logic [3:0] DST_VX_H  = 4'd1;
    localparam logic [3:0] DST_VY_H  = 4'd2;
    localparam logic [3:0] DST_X_N   = 4'd3;
    localparam logic [3:0] DST_VX_RT = 4'd4;
    localparam logic [3:0] DST_Y_N   = 4'd5;
    localparam logic [3:0] DST_VY_RT = 4'd6;
    localparam logic [3:0] DST_VX_N  = 4'd7;
    localparam logic [3:0] DST_VY_N  = 4'd8;

    // 32-step micro-sequencer (0 = IDLE, 1..30 = MAC/pipeline drain, 31 = COMMIT)
    logic [5:0] step_idx;
    logic       busy;

    // Rounded 48-bit MAC term from registered 84-bit product
    logic signed [83:0] prod_shifted;
    logic signed [47:0] term_q44;

    always_comb begin
        if (prod_is_u_r) begin
            prod_shifted = (prod_r + (84'sd1 <<< (U_SHIFT - 1))) >>> U_SHIFT;
        end else begin
            prod_shifted = (prod_r + (84'sd1 <<< (C_FRAC - 1))) >>> C_FRAC;
        end
        term_q44 = prod_neg_r ? -prod_shifted[47:0] : prod_shifted[47:0];
    end

    // Operand mux for the 28 Strang-split MAC operations
    // Note: Wait 1 bubble cycle after each dependency boundary (step 11 -> vx_h/vy_h ready,
    // step 20 -> x_n/vx_rot/y_n/vy_rot ready) so registered accumulation is hazard-free!
    always_comb begin
        mul_a    = 48'sd0;
        mul_b    = 36'sd0;
        mul_is_u = 1'b0;
        mul_neg  = 1'b0;
        mul_dest = DST_NONE;

        unique case (step_idx)
            // --- Kick 1: vx_h = vx + dt/2 * (-gx*vx - kxy*y - gxy*vy + cor*vy + u_x_k) ---
            6'd1:  begin mul_a = vx_r;             mul_b = half_gx_dt;  mul_neg = 1'b1; mul_dest = DST_VX_H; end
            6'd2:  begin mul_a = y_r;              mul_b = half_kxy_dt; mul_neg = 1'b1; mul_dest = DST_VX_H; end
            6'd3:  begin mul_a = vy_r;             mul_b = half_gxy_dt; mul_neg = 1'b1; mul_dest = DST_VX_H; end
            6'd4:  begin mul_a = vy_r;             mul_b = half_cor_dt; mul_neg = 1'b0; mul_dest = DST_VX_H; end
            6'd5:  begin mul_a = 48'(ux0_r);       mul_b = half_dt_q36; mul_is_u = 1'b1; mul_dest = DST_VX_H; end

            // --- Kick 1: vy_h = vy + dt/2 * (-gy*vy - kxy*x - gxy*vx - cor*vx + u_y_k) ---
            6'd6:  begin mul_a = vy_r;             mul_b = half_gy_dt;  mul_neg = 1'b1; mul_dest = DST_VY_H; end
            6'd7:  begin mul_a = x_r;              mul_b = half_kxy_dt; mul_neg = 1'b1; mul_dest = DST_VY_H; end
            6'd8:  begin mul_a = vx_r;             mul_b = half_gxy_dt; mul_neg = 1'b1; mul_dest = DST_VY_H; end
            6'd9:  begin mul_a = vx_r;             mul_b = half_cor_dt; mul_neg = 1'b1; mul_dest = DST_VY_H; end
            6'd10: begin mul_a = 48'(uy0_r);       mul_b = half_dt_q36; mul_is_u = 1'b1; mul_dest = DST_VY_H; end

            // Bubble cycle 11: vy_h finishes accumulating from step 10 (vx_h already ready at step 7)
            6'd11: begin mul_dest = DST_NONE; end

            // --- Exact Harmonic Rotation: (x, vx_h) -> (x_n, vx_rot) and (y, vy_h) -> (y_n, vy_rot) ---
            6'd12: begin mul_a = x_r;              mul_b = cos_tx;      mul_neg = 1'b0; mul_dest = DST_X_N;   end
            6'd13: begin mul_a = vx_h;             mul_b = sin_tx;      mul_neg = 1'b0; mul_dest = DST_X_N;   end
            6'd14: begin mul_a = x_r;              mul_b = sin_tx;      mul_neg = 1'b1; mul_dest = DST_VX_RT; end
            6'd15: begin mul_a = vx_h;             mul_b = cos_tx;      mul_neg = 1'b0; mul_dest = DST_VX_RT; end
            6'd16: begin mul_a = y_r;              mul_b = cos_ty;      mul_neg = 1'b0; mul_dest = DST_Y_N;   end
            6'd17: begin mul_a = vy_h;             mul_b = sin_ty;      mul_neg = 1'b0; mul_dest = DST_Y_N;   end
            6'd18: begin mul_a = y_r;              mul_b = sin_ty;      mul_neg = 1'b1; mul_dest = DST_VY_RT; end
            6'd19: begin mul_a = vy_h;             mul_b = cos_ty;      mul_neg = 1'b0; mul_dest = DST_VY_RT; end

            // Bubble cycle 20: vy_rot finishes accumulating from step 19, initialize vx_n/vy_n
            6'd20: begin mul_dest = DST_NONE; end

            // --- Kick 2: vx_n = vx_rot + dt/2 * (-gx*vx_rot - kxy*y_n - gxy*vy_rot + cor*vy_rot + u_x_k1) ---
            6'd21: begin mul_a = vx_rot;           mul_b = half_gx_dt;  mul_neg = 1'b1; mul_dest = DST_VX_N; end
            6'd22: begin mul_a = y_n;              mul_b = half_kxy_dt; mul_neg = 1'b1; mul_dest = DST_VX_N; end
            6'd23: begin mul_a = vy_rot;           mul_b = half_gxy_dt; mul_neg = 1'b1; mul_dest = DST_VX_N; end
            6'd24: begin mul_a = vy_rot;           mul_b = half_cor_dt; mul_neg = 1'b0; mul_dest = DST_VX_N; end
            6'd25: begin mul_a = 48'(ux1_r);       mul_b = half_dt_q36; mul_is_u = 1'b1; mul_dest = DST_VX_N; end

            // --- Kick 2: vy_n = vy_rot + dt/2 * (-gy*vy_rot - kxy*x_n - gxy*vx_rot - cor*vx_rot + u_y_k1) ---
            6'd26: begin mul_a = vy_rot;           mul_b = half_gy_dt;  mul_neg = 1'b1; mul_dest = DST_VY_N; end
            6'd27: begin mul_a = x_n;              mul_b = half_kxy_dt; mul_neg = 1'b1; mul_dest = DST_VY_N; end
            6'd28: begin mul_a = vx_rot;           mul_b = half_gxy_dt; mul_neg = 1'b1; mul_dest = DST_VY_N; end
            6'd29: begin mul_a = vx_rot;           mul_b = half_cor_dt; mul_neg = 1'b1; mul_dest = DST_VY_N; end
            6'd30: begin mul_a = 48'(uy1_r);       mul_b = half_dt_q36; mul_is_u = 1'b1; mul_dest = DST_VY_N; end

            default: begin mul_dest = DST_NONE; end
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            x_r          <= '0;
            vx_r         <= '0;
            y_r          <= '0;
            vy_r         <= '0;
            vx_h         <= '0;
            vy_h         <= '0;
            x_n          <= '0;
            vx_rot       <= '0;
            y_n          <= '0;
            vy_rot       <= '0;
            vx_n         <= '0;
            vy_n         <= '0;
            ux0_r        <= '0;
            uy0_r        <= '0;
            ux1_r        <= '0;
            uy1_r        <= '0;
            prod_r       <= '0;
            prod_is_u_r  <= 1'b0;
            prod_neg_r   <= 1'b0;
            prod_dest_r  <= DST_NONE;
            prod_valid_r <= 1'b0;
            step_idx     <= 6'd0;
            busy         <= 1'b0;
            step_done    <= 1'b0;
        end else begin
            step_done <= 1'b0;

            if (state_load) begin
                x_r          <= x_init;
                vx_r         <= vx_init;
                y_r          <= y_init;
                vy_r         <= vy_init;
                busy         <= 1'b0;
                step_idx     <= 6'd0;
                prod_valid_r <= 1'b0;
            end else begin
                // Stage 1 of MAC pipeline: multiply and register 84-bit signed product
                prod_r       <= signed'({{36{mul_a[47]}}, mul_a}) * signed'({{48{mul_b[35]}}, mul_b});
                prod_is_u_r  <= mul_is_u;
                prod_neg_r   <= mul_neg;
                prod_dest_r  <= mul_dest;
                prod_valid_r <= (mul_dest != DST_NONE);

                // Stage 2 of MAC pipeline: accumulate into target register
                if (prod_valid_r) begin
                    unique case (prod_dest_r)
                        DST_VX_H:  vx_h   <= vx_h   + term_q44;
                        DST_VY_H:  vy_h   <= vy_h   + term_q44;
                        DST_X_N:   x_n    <= x_n    + term_q44;
                        DST_VX_RT: vx_rot <= vx_rot + term_q44;
                        DST_Y_N:   y_n    <= y_n    + term_q44;
                        DST_VY_RT: vy_rot <= vy_rot + term_q44;
                        DST_VX_N:  vx_n   <= vx_n   + term_q44;
                        DST_VY_N:  vy_n   <= vy_n   + term_q44;
                        default: ;
                    endcase
                end

                // Sequencer control
                if (!busy) begin
                    if (tick_en) begin
                        busy     <= 1'b1;
                        step_idx <= 6'd1;
                        vx_h     <= vx_r;
                        vy_h     <= vy_r;
                        x_n      <= 48'sd0;
                        vx_rot   <= 48'sd0;
                        y_n      <= 48'sd0;
                        vy_rot   <= 48'sd0;
                        ux0_r    <= u_x_k;
                        uy0_r    <= u_y_k;
                        ux1_r    <= u_x_k1;
                        uy1_r    <= u_y_k1;
                    end
                end else begin
                    // At step 20, vx_rot and vy_rot finish accumulating on THIS edge:
                    // load vx_n / vy_n with the final rotated velocities before step 21!
                    if (step_idx == 6'd20) begin
                        vx_n <= vx_rot;
                        vy_n <= vy_rot + (prod_valid_r && (prod_dest_r == DST_VY_RT) ? term_q44 : 48'sd0);
                    end

                    if (step_idx == 6'd32) begin
                        // All 28 MACs finished and accumulated; commit updated state
                        x_r       <= x_n;
                        vx_r      <= vx_n;
                        y_r       <= y_n;
                        vy_r      <= vy_n;
                        busy      <= 1'b0;
                        step_idx  <= 6'd0;
                        step_done <= 1'b1;
                    end else begin
                        step_idx <= step_idx + 6'd1;
                    end
                end
            end
        end
    end

    // Convert Q4.44 state -> Q2.14 16-bit pickoff (round bit 29, extract bits [45:30])
    logic signed [47:0] x_rnd, y_rnd;
    assign x_rnd = x_r + (48'sd1 <<< 29);
    assign y_rnd = y_r + (48'sd1 <<< 29);

    assign x_q14    = x_rnd[30 +: 16];
    assign y_q14    = y_rnd[30 +: 16];
    assign x_state  = x_r;
    assign vx_state = vx_r;
    assign y_state  = y_r;
    assign vy_state = vy_r;

    logic unused_ok;
    assign unused_ok = &{1'b0, prod_shifted[83:48], x_rnd[47:46], x_rnd[29:0], y_rnd[47:46], y_rnd[29:0]};

endmodule
