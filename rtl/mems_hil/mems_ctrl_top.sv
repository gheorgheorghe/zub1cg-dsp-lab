// ============================================================================
// Module: mems_ctrl_top
// Description:
//   Complete Closed-Loop CVG Controller in SystemVerilog:
//     - Zero-DSP 32-bit CORDIC NCO (sv_nco_cordic)
//     - Dual 3-stage I/Q Lock-In Demodulators (sv_lockin) -> DX, DY, SX, SY
//     - 4x Anti-Windup PI Loops (sv_pi_ctrl) -> PLLAdj, AGCAdj, CLQAdj, FTRAdj
//     - Drive & Sense Carrier Re-Modulators -> u_x, u_y (Q10.22)
//
//   Fixed-point conventions:
//     - Pickoffs x_q14, y_q14 : signed 16-bit Q2.14 (1.0 = 16384)
//     - Demod DX, DY, SX, SY  : signed 24-bit Q2.22 (1.0 = 4,194,304)
//     - AGCAdj, CLQAdj, FTRAdj: signed 24-bit Q10.14 (1.0 = 16384, range [-512,+512))
//     - PLLAdj                : signed 24-bit frequency tuning word offset
//                               (shifted left by 4 and added to 32-bit base_freq_word)
//     - Actuation u_x, u_y    : signed 32-bit Q10.22 fed straight to mems_plant_2dof
// ============================================================================
`timescale 1ns / 1ps

module mems_ctrl_top (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               tick_en,        // Sample tick (e.g. 1 MHz or 200 kHz)

    // Pickoff inputs (Q2.14)
    input  logic signed [15:0] x_q14,
    input  logic signed [15:0] y_q14,

    // Configuration & Setpoints
    input  logic [31:0]        base_freq_word, // Nominal f0 NCO phase increment per tick
    input  logic [3:0]         lpf_shift,      // Lock-in LPF shift (e.g. 8 at 1 MHz)
    input  logic               enable_pll,
    input  logic               enable_agc,
    input  logic               enable_clq,
    input  logic               enable_ftr,

    input  logic signed [23:0] dx_setpoint,    // Q2.22 (default 1.0 = 24'sd4194304)
    input  logic signed [23:0] sx_setpoint,    // Q2.22 (default 0)
    input  logic signed [23:0] sy_setpoint,    // Q2.22 (default 0)

    // Loop PI gains (signed 24-bit, GAIN_FRAC = 16)
    input  logic signed [23:0] pll_kp, pll_ki,
    input  logic signed [23:0] agc_kp, agc_ki,
    input  logic signed [23:0] clq_kp, clq_ki,
    input  logic signed [23:0] ftr_kp, ftr_ki,

    // Preload hook for bumpless startup
    input  logic               pi_load,
    input  logic signed [23:0] pll_init,
    input  logic signed [23:0] agc_init,

    // Carrier-modulated control forces at t_k and t_{k+1} (Q10.22, signed 32-bit)
    output logic signed [31:0] u_x_k_out,
    output logic signed [31:0] u_y_k_out,
    output logic signed [31:0] u_x_k1_out,
    output logic signed [31:0] u_y_k1_out,
    output logic               plant_start,

    // Baseband telemetry matching the 12-panel dashboard
    output logic signed [23:0] dx_out,
    output logic signed [23:0] dy_out,
    output logic signed [23:0] sx_out,
    output logic signed [23:0] sy_out,
    output logic signed [23:0] pll_adj_out,
    output logic signed [23:0] agc_adj_out,
    output logic signed [23:0] clq_adj_out,
    output logic signed [23:0] ftr_adj_out
);

    // ------------------------------------------------------------------------
    // 1. Carrier NCO (advances by 1 phase step on each tick_en)
    // ------------------------------------------------------------------------
    logic [31:0]        nco_tdata;
    logic               nco_tvalid;
    logic signed [15:0] cos_q15;
    logic signed [15:0] sin_q15;
    logic [31:0]        eff_freq_word;

    // PLLAdj (24-bit signed) shifted left by 4 gives +-134M phase counts/step range
    // (at 1 MHz tick, +-31.25 kHz capture range with 0.0037 Hz LSB resolution!)
    assign eff_freq_word = enable_pll ? (base_freq_word + (32'(pll_adj_out) <<< 4)) : base_freq_word;

    // Step the pipelined CORDIC NCO cleanly on every tick_en:
    // Because NCO has 18 pipeline stages and tick_en arrives every 100 clocks,
    // keep NCO enabled only on tick_en and hold last valid {sin, cos}.
    logic signed [15:0] cos_hold, sin_hold;
    logic [31:0]        phase_acc;
    logic               nco_fire;

    // Use sv_nco_cordic driven by phase_offset = phase_acc with freq_word = 0 so
    // each tick_en produces an exact single-cycle phase sample after 19 clocks?
    // Even cleaner: run sv_nco_cordic with enable = tick_en, m_axis_tready = 1'b1!
    // After initial 19 ticks (19 us), every tick_en advances phase by eff_freq_word!
    // Wait: during tick_en = 0, sv_nco_cordic pushes invalid bubbles through the pipe,
    // so 19 clocks after tick_en, m_axis_tvalid pulses with the exact next (sin, cos)!
    sv_nco_cordic u_nco (
        .clk           (clk),
        .rst_n         (rst_n),
        .enable        (tick_en),
        .freq_word     (eff_freq_word),
        .phase_offset  (eff_freq_word),
        .m_axis_tdata  (nco_tdata),
        .m_axis_tvalid (nco_tvalid),
        .m_axis_tready (1'b1)
    );

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            cos_hold  <= 16'sd32760; // cos(0) = 1.0
            sin_hold  <= 16'sd0;     // sin(0) = 0.0
            phase_acc <= 32'd0;
            nco_fire  <= 1'b0;
        end else begin
            nco_fire <= nco_tvalid;
            if (nco_tvalid) begin
                cos_hold <= signed'(nco_tdata[15:0]);
                sin_hold <= signed'(nco_tdata[31:16]);
            end
            if (tick_en) begin
                phase_acc <= phase_acc + eff_freq_word;
            end
        end
    end

    assign cos_q15 = cos_hold;
    assign sin_q15 = sin_hold;

    // ------------------------------------------------------------------------
    // 2. Dual 3-Stage I/Q Lock-In Demodulators (Drive: DX/DY, Sense: SX/SY)
    // ------------------------------------------------------------------------
    logic lockin_valid_d, lockin_valid_s;

    sv_lockin #(.OUT_W(24)) u_lockin_drive (
        .clk       (clk),
        .rst_n     (rst_n),
        .tick_en   (tick_en),
        .lpf_shift (lpf_shift),
        .sig_in    (x_q14),
        .cos_in    (cos_q15),
        .sin_in    (sin_q15),
        .out_i     (dx_out),
        .out_q     (dy_out),
        .out_valid (lockin_valid_d)
    );

    sv_lockin #(.OUT_W(24)) u_lockin_sense (
        .clk       (clk),
        .rst_n     (rst_n),
        .tick_en   (tick_en),
        .lpf_shift (lpf_shift),
        .sig_in    (y_q14),
        .cos_in    (cos_q15),
        .sin_in    (sin_q15),
        .out_i     (sx_out),
        .out_q     (sy_out),
        .out_valid (lockin_valid_s)
    );

    // ------------------------------------------------------------------------
    // 3. Four Anti-Windup PI Controllers (run on lockin_valid_d)
    //    Error inputs are Q2.22; PI outputs are Q10.14 (shift error right by 8
    //    inside gain scaling, i.e., err is Q2.22 and gain includes 2^-8 factor).
    // ------------------------------------------------------------------------
    logic signed [23:0] err_pll, err_agc, err_clq, err_ftr;
    logic               pi_valid_pll, pi_valid_agc, pi_valid_clq, pi_valid_ftr;
    logic               sat_pll, sat_agc, sat_clq, sat_ftr;

    // Drive PLL drives DY -> 0 (DY > 0 => NCO lags resonance => increase freq)
    assign err_pll = enable_pll ? dy_out : 24'sd0;
    // Drive AGC drives DX -> dx_setpoint
    assign err_agc = enable_agc ? (dx_setpoint - dx_out) : 24'sd0;
    // Sense CLQ drives SY -> sy_setpoint (SY > 0 => apply positive CLQAdj*sin)
    assign err_clq = enable_clq ? (sy_out - sy_setpoint) : 24'sd0;
    // Sense FTR drives SX -> sx_setpoint (SX < 0 under +Omega => apply positive FTRAdj*cos)
    assign err_ftr = enable_ftr ? (sx_setpoint - sx_out) : 24'sd0;

    localparam logic signed [23:0] MAX_U_Q14 =  24'sd8000000; // ~+488 in Q10.14
    localparam logic signed [23:0] MIN_U_Q14 = -24'sd8000000;

    sv_pi_ctrl #(.DATA_W(24), .GAIN_W(24), .GAIN_FRAC(16)) u_pi_pll (
        .clk       (clk),
        .rst_n     (rst_n),
        .tick_en   (lockin_valid_d && enable_pll),
        .int_load  (pi_load),
        .int_init  (pll_init),
        .err       (err_pll),
        .kp        (pll_kp),
        .ki        (pll_ki),
        .out_min   (-24'sd4000000),
        .out_max   ( 24'sd4000000),
        .ctrl_out  (pll_adj_out),
        .sat_flag  (sat_pll),
        .out_valid (pi_valid_pll)
    );

    sv_pi_ctrl #(.DATA_W(24), .GAIN_W(24), .GAIN_FRAC(16)) u_pi_agc (
        .clk       (clk),
        .rst_n     (rst_n),
        .tick_en   (lockin_valid_d && enable_agc),
        .int_load  (pi_load),
        .int_init  (agc_init),
        .err       (err_agc),
        .kp        (agc_kp),
        .ki        (agc_ki),
        .out_min   (MIN_U_Q14),
        .out_max   (MAX_U_Q14),
        .ctrl_out  (agc_adj_out),
        .sat_flag  (sat_agc),
        .out_valid (pi_valid_agc)
    );

    sv_pi_ctrl #(.DATA_W(24), .GAIN_W(24), .GAIN_FRAC(16)) u_pi_clq (
        .clk       (clk),
        .rst_n     (rst_n),
        .tick_en   (lockin_valid_s && enable_clq),
        .int_load  (pi_load),
        .int_init  (24'sd0),
        .err       (err_clq),
        .kp        (clq_kp),
        .ki        (clq_ki),
        .out_min   (MIN_U_Q14),
        .out_max   (MAX_U_Q14),
        .ctrl_out  (clq_adj_out),
        .sat_flag  (sat_clq),
        .out_valid (pi_valid_clq)
    );

    sv_pi_ctrl #(.DATA_W(24), .GAIN_W(24), .GAIN_FRAC(16)) u_pi_ftr (
        .clk       (clk),
        .rst_n     (rst_n),
        .tick_en   (lockin_valid_s && enable_ftr),
        .int_load  (pi_load),
        .int_init  (24'sd0),
        .err       (err_ftr),
        .kp        (ftr_kp),
        .ki        (ftr_ki),
        .out_min   (MIN_U_Q14),
        .out_max   (MAX_U_Q14),
        .ctrl_out  (ftr_adj_out),
        .sat_flag  (sat_ftr),
        .out_valid (pi_valid_ftr)
    );

    // ------------------------------------------------------------------------
    // 4. Zero-Skew Carrier Re-Modulation (t_k and t_{k+1}):
    //      u_x = AGCAdj * cos(phi)
    //      u_y = FTRAdj * cos(phi) + CLQAdj * sin(phi)
    //    Q10.14 * Q1.15 = Q11.29 -> >>> 7 -> Q10.22 (32-bit signed)
    //    - On pi_valid (clock 3 after tick_en): latch u_x_k, u_y_k with phi_k.
    //    - On nco_tvalid (clock 19 after tick_en): latch u_x_k1, u_y_k1 with phi_{k+1}
    //      and pulse plant_start so mems_plant_2dof runs exact Strang-split!
    // ------------------------------------------------------------------------
    logic signed [15:0] cos_next_q15, sin_next_q15;
    assign cos_next_q15 = signed'(nco_tdata[15:0]);
    assign sin_next_q15 = signed'(nco_tdata[31:16]);

    logic signed [39:0] ux_k_prod, uy_k_ftr_prod, uy_k_clq_prod, uy_k_sum;
    logic signed [39:0] ux_k1_prod, uy_k1_ftr_prod, uy_k1_clq_prod, uy_k1_sum;

    assign ux_k_prod      = 40'(agc_adj_out) * 40'(cos_q15);
    assign uy_k_ftr_prod  = 40'(ftr_adj_out) * 40'(cos_q15);
    assign uy_k_clq_prod  = 40'(clq_adj_out) * 40'(sin_q15);
    assign uy_k_sum       = uy_k_ftr_prod + uy_k_clq_prod;

    assign ux_k1_prod     = 40'(agc_adj_out) * 40'(cos_next_q15);
    assign uy_k1_ftr_prod = 40'(ftr_adj_out) * 40'(cos_next_q15);
    assign uy_k1_clq_prod = 40'(clq_adj_out) * 40'(sin_next_q15);
    assign uy_k1_sum      = uy_k1_ftr_prod + uy_k1_clq_prod;

    logic               remod_k_en;
    assign remod_k_en = pi_valid_pll || pi_valid_agc || pi_valid_clq || pi_valid_ftr || lockin_valid_d;

    logic signed [39:0] ux_k_rnd, uy_k_rnd, ux_k1_rnd, uy_k1_rnd;
    assign ux_k_rnd  = (ux_k_prod  + 40'sd64) >>> 7;
    assign uy_k_rnd  = (uy_k_sum   + 40'sd64) >>> 7;
    assign ux_k1_rnd = (ux_k1_prod + 40'sd64) >>> 7;
    assign uy_k1_rnd = (uy_k1_sum  + 40'sd64) >>> 7;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            u_x_k_out   <= 32'sd0;
            u_y_k_out   <= 32'sd0;
            u_x_k1_out  <= 32'sd0;
            u_y_k1_out  <= 32'sd0;
            plant_start <= 1'b0;
        end else begin
            plant_start <= 1'b0;
            if (remod_k_en) begin
                u_x_k_out <= enable_agc ? ux_k_rnd[31:0] : 32'sd0;
                u_y_k_out <= (enable_ftr || enable_clq) ? uy_k_rnd[31:0] : 32'sd0;
            end
            if (nco_tvalid) begin
                u_x_k1_out  <= enable_agc ? ux_k1_rnd[31:0] : 32'sd0;
                u_y_k1_out  <= (enable_ftr || enable_clq) ? uy_k1_rnd[31:0] : 32'sd0;
                plant_start <= 1'b1;
            end
        end
    end

    logic unused_ok;
    assign unused_ok = &{1'b0, phase_acc, nco_fire, sat_pll, sat_agc, sat_clq, sat_ftr,
                         pi_valid_pll, pi_valid_agc, pi_valid_clq, pi_valid_ftr,
                         ux_k_rnd[39:32], uy_k_rnd[39:32], ux_k1_rnd[39:32], uy_k1_rnd[39:32]};

endmodule
