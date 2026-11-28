// ============================================================================
// Module: mems_hil_top
// Description:
//   Closed-loop Hardware-in-the-Loop (HIL) top combining:
//     - 48-bit Strang-split DRG/HRG physics plant (mems_plant_2dof)
//     - Closed-loop PLL + AGC + CLQ + FTR controller (mems_ctrl_top)
//     - Programmable clock divider generating 1 MHz (or 200 kHz) physics tick
//       from 100 MHz ZUBoard 1CG pl_clk0.
// ============================================================================
`timescale 1ns / 1ps

module mems_hil_top #(
    parameter int TICK_DIV = 100 // 100 MHz / 100 = 1 MHz physics & control rate
) (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               run_enable,
    input  logic [7:0]         tick_div_cfg, // 0 = use TICK_DIV param; >=34 = runtime Turbo divider!

    // Preload controls
    input  logic               state_load,
    input  logic signed [47:0] x_init, vx_init, y_init, vy_init,
    input  logic               pi_load,
    input  logic signed [31:0] pll_init, agc_init,

    // Plant parameters (cos/sin in Q2.34; kick coeffs in Q-6.42; half_dt in Q0.36)
    input  logic signed [35:0] cos_tx, sin_tx,
    input  logic signed [35:0] cos_ty, sin_ty,
    input  logic signed [35:0] half_gx_dt, half_gy_dt,
    input  logic signed [35:0] half_kxy_dt, half_gxy_dt,
    input  logic signed [35:0] half_cor_dt, half_dt_q36,

    // Optional PRNG thermal force, pickoff ADC, and RRW bias-walk injection
    input  logic [15:0]        force_noise_amp,
    input  logic [15:0]        adc_noise_amp,
    input  logic [15:0]        rrw_step_amp,
    input  logic [31:0]        noise_seed,

    // Controller parameters
    input  logic [31:0]        base_freq_word,
    input  logic [3:0]         lpf_shift,
    input  logic               enable_pll, enable_agc, enable_clq, enable_ftr,
    input  logic               mode_swap,
    input  logic signed [23:0] dx_setpoint, sx_setpoint, sy_setpoint,
    input  logic signed [23:0] pll_kp, pll_ki,
    input  logic signed [23:0] agc_kp, agc_ki,
    input  logic signed [23:0] clq_kp, clq_ki,
    input  logic signed [23:0] ftr_kp, ftr_ki,

    // Live telemetry outputs (32-bit High-Q PI outputs!)
    output logic               tick_pulse,
    output logic               step_done,
    output logic signed [15:0] x_q14, y_q14,
    output logic signed [23:0] dx_out, dy_out, sx_out, sy_out,
    output logic signed [31:0] pll_adj_out, agc_adj_out, clq_adj_out, ftr_adj_out
);

    // Runtime-programmable Turbo tick divider (clamped >= 58 so 20-cyc NCO + 32-cyc plant = 52 cyc never overruns!)
    logic [7:0] eff_tick_div;
    logic [7:0] div_cnt;
    assign eff_tick_div = (tick_div_cfg == 8'd0)  ? 8'(TICK_DIV) :
                          (tick_div_cfg < 8'd58)  ? 8'd58        :
                                                    tick_div_cfg;

    always_ff @(posedge clk) begin
        if (!rst_n || !run_enable) begin
            div_cnt    <= 8'd0;
            tick_pulse <= 1'b0;
        end else if (div_cnt >= (eff_tick_div - 8'd1)) begin
            div_cnt    <= 8'd0;
            tick_pulse <= 1'b1;
        end else begin
            div_cnt    <= div_cnt + 8'd1;
            tick_pulse <= 1'b0;
        end
    end

    logic signed [31:0] u_x_k_bus, u_y_k_bus, u_x_k1_bus, u_y_k1_bus;
    logic               plant_start_bus;
    logic signed [23:0] x_q22_bus, y_q22_bus;

    mems_ctrl_top u_ctrl (
        .clk            (clk),
        .rst_n          (rst_n),
        .tick_en        (tick_pulse),
        .x_q22          (x_q22_bus),
        .y_q22          (y_q22_bus),
        .base_freq_word (base_freq_word),
        .lpf_shift      (lpf_shift),
        .enable_pll     (enable_pll),
        .enable_agc     (enable_agc),
        .enable_clq     (enable_clq),
        .enable_ftr     (enable_ftr),
        .mode_swap      (mode_swap),
        .dx_setpoint    (dx_setpoint),
        .sx_setpoint    (sx_setpoint),
        .sy_setpoint    (sy_setpoint),
        .pll_kp         (pll_kp),
        .pll_ki         (pll_ki),
        .agc_kp         (agc_kp),
        .agc_ki         (agc_ki),
        .clq_kp         (clq_kp),
        .clq_ki         (clq_ki),
        .ftr_kp         (ftr_kp),
        .ftr_ki         (ftr_ki),
        .pi_load        (pi_load),
        .pll_init       (pll_init),
        .agc_init       (agc_init),
        .u_x_k_out      (u_x_k_bus),
        .u_y_k_out      (u_y_k_bus),
        .u_x_k1_out     (u_x_k1_bus),
        .u_y_k1_out     (u_y_k1_bus),
        .plant_start    (plant_start_bus),
        .dx_out         (dx_out),
        .dy_out         (dy_out),
        .sx_out         (sx_out),
        .sy_out         (sy_out),
        .pll_adj_out    (pll_adj_out),
        .agc_adj_out    (agc_adj_out),
        .clq_adj_out    (clq_adj_out),
        .ftr_adj_out    (ftr_adj_out)
    );

    logic signed [47:0] x_state_nc, vx_state_nc, y_state_nc, vy_state_nc;

    mems_plant_2dof u_plant (
        .clk         (clk),
        .rst_n       (rst_n),
        .tick_en     (plant_start_bus),
        .state_load  (state_load),
        .x_init      (x_init),
        .vx_init     (vx_init),
        .y_init      (y_init),
        .vy_init     (vy_init),
        .cos_tx      (cos_tx),
        .sin_tx      (sin_tx),
        .cos_ty      (cos_ty),
        .sin_ty      (sin_ty),
        .half_gx_dt  (half_gx_dt),
        .half_gy_dt  (half_gy_dt),
        .half_kxy_dt (half_kxy_dt),
        .half_gxy_dt (half_gxy_dt),
        .half_cor_dt (half_cor_dt),
        .half_dt_q36 (half_dt_q36),
        .u_x_k           (u_x_k_bus),
        .u_y_k           (u_y_k_bus),
        .u_x_k1          (u_x_k1_bus),
        .u_y_k1          (u_y_k1_bus),
        .force_noise_amp (force_noise_amp),
        .adc_noise_amp   (adc_noise_amp),
        .rrw_step_amp    (rrw_step_amp),
        .noise_seed      (noise_seed),
        .x_q14           (x_q14),
        .y_q14           (y_q14),
        .x_q22           (x_q22_bus),
        .y_q22           (y_q22_bus),
        .x_state     (x_state_nc),
        .vx_state    (vx_state_nc),
        .y_state     (y_state_nc),
        .vy_state    (vy_state_nc),
        .step_done   (step_done)
    );

    logic unused_ok;
    assign unused_ok = &{1'b0, x_state_nc, vx_state_nc, y_state_nc, vy_state_nc};

endmodule
