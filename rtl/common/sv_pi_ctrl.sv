// ============================================================================
// Module: sv_pi_ctrl
// Description:
//   Reusable saturated anti-windup fixed-point PI controller.
//   - Input `err`   : signed DATA_W (default 24-bit)
//   - Gains `kp,ki` : signed GAIN_W (default 24-bit, with GAIN_FRAC fractional bits;
//                     `ki` is pre-multiplied by sample period Ts)
//   - Internal 48-bit signed integrator prevents deadzone quantization for
//     high-Q / slow integral loops (DRG/HRG PLL, AGC, CLQ, FTR).
//   - Conditional-integration anti-windup freezes integrator when saturated
//     in the same direction as the incoming error.
// ============================================================================
`timescale 1ns / 1ps

module sv_pi_ctrl #(
    parameter int DATA_W    = 24,
    parameter int GAIN_W    = 24,
    parameter int GAIN_FRAC = 16,
    parameter int ACC_W     = 48
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     tick_en,

    // Runtime preload for bumpless mode transitions (e.g. R5F startup sweep -> lock)
    input  logic                     int_load,
    input  logic signed [DATA_W-1:0] int_init,

    input  logic signed [DATA_W-1:0] err,
    input  logic signed [GAIN_W-1:0] kp,
    input  logic signed [GAIN_W-1:0] ki,
    input  logic signed [DATA_W-1:0] out_min,
    input  logic signed [DATA_W-1:0] out_max,

    output logic signed [DATA_W-1:0] ctrl_out,
    output logic                     sat_flag,
    output logic                     out_valid
);

    // Integrator is stored shifted left by GAIN_FRAC so ki*err accumulates with full precision
    logic signed [ACC_W-1:0] int_acc;

    // Combinational P and I products (48-bit signed)
    logic signed [ACC_W-1:0] p_prod;
    logic signed [ACC_W-1:0] i_prod;
    logic signed [ACC_W-1:0] sum_unscaled;
    logic signed [ACC_W-1:0] sum_scaled;
    logic signed [ACC_W-1:0] min_ext;
    logic signed [ACC_W-1:0] max_ext;

    logic sat_hi;
    logic sat_lo;
    logic allow_integrate;

    assign p_prod = ACC_W'(err) * ACC_W'(kp);
    assign i_prod = ACC_W'(err) * ACC_W'(ki);

    assign sum_unscaled = int_acc + p_prod;
    assign sum_scaled   = (sum_unscaled + (ACC_W'(1) <<< (GAIN_FRAC - 1))) >>> GAIN_FRAC;

    assign min_ext = ACC_W'(out_min);
    assign max_ext = ACC_W'(out_max);

    assign sat_hi = (sum_scaled > max_ext);
    assign sat_lo = (sum_scaled < min_ext);

    // Anti-windup: freeze integrator only if saturated AND error pushes deeper into saturation
    assign allow_integrate = !((sat_hi && (i_prod > 0)) || (sat_lo && (i_prod < 0)));

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            int_acc   <= '0;
            ctrl_out  <= '0;
            sat_flag  <= 1'b0;
            out_valid <= 1'b0;
        end else begin
            out_valid <= 1'b0;
            if (int_load) begin
                int_acc  <= ACC_W'(int_init) <<< GAIN_FRAC;
                ctrl_out <= int_init;
                sat_flag <= 1'b0;
            end else if (tick_en) begin
                if (allow_integrate) begin
                    int_acc <= int_acc + i_prod;
                end

                if (sat_hi) begin
                    ctrl_out <= out_max;
                    sat_flag <= 1'b1;
                end else if (sat_lo) begin
                    ctrl_out <= out_min;
                    sat_flag <= 1'b1;
                end else begin
                    ctrl_out <= sum_scaled[DATA_W-1:0];
                    sat_flag <= 1'b0;
                end
                out_valid <= 1'b1;
            end
        end
    end

endmodule
