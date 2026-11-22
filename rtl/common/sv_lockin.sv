// ============================================================================
// Module: sv_lockin
// Description:
//   3-stage cascaded I/Q lock-in demodulator (zero-DSP shift-based LPF,
//   2 DSP48E2 multipliers for I/Q mixing).
//
//   Convention matching model/mems_gyro_model.py:
//     out_i = LPF_3[ 2 * sig_in * sin_in ]   (Drive: DX amplitude | Sense: SX rate)
//     out_q = LPF_3[ 2 * sig_in * cos_in ]   (Drive: DY phase err | Sense: SY quad)
//
//   Fixed-point scaling:
//     sig_in : signed 16-bit Q2.14 (1.0 = 16384)
//     sin_in, cos_in : signed 16-bit Q1.15 from sv_nco_cordic (1.0 = 32760)
//     2 * sig_in * sin_in -> Q3.29 (32-bit signed); top 24 bits = Q3.21.
//     3-stage 1-pole IIR: y[n+1] = y[n] + ((x[n] - y[n]) >>> lpf_shift)
//       At fs = 1 MHz, lpf_shift = 8 -> fc ~ 622 Hz, >90 dB rejection at 22 kHz.
//     out_i, out_q : signed 24-bit Q2.22 (1.0 = 4,194,304)
// ============================================================================
`timescale 1ns / 1ps

module sv_lockin #(
    parameter int OUT_W = 24
) (
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    tick_en,

    input  logic [3:0]              lpf_shift, // 1..12 (fc ~ fs / (2*pi*2^lpf_shift))
    input  logic signed [15:0]      sig_in,    // Q2.14 pickoff
    input  logic signed [15:0]      cos_in,    // Q1.15 NCO cos
    input  logic signed [15:0]      sin_in,    // Q1.15 NCO sin

    output logic signed [OUT_W-1:0] out_i,     // Q2.22: LPF(2 * sig * sin)
    output logic signed [OUT_W-1:0] out_q,     // Q2.22: LPF(2 * sig * cos)
    output logic                    out_valid
);

    // Stage 0: Registered I/Q mixing (maps cleanly to 2 DSP48E2 slices)
    // sig_in (Q2.14) * sin_in (Q1.15) = Q3.29; * 2 => Q2.30 (fits in signed 32-bit since |sig|<2, |sin|<=1)
    // Store internal LPF states in signed 36-bit Q4.32 for zero truncation noise at high lpf_shift.
    localparam int LPF_W = 36;

    logic signed [31:0] mix_i_reg;
    logic signed [31:0] mix_q_reg;
    logic               mix_valid;

    logic signed [LPF_W-1:0] i_s1, i_s2, i_s3;
    logic signed [LPF_W-1:0] q_s1, q_s2, q_s3;

    logic signed [LPF_W-1:0] mix_i_q32;
    logic signed [LPF_W-1:0] mix_q_q32;

    // Q2.14 * Q1.15 = Q3.29; *2 (lock-in 2x factor) => Q2.30; shift left 3 -> Q4.32 (total <<< 4)
    assign mix_i_q32 = LPF_W'(mix_i_reg) <<< 4;
    assign mix_q_q32 = LPF_W'(mix_q_reg) <<< 4;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            mix_i_reg <= '0;
            mix_q_reg <= '0;
            mix_valid <= 1'b0;
            i_s1      <= '0;
            i_s2      <= '0;
            i_s3      <= '0;
            q_s1      <= '0;
            q_s2      <= '0;
            q_s3      <= '0;
            out_valid <= 1'b0;
        end else begin
            mix_valid <= tick_en;
            if (tick_en) begin
                // Q2.14 * Q1.15 -> Q3.29 (shifted left by 3 above = *2 -> Q2.30, +2 guard -> Q4.32)
                mix_i_reg <= 32'(sig_in) * 32'(sin_in);
                mix_q_reg <= 32'(sig_in) * 32'(cos_in);
            end

            out_valid <= mix_valid;
            if (mix_valid) begin
                i_s1 <= i_s1 + ((mix_i_q32 - i_s1) >>> lpf_shift);
                i_s2 <= i_s2 + ((i_s1      - i_s2) >>> lpf_shift);
                i_s3 <= i_s3 + ((i_s2      - i_s3) >>> lpf_shift);

                q_s1 <= q_s1 + ((mix_q_q32 - q_s1) >>> lpf_shift);
                q_s2 <= q_s2 + ((q_s1      - q_s2) >>> lpf_shift);
                q_s3 <= q_s3 + ((q_s2      - q_s3) >>> lpf_shift);
            end
        end
    end

    // Convert Q4.32 -> Q2.22 (24-bit signed) with round-to-nearest
    logic signed [LPF_W-1:0] i_rnd;
    logic signed [LPF_W-1:0] q_rnd;
    assign i_rnd = i_s3 + (LPF_W'(1) <<< 9);
    assign q_rnd = q_s3 + (LPF_W'(1) <<< 9);

    assign out_i = i_rnd[10 +: OUT_W];
    assign out_q = q_rnd[10 +: OUT_W];

    logic unused_ok;
    assign unused_ok = &{1'b0, i_rnd[LPF_W-1:10+OUT_W], i_rnd[9:0],
                         q_rnd[LPF_W-1:10+OUT_W], q_rnd[9:0]};

endmodule
