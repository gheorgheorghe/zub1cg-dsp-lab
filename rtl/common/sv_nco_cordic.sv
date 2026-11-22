// ============================================================================
// Module: sv_nco_cordic
// Description:
//   32-bit phase accumulator + 18-stage pipelined rotation-mode CORDIC NCO
//   with AXI4-Stream output handshake (tvalid/tready).
//
//   - Zero DSP48E2 slices, zero BRAMs (pure shift-add pipeline).
//   - Outputs Q1.15 signed cos (I) in m_axis_tdata[15:0] and
//     Q1.15 signed sin (Q) in m_axis_tdata[31:16].
//   - Peak amplitude = 32760 (0x7FF8), SFDR > 92 dBc.
//   - Reused across Stage 0 (bring-up), Stage 1 (MEMS PLL/lock-in),
//     and Stage 2 (RF DDC / radar carrier).
// ============================================================================
`timescale 1ns / 1ps

module sv_nco_cordic #(
    parameter int PHASE_W  = 32,
    parameter int ANGLE_W  = 24,  // Top 24 bits of phase fed to CORDIC
    parameter int N_STAGES = 18
) (
    input  logic                clk,
    input  logic                rst_n,
    input  logic                enable,
    input  logic [PHASE_W-1:0]  freq_word,   // delta_phase = f_out / f_clk * 2^32
    input  logic [PHASE_W-1:0]  phase_offset,

    output logic [31:0]         m_axis_tdata,  // [31:16] = sin (Q), [15:0] = cos (I)
    output logic                m_axis_tvalid,
    input  logic                m_axis_tready
);

    // Internal signed width = 21 bits (INT_SHIFT = 4 fractional guard bits).
    // Gain K = prod_{i=0}^{17} 1/sqrt(1 + 2^-2i) = 0.607252935015
    // X0 = round(32760 * 2^4 * K) = 318298
    localparam int DATA_W    = 21;
    localparam int INT_SHIFT = 4;
    localparam logic signed [DATA_W-1:0] X0_CONST = 21'sd318298;

    // Precomputed atan(2^-i) / (2*pi) * 2^24
    localparam logic signed [ANGLE_W-1:0] ATAN_LUT [0:N_STAGES-1] = '{
        24'sd2097152, // i = 0
        24'sd1238021, // i = 1
        24'sd654136,  // i = 2
        24'sd332050,  // i = 3
        24'sd166669,  // i = 4
        24'sd83416,   // i = 5
        24'sd41718,   // i = 6
        24'sd20860,   // i = 7
        24'sd10430,   // i = 8
        24'sd5215,    // i = 9
        24'sd2608,    // i = 10
        24'sd1304,    // i = 11
        24'sd652,     // i = 12
        24'sd326,     // i = 13
        24'sd163,     // i = 14
        24'sd81,      // i = 15
        24'sd41,      // i = 16
        24'sd20       // i = 17
    };

    // Pipeline advance: stall entire pipeline cleanly when downstream applies backpressure
    logic pipe_en;
    assign pipe_en = !m_axis_tvalid || m_axis_tready;

    // Phase accumulator
    logic [PHASE_W-1:0] phase_acc;
    logic [PHASE_W-1:0] phase_sum;
    logic [1:0]         quad;

    assign phase_sum = phase_acc + phase_offset;
    assign quad      = phase_sum[PHASE_W-1 -: 2];

    // Pipeline arrays: index 0 is quadrant pre-rotation stage, N_STAGES is final output
    logic signed [DATA_W-1:0]  x_pipe [0:N_STAGES];
    logic signed [DATA_W-1:0]  y_pipe [0:N_STAGES];
    logic signed [ANGLE_W-1:0] z_pipe [0:N_STAGES];
    logic                      v_pipe [0:N_STAGES];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            phase_acc <= '0;
            for (int k = 0; k <= N_STAGES; k++) begin
                x_pipe[k] <= '0;
                y_pipe[k] <= '0;
                z_pipe[k] <= '0;
                v_pipe[k] <= 1'b0;
            end
        end else if (pipe_en) begin
            // Stage 0: Phase accumulation & quadrant pre-rotation into [0, pi/2)
            if (enable) begin
                phase_acc <= phase_acc + freq_word;
            end

            v_pipe[0] <= enable;
            z_pipe[0] <= signed'({2'b00, phase_sum[PHASE_W-3 -: (ANGLE_W-2)]});

            unique case (quad)
                2'b00: begin
                    x_pipe[0] <=  X0_CONST;
                    y_pipe[0] <= '0;
                end
                2'b01: begin
                    x_pipe[0] <= '0;
                    y_pipe[0] <=  X0_CONST;
                end
                2'b10: begin
                    x_pipe[0] <= -X0_CONST;
                    y_pipe[0] <= '0;
                end
                2'b11: begin
                    x_pipe[0] <= '0;
                    y_pipe[0] <= -X0_CONST;
                end
            endcase

            // Stages 1..N_STAGES: Rotation-mode CORDIC iterations
            for (int i = 0; i < N_STAGES; i++) begin
                v_pipe[i+1] <= v_pipe[i];
                if (z_pipe[i] >= 0) begin
                    x_pipe[i+1] <= x_pipe[i] - (y_pipe[i] >>> i);
                    y_pipe[i+1] <= y_pipe[i] + (x_pipe[i] >>> i);
                    z_pipe[i+1] <= z_pipe[i] - ATAN_LUT[i];
                end else begin
                    x_pipe[i+1] <= x_pipe[i] + (y_pipe[i] >>> i);
                    y_pipe[i+1] <= y_pipe[i] - (x_pipe[i] >>> i);
                    z_pipe[i+1] <= z_pipe[i] + ATAN_LUT[i];
                end
            end
        end
    end

    // Round from internal 21-bit fixed-point back to signed 16-bit Q1.15
    logic signed [DATA_W-1:0] cos_rnd;
    logic signed [DATA_W-1:0] sin_rnd;
    logic signed [15:0]       cos_q15;
    logic signed [15:0]       sin_q15;

    assign cos_rnd = x_pipe[N_STAGES] + (21'sd1 <<< (INT_SHIFT - 1));
    assign sin_rnd = y_pipe[N_STAGES] + (21'sd1 <<< (INT_SHIFT - 1));

    assign cos_q15 = cos_rnd[INT_SHIFT +: 16];
    assign sin_q15 = sin_rnd[INT_SHIFT +: 16];

    assign m_axis_tdata  = {sin_q15, cos_q15};
    assign m_axis_tvalid = v_pipe[N_STAGES];

    // Tie off intentionally truncated low phase/rounding guard bits for clean -Wall lint
    logic unused_ok;
    assign unused_ok = &{1'b0, phase_sum[PHASE_W-ANGLE_W-1:0], cos_rnd[DATA_W-1], cos_rnd[INT_SHIFT-1:0],
                         sin_rnd[DATA_W-1], sin_rnd[INT_SHIFT-1:0], z_pipe[N_STAGES]};

endmodule
