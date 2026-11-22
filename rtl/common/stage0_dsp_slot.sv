// ============================================================================
// Module: stage0_dsp_slot
// Description:
//   Stage 0 swappable AXI4-Stream DSP slot for the ZUBoard 1CG shell.
//   Supports two runtime-selectable modes via AXI-Lite CTRL[1]:
//     - loopback == 0: Streams Q1.15 {sin, cos} from sv_nco_cordic and inserts
//                      m_axis_tlast every `pkt_len` samples for AXI-DMA S2MM.
//     - loopback == 1: Passes s_axis (AXI-DMA MM2S) straight to m_axis (S2MM).
// ============================================================================
`timescale 1ns / 1ps

module stage0_dsp_slot (
    input  logic        aclk,
    input  logic        aresetn,

    // Control inputs from axil_regfile
    input  logic        ctrl_enable,
    input  logic        ctrl_loopback,
    input  logic        ctrl_soft_rst,
    input  logic [31:0] nco_freq_word,
    input  logic [31:0] pkt_len,
    input  logic [31:0] nco_phase_off,

    // Status outputs to axil_regfile
    output logic [31:0] status_out,
    output logic [31:0] sample_cnt_out,

    // AXI4-Stream Slave (from AXI-DMA MM2S)
    input  logic [31:0] s_axis_tdata,
    input  logic        s_axis_tvalid,
    output logic        s_axis_tready,
    input  logic        s_axis_tlast,

    // AXI4-Stream Master (to AXI-DMA S2MM)
    output logic [31:0] m_axis_tdata,
    output logic        m_axis_tvalid,
    input  logic        m_axis_tready,
    output logic        m_axis_tlast
);

    logic slot_rst_n;
    assign slot_rst_n = aresetn && !ctrl_soft_rst;

    // NCO instance
    logic [31:0] nco_tdata;
    logic        nco_tvalid;
    logic        nco_tready;

    sv_nco_cordic u_nco (
        .clk           (aclk),
        .rst_n         (slot_rst_n),
        .enable        (ctrl_enable && !ctrl_loopback),
        .freq_word     (nco_freq_word),
        .phase_offset  (nco_phase_off),
        .m_axis_tdata  (nco_tdata),
        .m_axis_tvalid (nco_tvalid),
        .m_axis_tready (nco_tready)
    );

    // Frame length counter for NCO mode (AXI-DMA S2MM requires TLAST to complete a transfer)
    logic [31:0] pkt_cnt;
    logic [31:0] eff_pkt_len;
    logic        nco_tlast;

    assign eff_pkt_len = (pkt_len == 32'd0) ? 32'd1 : pkt_len;
    assign nco_tlast   = (pkt_cnt == (eff_pkt_len - 32'd1));

    // Output mux between NCO stream and MM2S loopback
    assign m_axis_tdata  = ctrl_loopback ? s_axis_tdata  : nco_tdata;
    assign m_axis_tvalid = ctrl_loopback ? s_axis_tvalid : nco_tvalid;
    assign m_axis_tlast  = ctrl_loopback ? s_axis_tlast  : nco_tlast;

    assign s_axis_tready = ctrl_loopback ? m_axis_tready : 1'b0;
    assign nco_tready    = ctrl_loopback ? 1'b0          : m_axis_tready;

    // Counters & status
    logic [31:0] total_samples;

    always_ff @(posedge aclk) begin
        if (!slot_rst_n) begin
            pkt_cnt       <= 32'd0;
            total_samples <= 32'd0;
        end else if (m_axis_tvalid && m_axis_tready) begin
            total_samples <= total_samples + 32'd1;
            if (!ctrl_loopback) begin
                if (nco_tlast) begin
                    pkt_cnt <= 32'd0;
                end else begin
                    pkt_cnt <= pkt_cnt + 32'd1;
                end
            end else begin
                pkt_cnt <= 32'd0;
            end
        end
    end

    assign sample_cnt_out = total_samples;
    assign status_out     = {16'h0000, 8'h00 /* Stage 0 */, 5'b0, ctrl_loopback, ctrl_enable, nco_tvalid};

endmodule
