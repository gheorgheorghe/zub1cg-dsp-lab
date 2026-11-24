// ============================================================================
// Module: axil_regfile
// Description:
//   Clean, AXI4-Lite compliant 32-bit register bank designed to work reliably
//   with Vivado AXI SmartConnect (independent AW and W channel acceptance).
//
//   Register Map (byte offsets):
//     0x00 (RW) CTRL          [0]=enable, [1]=loopback_mode, [2]=soft_reset
//     0x04 (RW) NCO_FREQ_WORD 32-bit NCO phase increment
//     0x08 (RW) PKT_LEN       AXI-DMA frame length in 32-bit samples (default 1024)
//     0x0C (RW) NCO_PHASE_OFF 32-bit NCO phase offset
//     0x10..0x2C (RW) USER_CFG[0..7] Reserved for Stage 1..3 parameters
//     0x30 (RO) STATUS        [0]=slot_valid, [1]=enable_mirror, [15:8]=stage_id
//     0x34 (RO) SAMPLE_CNT    Rolling count of S2MM samples emitted
//     0x38 (RO) VERSION       32'h5A55_0001 ("ZU" Stage 0 v1)
// ============================================================================
`timescale 1ns / 1ps

module axil_regfile #(
    parameter int ADDR_W = 6,   // 64-byte address space (16 x 32-bit regs)
    parameter logic [31:0] VERSION_ID = 32'h5A55_0001
) (
    input  logic              aclk,
    input  logic              aresetn,

    // AXI4-Lite Slave Interface
    input  logic [ADDR_W-1:0] s_axi_awaddr,
    input  logic              s_axi_awvalid,
    output logic              s_axi_awready,

    input  logic [31:0]       s_axi_wdata,
    input  logic [3:0]        s_axi_wstrb,
    input  logic              s_axi_wvalid,
    output logic              s_axi_wready,

    output logic [1:0]        s_axi_bresp,
    output logic              s_axi_bvalid,
    input  logic              s_axi_bready,

    input  logic [ADDR_W-1:0] s_axi_araddr,
    input  logic              s_axi_arvalid,
    output logic              s_axi_arready,

    output logic [31:0]       s_axi_rdata,
    output logic [1:0]        s_axi_rresp,
    output logic              s_axi_rvalid,
    input  logic              s_axi_rready,

    // Decoded Control Outputs to DSP Slot
    output logic              ctrl_enable,
    output logic              ctrl_loopback,
    output logic              ctrl_soft_rst,
    output logic [31:0]       nco_freq_word,
    output logic [31:0]       pkt_len,
    output logic [31:0]       nco_phase_off,
    output logic [255:0]      user_cfg,

    // Status Inputs from DSP Slot
    input  logic [31:0]       status_in,
    input  logic [31:0]       sample_cnt_in
);

    localparam int NUM_RW_REGS = 12; // 0x00 .. 0x2C
    logic [31:0] rw_regs [0:NUM_RW_REGS-1];

    // Write address / data latching for independent AW and W handshakes
    logic              aw_latched;
    logic [ADDR_W-1:0] aw_addr_reg;
    logic              w_latched;
    logic [31:0]       w_data_reg;
    logic [3:0]        w_strb_reg;

    assign s_axi_awready = !aw_latched && !s_axi_bvalid;
    assign s_axi_wready  = !w_latched  && !s_axi_bvalid;
    assign s_axi_bresp   = 2'b00; // OKAY

    logic wr_fire;
    assign wr_fire = (aw_latched || (s_axi_awvalid && s_axi_awready)) &&
                     (w_latched  || (s_axi_wvalid  && s_axi_wready));

    logic [ADDR_W-1:0] wr_addr_eff;
    logic [31:0]       wr_data_eff;
    logic [3:0]        wr_strb_eff;
    logic [3:0]        wr_reg_idx;

    assign wr_addr_eff = aw_latched ? aw_addr_reg : s_axi_awaddr;
    assign wr_data_eff = w_latched  ? w_data_reg  : s_axi_wdata;
    assign wr_strb_eff = w_latched  ? w_strb_reg  : s_axi_wstrb;
    assign wr_reg_idx  = wr_addr_eff[5:2];

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            aw_latched   <= 1'b0;
            w_latched    <= 1'b0;
            aw_addr_reg  <= '0;
            w_data_reg   <= '0;
            w_strb_reg   <= '0;
            s_axi_bvalid <= 1'b0;

            for (int i = 0; i < NUM_RW_REGS; i++) begin
                rw_regs[i] <= 32'h0;
            end
            // Sensible Stage 0 defaults: 16 kHz @ 100 MHz, 1024-sample DMA packets
            rw_regs[1] <= 32'd687195; // ~16.000 kHz at 100 MHz
            rw_regs[2] <= 32'd1024;   // default pkt_len
        end else begin
            // Clear response when accepted
            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end

            if (wr_fire) begin
                if (wr_reg_idx < 4'd12) begin
                    for (int b = 0; b < 4; b++) begin
                        if (wr_strb_eff[b]) begin
                            rw_regs[wr_reg_idx][b*8 +: 8] <= wr_data_eff[b*8 +: 8];
                        end
                    end
                end
                aw_latched   <= 1'b0;
                w_latched    <= 1'b0;
                s_axi_bvalid <= 1'b1;
            end else begin
                if (s_axi_awvalid && s_axi_awready) begin
                    aw_latched  <= 1'b1;
                    aw_addr_reg <= s_axi_awaddr;
                end
                if (s_axi_wvalid && s_axi_wready) begin
                    w_latched  <= 1'b1;
                    w_data_reg <= s_axi_wdata;
                    w_strb_reg <= s_axi_wstrb;
                end
            end
        end
    end

    // Read channel
    assign s_axi_arready = !s_axi_rvalid;
    assign s_axi_rresp   = 2'b00; // OKAY

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            s_axi_rvalid <= 1'b0;
            s_axi_rdata  <= 32'h0;
        end else begin
            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
            if (s_axi_arvalid && s_axi_arready) begin
                s_axi_rvalid <= 1'b1;
                unique case (s_axi_araddr[5:2])
                    4'h0, 4'h1, 4'h2, 4'h3,
                    4'h4, 4'h5, 4'h6, 4'h7,
                    4'h8, 4'h9, 4'hA, 4'hB: s_axi_rdata <= rw_regs[s_axi_araddr[5:2]];
                    4'hC:                   s_axi_rdata <= status_in;
                    4'hD:                   s_axi_rdata <= sample_cnt_in;
                    4'hE:                   s_axi_rdata <= VERSION_ID;
                    default:                s_axi_rdata <= 32'hDEAD_BEEF;
                endcase
            end
        end
    end

    // Map outputs
    assign ctrl_enable   = rw_regs[0][0];
    assign ctrl_loopback = rw_regs[0][1];
    assign ctrl_soft_rst = rw_regs[0][2];
    assign nco_freq_word = rw_regs[1];
    assign pkt_len       = rw_regs[2];
    assign nco_phase_off = rw_regs[3];

    always_comb begin
        for (int i = 0; i < 8; i++) begin
            user_cfg[i*32 +: 32] = rw_regs[4 + i];
        end
    end

    logic unused_ok;
    assign unused_ok = &{1'b0, wr_addr_eff[1:0], s_axi_araddr[1:0], rw_regs[0][31:3]};

endmodule
