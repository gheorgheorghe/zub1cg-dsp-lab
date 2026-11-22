// ============================================================================
// Module: stage0_bd_wrap
// Description:
//   Thin Verilog-2001 wrapper combining axil_regfile and stage0_dsp_slot so
//   Vivado IP Integrator (`create_bd_cell -type module`) can instantiate the
//   SystemVerilog modules cleanly with automatic AXI-Lite & AXI-Stream inference.
// ============================================================================
`timescale 1ns / 1ps

module stage0_bd_wrap (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi:s_axis:m_axis, ASSOCIATED_RESET aresetn" *)
    input  wire        aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        aresetn,

    // AXI4-Lite Slave
    input  wire [5:0]  s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,
    output wire [1:0]  s_axi_bresp,
    output wire        s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [5:0]  s_axi_araddr,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,
    output wire [31:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,
    output wire        s_axi_rvalid,
    input  wire        s_axi_rready,

    // AXI4-Stream Slave (from AXI-DMA MM2S)
    input  wire [31:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast,

    // AXI4-Stream Master (to AXI-DMA S2MM)
    output wire [31:0] m_axis_tdata,
    output wire        m_axis_tvalid,
    input  wire        m_axis_tready,
    output wire        m_axis_tlast
);

    wire        ctrl_enable;
    wire        ctrl_loopback;
    wire        ctrl_soft_rst;
    wire [31:0] nco_freq_word;
    wire [31:0] pkt_len;
    wire [31:0] nco_phase_off;
    wire [31:0] status_bus;
    wire [31:0] sample_cnt_bus;

    axil_regfile u_regfile (
        .aclk          (aclk),
        .aresetn       (aresetn),
        .s_axi_awaddr  (s_axi_awaddr),
        .s_axi_awvalid (s_axi_awvalid),
        .s_axi_awready (s_axi_awready),
        .s_axi_wdata   (s_axi_wdata),
        .s_axi_wstrb   (s_axi_wstrb),
        .s_axi_wvalid  (s_axi_wvalid),
        .s_axi_wready  (s_axi_wready),
        .s_axi_bresp   (s_axi_bresp),
        .s_axi_bvalid  (s_axi_bvalid),
        .s_axi_bready  (s_axi_bready),
        .s_axi_araddr  (s_axi_araddr),
        .s_axi_arvalid (s_axi_arvalid),
        .s_axi_arready (s_axi_arready),
        .s_axi_rdata   (s_axi_rdata),
        .s_axi_rresp   (s_axi_rresp),
        .s_axi_rvalid  (s_axi_rvalid),
        .s_axi_rready  (s_axi_rready),
        .ctrl_enable   (ctrl_enable),
        .ctrl_loopback (ctrl_loopback),
        .ctrl_soft_rst (ctrl_soft_rst),
        .nco_freq_word (nco_freq_word),
        .pkt_len       (pkt_len),
        .nco_phase_off (nco_phase_off),
        .user_cfg      (),
        .status_in     (status_bus),
        .sample_cnt_in (sample_cnt_bus)
    );

    stage0_dsp_slot u_slot (
        .aclk           (aclk),
        .aresetn        (aresetn),
        .ctrl_enable    (ctrl_enable),
        .ctrl_loopback  (ctrl_loopback),
        .ctrl_soft_rst  (ctrl_soft_rst),
        .nco_freq_word  (nco_freq_word),
        .pkt_len        (pkt_len),
        .nco_phase_off  (nco_phase_off),
        .status_out     (status_bus),
        .sample_cnt_out (sample_cnt_bus),
        .s_axis_tdata   (s_axis_tdata),
        .s_axis_tvalid  (s_axis_tvalid),
        .s_axis_tready  (s_axis_tready),
        .s_axis_tlast   (s_axis_tlast),
        .m_axis_tdata   (m_axis_tdata),
        .m_axis_tvalid  (m_axis_tvalid),
        .m_axis_tready  (m_axis_tready),
        .m_axis_tlast   (m_axis_tlast)
    );

endmodule
