// ============================================================================
// Module: mems_hil_axis_wrap
// Description:
//   Stage 1 ZUBoard 1CG Top-Level Slot combining:
//     - 256-byte AXI4-Lite slave register file (runtime DRG/HRG preset loading,
//       live Omega_z(t) & thermal frequency drift injection, live RO snapshots)
//     - mems_hil_top (48-bit Strang-split DRG/HRG plant + PLL/AGC/CLQ/FTR loops)
//     - Decimated 8-word AXI4-Stream telemetry framer -> AXI-DMA S2MM:
//         Word 0: DX (Q2.22, sign-extended to 32-bit)
//         Word 1: DY (Q2.22)
//         Word 2: SX (Q2.22)
//         Word 3: SY (Q2.22)
//         Word 4: PLLAdj (24-bit signed, sign-extended)
//         Word 5: AGCAdj (Q10.14)
//         Word 6: CLQAdj (Q10.14)
//         Word 7: FTRAdj (Q10.14)  <-- TLAST asserted on last frame of burst
// ============================================================================
`timescale 1ns / 1ps

module mems_hil_axis_wrap #(
    parameter int TICK_DIV = 100 // 100 MHz / 100 = 1 MHz HIL physics & control rate
) (
    input  logic        aclk,
    input  logic        aresetn,

    // AXI4-Lite Slave (256-byte address space)
    input  logic [7:0]  s_axi_awaddr,
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0]  s_axi_wstrb,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready,
    input  logic [7:0]  s_axi_araddr,
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,
    output logic [31:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready,

    // AXI4-Stream Slave (from AXI-DMA MM2S: optional live rate stimulus stream)
    input  logic [31:0] s_axis_tdata,
    input  logic        s_axis_tvalid,
    output logic        s_axis_tready,
    input  logic        s_axis_tlast,

    // AXI4-Stream Master (to AXI-DMA S2MM: 8-word decimated telemetry frames)
    output logic [31:0] m_axis_tdata,
    output logic        m_axis_tvalid,
    input  logic        m_axis_tready,
    output logic        m_axis_tlast
);

    localparam logic [31:0] VERSION_ID = 32'h5A55_0101; // "ZU" Stage 1 v1
    localparam int NUM_RW_REGS = 41;                   // Byte offsets 0x00 .. 0xA0

    logic [31:0] rw_regs [0:NUM_RW_REGS-1];

    // ------------------------------------------------------------------------
    // 1. AXI4-Lite Slave (independent AW/W acceptance for Vivado SmartConnect)
    // ------------------------------------------------------------------------
    logic       aw_latched, w_latched;
    logic [7:0] aw_addr_reg;
    logic [31:0] w_data_reg;
    logic [3:0]  w_strb_reg;

    assign s_axi_awready = !aw_latched && !s_axi_bvalid;
    assign s_axi_wready  = !w_latched  && !s_axi_bvalid;
    assign s_axi_bresp   = 2'b00;

    logic wr_fire;
    assign wr_fire = (aw_latched || (s_axi_awvalid && s_axi_awready)) &&
                     (w_latched  || (s_axi_wvalid  && s_axi_wready));

    logic [7:0]  wr_addr_eff;
    logic [31:0] wr_data_eff;
    logic [3:0]  wr_strb_eff;
    logic [5:0]  wr_idx;

    assign wr_addr_eff = aw_latched ? aw_addr_reg : s_axi_awaddr;
    assign wr_data_eff = w_latched  ? w_data_reg  : s_axi_wdata;
    assign wr_strb_eff = w_latched  ? w_strb_reg  : s_axi_wstrb;
    assign wr_idx      = wr_addr_eff[7:2];

    // Auto-clearing 1-cycle strobes triggered by writes to CTRL[6] / CTRL[7]
    logic state_load_pulse;
    logic pi_load_pulse;

    // Live telemetry buses from mems_hil_top
    logic               tick_pulse;
    logic               step_done;
    logic signed [15:0] x_q14, y_q14;
    logic signed [23:0] dx_bus, dy_bus, sx_bus, sy_bus;
    logic signed [23:0] pll_adj_bus, agc_adj_bus, clq_adj_bus, ftr_adj_bus;
    logic [31:0]        frame_cnt;
    logic               stream_ovf;
    logic               step_done_sticky;

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            aw_latched       <= 1'b0;
            w_latched        <= 1'b0;
            aw_addr_reg      <= '0;
            w_data_reg       <= '0;
            w_strb_reg       <= '0;
            s_axi_bvalid     <= 1'b0;
            state_load_pulse <= 1'b0;
            pi_load_pulse    <= 1'b0;

            for (int i = 0; i < NUM_RW_REGS; i++) begin
                rw_regs[i] <= 32'h0;
            end

            // Sensible Stage 1 defaults (DRG 11 kHz, Q=100k @ 1 MHz HIL tick)
            rw_regs[0]  <= 32'h0000_001E; // enable PLL/AGC/CLQ/FTR, run=0 initially
            rw_regs[1]  <= 32'h0008_01F4; // lpf_shift=8 (~622 Hz @ 1MHz), decim=500 (2 kHz)
            rw_regs[2]  <= 32'd256;       // 256 frames (2048 words) per DMA TLAST burst
            rw_regs[3]  <= 32'd47244640;  // 11 kHz @ 1 MHz NCO tuning word
            rw_regs[4]  <= 32'd4194304;   // DX setpoint = 1.0 in Q2.22
            rw_regs[39] <= 32'd34360;     // half_dt_q36 = 0.5e-6 * 2^36
        end else begin
            state_load_pulse <= 1'b0;
            pi_load_pulse    <= 1'b0;

            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end

            if (wr_fire) begin
                if (wr_idx < 6'd41) begin
                    for (int b = 0; b < 4; b++) begin
                        if (wr_strb_eff[b]) begin
                            rw_regs[wr_idx][b*8 +: 8] <= wr_data_eff[b*8 +: 8];
                        end
                    end
                    // Auto-clear load pulse bits in CTRL (offset 0x00)
                    if (wr_idx == 6'd0 && wr_strb_eff[0]) begin
                        state_load_pulse <= wr_data_eff[6];
                        pi_load_pulse    <= wr_data_eff[7];
                        rw_regs[0][7:6]  <= 2'b00;
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
    assign s_axi_rresp   = 2'b00;

    logic [5:0] rd_idx;
    assign rd_idx = s_axi_araddr[7:2];

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
                if (rd_idx < 6'd41) begin
                    s_axi_rdata <= rw_regs[rd_idx];
                end else begin
                    unique case (rd_idx)
                        6'd48: s_axi_rdata <= {{8{dx_bus[23]}},      dx_bus};       // 0xC0 DX_RO
                        6'd49: s_axi_rdata <= {{8{dy_bus[23]}},      dy_bus};       // 0xC4 DY_RO
                        6'd50: s_axi_rdata <= {{8{sx_bus[23]}},      sx_bus};       // 0xC8 SX_RO
                        6'd51: s_axi_rdata <= {{8{sy_bus[23]}},      sy_bus};       // 0xCC SY_RO
                        6'd52: s_axi_rdata <= {{8{pll_adj_bus[23]}}, pll_adj_bus};  // 0xD0 PLL_ADJ_RO
                        6'd53: s_axi_rdata <= {{8{agc_adj_bus[23]}}, agc_adj_bus};  // 0xD4 AGC_ADJ_RO
                        6'd54: s_axi_rdata <= {{8{clq_adj_bus[23]}}, clq_adj_bus};  // 0xD8 CLQ_ADJ_RO
                        6'd55: s_axi_rdata <= {{8{ftr_adj_bus[23]}}, ftr_adj_bus};  // 0xDC FTR_ADJ_RO
                        6'd56: s_axi_rdata <= {y_q14, x_q14};                       // 0xE0 PICKOFF_XY
                        6'd57: s_axi_rdata <= {16'h0001, 12'd0, stream_ovf,
                                               rw_regs[0][0], step_done_sticky, 1'b1}; // 0xE4 STATUS
                        6'd58: s_axi_rdata <= frame_cnt;                            // 0xE8 FRAME_CNT
                        6'd59: s_axi_rdata <= VERSION_ID;                           // 0xEC VERSION
                        default: s_axi_rdata <= 32'hDEAD_BEEF;
                    endcase
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // 2. Decode Register Fields -> mems_hil_top
    // ------------------------------------------------------------------------
    logic core_rst_n;
    logic run_en;
    logic stream_rate_en;

    assign run_en         = rw_regs[0][0];
    assign core_rst_n     = aresetn && !rw_regs[0][5];
    assign stream_rate_en = rw_regs[0][8];

    logic [15:0] decim_div;
    logic [3:0]  lpf_shift;
    logic [31:0] pkt_frames;

    assign decim_div  = (rw_regs[1][15:0] == 16'd0) ? 16'd1 : rw_regs[1][15:0];
    assign lpf_shift  = rw_regs[1][19:16];
    assign pkt_frames = (rw_regs[2] == 32'd0) ? 32'd1 : rw_regs[2];

    // Optional MM2S live rate stimulus: consumes 1 signed 32-bit half_cor_dt per decimated frame
    logic               decim_strobe;
    logic signed [35:0] stream_cor_reg;
    logic signed [35:0] eff_half_cor_dt;

    assign s_axis_tready   = stream_rate_en && decim_strobe;
    assign eff_half_cor_dt = stream_rate_en ? stream_cor_reg
                                            : signed'({rw_regs[38][3:0], rw_regs[37]});

    mems_hil_top #(.TICK_DIV(TICK_DIV)) u_hil (
        .clk            (aclk),
        .rst_n          (core_rst_n),
        .run_enable     (run_en),

        .state_load     (state_load_pulse),
        .x_init         (signed'({rw_regs[18][15:0], rw_regs[17]})),
        .vx_init        (signed'({rw_regs[20][15:0], rw_regs[19]})),
        .y_init         (48'sd0),
        .vy_init        (48'sd0),

        .pi_load        (pi_load_pulse),
        .pll_init       (signed'(rw_regs[15][23:0])),
        .agc_init       (signed'(rw_regs[16][23:0])),

        .cos_tx         (signed'({rw_regs[22][3:0], rw_regs[21]})),
        .sin_tx         (signed'({rw_regs[24][3:0], rw_regs[23]})),
        .cos_ty         (signed'({rw_regs[26][3:0], rw_regs[25]})),
        .sin_ty         (signed'({rw_regs[28][3:0], rw_regs[27]})),
        .half_gx_dt     (signed'({rw_regs[30][3:0], rw_regs[29]})),
        .half_gy_dt     (signed'({rw_regs[32][3:0], rw_regs[31]})),
        .half_kxy_dt    (signed'({rw_regs[34][3:0], rw_regs[33]})),
        .half_gxy_dt    (signed'({rw_regs[36][3:0], rw_regs[35]})),
        .half_cor_dt    (eff_half_cor_dt),
        .half_dt_q36    (signed'({rw_regs[40][3:0], rw_regs[39]})),

        .base_freq_word (rw_regs[3]),
        .lpf_shift      (lpf_shift),
        .enable_pll     (rw_regs[0][1]),
        .enable_agc     (rw_regs[0][2]),
        .enable_clq     (rw_regs[0][3]),
        .enable_ftr     (rw_regs[0][4]),

        .dx_setpoint    (signed'(rw_regs[4][23:0])),
        .sx_setpoint    (signed'(rw_regs[5][23:0])),
        .sy_setpoint    (signed'(rw_regs[6][23:0])),

        .pll_kp         (signed'(rw_regs[7][23:0])),
        .pll_ki         (signed'(rw_regs[8][23:0])),
        .agc_kp         (signed'(rw_regs[9][23:0])),
        .agc_ki         (signed'(rw_regs[10][23:0])),
        .clq_kp         (signed'(rw_regs[11][23:0])),
        .clq_ki         (signed'(rw_regs[12][23:0])),
        .ftr_kp         (signed'(rw_regs[13][23:0])),
        .ftr_ki         (signed'(rw_regs[14][23:0])),

        .tick_pulse     (tick_pulse),
        .step_done      (step_done),
        .x_q14          (x_q14),
        .y_q14          (y_q14),
        .dx_out         (dx_bus),
        .dy_out         (dy_bus),
        .sx_out         (sx_bus),
        .sy_out         (sy_bus),
        .pll_adj_out    (pll_adj_bus),
        .agc_adj_out    (agc_adj_bus),
        .clq_adj_out    (clq_adj_bus),
        .ftr_adj_out    (ftr_adj_bus)
    );

    // ------------------------------------------------------------------------
    // 3. Decimated 8-Word AXI4-Stream Telemetry Packetizer -> AXI-DMA S2MM
    // ------------------------------------------------------------------------
    logic [15:0] decim_cnt;
    logic [31:0] snap_words [0:7];
    logic [2:0]  word_idx;
    logic        tx_active;
    logic [31:0] burst_frame_cnt;

    assign decim_strobe = step_done && (decim_cnt == (decim_div - 16'd1));

    always_ff @(posedge aclk) begin
        if (!core_rst_n) begin
            decim_cnt        <= 16'd0;
            tx_active        <= 1'b0;
            word_idx         <= 3'd0;
            burst_frame_cnt  <= 32'd0;
            frame_cnt        <= 32'd0;
            stream_ovf       <= 1'b0;
            step_done_sticky <= 1'b0;
            stream_cor_reg   <= 36'sd0;
            for (int i = 0; i < 8; i++) begin
                snap_words[i] <= 32'd0;
            end
        end else begin
            if (step_done) begin
                step_done_sticky <= 1'b1;
                if (decim_cnt == (decim_div - 16'd1)) begin
                    decim_cnt <= 16'd0;
                end else begin
                    decim_cnt <= decim_cnt + 16'd1;
                end
            end

            if (s_axis_tvalid && s_axis_tready) begin
                stream_cor_reg <= signed'({{4{s_axis_tdata[31]}}, s_axis_tdata});
            end

            // Launch 8-word telemetry frame on each decimated strobe
            if (decim_strobe) begin
                if (tx_active) begin
                    stream_ovf <= 1'b1;
                end else begin
                    snap_words[0] <= {{8{dx_bus[23]}},      dx_bus};
                    snap_words[1] <= {{8{dy_bus[23]}},      dy_bus};
                    snap_words[2] <= {{8{sx_bus[23]}},      sx_bus};
                    snap_words[3] <= {{8{sy_bus[23]}},      sy_bus};
                    snap_words[4] <= {{8{pll_adj_bus[23]}}, pll_adj_bus};
                    snap_words[5] <= {{8{agc_adj_bus[23]}}, agc_adj_bus};
                    snap_words[6] <= {{8{clq_adj_bus[23]}}, clq_adj_bus};
                    snap_words[7] <= {{8{ftr_adj_bus[23]}}, ftr_adj_bus};
                    tx_active     <= 1'b1;
                    word_idx      <= 3'd0;
                    frame_cnt     <= frame_cnt + 32'd1;
                end
            end else if (tx_active && m_axis_tready) begin
                if (word_idx == 3'd7) begin
                    tx_active <= 1'b0;
                    word_idx  <= 3'd0;
                    if (burst_frame_cnt == (pkt_frames - 32'd1)) begin
                        burst_frame_cnt <= 32'd0;
                    end else begin
                        burst_frame_cnt <= burst_frame_cnt + 32'd1;
                    end
                end else begin
                    word_idx <= word_idx + 3'd1;
                end
            end
        end
    end

    assign m_axis_tdata  = snap_words[word_idx];
    assign m_axis_tvalid = tx_active;
    assign m_axis_tlast  = tx_active && (word_idx == 3'd7) &&
                           (burst_frame_cnt == (pkt_frames - 32'd1));

    logic unused_ok;
    assign unused_ok = &{1'b0, wr_addr_eff[1:0], s_axi_araddr[1:0], s_axis_tlast, tick_pulse,
                         rw_regs[0][31:9], rw_regs[1][31:20],
                         rw_regs[4][31:24], rw_regs[5][31:24], rw_regs[6][31:24],
                         rw_regs[7][31:24], rw_regs[8][31:24], rw_regs[9][31:24],
                         rw_regs[10][31:24], rw_regs[11][31:24], rw_regs[12][31:24],
                         rw_regs[13][31:24], rw_regs[14][31:24], rw_regs[15][31:24],
                         rw_regs[16][31:24], rw_regs[18][31:16], rw_regs[20][31:16],
                         rw_regs[22][31:4], rw_regs[24][31:4], rw_regs[26][31:4],
                         rw_regs[28][31:4], rw_regs[30][31:4], rw_regs[32][31:4],
                         rw_regs[34][31:4], rw_regs[36][31:4], rw_regs[38][31:4],
                         rw_regs[40][31:4]};

endmodule
