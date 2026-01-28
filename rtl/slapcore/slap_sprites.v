//============================================================================
// Sprite Layer Module
//============================================================================

module sprite_layer (
	 input wire        reset_n,
    input wire        master_clk,
    input wire        pixel_clk,
    input wire        npixel_clk,
    input wire        pixel_clk_lb,
    
    input wire [7:0]  VPIX,
    input wire [11:0] HPIX,
    input wire [8:0]  HPIX_LT,
    input wire        SCREEN_FLIP,
    
    input wire        SPRITE_RAM,
    input wire        Z80_WR,
    input wire        Z80_RD,
    input wire        CPU_RAM_SYNC,
    input wire        CPU_RAM_SELECT,
    input wire        CPU_RAM_LBUF,
    input wire [15:0] CPU_ADDR,
    input wire [7:0]  CPU_DIN,
    
    input wire [24:0] dn_addr,
    input wire [7:0]  dn_data,
    input wire        ep9_cs_i,
    input wire        ep10_cs_i,
    input wire        ep11_cs_i,
    input wire        ep12_cs_i,
    input wire        dn_wr,
    
    output wire [7:0] SP_RAMD_out,
    output reg  [7:0] pixel_output
);

//============================================================================
// Parameters
//============================================================================

localparam SPRITE_RAM_ADDR_WIDTH = 11;
localparam SPRITE_32K_ADDR_WIDTH = 12;

localparam CMD_IDLE     = 3'b000;
localparam CMD_LAT_HPOS = 3'b001;
localparam CMD_LAT_XDAT = 3'b010;
localparam CMD_LAT_VPOS = 3'b011;
localparam CMD_LAT_INDX = 3'b100;
localparam CMD_CTRL_101 = 3'b101;
localparam CMD_CTRL_110 = 3'b110;
localparam CMD_CTRL_111 = 3'b111;

//============================================================================
// Internal Signals
//============================================================================

wire cpu_ram_select_active = CPU_RAM_SELECT;
wire cpu_ram_sync_n        = ~CPU_RAM_SYNC;

reg [11:0] SPR_CNT;
reg  [7:0] ROM18_addr;
wire [7:0] ROM18_out;

reg  [7:0] control_latch;
wire [7:0] control_out;


wire SPR_REG_OE, SPR_ROM_LD, SPR_LB_LD, SPR_8HPIX;
wire ctrl_u4a_3, ctrl_u4a_6, ctrl_u4a_10, ctrl_u4a_11, RESET_LD_CTR;

reg  [7:0] SPR_ROM1617_ADDR;
wire [7:0] ROM17_out, ROM16_out;
reg  [2:0] CMD_decode_latch;
reg        SPR_INC_CNT_latch, RST_REG_CTR_latch;
wire [2:0] CMD_decode;
wire       SPR_INC_CNT, RST_REG_CTR;

wire [7:0] SPRITE_RAM_D;
reg  [7:0] SPR_VPOS_D, SPR_EXT_D;
reg  [9:0] SPR_IDX_D;
reg  [8:0] SPR_HPOS_D;

reg  [4:0] SPR_VPIX_CNT;
reg  [7:0] SPR_VPOS_CNT;
wire [3:0] SPR_VPIX;

wire [11:0] SPR_32K_A;
reg  [3:0]  SPR_32K_HI;
wire [3:0]  SPR_32K_HI_next, SPR_32K_HI_ram_out;
wire [7:0]  SPR_VPIX_out, SPR_IDX_out, SPR_EXTRA_out, SPR_HPOS_out;

wire [14:0] SPROM_ADDR;
wire [7:0]  SP_09_out, SP_10_out, SP_11_out, SP_12_out;
reg  [7:0]  Qout_09, Qout_10, Qout_11, Qout_12;
wire        SPR_PIX_A, SPR_PIX_B, SPR_PIX_C, SPR_PIX_D;

wire        SPR_LINEA, SPR_LINEB;
reg  [8:0]  LNBF_CNT;
wire [8:0]  LINEBUF_A_A, LINEBUF_B_A;
wire [7:0]  LINEBUF_A_D_in, LINEBUF_A_D_out;
wire [7:0]  LINEBUF_B_D_in, LINEBUF_B_D_out;
wire        LINEBUF_A_nWE, LINEBUF_B_nWE;
reg  [3:0]  SP_PX_SEL_D;
reg  [7:0]  LINEA_PIXEL, LINEB_PIXEL;

wire SPR32_CLK, SPR32_WR, SPR32_BUF_WR;

//============================================================================
// Sequencer ROM
//============================================================================

always @(posedge pixel_clk or negedge cpu_ram_sync_n) begin
    if (~cpu_ram_sync_n)
        ROM18_addr <= 8'd0;
    else if (~RESET_LD_CTR)
        ROM18_addr <= 8'b00000100;
    else
        ROM18_addr <= ROM18_addr + 8'd1;
end

ROM18 S2_U2B_ROM18 (
    .clk  (master_clk),
    .addr (ROM18_addr[4:0]),
    .data (ROM18_out)
);

//============================================================================
// Control Registers (ls175 behavior with async clear)
//============================================================================

always @(posedge pixel_clk) control_latch <= ROM18_out[7:0];

assign control_out = ~cpu_ram_sync_n ? 8'b00000000 : control_latch;

assign SPR_REG_OE    = control_out[0];
assign SPR_ROM_LD    = ~control_out[1] | CPU_RAM_SYNC;
assign SPR_LB_LD     = ~control_out[2];
assign SPR_8HPIX     = ~control_out[3];
assign ctrl_u4a_3    = ~control_out[4];
assign ctrl_u4a_6    = ~control_out[5];
assign ctrl_u4a_10   =  control_out[6];
assign ctrl_u4a_11   = ~control_out[6];
assign RESET_LD_CTR  = ~control_out[7];

//============================================================================
// Command Sequencer
//============================================================================

wire rom_addr_reset = RST_REG_CTR | cpu_ram_select_active;

always @(posedge npixel_clk or posedge rom_addr_reset) begin
    if (rom_addr_reset)
        SPR_ROM1617_ADDR <= 8'd0;
    else
        SPR_ROM1617_ADDR <= SPR_ROM1617_ADDR + 8'd1;
end

ROM17 S2_U1C_ROM17 (.clk(master_clk), .addr(SPR_ROM1617_ADDR), .data(ROM17_out));
ROM16 S2_U1E_ROM16 (.clk(master_clk), .addr(SPR_ROM1617_ADDR), .data(ROM16_out));

always @(posedge pixel_clk) begin
    RST_REG_CTR_latch  <= ROM16_out[1];
    SPR_INC_CNT_latch  <= ROM17_out[3];
    CMD_decode_latch   <= ROM17_out[2:0];
end

assign RST_REG_CTR = cpu_ram_select_active ? 1'b0 : RST_REG_CTR_latch;
assign SPR_INC_CNT = cpu_ram_select_active ? 1'b0 : SPR_INC_CNT_latch;
assign CMD_decode  = cpu_ram_select_active ? 3'b000 : CMD_decode_latch;

//============================================================================
// Sprite Counter
//============================================================================

always @(posedge SPR_INC_CNT or posedge cpu_ram_select_active) begin
    if (cpu_ram_select_active)
        SPR_CNT <= 12'd0;
    else
        SPR_CNT <= SPR_CNT + 12'd1;
end

//============================================================================
// Command Decoder
//============================================================================

wire lat_hpos_en = (CMD_decode == CMD_LAT_HPOS);
wire lat_xdat_en = (CMD_decode == CMD_LAT_XDAT);
wire lat_vpos_en = (CMD_decode == CMD_LAT_VPOS);
wire lat_indx_en = (CMD_decode == CMD_LAT_INDX);
wire ctrl_101_en = (CMD_decode == CMD_CTRL_101);
wire ctrl_110_en = (CMD_decode == CMD_CTRL_110);
wire ctrl_111_en = (CMD_decode == CMD_CTRL_111);

wire SPR_LAT_HPOS = pixel_clk | ~lat_hpos_en;
wire SPR_LAT_XDAT = pixel_clk | ~lat_xdat_en;
wire SPR_LAT_VPOS = pixel_clk | ~lat_vpos_en;
wire SPR_LAT_INDX = pixel_clk | ~lat_indx_en;
wire S2_U1G_10    = pixel_clk | ~ctrl_101_en;
wire S2_U1G_9     = pixel_clk | ~ctrl_110_en;
wire S2_U1G_7     = pixel_clk | ~ctrl_111_en;

//============================================================================
// Main Sprite RAM
//============================================================================

dpram_dc #(.widthad_a(SPRITE_RAM_ADDR_WIDTH)) SP_U2L (
    .clock_a(master_clk), .address_a(SPR_CNT[10:0]), .data_a(CPU_DIN),
    .wren_a(1'b0), .q_a(SPRITE_RAM_D),
    .clock_b(master_clk), .address_b(CPU_ADDR[10:0]), .data_b(CPU_DIN),
    .wren_b(~Z80_WR & ~SPRITE_RAM), .q_b(SP_RAMD_out)
);

//============================================================================
// Sprite Attribute Registers
//============================================================================

always @(posedge SPR_LAT_VPOS) SPR_VPOS_D <= SPRITE_RAM_D;
always @(posedge SPR_LAT_INDX) SPR_IDX_D[7:0] <= SPRITE_RAM_D;
always @(posedge SPR_LAT_HPOS) SPR_HPOS_D[7:0] <= SPRITE_RAM_D;
always @(posedge SPR_LAT_XDAT) begin
    SPR_HPOS_D[8]   <= SPRITE_RAM_D[0];
    SPR_IDX_D[9:8]  <= SPRITE_RAM_D[7:6];
    SPR_EXT_D       <= SPRITE_RAM_D;
end

//============================================================================
// S2_U7B - LS74 Dual Flip-Flop
//============================================================================

wire S2_U9B_D = SPR32_CLK | ~(SPR_VPIX_CNT[0] & SPR_VPIX_CNT[1] & 
                               SPR_VPIX_CNT[2] & SPR_VPIX_CNT[3]);

reg S2_U7BA_nQ,S2_U7BB_Q,S2_U9B_D_prev;

wire S2_U7B_AQ, S2_U7B_AnQ;

always @(posedge master_clk or negedge reset_n) begin
    if (!reset_n) begin
        S2_U7BA_nQ <= 1'b1;  // Reset: Q=0, nQ=1
    end else begin
        // Clear has priority (Q=0 → nQ=1)
        if (!S2_U1G_9)
            S2_U7BA_nQ <= 1'b1;
        else if (!S2_U1G_10)  // Preset (Q=1 → nQ=0)
            S2_U7BA_nQ <= 1'b0;
    end
end

always @(posedge master_clk or negedge reset_n) begin
    if (!reset_n) begin
        S2_U7BB_Q <= 1'b0;
        S2_U9B_D_prev <= 1'b0;
    end else begin
        S2_U9B_D_prev <= S2_U9B_D;
        
        // Preset (active-low)
        if (!S2_U7BA_nQ)
            S2_U7BB_Q <= 1'b1;
        // Rising edge of clock, D=0
        else if (S2_U9B_D && !S2_U9B_D_prev)
            S2_U7BB_Q <= 1'b0;
    end
end

wire S2_U7BB_nQ = ~S2_U7BB_Q;
assign S2_U7B_AQ  = ~S2_U1G_10 & S2_U1G_9;
assign S2_U7B_AnQ = ~S2_U7B_AQ;

//============================================================================
// Sprite 32K RAM Control Signals
//============================================================================

wire S2_U4C_A = S2_U7BB_nQ | S2_U1G_10;
wire S2_U4C_B = S2_U7BB_nQ | S2_U1G_9;
wire S2_U4C_C = S2_U7BB_nQ | S2_U1G_7;
wire S2_U5A_A = S2_U4C_B & ctrl_u4a_3;

assign SPR32_WR     = S2_U4C_C & ctrl_u4a_6;
assign SPR32_CLK    = S2_U4C_C & S2_U4C_A;
assign SPR32_BUF_WR = S2_U4C_C & ctrl_u4a_6;

//============================================================================
// Vertical Pixel Counter
//============================================================================
// CRITICAL FIX: This must be SYNCHRONOUS, not async reset!
// Original code: checks S2_U7B_AnQ as a MUX select on the clock edge
// NOT as an async reset signal.

always @(posedge SPR32_CLK) begin
    // SYNCHRONOUS check - matches original behavior exactly
    SPR_VPIX_CNT <= (~S2_U7B_AnQ) ? 5'd0            : 
                    (S2_U7BB_Q)   ? SPR_VPIX_CNT + 1 : 
                                   SPR_VPIX_CNT;
                                   
    SPR_VPOS_CNT <= (~S2_U7B_AnQ) ? SPR_VPOS_D       : 
                    (S2_U7BB_Q)   ? SPR_VPOS_CNT + 1 : 
                                   SPR_VPOS_CNT;
end

assign SPR_VPIX = (~SPR_REG_OE) ? SPR_VPIX_CNT[3:0] : 4'd0;

//============================================================================
// Sprite 32K Address
//============================================================================

assign SPR_32K_HI_next = SPR_32K_HI + 4'd1;

always @(posedge S2_U5A_A) begin
    if (~SPR32_BUF_WR)
        SPR_32K_HI <= SPR_32K_HI_next;
    else
        SPR_32K_HI <= SPR_32K_HI_ram_out;
end

assign SPR_32K_A[7:0]  = CPU_RAM_SYNC ? SPR_VPOS_CNT : (VPIX - 8'd1);
assign SPR_32K_A[11:8] = SPR_32K_HI;

m6148_7 S2_U2F (
    .clk(master_clk), .addr(SPR_32K_A[7:0]),
    .data(SPR_32K_HI_next), .nWE(SPR32_BUF_WR), .q(SPR_32K_HI_ram_out)
);

//============================================================================
// Sprite 32K RAMs
//============================================================================

dpram_dc #(.widthad_a(SPRITE_32K_ADDR_WIDTH)) SP_3H (
    .clock_a(master_clk), .address_a(SPR_32K_A),
    .data_a({4'b0000, SPR_VPIX}), .wren_a(~SPR32_WR), .q_a(SPR_VPIX_out),
    .clock_b(master_clk), .address_b(SPR_32K_A),
    .data_b(8'd0), .wren_b(1'b0), .q_b()
);

dpram_dc #(.widthad_a(SPRITE_32K_ADDR_WIDTH)) SP_3J (
    .clock_a(master_clk), .address_a(SPR_32K_A),
    .data_a(SPR_IDX_D[7:0]), .wren_a(~SPR32_WR), .q_a(SPR_IDX_out),
    .clock_b(master_clk), .address_b(SPR_32K_A),
    .data_b(8'd0), .wren_b(1'b0), .q_b()
);

dpram_dc #(.widthad_a(SPRITE_32K_ADDR_WIDTH)) SP_3K (
    .clock_a(master_clk), .address_a(SPR_32K_A),
    .data_a(SPR_EXT_D), .wren_a(~SPR32_WR), .q_a(SPR_EXTRA_out),
    .clock_b(master_clk), .address_b(SPR_32K_A),
    .data_b(8'd0), .wren_b(1'b0), .q_b()
);

dpram_dc #(.widthad_a(SPRITE_32K_ADDR_WIDTH)) SP_3L (
    .clock_a(master_clk), .address_a(SPR_32K_A),
    .data_a(SPR_HPOS_D[7:0]), .wren_a(~SPR32_WR), .q_a(SPR_HPOS_out),
    .clock_b(master_clk), .address_b(SPR_32K_A),
    .data_b(8'd0), .wren_b(1'b0), .q_b()
);

//============================================================================
// Sprite Pattern ROMs
//============================================================================

assign SPROM_ADDR = {SPR_EXTRA_out[7:6], SPR_IDX_out, SPR_VPIX_out[3:0], SPR_8HPIX};

eprom_9  SP_09 (.ADDR(SPROM_ADDR), .CLK(master_clk), .DATA(SP_09_out),
                .ADDR_DL(dn_addr), .CLK_DL(~master_clk), .DATA_IN(dn_data),
                .CS_DL(ep9_cs_i), .WR(dn_wr));
eprom_10 SP_10 (.ADDR(SPROM_ADDR), .CLK(master_clk), .DATA(SP_10_out),
                .ADDR_DL(dn_addr), .CLK_DL(~master_clk), .DATA_IN(dn_data),
                .CS_DL(ep10_cs_i), .WR(dn_wr));
eprom_11 SP_11 (.ADDR(SPROM_ADDR), .CLK(master_clk), .DATA(SP_11_out),
                .ADDR_DL(dn_addr), .CLK_DL(~master_clk), .DATA_IN(dn_data),
                .CS_DL(ep11_cs_i), .WR(dn_wr));
eprom_12 SP_12 (.ADDR(SPROM_ADDR), .CLK(master_clk), .DATA(SP_12_out),
                .ADDR_DL(dn_addr), .CLK_DL(~master_clk), .DATA_IN(dn_data),
                .CS_DL(ep12_cs_i), .WR(dn_wr));

//============================================================================
// Pixel Shift Registers
//============================================================================

always @(posedge pixel_clk) begin
    if (~SPR_ROM_LD) begin
        Qout_09 <= SP_09_out;
        Qout_10 <= SP_10_out;
        Qout_11 <= SP_11_out;
        Qout_12 <= SP_12_out;
    end else begin
        Qout_09 <= {1'b0, Qout_09[7:1]};
        Qout_10 <= {1'b0, Qout_10[7:1]};
        Qout_11 <= {1'b0, Qout_11[7:1]};
        Qout_12 <= {1'b0, Qout_12[7:1]};
    end
end

assign SPR_PIX_A = Qout_09[0];
assign SPR_PIX_B = Qout_11[0];
assign SPR_PIX_C = Qout_10[0];
assign SPR_PIX_D = Qout_12[0];

//============================================================================
// Line Buffer Selection
//============================================================================

reg linebuf_select_reg = 1'b0;

always @(negedge CPU_RAM_SYNC) begin
    linebuf_select_reg <= ~linebuf_select_reg;
end

assign SPR_LINEA = linebuf_select_reg;
assign SPR_LINEB = ~linebuf_select_reg;

//============================================================================
// Line Buffer Address Counter
//============================================================================

wire S2_U5A_D = SPR_LB_LD & ctrl_u4a_11;

always @(posedge pixel_clk) begin
    if (~SPR_LB_LD)
        LNBF_CNT <= {SPR_EXTRA_out[0], SPR_HPOS_out};
    else if (~S2_U5A_D)
        LNBF_CNT <= LNBF_CNT - 9'd1;
end

always @(posedge SPR_ROM_LD) SP_PX_SEL_D <= SPR_EXTRA_out[4:1];

//============================================================================
// Line Buffer Address Mux
//============================================================================

assign LINEBUF_A_A = CPU_RAM_LBUF ? 9'd0 : (~SPR_LINEA) ? HPIX_LT : LNBF_CNT;
assign LINEBUF_B_A = CPU_RAM_LBUF ? 9'd0 : (~SPR_LINEB) ? HPIX_LT : LNBF_CNT;

//============================================================================
// Line Buffer Data
//============================================================================
// Combine pixel bits with color/priority information

wire SPR_LINEA_PIX_A = SPR_PIX_A & SPR_LINEA;
wire SPR_LINEA_PIX_B = SPR_PIX_B & SPR_LINEA;
wire SPR_LINEA_PIX_C = SPR_PIX_C & SPR_LINEA;
wire SPR_LINEA_PIX_D = SPR_PIX_D & SPR_LINEA;

wire SPR_LINEB_PIX_A = SPR_PIX_A & SPR_LINEB;
wire SPR_LINEB_PIX_B = SPR_PIX_B & SPR_LINEB;
wire SPR_LINEB_PIX_C = SPR_PIX_C & SPR_LINEB;
wire SPR_LINEB_PIX_D = SPR_PIX_D & SPR_LINEB;

// Format: {pixel_D, pixel_B, pixel_C, pixel_A, color_select[3:0]}
assign LINEBUF_A_D_in = {SPR_LINEA_PIX_D, SPR_LINEA_PIX_B, 
                         SPR_LINEA_PIX_C, SPR_LINEA_PIX_A, SP_PX_SEL_D};
assign LINEBUF_B_D_in = {SPR_LINEB_PIX_D, SPR_LINEB_PIX_B, 
                         SPR_LINEB_PIX_C, SPR_LINEB_PIX_A, SP_PX_SEL_D};

//============================================================================
// Line Buffer Write Enable
//============================================================================

wire S2_U7M_B = ~(SPR_LINEA_PIX_B | SPR_LINEA_PIX_C);
wire S2_U7M_A = ~(SPR_LINEA_PIX_A | SPR_LINEA_PIX_D);
wire S2_U8M_A = ~(S2_U7M_A & S2_U7M_B);
wire S2_U1R_A = ~(S2_U8M_A & SPR_LINEA & ctrl_u4a_10);
wire S2_U1T_D = S2_U1R_A & SPR_LINEA;
assign LINEBUF_A_nWE = pixel_clk | S2_U1T_D;

wire S2_U7M_D = ~(SPR_LINEB_PIX_B | SPR_LINEB_PIX_C);
wire S2_U7M_C = ~(SPR_LINEB_PIX_A | SPR_LINEB_PIX_D);
wire S2_U8M_C = ~(S2_U7M_C & S2_U7M_D);
wire S2_U1R_B = ~(S2_U8M_C & SPR_LINEB & ctrl_u4a_10);
wire S2_U1T_C = S2_U1R_B & SPR_LINEB;
assign LINEBUF_B_nWE = pixel_clk | S2_U1T_C;

//============================================================================
// Line Buffer RAMs
//============================================================================

m6148x2 S2_U5T (.data(LINEBUF_A_D_in), .clk(master_clk), 
                .addr(LINEBUF_A_A), .nWE(LINEBUF_A_nWE), .q(LINEBUF_A_D_out));
m6148x2 S2_U5M (.data(LINEBUF_B_D_in), .clk(master_clk), 
                .addr(LINEBUF_B_A), .nWE(LINEBUF_B_nWE), .q(LINEBUF_B_D_out));

//============================================================================
// Line Buffer Output
//============================================================================

always @(posedge pixel_clk_lb) begin
    LINEA_PIXEL <= {LINEBUF_A_D_out[3:0], LINEBUF_A_D_out[7:4]};
    LINEB_PIXEL <= {LINEBUF_B_D_out[3:0], LINEBUF_B_D_out[7:4]};
end

always @(posedge pixel_clk) begin
    pixel_output <= (~SPR_LINEA) ? LINEA_PIXEL : LINEB_PIXEL;
end

endmodule
