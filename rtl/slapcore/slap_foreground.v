//============================================================================
// Foreground Layer Module 
// 
// Description: Foreground/text tile layer
//              Handles 2bpp tile graphics with screen flip 
//


module foreground_layer (
	input wire        reset_n,
	input wire        master_clk,
	input wire        pixel_clk,
	input wire  [7:0] VPIX,
	input wire  [8:0] HPIX,          // Reduced from 12 bits - only [8:0] were used
	input wire        SCREEN_FLIP,
	input wire        ATRRAM,
	input wire        CHARAM,
	input wire        Z80_WR,
	input wire        CPU_RAM_SYNC,
	input wire [15:0] CPU_ADDR,
	input wire  [7:0] CPU_DIN,

	input wire [24:0] dn_addr,
	input wire  [7:0] dn_data,
	input wire        ep3_cs_i,
	input wire        ep4_cs_i,
	input wire        dn_wr,
	
	output wire [8:0] HPIX_LT_out,
	output wire [7:0] FG_HI_out,
	output wire [7:0] FG_LO_out,
	output wire [7:0] pixel_output,
	output wire       FG_WAIT
);

//============================================================================
// Parameters
//============================================================================
localparam [7:0] WAIT_INIT = 8'b11000000;  // Wait state shift register init (2 cycles)

//============================================================================
// Internal Signals
//============================================================================
reg  [8:0]  HPIX_LT;
reg  [5:0]  FG_PX_colour;  // Color/attribute bits [7:2]
wire [15:0] FG_RAMD;

//============================================================================
// Horizontal Pixel Latch
// Creates a pipeline stage for timing margin
//============================================================================
always @(posedge pixel_clk or negedge reset_n) begin 
	if (!reset_n) begin
		HPIX_LT <= 9'b0;
	end else begin
		HPIX_LT <= HPIX;
	end
end

assign HPIX_LT_out = HPIX_LT;

//============================================================================
// Timing Generation
//============================================================================
// FG_CLK: Tile clock, toggles every 8 pixels
// Inverted when screen is flipped for cocktail cabinet support
wire FG_CLK = (SCREEN_FLIP ^ !HPIX_LT[2]);

// FG_SYNC: Active at tile boundary (pixel positions 0 or 7 within 8-pixel tile)
// Used to trigger loading of new tile data into shift registers
wire FG_SYNC = !(FG_CLK & (SCREEN_FLIP ^ !HPIX_LT[0]) & (SCREEN_FLIP ^ HPIX_LT[1]));

// Original colour_copy signal - directly used as latch enable
wire colour_copy = FG_SYNC | pixel_clk;

// Mux select for shift register operation
wire FG_SELECT = (CPU_RAM_SYNC | FG_SYNC);
wire FG_S0     = !(SCREEN_FLIP & FG_SELECT);
wire FG_S1     = !(!SCREEN_FLIP & FG_SELECT);

// Shift register control signals
wire load        =  FG_S0 &  FG_S1;  // Load new tile data
wire shift_left  =  FG_S0 & ~FG_S1;  // Shift for normal orientation
wire shift_right = ~FG_S0 &  FG_S1;  // Shift for flipped orientation

//============================================================================
// Foreground Tile Map RAM
// Dual-port RAM: Port A for video generation, Port B for CPU access
//============================================================================

// Attribute RAM (colors/flags) - active low chip select
dpram_dc #(.widthad_a(11)) FG_U4G 
(
	.clock_a(master_clk),
	.address_a({VPIX[7:3], HPIX_LT[8:3]}),
	.data_a(CPU_DIN),
	.wren_a(1'b0),
	.q_a(FG_RAMD[15:8]),

	.clock_b(master_clk),
	.address_b(CPU_ADDR[10:0]),
	.data_b(CPU_DIN),
	.wren_b(!ATRRAM & !Z80_WR),
	.q_b(FG_HI_out)
);

// Character/Tile Index RAM - active low chip select
dpram_dc #(.widthad_a(11)) FG_U4F 
(
	.clock_a(master_clk),
	.address_a({VPIX[7:3], HPIX_LT[8:3]}),
	.data_a(CPU_DIN),
	.wren_a(1'b0),
	.q_a(FG_RAMD[7:0]),

	.clock_b(master_clk),
	.address_b(CPU_ADDR[10:0]),
	.data_b(CPU_DIN),
	.wren_b(!CHARAM & !Z80_WR),
	.q_b(FG_LO_out)
);

//============================================================================
// Foreground Tile Graphics ROMs
// Two EPROMs provide 2 bits per pixel
//============================================================================
wire [7:0] U6G_FG_A77_03_out;
wire [7:0] U6F_FG_A77_04_out;

eprom_3 U6G_FG_A77_03
(
	.ADDR({FG_RAMD, VPIX[2:0]}),  // ROM Address: tile index + row within tile
	.CLK(master_clk),
	.DATA(U6G_FG_A77_03_out),

	.ADDR_DL(dn_addr),
	.CLK_DL(master_clk),
	.DATA_IN(dn_data),
	.CS_DL(ep3_cs_i),
	.WR(dn_wr)
);

eprom_4 U6G_FG_A77_04
(
	.ADDR({FG_RAMD, VPIX[2:0]}),  // ROM Address: tile index + row within tile
	.CLK(master_clk),
	.DATA(U6F_FG_A77_04_out),

	.ADDR_DL(dn_addr),
	.CLK_DL(master_clk),
	.DATA_IN(dn_data),
	.CS_DL(ep4_cs_i),
	.WR(dn_wr)
);

//============================================================================
// Pixel Shift Registers
// Serialize 8 pixels from ROM into sequential pixel output
//============================================================================
reg [7:0] Qout_03;
reg [7:0] Qout_04;

always @(posedge pixel_clk or negedge reset_n) begin
	if (!reset_n) begin
		Qout_03 <= 8'b0;
		Qout_04 <= 8'b0;
	end else if (load) begin
		// Load new tile row from ROMs
		Qout_03 <= U6G_FG_A77_03_out;
		Qout_04 <= U6F_FG_A77_04_out;
	end else if (shift_left) begin
		// Normal orientation: shift MSB out first
		Qout_03 <= {Qout_03[6:0], 1'b0};
		Qout_04 <= {Qout_04[6:0], 1'b0};
	end else if (shift_right) begin
		// Flipped orientation: shift LSB out first
		Qout_03 <= {1'b0, Qout_03[7:1]};
		Qout_04 <= {1'b0, Qout_04[7:1]};
	end
	// else: hold current value
end

//============================================================================
// Pixel Output Generation
//============================================================================
// Select pixel bits based on screen orientation
// Normal: MSB (bit 7), Flipped: LSB (bit 0)
wire [1:0] pixel_bits = (SCREEN_FLIP) ? 
	{Qout_04[0], Qout_03[0]} : 
	{Qout_04[7], Qout_03[7]};

// Color/attribute latch - uses colour_copy as clock (matches original hardware timing)
// This is a level-sensitive latch that captures FG_RAMD when colour_copy goes high
always @(posedge colour_copy or negedge reset_n) begin
	if (!reset_n) begin
		FG_PX_colour <= 6'b0;
	end else begin
		FG_PX_colour <= FG_RAMD[15:10];
	end
end

// Combine color attributes with pixel data
assign pixel_output = {FG_PX_colour, pixel_bits};

//============================================================================
// CPU Wait State Generator
// Generates wait states when CPU accesses video RAM during active display
//============================================================================
wire FG_CHIP_SEL = (ATRRAM & CHARAM);  // Active when neither RAM selected (active-low selects)
wire nFG_CLK = !FG_CLK;

reg [7:0] count_wait;
reg       FG_WAIT_out;
reg       FG_CS_OLD;

always @(posedge nFG_CLK or negedge reset_n) begin
	if (!reset_n) begin
		count_wait  <= 8'b0;
		FG_WAIT_out <= 1'b0;
		FG_CS_OLD   <= 1'b1;
	end else begin
		// Detect falling edge of chip select (start of CPU access)
		if (!FG_CHIP_SEL && FG_CS_OLD) begin
			count_wait <= WAIT_INIT;
		end else begin
			count_wait <= count_wait << 1;
		end
		FG_WAIT_out <= count_wait[7];
		FG_CS_OLD   <= FG_CHIP_SEL;
	end
end

// WAIT signal: active when no chip selected OR during wait countdown
assign FG_WAIT = FG_CHIP_SEL | FG_WAIT_out;

endmodule
