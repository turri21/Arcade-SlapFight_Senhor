module background_layer (
	input reset_n,
	input master_clk,			//Main system clock for RAM and ROM operations
	input pixel_clk,
	input pcb,					//PCB type selector (0=Slap Fight, 1=Tiger Heli)
	input [7:0] VPIXSCRL,	//Vertical pixel position with scroll offset
	input [8:0] HPIXSCRL,	//Horizontal pixel position with scroll offset
	input SCREEN_FLIP,
	input BACKGRAM_1,			//Chip select for background RAM bank 1 (active low)
	input BACKGRAM_2,			//Chip select for background RAM bank 2 (active low)
	input Z80_WR,				//Z80 write strobe (active low)
	input CPU_RAM_SYNC,		//CPU/video RAM access synchronization
	input [15:0] CPU_ADDR,	//CPU address bus
	input  [7:0] CPU_DIN,	//CPU databus input
	input [24:0] dn_addr,	//Download address for ROM loading
	input [7:0] dn_data,		//Download data for ROM loading
	input ep5_cs_i,			
	input ep6_cs_i,
	input ep7_cs_i,
	input ep8_cs_i,
	input dn_wr,				//Download write strobe
	output [7:0] BG_HI_out,
	output [7:0] BG_LO_out,
	output [7:0] pixel_output,	//Final pixel data: [7:4]=color, [3:0]=tile pixel
	output BG_WAIT
);

wire [15:0] BG_RAMD;
reg [7:0] BG_PX_D;
reg bg_colour_copy_d;
	
	wire SH_REG_DIR=!SCREEN_FLIP;
	wire BG_CLK=(SCREEN_FLIP^!HPIXSCRL[2]);
	wire BG_SYNC=!(BG_CLK&(SCREEN_FLIP^!HPIXSCRL[0])&(SCREEN_FLIP^HPIXSCRL[1]));
	wire BG_COLOUR_COPY=BG_SYNC|pixel_clk;
	wire BG_SELECT=BG_SYNC|CPU_RAM_SYNC;
	wire BG_S0=!(BG_SELECT& SCREEN_FLIP);
	wire BG_S1=!(BG_SELECT&!SCREEN_FLIP);
	

dpram_dc #(.widthad_a(11)) BG_U4N //sf
(
	.clock_a(master_clk),
	.address_a({VPIXSCRL[7:3],HPIXSCRL[8:3]}), //
	.data_a(CPU_DIN),
	.wren_a(1'b0),
	.q_a(BG_RAMD[7:0]),
	
	.clock_b(master_clk),
	.address_b(CPU_ADDR[10:0]),
	.data_b(CPU_DIN),
	.wren_b(!BACKGRAM_1 & !Z80_WR),
	.q_b(BG_LO_out)		

);

dpram_dc #(.widthad_a(11)) BG_U4P //sf
(
	.clock_a(master_clk),
	.address_a({VPIXSCRL[7:3],HPIXSCRL[8:3]}), //{VPIXSCRL[7:3],HPIXSCRL[8:3]}
	.data_a(CPU_DIN),
	.wren_a(1'b0),
	.q_a(BG_RAMD[15:8]),

	.clock_b(master_clk),
	.address_b(CPU_ADDR[10:0]),
	.data_b(CPU_DIN),
	.wren_b(!BACKGRAM_2 & !Z80_WR),
	.q_b(BG_HI_out)
);

wire [7:0] U6P_BG_A77_05_out;
wire [7:0] U6N_BG_A77_06_out;
wire [7:0] U6M_BG_A77_07_out;
wire [7:0] U6K_BG_A77_08_out; //eprom data output

wire [14:0] BGROM_ADDR = (pcb) ? {1'b0, BG_RAMD[10:0], VPIXSCRL[2:0]} :  {BG_RAMD[11:0], VPIXSCRL[2:0]};
	 
eprom_5 U6P_BG_A77_05
(
	.ADDR(BGROM_ADDR),//BG_RAMD[11:0]
	.CLK(master_clk),//
	.DATA(U6P_BG_A77_05_out),//
	.ADDR_DL(dn_addr),
	.CLK_DL(master_clk),//
	.DATA_IN(dn_data),
	.CS_DL(ep5_cs_i),
	.WR(dn_wr)
);


eprom_6 U6N_BG_A77_06
(
	.ADDR(BGROM_ADDR),//
	.CLK(master_clk),//
	.DATA(U6N_BG_A77_06_out),//
	.ADDR_DL(dn_addr),
	.CLK_DL(master_clk),//
	.DATA_IN(dn_data),
	.CS_DL(ep6_cs_i),
	.WR(dn_wr)
);

eprom_7 U6M_BG_A77_07
(
	.ADDR(BGROM_ADDR),//
	.CLK(master_clk),//
	.DATA(U6M_BG_A77_07_out),//
	.ADDR_DL(dn_addr),
	.CLK_DL(master_clk),//
	.DATA_IN(dn_data),
	.CS_DL(ep7_cs_i),
	.WR(dn_wr)
);

eprom_8 U6K_BG_A77_08
(
	.ADDR(BGROM_ADDR),//
	.CLK(master_clk),//
	.DATA(U6K_BG_A77_08_out),//
	.ADDR_DL(dn_addr),
	.CLK_DL(master_clk),//
	.DATA_IN(dn_data),
	.CS_DL(ep8_cs_i),
	.WR(dn_wr)
);

	 reg [7:0] Qout_05,Qout_06,Qout_07,Qout_08;

    wire load       =  BG_S0 &  BG_S1;
    wire shift_left =  BG_S0 & ~BG_S1;
    wire shift_right= ~BG_S0 &  BG_S1;

    always @(posedge pixel_clk) begin
        if (load) begin
            Qout_05 <= U6P_BG_A77_05_out;
            Qout_06 <= U6N_BG_A77_06_out;
            Qout_07 <= U6M_BG_A77_07_out;
            Qout_08 <= U6K_BG_A77_08_out;
		  end	
        else if (shift_left) begin
            Qout_05 <= {Qout_05[6:0], 1'b0};
            Qout_06 <= {Qout_06[6:0], 1'b0};
            Qout_07 <= {Qout_07[6:0], 1'b0};
            Qout_08 <= {Qout_08[6:0], 1'b0};				
		  end
        else if (shift_right) begin
            Qout_05 <= {1'b0, Qout_05[7:1]};
            Qout_06 <= {1'b0, Qout_06[7:1]};
            Qout_07 <= {1'b0, Qout_07[7:1]};
            Qout_08 <= {1'b0, Qout_08[7:1]};				
		  end
        // else: implicit hold

    end
	 
// Lower 4 bits - combinational from shift registers
wire [3:0] pixel_bits = SH_REG_DIR ? {Qout_08[7], Qout_07[7], Qout_06[7], Qout_05[7]}
                                   : {Qout_08[0], Qout_07[0], Qout_06[0], Qout_05[0]};

// Upper 4 bits - latch at tile boundary (when BG_SYNC is low)
reg [3:0] BG_PX_D_color;

always @(posedge master_clk or negedge reset_n) begin
    if (!reset_n) begin
        BG_PX_D_color <= 4'b0;
    end else if (pixel_clk && !BG_SYNC) begin  // Tile boundary: latch color
        BG_PX_D_color <= BG_RAMD[15:12];
    end
end

assign pixel_output = {BG_PX_D_color, pixel_bits};

	//background wait states
	wire BG_CHIP_SEL=BACKGRAM_1&BACKGRAM_2;

	reg [7:0] count_wait;
	reg       BG_CS_OLD;
	reg       BG_WAIT_out;
	reg       BG_CLK_prev;

	always @(posedge master_clk or negedge reset_n) begin
		if (!reset_n) begin
			count_wait  <= 8'b0;
			BG_WAIT_out <= 1'b1;
			BG_CS_OLD   <= 1'b1;
			BG_CLK_prev <= 1'b0;
		end else begin
			BG_CLK_prev <= BG_CLK;
			
			// Falling edge of BG_CLK
			if (!BG_CLK && BG_CLK_prev) begin
				count_wait  <= (!BG_CHIP_SEL & BG_CS_OLD) ? 8'b11000000 : {count_wait[6:0], 1'b0};
				BG_WAIT_out <= count_wait[7];
				BG_CS_OLD   <= BG_CHIP_SEL;
			end
		end
	end
	
 	assign BG_WAIT=BG_CHIP_SEL|BG_WAIT_out;

endmodule
