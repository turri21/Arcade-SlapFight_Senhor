//============================================================================
// MCU Handshake Flip-Flops (directly from TTL 74LS74)
// Used for communication with external 68705 MCU (Slap Fight protection)
// This code does not function currently
//============================================================================
/*
wire U4B_B   = SEL_MCU_PORT | Z80_RD;
wire U9A_SF_B = RESET_68705 & U4B_B;

ttl_7474 #(.BLOCKS(1), .DELAY_RISE(0), .DELAY_FALL(0)) U7A_A(
	.n_pre(1'b0),
	.n_clr(U9A_SF_B),
	.d(1'b0),
	.clk(U4B_B),
	.q(),
	.n_q(RD_BUFFER_FULL_68705)
);

ttl_7474 #(.BLOCKS(1), .DELAY_RISE(0), .DELAY_FALL(0)) U7A_B(
	.n_pre(MCU_PORT_WR),
	.n_clr(U9A_SF_B),
	.d(1'b0),
	.clk(1'b0),
	.q(),
	.n_q(WR_BUFFER_FULL_68705)
);*/

