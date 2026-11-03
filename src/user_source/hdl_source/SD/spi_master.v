
// ============================================================================
// spi_master —— SPI 物理层：一次收发一个字节（SD 卡的 SPI 模式）
// ----------------------------------------------------------------------------
// 上层（sd_card_cmd / sd_card_sec_read_write）摆好 data_in、拉 wr_req，
// 本模块就开始打 8 个 DCLK 周期、同时收 8 个 MISO 位，收完在 ACK 状态给出
// wr_ack 并保持一个周期，之后多等一拍 ACK_WAIT 才回 IDLE —— 这一拍是留给
// 上层在看到 ack 的那一拍把 wr_req 撤掉的，否则会被当成又一次新请求。
//
// 时钟与采样沿
//   · DCLK 的半周期 = (clk_div + 2) 个 sys_clk，全周期 = 2 × (clk_div + 2)；
//     那个 +2 是状态机开销：每个半周期都要先走 DCLK_IDLE 数到 clk_div
//     （clk_div+1 拍），再走一拍 DCLK_EDGE 翻转。所以 clk_div = 0 已经是
//     最快档，100MHz 的 sys_clk 只能得到 25MHz DCLK —— 正好是 SD 卡 SPI
//     模式的上限，再快就要提 sys_clk；
//   · 一个字节共 16 个 DCLK 沿（clk_edge_cnt 数到 15 结束）；
//   · CPOL 决定空闲电平，CPHA 决定 MOSI 在哪个沿换、MISO 在哪个沿采：
//       CPHA=0 → MOSI 在奇数沿移位，MISO 在偶数沿采样
//       CPHA=1 → MOSI 在非零偶数沿移位，MISO 在奇数沿采样
//     两路互补，保证「换的时候不采、采的时候不换」。
//
// 注意
//   · MOSI 直接取移位寄存器最高位（MSB first），SD 卡 SPI 模式就是这个顺序；
//   · nCS 在本模块里是直通（nCS = nCS_ctrl），片选完全由上层按命令序列控制，
//     本模块不管 —— 所以上层必须自己保证「拉低 nCS 期间不要插别的命令」。
// ============================================================================
module spi_master
(
	input                       sys_clk,
	input                       rst,
	output                      nCS,       //chip select (SPI mode)
	output                      DCLK,      //spi clock
	output                      MOSI,      //spi data output
	input                       MISO,      //spi input
	input                       CPOL,
	input                       CPHA,
	input                       nCS_ctrl,
	input[15:0]                 clk_div,
	input                       wr_req,
	output                      wr_ack,
	input[7:0]                  data_in,
	output[7:0]                 data_out
);
localparam                   IDLE            = 0;
localparam                   DCLK_EDGE       = 1;
localparam                   DCLK_IDLE       = 2;
localparam                   ACK             = 3;
localparam                   LAST_HALF_CYCLE = 4;
localparam                   ACK_WAIT        = 5;
reg                          DCLK_reg;
reg[7:0]                     MOSI_shift;
reg[7:0]                     MISO_shift;
reg[2:0]                     state;
reg[2:0]                     next_state;
reg [15:0]                   clk_cnt;
reg[4:0]                     clk_edge_cnt;
assign MOSI = MOSI_shift[7];
assign DCLK = DCLK_reg;
assign data_out = MISO_shift;
assign wr_ack = (state == ACK);
assign nCS = nCS_ctrl;
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		state <= IDLE;
	else
		state <= next_state;
end

always@(*)
begin
	case(state)
		IDLE:
			if(wr_req == 1'b1)
				next_state <= DCLK_IDLE;
			else
				next_state <= IDLE;
		DCLK_IDLE:
			//half a SPI clock cycle produces a clock edge
			if(clk_cnt == clk_div)
				next_state <= DCLK_EDGE;
			else
				next_state <= DCLK_IDLE;
		DCLK_EDGE:
			//a SPI byte with a total of 16 clock edges
			if(clk_edge_cnt == 5'd15)
				next_state <= LAST_HALF_CYCLE;
			else
				next_state <= DCLK_IDLE;
		//this is the last data edge
		LAST_HALF_CYCLE:
			if(clk_cnt == clk_div)
				next_state <= ACK;
			else
				next_state <= LAST_HALF_CYCLE;
		//send one byte complete
		ACK:
			next_state <= ACK_WAIT;
		//wait for one clock cycle, to ensure that the cancel request signal
		ACK_WAIT:
			next_state <= IDLE;
		default:
			next_state <= IDLE;
	endcase
end

always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		DCLK_reg <= 1'b0;
	else if(state == IDLE)
		DCLK_reg <= CPOL;
	else if(state == DCLK_EDGE)
		DCLK_reg <= ~DCLK_reg;//SPI clock edge
end
//SPI clock wait counter
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		clk_cnt <= 16'd0;
	else if(state == DCLK_IDLE || state == LAST_HALF_CYCLE)
		clk_cnt <= clk_cnt + 16'd1;
	else
		clk_cnt <= 16'd0;
end
//SPI clock edge counter
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		clk_edge_cnt <= 5'd0;
	else if(state == DCLK_EDGE)
		clk_edge_cnt <= clk_edge_cnt + 5'd1;
	else if(state == IDLE)
		clk_edge_cnt <= 5'd0;
end
//SPI data output
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		MOSI_shift <= 8'd0;
	else if(state == IDLE && wr_req)
		MOSI_shift <= data_in;
	else if(state == DCLK_EDGE)
		if(CPHA == 1'b0 && clk_edge_cnt[0] == 1'b1)
			MOSI_shift <= {MOSI_shift[6:0],MOSI_shift[7]};
		else if(CPHA == 1'b1 && (clk_edge_cnt != 5'd0 && clk_edge_cnt[0] == 1'b0))
			MOSI_shift <= {MOSI_shift[6:0],MOSI_shift[7]};
end
//SPI data input
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		MISO_shift <= 8'd0;
	else if(state == IDLE && wr_req)
		MISO_shift <= 8'h00;
	else if(state == DCLK_EDGE)
		if(CPHA == 1'b0 && clk_edge_cnt[0] == 1'b0)
			MISO_shift <= {MISO_shift[6:0],MISO};
		else if(CPHA == 1'b1 && (clk_edge_cnt[0] == 1'b1))
			MISO_shift <= {MISO_shift[6:0],MISO};
end
endmodule