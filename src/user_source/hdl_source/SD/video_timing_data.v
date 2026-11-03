// ============================================================================
// video_timing_data —— 顶层视频时序入口（产生 hs/vs/de + 每帧读请求）
// ----------------------------------------------------------------------------
// 内部只做三件事：
//   1) 例化 color_bar 做时序发生器。色彩输出（rgb_r/g/b）在这里被悬空——
//      彩色画面不参与显示，真正的画面数据来自 SDRAM 读侧，所以不必接；
//   2) 把 color_bar 输出的 hs/vs/de 再打两拍（_d0/_d1）后送出。多打的这两拍
//      是为了给下游的取数/处理流水留对齐余量：顶层最终送出的是 _d1，
//      与之配套的像素数据也必须延迟同样的拍数（见 video_delay 的 21 拍）；
//   3) 用 vs 的下降沿产生 read_req ——「这一帧结束了，去预取下一帧」。
//      read_req 一直保持到 read_req_ack 回来才撤销，是标准的
//      「请求-应答」握手机制，避免读侧还在处理时被重复触发。
//
// 相关：video_delay（像素数据延迟对齐）、frame_fifo_read（接收 read_req 的
// 另一侧）、color_bar（时序本体与彩条）。
// ============================================================================
module video_timing_data
#(
	parameter DATA_WIDTH = 16                       // Video data one clock data width
)
(
	input                       video_clk,          // Video pixel clock
	input                       rst,
	output reg                  read_req,           // Start reading a frame of data     
	input                       read_req_ack,       // Read request response
	//output                      read_en,            // Read data enable
	//input[DATA_WIDTH - 1:0]     read_data,          // Read data
	output                      hs,                 // horizontal synchronization
	output                      vs,                 // vertical synchronization
	output                      de                  // video valid
	//output[DATA_WIDTH - 1:0]    vout_data           // video data
);
wire                   video_hs;
wire                   video_vs;
wire                   video_de;
//delay video_hs video_vs  video_de 2 clock cycles
reg                    video_hs_d0;
reg                    video_vs_d0;
reg                    video_de_d0;
reg                    video_hs_d1;
reg                    video_vs_d1;
reg                    video_de_d1;

//reg[DATA_WIDTH - 1:0]  vout_data_r;
//assign read_en = video_de;
assign hs = video_hs_d1;
assign vs = video_vs_d1;
assign de = video_de_d1;
//assign vout_data = vout_data_r;
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		video_hs_d0 <= 1'b0;
		video_vs_d0 <= 1'b0;
		video_de_d0 <= 1'b0;
	end
	else
	begin
		//delay video_hs video_vs  video_de 2 clock cycles
		video_hs_d0 <= video_hs;
		video_vs_d0 <= video_vs;
		video_de_d0 <= video_de;
		video_hs_d1 <= video_hs_d0;
		video_vs_d1 <= video_vs_d0;
		video_de_d1 <= video_de_d0;		
	end
end
/*
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		vout_data_r <= {DATA_WIDTH{1'b0}};
	else if(video_de_d0)
		vout_data_r <= read_data;
	else
		vout_data_r <= {DATA_WIDTH{1'b0}};
end
*/
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		read_req <= 1'b0;
	else if(video_vs_d0 & ~video_vs) //vertical synchronization edge (the rising or falling edges are OK)
		read_req <= 1'b1;
	else if(read_req_ack)
		read_req <= 1'b0;
end

color_bar color_bar_m0(
	.clk(video_clk),
	.rst(rst),
	.hs(video_hs),
	.vs(video_vs),
	.de(video_de),
	.rgb_r(),
	.rgb_g(),
	.rgb_b()
);
endmodule 