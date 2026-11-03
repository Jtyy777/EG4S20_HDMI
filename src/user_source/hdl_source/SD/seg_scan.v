// ============================================================================
// seg_scan —— 6 位七段数码管动态扫描显示
// ----------------------------------------------------------------------------
// 6 个数码管共用一组段线，靠 seg_sel 位选轮流点亮：一次只点一位，
// 靠人眼视觉暂留看成 6 位同时亮。整屏刷新率 SCAN_FREQ = 200Hz，
// 所以每位停留 CLK_FREQ/(SCAN_FREQ×6) = 50MHz/1200 ≈ 41667 个时钟（约 0.83ms）。
//
// 端口：
//   seg_sel[5:0]  位选，低电平有效（同一时刻只有一位为 0）
//   seg_data[7:0] 段码，最高位保留（小数点/固定），低 7 位给段线
//   seg_data_N    第 N 位数码管要显示的内容，由调用方按需拼好
//
// 注意：
//   · 段码是「共阳」极性（0 亮 1 灭），与 seg_decoder 的输出一致；
//   · SCAN_FREQ 调太低会看到闪烁，调太高则单管亮度下降，一般 100~1000Hz；
//   · 复位时 seg_sel 全 1、seg_data 全 1 = 全部熄灭，避免上电瞬间乱亮。
// ============================================================================
module seg_scan(
	input           clk,
	input           rst_n,
	output reg[5:0] seg_sel,
	output reg[7:0] seg_data,
	input[7:0]      seg_data_0,
	input[7:0]      seg_data_1,
	input[7:0]      seg_data_2,
	input[7:0]      seg_data_3,
	input[7:0]      seg_data_4,
	input[7:0]      seg_data_5
);
parameter SCAN_FREQ = 200;
parameter CLK_FREQ = 50000000;

parameter SCAN_COUNT = CLK_FREQ /(SCAN_FREQ * 6) - 1;

reg[31:0] scan_timer;
reg[3:0] scan_sel;
always@(posedge clk or negedge rst_n)
begin
	if(rst_n == 1'b0)
	begin
		scan_timer <= 32'd0;
		scan_sel <= 4'd0;
	end
	else if(scan_timer >= SCAN_COUNT)
	begin
		scan_timer <= 32'd0;
		if(scan_sel == 4'd5)
			scan_sel <= 4'd0;
		else
			scan_sel <= scan_sel + 4'd1;
	end
	else
		begin
			scan_timer <= scan_timer + 32'd1;
		end
end
always@(posedge clk or negedge rst_n)
begin
	if(rst_n == 1'b0)
	begin
		seg_sel <= 6'b111111;
		seg_data <= 8'hff;
	end
	else
	begin
		case(scan_sel)
			4'd0:
			begin
				seg_sel <= 6'b11_1110;
				seg_data <= seg_data_0;
			end
			4'd1:
			begin
				seg_sel <= 6'b11_1101;
				seg_data <= seg_data_1;
			end
			4'd2:
			begin
				seg_sel <= 6'b11_1011;
				seg_data <= seg_data_2;
			end
			4'd3:
			begin
				seg_sel <= 6'b11_0111;
				seg_data <= seg_data_3;
			end
			4'd4:
			begin
				seg_sel <= 6'b10_1111;
				seg_data <= seg_data_4;
			end
			4'd5:
			begin
				seg_sel <= 6'b01_1111;
				seg_data <= seg_data_5;
			end
			default:
			begin
				seg_sel <= 6'b11_1111;
				seg_data <= 8'hff;
			end
		endcase
	end
end

endmodule
