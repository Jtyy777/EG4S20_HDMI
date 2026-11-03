// ============================================================================
// seg4_driver.v  —— 核心板 4 位数码管驱动（纯自研模块，适配 DMZ_EG4S20）
//
// 功能：驱动核心板共阴 4 位七段数码管，动态扫描显示。
//   - 片选 DigitronCS[3:0]（低电平选中，从右到左 DIG0~DIG3）
//   - 段码 Digitron_Out[7:0]（bit0=A ... bit7=DOT，高电平点亮）
//
// 一、扫描节奏
//   每位停留 CLK_FREQ/(SCAN_FREQ*4)-1 = 62499 拍，即 1.25ms；
//   4 位轮一遍 5ms，整屏刷新 200Hz（SCAN_FREQ_HZ 描述的是这一整屏速率）。
//   200Hz 远高于人眼闪烁融合频率（约 60Hz），看不到闪烁。
//   选 200Hz 而不是更快，是因为 seg_sel / seg_data 这 12 根 IO 的翻转次数
//   跟扫描频率成正比：从 1kHz 降到 200Hz，这部分的开关功耗降到 1/5，
//   而占空比仍是 1/4、单段平均电流仍是 2mA，显示亮度不受影响。
//
// 二、段码表
//   用 function 查表（seg_lut），0~9 + A~F 共 16 项，{DOT,G,F,E,D,C,B,A} 排列。
//   共阴管的段码和共阳管正好互补 —— 换板子时要确认这个极性，搞反了会
//   显示成「该亮的灭、该灭的亮」。
//
// 三、一拍延迟
//   seg_sel 与 seg_data 都是寄存器输出，seg_data 直接用上一拍的 cur_digit
//   查表，所以切换那一位时两者会错开 1 个时钟（20ns）。人眼完全看不见，
//   但示波器上能抓到，属于正常现象。
// ============================================================================

module seg4_driver #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer SCAN_FREQ_HZ = 200       // 整屏刷新率；200Hz 无闪烁，再快只是白烧 IO
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [3:0]  digit0,       // 最右位 DIG0
    input  wire [3:0]  digit1,       // DIG1
    input  wire [3:0]  digit2,       // DIG2
    input  wire [3:0]  digit3,       // 最左位 DIG3

    output reg  [3:0]  seg_sel,      // DigitronCS[3:0]，低有效
    output reg  [7:0]  seg_data      // Digitron_Out[7:0]，高电平点亮段
);

    // 段码查找（共阴，高电平点亮）：{DOT,G,F,E,D,C,B,A}
    function [7:0] seg_lut;
        input [3:0] val;
        begin
            case (val)
                4'd0: seg_lut = 8'b0011_1111;   // 0
                4'd1: seg_lut = 8'b0000_0110;   // 1
                4'd2: seg_lut = 8'b0101_1011;   // 2
                4'd3: seg_lut = 8'b0100_1111;   // 3
                4'd4: seg_lut = 8'b0110_0110;   // 4
                4'd5: seg_lut = 8'b0110_1101;   // 5
                4'd6: seg_lut = 8'b0111_1101;   // 6
                4'd7: seg_lut = 8'b0000_0111;   // 7
                4'd8: seg_lut = 8'b0111_1111;   // 8
                4'd9: seg_lut = 8'b0110_1111;   // 9
                4'ha: seg_lut = 8'b0111_0111;   // A
                4'hb: seg_lut = 8'b0111_1100;   // b
                4'hc: seg_lut = 8'b0011_1001;   // C
                4'hd: seg_lut = 8'b0101_1110;   // d
                4'he: seg_lut = 8'b0111_1001;   // E
                4'hf: seg_lut = 8'b0111_0001;   // F
                default: seg_lut = 8'b0000_0000;
            endcase
        end
    endfunction

    reg [31:0] scan_timer;
    reg [1:0]  scan_idx;
    reg [3:0]  cur_digit;

    localparam integer SCAN_COUNT = CLK_FREQ_HZ / (SCAN_FREQ_HZ * 4) - 1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scan_timer <= 32'd0;
            scan_idx   <= 2'd0;
            seg_sel    <= 4'b1111;
            seg_data   <= 8'b0000_0000;
        end else begin
            if (scan_timer >= SCAN_COUNT) begin
                scan_timer <= 32'd0;
                scan_idx   <= scan_idx + 2'd1;
            end else begin
                scan_timer <= scan_timer + 32'd1;
            end

            // 当前位片选（低有效）与段码
            case (scan_idx)
                2'd0: begin
                    seg_sel  <= 4'b1110;      // DIG0（最右）
                    cur_digit <= digit0;
                end
                2'd1: begin
                    seg_sel  <= 4'b1101;      // DIG1
                    cur_digit <= digit1;
                end
                2'd2: begin
                    seg_sel  <= 4'b1011;      // DIG2
                    cur_digit <= digit2;
                end
                2'd3: begin
                    seg_sel  <= 4'b0111;      // DIG3（最左）
                    cur_digit <= digit3;
                end
                default: begin
                    seg_sel  <= 4'b1111;
                    cur_digit <= 4'd0;
                end
            endcase

            seg_data <= seg_lut(cur_digit);
        end
    end

endmodule
