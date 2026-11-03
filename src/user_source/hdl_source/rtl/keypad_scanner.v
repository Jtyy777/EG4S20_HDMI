// ============================================================================
// keypad_scanner.v  —— 4x4 矩阵键盘扫描（纯自研模块）
// ----------------------------------------------------------------------------
// 功能：扫描核心板（DMZ_EG4S20）的 4x4 矩阵键盘，输出「按下脉冲」与「按住电平」。
//       本工程只用它接两颗扩展按键，但 16 个键的映射都完整实现了。
//
// 按键布局（KEY0~KEY15，4 行 4 列）
//   Col0  Col1  Col2  Col3
//   Row0: KEY0   KEY1   KEY2   KEY3
//   Row1: KEY4   KEY5   KEY6   KEY7
//   Row2: KEY8   KEY9   KEY10  KEY11
//   Row3: KEY12  KEY13  KEY14  KEY15
//
// 一、扫描时序
//   一次扫描分两个相位：稳定期（让行线上的电平建立）+ 采样期（读列）。
//   每个相位 SCAN_DIV = 25000 拍 = 0.5ms（50MHz），所以
//     · 每行 1ms，4 行轮一遍 = 4ms 一次完整扫描（即每秒约 250 次）；
//     · 行选是「低电平有效、其余为高」的独热码：1110 / 1101 / 1011 / 0111。
//
// 二、消抖方式
//   这里不是计时消抖，而是「采样式消抖」：每个按键的电平每 4ms 才被采一次，
//   存进 col_sync0，再无延迟打一拍得到 col_sync1。毛刺撑不过一个采样点
//   就自然被滤掉；能连续跨过两个采样点的抖动才可能被骗过去（实际极少）。
//   key_press_pulse = ~col_sync1 & col_stable，即「这一拍按下、上一拍还松着」，
//   也就是下降沿（高→低 = 按下），单拍脉冲，一条按键只发一次。
//
// 三、给 param_controller 用的 key_hold
//   key_hold = ~col_sync1，高电平表示当前被按住。它的存在是为了支持
//   「长按连续调节」——按住不放要不停地改参数，靠单次脉冲做不到。
// ============================================================================

module keypad_scanner #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer SCAN_DIV     = 25000    // 每个扫描状态持续 0.5ms
)(
    input  wire        clk,
    input  wire        rst,

    // 矩阵键盘物理接口（Row 输出扫描，Col 输入读取）
    output reg  [3:0]  key_row,     // Key_Row[3:0]（输出，扫描时拉低）
    input  wire [3:0]  key_col,     // Key_Col[3:0]（输入，按下为低）

    // 按键事件输出（去抖后单拍脉冲）
    output reg  [15:0] key_press_pulse,  // 每位对应 KEY0~KEY15 按下脉冲

    // 按住电平（去抖后，高电平 = 该键当前被按住）
    // 供 param_controller 实现"长按连续调节"（连发）使用。
    output wire [15:0] key_hold
);

    // col_sync1 是"低=按下"的电平，取反得到高有效的按住状态
    reg [1:0]  scan_row;         // 当前扫描的行 0~3
    reg [15:0] scan_div_cnt;     // 分频计数（半周期）
    reg        scan_phase;       // 0: 行稳定期，1: 采样期

    // 每个按键的同步与去抖
    reg [15:0] col_sync0;
    reg [15:0] col_sync1;
    reg [15:0] col_stable;

    assign key_hold = ~col_sync1;

    integer i;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            scan_row        <= 2'd0;
            scan_div_cnt    <= 16'd0;
            scan_phase      <= 1'b0;
            key_row         <= 4'b1110;   // 初始扫描 Row0
            col_sync0       <= 16'hFFFF;
            col_sync1       <= 16'hFFFF;
            col_stable      <= 16'hFFFF;
            key_press_pulse <= 16'd0;
        end else begin
            key_press_pulse <= 16'd0;

            // 分频计数
            if (scan_div_cnt >= SCAN_DIV - 1) begin
                scan_div_cnt <= 16'd0;
                scan_phase   <= ~scan_phase;

                if (scan_phase == 1'b1) begin
                    // 采样期结束：读取当前行的列状态，然后切换到下一行
                    case (scan_row)
                        2'd0: col_sync0[3:0]   <= key_col;
                        2'd1: col_sync0[7:4]   <= key_col;
                        2'd2: col_sync0[11:8]  <= key_col;
                        2'd3: col_sync0[15:12] <= key_col;
                    endcase

                    // 切换到下一行
                    scan_row <= scan_row + 2'd1;
                    case (scan_row + 2'd1)
                        2'd0: key_row <= 4'b1110;
                        2'd1: key_row <= 4'b1101;
                        2'd2: key_row <= 4'b1011;
                        default: key_row <= 4'b0111;
                    endcase
                end
                // 稳定期结束不做事，等待下一半周期采样
            end else begin
                scan_div_cnt <= scan_div_cnt + 16'd1;
            end

            // 采样式消抖：col_sync0 每 4ms 才更新一次（每行轮到才采），
            // col_sync1 再跟一拍。key_press_pulse 是「当前按下 & 上一拍还松」，
            // 也就是按下那一瞬间的单拍脉冲。
            // （上面第 62 行还有一处清零，会在本行被覆盖 —— 非阻塞赋值后写生效，
            //   保留它是无害的，实际起作用的只有这一行。）
            col_sync1  <= col_sync0;
            col_stable <= col_sync1;
            key_press_pulse <= (~col_sync1) & col_stable;
        end
    end

endmodule
