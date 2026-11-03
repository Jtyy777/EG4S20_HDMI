// ============================================================================
// param_controller —— 交互参数控制器：按键事件 → 图像参数
// ----------------------------------------------------------------------------
// 功能：把矩阵键盘的按键事件映射为图像处理参数，支撑"实时参数调节 + OSD"。
//   - 亮度   bright       （-96 ~ +96，步进 4，共 49 级）
//   - 对比度 contrast_q8  （128 ~ 512，即 0.50x ~ 2.00x，步进 8 = 0.03125x）
//   - 自动播放 / 手动切图控制
//
// 按键定义（矩阵键盘 KEY0~KEY15；删掉原 KEY2 转场类型切换后，
//           原 KEY3~KEY7 整体前移一位，再于 11-03 补入 KEY7 显隐键，
//           当前有效键为 KEY0~KEY7 共 8 个）：
//   KEY0 (ROW0,COL0): 手动切下一张
//   KEY1 (ROW0,COL1): 自动播放 开/关
//   KEY2 (ROW0,COL2): 亮度 +4（长按连发）
//   KEY3 (ROW0,COL3): 亮度 -4（长按连发）
//   KEY4 (ROW1,COL0): 对比度 +0.03125x（长按连发）
//   KEY5 (ROW1,COL1): 对比度 -0.03125x（长按连发）
//   KEY6 (ROW1,COL2): 复位参数到默认
//   KEY7 (ROW1,COL3): OSD 信息层显隐（亮度/对比度 + 切图耗时 T/W/L 一起显隐）
//   KEY8~KEY15      : 未使用
//
// OSD 显隐说明：上电默认隐藏（osd_show=0），按一次 KEY7 显示，再按一次消失。
//   受控的是两处信息层 —— 左上角 "L:/C:" 与右上角 "T:/W:/L:" 三行；
//   右上角图片计数器 "2/4" 不受影响，始终显示。
//   参数调节键（KEY2~KEY5）在隐藏状态下依然生效，只是看不到读数。
//
// 原 KEY2 是"切换转场类型（trans_sel 0~3 循环）"，但 transition_effect 本版
// 统一白场淡化、内部不引用 trans_sel，按了画面毫无变化——是个"看着有功能
// 其实是死键"的坑，已删除，trans_sel 端口一并去掉。
//
// 长按连发：按住 400ms 后开始自动重复，之后每 100ms 一档，
//           所以从 1.00x 拉到 2.00x 只需按住约 3.4 秒。
// ============================================================================

module param_controller (
    input  wire        clk,
    input  wire        rst,

    input  wire [15:0] key_press,       // 矩阵键盘去抖单拍脉冲
    input  wire [15:0] key_hold,        // 矩阵键盘去抖按住电平（高=按住）

    output reg         key_next,        // 手动切图（低有效，保持 100ms）
    output reg         key_auto,        // 自动播放开关（低有效）
    output reg         osd_show,        // OSD 信息层显隐（0=隐藏，1=显示），KEY7 翻转
    output reg  [7:0]  bright,          // 亮度（有符号，-96~+96）
    output reg  [9:0]  contrast_q8      // 对比度（256=1.0x，范围 128~512）
);

    // ---------------- 步进与边界 ----------------
    // 亮度：每按一次 ±4（原来 ±16，太粗）
    localparam signed [8:0] BRIGHT_STEP   = 9'sd4;
    localparam signed [8:0] BRIGHT_MAX_P  = 9'sd92;    // 再加一档不越过 ±96
    localparam signed [8:0] BRIGHT_MIN_P  = -9'sd92;   // 再减一档不越过 ±96

    // 对比度：每按一次 ±8（= 0.03125x，原来 ±32 = 0.125x）
    localparam [9:0] CONTRAST_STEP = 10'd8;
    localparam [9:0] CONTRAST_MAX  = 10'd512;          // 2.00x
    localparam [9:0] CONTRAST_MIN  = 10'd128;          // 0.50x

    // 默认参数
    localparam [7:0]  DEFAULT_BRIGHT   = 8'd0;         // 0
    localparam [9:0]  DEFAULT_CONTRAST = 10'd256;      // 1.0x

    // 按住连发计时（50MHz）
    //   400ms = 20_000_000 拍；100ms = 5_000_000 拍
    localparam [24:0] REP_DELAY  = 25'd20_000_000;
    localparam [24:0] REP_PERIOD = 25'd5_000_000;

    // 亮度是 8bit 二进制补码（-128~+127），必须按有符号数比较边界。
    // 原来写成 "bright < 8'd127" / "bright > -8'd128"：
    //   前者把 bright 当无符号数，bright=-16(8'hF0=240) 判为 >127，亮度+ 失效；
    //   后者 8 位无符号量与负数比较，结果恒为假，亮度- 完全失效。
    // 这里扩展成 9 位有符号视图，判断准确且不会溢出。
    wire signed [8:0] bright_s = {bright[7], bright};

    // ---------------- 长按连发 ----------------
    // 四个调节键：KEY2(亮+)、KEY3(亮-)、KEY4(对比+)、KEY5(对比-)
    wire [3:0] adj_hold = {key_hold[5], key_hold[4], key_hold[3], key_hold[2]};
    wire       adj_any  = |adj_hold;

    reg [24:0] rep_cnt;
    reg        rep_en;      // 0=首次延时阶段 1=周期重复阶段

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rep_cnt <= 25'd0;
            rep_en  <= 1'b0;
        end else if (!adj_any) begin
            // 全部松开：重新计时
            rep_cnt <= 25'd0;
            rep_en  <= 1'b0;
        end else if (!rep_en) begin
            // 首次延时 400ms
            if (rep_cnt >= REP_DELAY - 25'd1) begin
                rep_cnt <= 25'd0;
                rep_en  <= 1'b1;
            end else begin
                rep_cnt <= rep_cnt + 25'd1;
            end
        end else begin
            // 之后每 100ms 一档
            if (rep_cnt >= REP_PERIOD - 25'd1)
                rep_cnt <= 25'd0;
            else
                rep_cnt <= rep_cnt + 25'd1;
        end
    end

    // 连发脉冲（与上面计数器同拍产生，1 拍宽）
    wire rep_pulse = rep_en ? (rep_cnt == REP_PERIOD - 25'd1)
                            : (rep_cnt == REP_DELAY  - 25'd1);

    // 每个调节键的"生效事件"：首次按下 或 长按连发
    wire ev_bright_up   = key_press[2] || (rep_pulse && key_hold[2]);
    wire ev_bright_dn   = key_press[3] || (rep_pulse && key_hold[3]);
    wire ev_contrast_up = key_press[4] || (rep_pulse && key_hold[4]);
    wire ev_contrast_dn = key_press[5] || (rep_pulse && key_hold[5]);

    // ---------------- 手动切图 / 自动播放的脉冲展宽 ----------------
    // 按键低电平保持计数（50MHz 下 100ms ≈ 5,000,000 拍）
    reg [22:0] key_low_cnt;
    reg        key_low_active;
    reg [22:0] key_auto_low_cnt;
    reg        key_auto_low_active;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            key_next           <= 1'b1;    // 低有效，默认松开
            key_auto           <= 1'b1;
            osd_show           <= 1'b0;    // 上电默认隐藏信息层
            bright             <= DEFAULT_BRIGHT;
            contrast_q8        <= DEFAULT_CONTRAST;
            key_low_cnt        <= 23'd0;
            key_low_active     <= 1'b0;
            key_auto_low_cnt   <= 23'd0;
            key_auto_low_active<= 1'b0;
        end else begin
            key_next <= 1'b1;
            key_auto <= 1'b1;

            // KEY0: 手动切下一张（触发 40ms 低电平，sd_card_bmp 去抖 20ms）
            // 原为 100ms：与去抖 50ms 串起来，按一次键就是 150ms 的固定延时，
            // 而这段时间画面完全没反应。40ms 已足够越过 20ms 去抖，余量 20ms。
            if (key_press[0]) begin
                key_low_active <= 1'b1;
                key_low_cnt    <= 23'd0;
            end
            if (key_low_active) begin
                key_next <= 1'b0;
                if (key_low_cnt >= 23'd2_000_000) begin    // 40ms @50MHz
                    key_low_active <= 1'b0;
                end else begin
                    key_low_cnt <= key_low_cnt + 23'd1;
                end
            end

            // KEY1: 自动播放开/关（触发 100ms 低电平脉冲，sd_card_bmp 检测下降沿）
            if (key_press[1]) begin
                key_auto_low_active <= 1'b1;
                key_auto_low_cnt    <= 23'd0;
            end
            if (key_auto_low_active) begin
                key_auto <= 1'b0;
                if (key_auto_low_cnt >= 23'd5_000_000) begin
                    key_auto_low_active <= 1'b0;
                end else begin
                    key_auto_low_cnt <= key_auto_low_cnt + 23'd1;
                end
            end

            // KEY2: 亮度 +4
            if (ev_bright_up && (bright_s <= BRIGHT_MAX_P))
                bright <= bright + 8'd4;

            // KEY3: 亮度 -4
            if (ev_bright_dn && (bright_s >= BRIGHT_MIN_P))
                bright <= bright - 8'd4;

            // KEY4: 对比度 +0.03125x
            // 注意：contrast_q8 已扩成 10 位（0~1023），所以 10'd512 不会再被
            // 截断成 0（旧代码 "9'd512" 在 9 位里等于 0，导致对比度+ 恒失效）。
            if (ev_contrast_up && (contrast_q8 < CONTRAST_MAX))
                contrast_q8 <= contrast_q8 + CONTRAST_STEP;

            // KEY5: 对比度 -0.03125x
            if (ev_contrast_dn && (contrast_q8 > CONTRAST_MIN))
                contrast_q8 <= contrast_q8 - CONTRAST_STEP;

            // KEY6: 复位默认参数
            if (key_press[6]) begin
                bright      <= DEFAULT_BRIGHT;
                contrast_q8 <= DEFAULT_CONTRAST;
            end

            // KEY7: OSD 信息层显隐翻转
            //   key_press 已经是去抖后的单拍脉冲，这里只做电平翻转，不再展宽。
            //   跨到 video_clk 域由顶层两级同步完成（单 bit 慢变电平，不需要握手）。
            //   只翻 osd_show，不碰 bright/contrast：隐藏 OSD 不等于恢复默认值。
            if (key_press[7])
                osd_show <= ~osd_show;
        end
    end

endmodule
