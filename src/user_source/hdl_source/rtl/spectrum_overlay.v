// ============================================================================
// spectrum_overlay.v  —— 频谱柱状图叠加（纯自研模块）
//
// ============================ 布局与画法说明 ==========================
// 用户反馈："美化一下并缩小音量条放在左下角，同时不要有黑框遮挡图片"
//
// 相对上一版（v3，居中 + 背景板 + 暗轨道）的三处结构性变化：
//
// 1) **去掉背景板**（原来 x184~455 / y348~435 那块近黑矩形）
//    用户说的"黑框"就是它。现在直接画在原始图像上，四周完全透明，
//    除了柱子本身和一条细基线，一个像素都不遮挡图片。
//
// 2) **去掉暗色轨道**（原来每根柱都有 72 行暗绿底条，8 根 = 8 个色块）
//    轨道本质也是"遮住图片"。现在把柱高下限抬到 26 像素，最矮的柱子
//    也清晰可见，不需要轨道来占位。柱底只留一条 2 像素基底线当底座。
//
// 3) **缩小 + 移到左下角**
//      8 根 × PITCH 24 = 192 宽，START_X = 16  → x 16~207
//      柱身宽 16，间隔 8；柱底 y=428；轨道高 60 → 顶 y=368
//    占地 192×60（上一版 272×88，面积再减 51%），紧贴左下角。
//    柱底 428 + 基线 2 行 = 430，距 OSD 字幕条（y>=440）还有 10 行空隙。
//
// 美化手段（全部是纯组合逻辑，不增加任何遮挡面积）：
//   · 柱身竖向三段渐变：柱顶 2 行提亮(+48) → 中段主色 → 下段 75% 暗调，
//     像一根有体积感的发光柱，而不是一块死板的色条。
//   · 颜色随柱高连续变化：低柱纯绿 → 中柱黄绿 → 高柱橙红
//     r = (h-24)<<3（饱和到 255），g = 255-r，蓝分量固定让颜色更通透。
//   · 当前能量最强的柱（active_col）顶部 4 行纯白顶帽，
//     用来标出这一段音频的主导频带落在哪一根。
//
// ============================ 关键坐标系 ====================================
// 本模块内部用**自己的 de/vs 现算 x/y**，与 rgb_in 天然同拍，
// 不依赖顶层 px_x/px_y 的任何延迟假设（v2 就栽在坐标与数据不同拍，
// 整块图形相对画面横向漂移）。规则同 pixel_coord_gen：
//   de 期间 x 自增 → 满 H_ACTIVE 归零并 y+1 → vs 上升沿全部清零。
//
// ============================ 柱宽 / PITCH 说明 ============================
// PITCH = 24 不是 2 的幂，没法直接用位选当除法，这里用嵌套除法恒等式：
//   band = xr / 24 = (xr / 8) / 3 = (xr >> 3) / 3
// 其中 /3 用 q*11>>5 近似（对 q ∈ [0,23] 结果与 floor(q/3) 完全一致）。
// floor(floor(x/8)/3) == floor(x/24) 是整数除法的标准恒等式，无误差。
// 这样既避免了组合除法（安路 TD 对组合除法支持不可靠），也不用乘法器。
// ============================================================================

module spectrum_overlay #(
    parameter integer H_ACTIVE  = 640,
    parameter integer V_ACTIVE  = 480,
    parameter integer NUM_BANDS = 8,
    parameter integer START_X   = 16,    // 第一根柱左边（左下角）
    parameter integer PITCH     = 24,    // 柱间距
    parameter integer BODY_W    = 16,    // 柱身宽（其余为间隔）
    parameter integer BASE_Y    = 428,   // 柱底 y
    parameter integer TRACK_H   = 60,    // 柱高上限（= audio_spectrum 的 BAR_MAX_H）
    parameter integer HAT_H     = 4      // 当前更新柱的白色顶帽高度（压在柱身顶部）
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        de,          // 与 rgb_in 同拍的有效使能
    input  wire        vs,          // 帧同步（本模块内部重建坐标用）

    input  wire [23:0] rgb_in,
    // 扁平化柱高向量，band_i 在 [i*8+7 : i*8]（每根 8bit）
    input  wire [NUM_BANDS*8-1:0] bar_h,
    // 能量最强的频带（来自 audio_spectrum 的 argmax），本模块在它顶上画白帽
    input  wire [2:0]  active_col,

    output wire [23:0] rgb_out,
    output reg         de_out
);

    // ------------------------------------------------------------------
    // 0. 内部坐标：用本模块自己的 de/vs 现算，天然与 rgb_in 同拍
    // ------------------------------------------------------------------
    localparam [10:0] H_LAST = H_ACTIVE - 1;
    localparam [10:0] V_LAST = V_ACTIVE - 1;

    reg  [10:0] x, y;
    reg         vs_d1;
    wire        vs_rise = vs & ~vs_d1;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            vs_d1 <= 1'b0;
            x     <= 11'd0;
            y     <= 11'd0;
        end else begin
            vs_d1 <= vs;
            if (vs_rise) begin
                x <= 11'd0;
                y <= 11'd0;
            end else if (de) begin
                if (x >= H_LAST) begin
                    x <= 11'd0;
                    y <= (y >= V_LAST) ? 11'd0 : (y + 11'd1);
                end else begin
                    x <= x + 11'd1;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // 1. 几何常量（全部先落成定宽 localparam，避免 integer 直连比较）
    // ------------------------------------------------------------------
    localparam [10:0] START_V   = START_X;                       // 16
    localparam [10:0] BAND_END  = START_X + NUM_BANDS * PITCH;   // 16+192 = 208
    localparam [10:0] BASE_V    = BASE_Y;                        // 428
    localparam [10:0] TRACK_TOP = BASE_Y - TRACK_H;              // 368
    localparam [7:0]  BODY_V    = BODY_W;                        // 16
    localparam [10:0] HAT_V     = HAT_H;                         // 4
    localparam [7:0]  TRACKH_V  = TRACK_H;                       // 必须 < 128
    localparam [10:0] LINE_X0   = START_X - 4;                   // 12（基线两端各出头 4）
    localparam [10:0] LINE_X1   = BAND_END + 4;                  // 212

    // ------------------------------------------------------------------
    // 2. 定位到柱：band = xr/24（嵌套除法），col_off = xr - band*24
    // ------------------------------------------------------------------
    wire        in_x = (x >= START_V) && (x < BAND_END);
    wire [10:0] xr   = in_x ? (x - START_V) : 11'd0;
    wire [7:0]  xr8  = xr[7:0];                  // 0 ~ 191

    wire [4:0]  q8   = xr8[7:3];                            // xr >> 3，0 ~ 23
    wire [7:0]  qe   = {3'd0, q8};
    wire [7:0]  q11d = (qe << 3) + (qe << 1) + qe;          // q*11，最大 253
    wire [2:0]  band_idx = q11d[7:5];                       // /32 ≈ /3，0 ~ 7

    // band*24 = band*16 + band*8
    wire [7:0]  band_base = ({5'd0, band_idx} << 4) + ({5'd0, band_idx} << 3);
    wire [7:0]  col_off   = xr8 - band_base;                // 0 ~ 23
    wire        in_body   = in_x && (col_off < BODY_V);

    // band_i 的柱高：用显式 8 路选择，不用 bar_h[band_base +: 8]。
    // 变量索引的部分选择（variable part-select）在部分综合器上会被推断成
    // 奇怪的结构，甚至只让低几根柱拿到正确值，所以写成定宽 case。
    reg [7:0] this_h;
    always @(*) begin
        case (band_idx)
            3'd0: this_h = bar_h[7:0];
            3'd1: this_h = bar_h[15:8];
            3'd2: this_h = bar_h[23:16];
            3'd3: this_h = bar_h[31:24];
            3'd4: this_h = bar_h[39:32];
            3'd5: this_h = bar_h[47:40];
            3'd6: this_h = bar_h[55:48];
            default: this_h = bar_h[63:56];
        endcase
    end

    // 越界保护：历史遗留值大于轨道高时也不许把柱顶顶到板子外面
    wire [7:0]  h_val   = (this_h > TRACKH_V) ? TRACKH_V : this_h;
    wire [10:0] bar_top = BASE_V - {3'd0, h_val};

    // ------------------------------------------------------------------
    // 3. 图层判定（只有柱身 / 顶帽 / 基线三层，没有任何色块底板）
    // ------------------------------------------------------------------
    wire in_lit  = in_body && (y >= bar_top) && (y < BASE_V);

    // 白色顶帽：覆盖柱身内部的顶部 4 行（不是柱顶之上）
    //   v4 曾把帽画在柱顶"之上"，也就是图片区域。去掉暗色轨道后那 3 行全是
    //   原图，纯白直接融进浅色图片 → 用户报"白帽没了"。现在压在彩色柱身自己
    //   身上，对比永远是"纯白 vs 彩绿/橙红"，且不向图片区多占一个像素。
    //   h_val=0（该柱还没轮到/无音）时 bar_top = BASE_V，帽落在基线上，
    //   优先级高于基线，仍是一个可见的白块。
    wire in_hat  = in_body && (band_idx == active_col) &&
                   (y >= bar_top) && (y < (bar_top + HAT_V));

    wire in_base = (x >= LINE_X0) && (x < LINE_X1) &&
                   (y >= BASE_V) && (y < (BASE_V + 11'd2));

    // ------------------------------------------------------------------
    // 4. 柱身竖向渐变：柱顶 2 行提亮 → 中段主色 → 下段 75% 暗调
    // ------------------------------------------------------------------
    wire [10:0] dtop  = in_lit ? (y - bar_top) : 11'd0;   // 0 = 柱顶那一行
    wire [10:0] halfh = {3'd0, h_val[7:1]};               // 柱高的一半
    wire        z_top = (dtop < 11'd2);
    wire        z_low = (dtop >= halfh);

    // ------------------------------------------------------------------
    // 5. 颜色：r = (h-24)<<3（饱和 255），g = 255-r，蓝分量固定
    //    实测 26 30 35 37 43 49 55 58 → 纯绿 → 黄绿 → 橙红
    // ------------------------------------------------------------------
    wire [7:0]  h_off = (h_val > 8'd24) ? (h_val - 8'd24) : 8'd0;   // 0 ~ 36
    wire [8:0]  r9    = {1'b0, h_off} << 3;                          // 0 ~ 288
    wire [7:0]  lr    = r9[8] ? 8'd255 : r9[7:0];
    wire [7:0]  lg    = 8'd255 - lr;

    // 提亮 +48（9 位承接，防溢出）
    wire [8:0]  rh9   = {1'b0, lr} + 9'd48;
    wire [8:0]  gh9   = {1'b0, lg} + 9'd48;
    wire [7:0]  lr_hi = rh9[8] ? 8'd255 : rh9[7:0];
    wire [7:0]  lg_hi = gh9[8] ? 8'd255 : gh9[7:0];

    // 暗调 ×0.75 = x/2 + x/4
    wire [7:0]  lr_lo = {1'b0, lr[7:1]} + {2'd0, lr[7:2]};
    wire [7:0]  lg_lo = {1'b0, lg[7:1]} + {2'd0, lg[7:2]};

    wire [23:0] col_hi   = {lr_hi, lg_hi, 8'h70};
    wire [23:0] col_md   = {lr,    lg,    8'h28};
    wire [23:0] col_lo   = {lr_lo, lg_lo, 8'h1C};
    wire [23:0] col_hat  = 24'hFFFFFF;      // 顶帽纯白
    wire [23:0] col_base = 24'h3A5A78;      // 基底线：中灰蓝，压在原图上不刺眼

    // 优先级：顶帽 > 柱身(三段) > 基线 > 原图
    wire [23:0] pix = in_hat  ? col_hat :
                      in_lit  ? (z_top ? col_hi : (z_low ? col_lo : col_md)) :
                      in_base ? col_base : rgb_in;

    // ------------------------------------------------------------------
    // 6. 输出打一拍
    // ------------------------------------------------------------------
    reg [23:0] rgb_out_r;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rgb_out_r <= 24'd0;
            de_out    <= 1'b0;
        end else begin
            de_out <= de;
            if (de)
                rgb_out_r <= pix;
        end
    end

    assign rgb_out = rgb_out_r;

endmodule
