// ============================================================================
// osd_overlay.v  —— OSD 字幕后叠加（纯自研模块）
// ----------------------------------------------------------------------------
// 在视频画面上叠五层内容，全部由 h_cnt / v_cnt 逐像素判断「这个点画不画」：
//   1. 底部滚动字幕条 —— 32 字符标语循环左滚，2 帧移 1 像素；
//   2. 右上角图片计数器 —— "当前/总数"，如 2/4；
//   3. 右上角切图耗时   —— 计数器正下方三行：总耗时 T、等待 W、加载 L，
//                          单位 0.01s，用来定位一次切图慢在哪一段；
//   4. 左上角参数显示   —— 两行：亮度 "L:+xx"、对比度 "C:x.xx"；
//   5. 转场提示         —— 画面正中的 2px 红框 + "转场提示"四个 16×16 汉字，
//                          只在 trans_active 期间出现，红框内部保持透明
//                          （能看见画面本身，只压一圈框）。当前顶层把
//                          trans_active 接了常数 0，所以这层实际不出现。
//
// 显隐控制（osd_on，由 KEY7 翻转）：
//   只控制第 3 层（T/W/L 三行）和第 4 层（L:/C: 两行）—— 即"读数类"信息。
//   第 1 层字幕条、第 2 层图片计数器始终显示，不受 osd_on 影响。
//   上电默认 osd_on=0（隐藏），画面干净；按键后才出现读数。
//
// 设计要点
//   · 不占帧缓冲：每层只算「当前扫描点到本层局部的相对坐标」，再去查字模
//     取这一位。省掉一整套读改写，代价是每层都要几个比较器 + 一个坐标减法。
//   · 字形靠 function 查表：ASCII 用 5×7 点阵（glyph_row），中文用 16×16
//     （cn_row）。放大 2 倍的做法是把字模左对齐成 8 位/16 位再按列取位，
//     于是字符内置的空白就天然变成了字间距，不用额外算间隔。
//   · 数值显示不用除法器：亮度/对比度转十进制全部用比较链和移位加，
//     避免安路 TD 的组合除法（又慢又不可靠）。
//
// 层间是「短路覆盖」而不是混合：最终叠加是一串 if / else if，命中哪层就用
// 哪层的颜色，所以层的书写顺序就是优先级 —— 字幕条 > 计数器 > 切图耗时 >
// 参数区 > 转场框 > 转场文字。改动顺序等于改动遮挡关系。
//
// 底框：2026-11-03 改为「只留字幕条、其余全透明」。
// 保留：底部滚动字幕条仍铺一层深灰底带（0x202020），保证白字在任何亮度的
//       画面上都能读清 —— 这一条是整幅宽、36 行高，必须靠底带才有"字幕条"观感。
// 取消：右上角计数面板底、右上角 T/W/L 面板底、左上角参数区底（0x303030）。
//       这三处只有字形像素会改写输出，其余像素透传 rgb_in，直接浮在画面上。
//
// 一个前提：所有坐标都以 H_ACTIVE/V_ACTIVE = 640×480 为基准写死，
// 换分辨率必须同步改这里的常量，否则各层的框会错位。
// ============================================================================

module osd_overlay #(
    parameter integer H_ACTIVE = 640,
    parameter integer V_ACTIVE = 480
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        de,
    input  wire        vs,
    input  wire [10:0] h_cnt,
    input  wire [10:0] v_cnt,

    input  wire [23:0] rgb_in,

    // 图片计数
    input  wire [2:0]  img_count,      // 已发现图片数
    input  wire [1:0]  img_index,      // 当前显示第几张（0-based）

    // 亮度/对比度数值
    input  wire [7:0]  bright,
    input  wire [9:0]  contrast_q8,

    // 切图耗时（单位 10ms，即 centisecond；0~999 对应显示 "x.xx" 秒）
    //   三行描述的都是**当前屏幕上这张图**（发布时刻挂在"本张上屏 + 转场结束"，
    //   不是"读卡写完"，否则三缓冲预读会让这张图配着下一张的数字）：
    //   sw_total_cs 请求 → 新图完全可见（请求 = 按键；自动轮播取开始读卡时刻）
    //   sw_wait_cs  请求 → 开始读卡（预读命中时恒为 0，帧早已就绪）
    //   sw_load_cs  开始读卡 → 整帧写完
    input  wire [9:0]  sw_wait_cs,
    input  wire [9:0]  sw_load_cs,
    input  wire [9:0]  sw_total_cs,

    // 转场进行中标志
    input  wire        trans_active,

    // 信息层显隐（KEY7 翻转，已在顶层同步到本域）
    //   1 = 显示左上角 "L:/C:" 与右上角 "T:/W:/L:" 四行读数
    //   0 = 隐藏这两块，画面只剩字幕条与右上角 "2/4" 计数器
    input  wire        osd_on,

    output wire [23:0] rgb_out,
    output reg         de_out
);

    // ==================== 字符渲染（标准 5x7 点阵） ====================
    // 每个字符 5 列 x 7 行，bit[4] 为最左列。
    // 渲染时把 5 位字模左对齐成 8 位（{glyph,3'b000}）再按列取位，
    // 这样 col 取 0~7 都不会越界，且天然得到 3 列（放大后 6 像素）字符间距。
    function [4:0] glyph_row;
        input [7:0] ch;      // ASCII 码
        input [2:0] row;     // 0~6
        reg [4:0] bits;
        begin
            bits = 5'b00000;
            case (ch)
                "A": case (row)
                    3'd0: bits = 5'b01110; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b10001; 3'd3: bits = 5'b11111;
                    3'd4: bits = 5'b10001; 3'd5: bits = 5'b10001;
                    3'd6: bits = 5'b10001;
                    default: bits = 5'b00000;
                endcase
                "B": case (row)
                    3'd0: bits = 5'b11110; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b11110; 3'd3: bits = 5'b10001;
                    3'd4: bits = 5'b10001; 3'd5: bits = 5'b10001;
                    3'd6: bits = 5'b11110;
                    default: bits = 5'b00000;
                endcase
                "C": case (row)
                    3'd0: bits = 5'b01110; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b10000; 3'd3: bits = 5'b10000;
                    3'd4: bits = 5'b10000; 3'd5: bits = 5'b10001;
                    3'd6: bits = 5'b01110;
                    default: bits = 5'b00000;
                endcase
                "D": case (row)
                    3'd0: bits = 5'b11110; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b10001; 3'd3: bits = 5'b10001;
                    3'd4: bits = 5'b10001; 3'd5: bits = 5'b10001;
                    3'd6: bits = 5'b11110;
                    default: bits = 5'b00000;
                endcase
                "E": case (row)
                    3'd0: bits = 5'b11111; 3'd1: bits = 5'b10000;
                    3'd2: bits = 5'b11110; 3'd3: bits = 5'b10000;
                    3'd4: bits = 5'b10000; 3'd5: bits = 5'b10000;
                    3'd6: bits = 5'b11111;
                    default: bits = 5'b00000;
                endcase
                "F": case (row)
                    3'd0: bits = 5'b11111; 3'd1: bits = 5'b10000;
                    3'd2: bits = 5'b11110; 3'd3: bits = 5'b10000;
                    3'd4: bits = 5'b10000; 3'd5: bits = 5'b10000;
                    3'd6: bits = 5'b10000;
                    default: bits = 5'b00000;
                endcase
                "G": case (row)
                    3'd0: bits = 5'b01110; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b10000; 3'd3: bits = 5'b10111;
                    3'd4: bits = 5'b10001; 3'd5: bits = 5'b10001;
                    3'd6: bits = 5'b01111;
                    default: bits = 5'b00000;
                endcase
                "H": case (row)
                    3'd0: bits = 5'b10001; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b11111; 3'd3: bits = 5'b10001;
                    3'd4: bits = 5'b10001; 3'd5: bits = 5'b10001;
                    3'd6: bits = 5'b10001;
                    default: bits = 5'b00000;
                endcase
                "I": case (row)
                    3'd0: bits = 5'b11111; 3'd1: bits = 5'b00100;
                    3'd2: bits = 5'b00100; 3'd3: bits = 5'b00100;
                    3'd4: bits = 5'b00100; 3'd5: bits = 5'b00100;
                    3'd6: bits = 5'b11111;
                    default: bits = 5'b00000;
                endcase
                "J": case (row)
                    3'd0: bits = 5'b00111; 3'd1: bits = 5'b00010;
                    3'd2: bits = 5'b00010; 3'd3: bits = 5'b00010;
                    3'd4: bits = 5'b00010; 3'd5: bits = 5'b10010;
                    3'd6: bits = 5'b01100;
                    default: bits = 5'b00000;
                endcase
                "K": case (row)
                    3'd0: bits = 5'b10001; 3'd1: bits = 5'b10010;
                    3'd2: bits = 5'b10100; 3'd3: bits = 5'b11000;
                    3'd4: bits = 5'b10100; 3'd5: bits = 5'b10010;
                    3'd6: bits = 5'b10001;
                    default: bits = 5'b00000;
                endcase
                "L": case (row)
                    3'd0: bits = 5'b10000; 3'd1: bits = 5'b10000;
                    3'd2: bits = 5'b10000; 3'd3: bits = 5'b10000;
                    3'd4: bits = 5'b10000; 3'd5: bits = 5'b10000;
                    3'd6: bits = 5'b11111;
                    default: bits = 5'b00000;
                endcase
                "M": case (row)
                    3'd0: bits = 5'b10001; 3'd1: bits = 5'b11011;
                    3'd2: bits = 5'b10101; 3'd3: bits = 5'b10101;
                    3'd4: bits = 5'b10001; 3'd5: bits = 5'b10001;
                    3'd6: bits = 5'b10001;
                    default: bits = 5'b00000;
                endcase
                "N": case (row)
                    3'd0: bits = 5'b10001; 3'd1: bits = 5'b11001;
                    3'd2: bits = 5'b10101; 3'd3: bits = 5'b10011;
                    3'd4: bits = 5'b10001; 3'd5: bits = 5'b10001;
                    3'd6: bits = 5'b10001;
                    default: bits = 5'b00000;
                endcase
                "O": case (row)
                    3'd0: bits = 5'b01110; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b10001; 3'd3: bits = 5'b10001;
                    3'd4: bits = 5'b10001; 3'd5: bits = 5'b10001;
                    3'd6: bits = 5'b01110;
                    default: bits = 5'b00000;
                endcase
                "P": case (row)
                    3'd0: bits = 5'b11110; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b10001; 3'd3: bits = 5'b11110;
                    3'd4: bits = 5'b10000; 3'd5: bits = 5'b10000;
                    3'd6: bits = 5'b10000;
                    default: bits = 5'b00000;
                endcase
                "Q": case (row)
                    3'd0: bits = 5'b01110; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b10001; 3'd3: bits = 5'b10001;
                    3'd4: bits = 5'b10101; 3'd5: bits = 5'b10010;
                    3'd6: bits = 5'b01101;
                    default: bits = 5'b00000;
                endcase
                "R": case (row)
                    3'd0: bits = 5'b11110; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b10001; 3'd3: bits = 5'b11110;
                    3'd4: bits = 5'b10100; 3'd5: bits = 5'b10010;
                    3'd6: bits = 5'b10001;
                    default: bits = 5'b00000;
                endcase
                "S": case (row)
                    3'd0: bits = 5'b01111; 3'd1: bits = 5'b10000;
                    3'd2: bits = 5'b10000; 3'd3: bits = 5'b01110;
                    3'd4: bits = 5'b00001; 3'd5: bits = 5'b00001;
                    3'd6: bits = 5'b11110;
                    default: bits = 5'b00000;
                endcase
                "T": case (row)
                    3'd0: bits = 5'b11111; 3'd1: bits = 5'b00100;
                    3'd2: bits = 5'b00100; 3'd3: bits = 5'b00100;
                    3'd4: bits = 5'b00100; 3'd5: bits = 5'b00100;
                    3'd6: bits = 5'b00100;
                    default: bits = 5'b00000;
                endcase
                "U": case (row)
                    3'd0: bits = 5'b10001; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b10001; 3'd3: bits = 5'b10001;
                    3'd4: bits = 5'b10001; 3'd5: bits = 5'b10001;
                    3'd6: bits = 5'b01110;
                    default: bits = 5'b00000;
                endcase
                "V": case (row)
                    3'd0: bits = 5'b10001; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b10001; 3'd3: bits = 5'b10001;
                    3'd4: bits = 5'b10001; 3'd5: bits = 5'b01010;
                    3'd6: bits = 5'b00100;
                    default: bits = 5'b00000;
                endcase
                "W": case (row)
                    3'd0: bits = 5'b10001; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b10001; 3'd3: bits = 5'b10101;
                    3'd4: bits = 5'b10101; 3'd5: bits = 5'b11011;
                    3'd6: bits = 5'b10001;
                    default: bits = 5'b00000;
                endcase
                "X": case (row)
                    3'd0: bits = 5'b10001; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b01010; 3'd3: bits = 5'b00100;
                    3'd4: bits = 5'b01010; 3'd5: bits = 5'b10001;
                    3'd6: bits = 5'b10001;
                    default: bits = 5'b00000;
                endcase
                "Y": case (row)
                    3'd0: bits = 5'b10001; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b01010; 3'd3: bits = 5'b00100;
                    3'd4: bits = 5'b00100; 3'd5: bits = 5'b00100;
                    3'd6: bits = 5'b00100;
                    default: bits = 5'b00000;
                endcase
                "Z": case (row)
                    3'd0: bits = 5'b11111; 3'd1: bits = 5'b00001;
                    3'd2: bits = 5'b00010; 3'd3: bits = 5'b00100;
                    3'd4: bits = 5'b01000; 3'd5: bits = 5'b10000;
                    3'd6: bits = 5'b11111;
                    default: bits = 5'b00000;
                endcase
                "0": case (row)
                    3'd0: bits = 5'b01110; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b10011; 3'd3: bits = 5'b10101;
                    3'd4: bits = 5'b11001; 3'd5: bits = 5'b10001;
                    3'd6: bits = 5'b01110;
                    default: bits = 5'b00000;
                endcase
                "1": case (row)
                    3'd0: bits = 5'b00100; 3'd1: bits = 5'b01100;
                    3'd2: bits = 5'b00100; 3'd3: bits = 5'b00100;
                    3'd4: bits = 5'b00100; 3'd5: bits = 5'b00100;
                    3'd6: bits = 5'b01110;
                    default: bits = 5'b00000;
                endcase
                "2": case (row)
                    3'd0: bits = 5'b01110; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b00001; 3'd3: bits = 5'b00010;
                    3'd4: bits = 5'b00100; 3'd5: bits = 5'b01000;
                    3'd6: bits = 5'b11111;
                    default: bits = 5'b00000;
                endcase
                "3": case (row)
                    3'd0: bits = 5'b11111; 3'd1: bits = 5'b00010;
                    3'd2: bits = 5'b00100; 3'd3: bits = 5'b00010;
                    3'd4: bits = 5'b00001; 3'd5: bits = 5'b10001;
                    3'd6: bits = 5'b01110;
                    default: bits = 5'b00000;
                endcase
                "4": case (row)
                    3'd0: bits = 5'b00010; 3'd1: bits = 5'b00110;
                    3'd2: bits = 5'b01010; 3'd3: bits = 5'b10010;
                    3'd4: bits = 5'b11111; 3'd5: bits = 5'b00010;
                    3'd6: bits = 5'b00010;
                    default: bits = 5'b00000;
                endcase
                "5": case (row)
                    3'd0: bits = 5'b11111; 3'd1: bits = 5'b10000;
                    3'd2: bits = 5'b11110; 3'd3: bits = 5'b00001;
                    3'd4: bits = 5'b00001; 3'd5: bits = 5'b10001;
                    3'd6: bits = 5'b01110;
                    default: bits = 5'b00000;
                endcase
                "6": case (row)
                    3'd0: bits = 5'b00110; 3'd1: bits = 5'b01000;
                    3'd2: bits = 5'b10000; 3'd3: bits = 5'b11110;
                    3'd4: bits = 5'b10001; 3'd5: bits = 5'b10001;
                    3'd6: bits = 5'b01110;
                    default: bits = 5'b00000;
                endcase
                "7": case (row)
                    3'd0: bits = 5'b11111; 3'd1: bits = 5'b00001;
                    3'd2: bits = 5'b00010; 3'd3: bits = 5'b00100;
                    3'd4: bits = 5'b01000; 3'd5: bits = 5'b01000;
                    3'd6: bits = 5'b01000;
                    default: bits = 5'b00000;
                endcase
                "8": case (row)
                    3'd0: bits = 5'b01110; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b10001; 3'd3: bits = 5'b01110;
                    3'd4: bits = 5'b10001; 3'd5: bits = 5'b10001;
                    3'd6: bits = 5'b01110;
                    default: bits = 5'b00000;
                endcase
                "9": case (row)
                    3'd0: bits = 5'b01110; 3'd1: bits = 5'b10001;
                    3'd2: bits = 5'b10001; 3'd3: bits = 5'b01111;
                    3'd4: bits = 5'b00001; 3'd5: bits = 5'b00010;
                    3'd6: bits = 5'b01100;
                    default: bits = 5'b00000;
                endcase
                " ": case (row)
                    3'd0: bits = 5'b00000; 3'd1: bits = 5'b00000;
                    3'd2: bits = 5'b00000; 3'd3: bits = 5'b00000;
                    3'd4: bits = 5'b00000; 3'd5: bits = 5'b00000;
                    3'd6: bits = 5'b00000;
                    default: bits = 5'b00000;
                endcase
                ".": case (row)
                    3'd0: bits = 5'b00000; 3'd1: bits = 5'b00000;
                    3'd2: bits = 5'b00000; 3'd3: bits = 5'b00000;
                    3'd4: bits = 5'b00000; 3'd5: bits = 5'b01100;
                    3'd6: bits = 5'b01100;
                    default: bits = 5'b00000;
                endcase
                ":": case (row)
                    3'd0: bits = 5'b00000; 3'd1: bits = 5'b01100;
                    3'd2: bits = 5'b01100; 3'd3: bits = 5'b00000;
                    3'd4: bits = 5'b01100; 3'd5: bits = 5'b01100;
                    3'd6: bits = 5'b00000;
                    default: bits = 5'b00000;
                endcase
                "-": case (row)
                    3'd0: bits = 5'b00000; 3'd1: bits = 5'b00000;
                    3'd2: bits = 5'b00000; 3'd3: bits = 5'b11111;
                    3'd4: bits = 5'b00000; 3'd5: bits = 5'b00000;
                    3'd6: bits = 5'b00000;
                    default: bits = 5'b00000;
                endcase
                "/": case (row)
                    3'd0: bits = 5'b00001; 3'd1: bits = 5'b00010;
                    3'd2: bits = 5'b00010; 3'd3: bits = 5'b00100;
                    3'd4: bits = 5'b01000; 3'd5: bits = 5'b01000;
                    3'd6: bits = 5'b10000;
                    default: bits = 5'b00000;
                endcase
                default: bits = 5'b00000;
            endcase
            glyph_row = bits;
        end
    endfunction

    // ==================== 滚动字幕参数 ====================
    // 滚动字幕：固定字符串，逐列左移。
    // 字符数取 64（2 的幂），"对 64 取模"直接就是截取低 6 位，无需减法器；
    // 同时 64 x 16 = 1024 像素 > 屏幕宽 640，右侧永远不会露出空白，
    // 滚动时字幕始终铺满整行。标语本身 32 字符，末尾 4 个空格，
    // 由 scroll_char 只取低 5 位实现自动重复两遍。
    localparam integer SCROLL_CHARS = 64;
    // 字符串索引到 ASCII：用 function 返回第 n 个字符
    function [7:0] scroll_char;
        input [5:0] n;
        reg [7:0] c;
        begin
            case (n[4:0])
                5'd0:  c = "A"; 5'd1:  c = "N"; 5'd2:  c = "L"; 5'd3:  c = "O";
                5'd4:  c = "G"; 5'd5:  c = "I"; 5'd6:  c = "C"; 5'd7:  c = "-";
                5'd8:  c = "H"; 5'd9:  c = "D"; 5'd10: c = "M"; 5'd11: c = "I";
                5'd12: c = " "; 5'd13: c = " "; 5'd14: c = " "; 5'd15: c = "2";
                5'd16: c = "0"; 5'd17: c = "2"; 5'd18: c = "6"; 5'd19: c = " ";
                5'd20: c = "E"; 5'd21: c = "G"; 5'd22: c = "4"; 5'd23: c = "S";
                5'd24: c = "2"; 5'd25: c = "0"; 5'd26: c = "B"; 5'd27: c = "G";
                5'd28: c = "2"; 5'd29: c = "5"; 5'd30: c = "6"; 5'd31: c = " ";
                default: c = " ";
            endcase
            scroll_char = c;
        end
    endfunction

    // ---- 滚动偏移：每 2 帧左移 1 像素，一个滚动周期 = SCROLL_CHARS * 16 ----
    // 字形放大 2 倍后字符格宽 16 像素（5 列字模 x2 = 10 列 + 6 列间隔）
    reg [11:0] scroll_off;
    reg        vs_d1;
    reg        scroll_div;       // 分频：每 2 帧滚 1 像素
    wire       vs_rise;
    localparam integer SCROLL_PIXELS = SCROLL_CHARS * 16;

    assign vs_rise = vs & ~vs_d1;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            vs_d1      <= 1'b0;
            scroll_off <= 12'd0;
            scroll_div <= 1'b0;
        end else begin
            vs_d1 <= vs;
            if (vs_rise) begin
                scroll_div <= ~scroll_div;
                // 每 2 帧（scroll_div 翻转）才滚动 1 像素
                if (scroll_div)
                    scroll_off <= (scroll_off >= SCROLL_PIXELS - 1) ? 12'd0 : scroll_off + 12'd1;
            end
        end
    end

    // ==================== 字幕条坐标 ====================
    // 底部字幕条：y 从 V_ACTIVE-40 到 V_ACTIVE-4，高 36 行；
    // 其中真正画字形的只有第 11~24 行（14 行 = 7 行字模 ×2 倍放大），
    // 上下各留一段边距，看起来才有「一条带子」的观感，而不是贴死画面底边。
    localparam integer BAND_TOP    = V_ACTIVE - 40;
    localparam integer BAND_BOTTOM = V_ACTIVE - 4;
    localparam integer CHAR_H      = 8;

    wire in_band = (v_cnt >= BAND_TOP) && (v_cnt < BAND_BOTTOM);

    // ---- 字幕条内：计算当前列对应哪个字符 ----
    // 全局列 = h_cnt + scroll_off；字符索引 = 全局列 / 16；字符内列 = 全局列 % 16
    // 字模 5 列按 2 倍放大占 10 列，其余列为字符间隔。
    // 字符数固定 64，取模 64 等价于取低 6 位，无需减法器。
    wire [11:0] g_col   = {1'b0, h_cnt} + {1'b0, scroll_off};
    wire [6:0]  ch_idx_full = g_col[10:4];        // /16，范围 0~103
    wire [5:0]  ch_idx = ch_idx_full[5:0];        // 对 64 取模 → 0~63
    wire [3:0]  ch_col_raw = g_col[3:0];          // 字符内列 0~15
    wire [2:0]  ch_col = ch_col_raw[3:1];         // 字模列 0~7（2 倍放大）
    wire [10:0] band_row = v_cnt - BAND_TOP[10:0]; // 字幕条内行 0~35
    // 字形 7 行放大 2 倍 = 14 行，在 36 行高的字幕带内居中（行 11~24）
    wire        band_glyph_en = (band_row >= 11'd11) && (band_row < 11'd25);
    wire [10:0] band_glyph_off = band_row - 11'd11;
    wire [2:0]  ch_row = band_glyph_off[3:1];     // /2，字形内行号 0~6

    wire [7:0]  cur_ch = scroll_char(ch_idx);
    wire [4:0]  cur_glyph = glyph_row(cur_ch, ch_row);
    // 5 位字模左对齐成 8 位：bit[7] 是字形最左列，col 0~7 取位全部安全，
    // col=5~7 落在补的 0 上，自然形成字符间距（放大 2 倍即 6 像素）。
    wire [7:0]  band_gl = {cur_glyph, 3'b000};
    wire        char_pixel_on = band_glyph_en && band_gl[3'd7 - ch_col];

    // 字幕条保留底带（2026-11-03 按实际观感确认：保留这一处，其余三处改透明）。
    // 整条 36 行高的带子统一被改写，字形像素取白、其余取深灰，
    // 好处是白字在任何亮度的画面上都能读清。
    wire [23:0] band_pixel = char_pixel_on ? 24'hFFFFFF : 24'h202020;

    // ==================== 右上角图片计数器 ====================
    // 显示 "当前/总数"（如 2/4），位置：右上角 x 520~571, y 10~42
    // （3 个字符格 x 16 像素 = 48 像素，右边留 4 像素边距）
    wire in_counter = (h_cnt >= 11'd520) && (h_cnt < 11'd572) &&
                      (v_cnt >= 11'd10)  && (v_cnt < 11'd42);
    wire [10:0] cnt_row_full = v_cnt - 11'd10;   // 局部行 0~31（完整位宽）
    // 字形 7 行放大 2 倍 = 14 行，在 32 行高的计数区内居中（行 9~22）
    wire        cnt_glyph_en = (cnt_row_full >= 11'd9) && (cnt_row_full < 11'd23);
    wire [10:0] cnt_glyph_off = cnt_row_full - 11'd9;
    wire [2:0]  cnt_row = cnt_glyph_off[3:1];    // /2，字形内行号 0~6
    // 字符布局：数字(列0-15) "/"(列16-31) 数字(列32-47)
    // 每个字符 16 像素宽（用 8 像素字形 + 空 8 像素，字号翻倍感）
    wire [11:0] cx = h_cnt - 11'd520;
    reg  [7:0]  cnt_char_sel;
    wire [3:0]  cnt_num = {2'b0, img_index} + 4'd1;   // 当前编号 1~4（3bit，防溢出）

    always @(*) begin
        if (cx < 12'd16)        cnt_char_sel = 8'd48 + cnt_num;   // '0'+当前编号
        else if (cx < 12'd32)   cnt_char_sel = "/";
        else if (cx < 12'd48)   cnt_char_sel = 8'd48 + img_count; // '0'+总数
        else                    cnt_char_sel = " ";
    end

    wire [4:0] cnt_glyph = glyph_row(cnt_char_sel, cnt_row);
    // 字符格 16 像素宽：5 位字模放大 2 倍占 10 列，其余为间隔
    wire [3:0] cnt_col_raw = cx[3:0];
    wire [2:0] cnt_col = cnt_col_raw[3:1];
    wire [7:0] cnt_gl = {cnt_glyph, 3'b000};
    // 必须用 cnt_glyph_en 把字形限制在 14 行的窗口内，
    // 否则行偏移会回绕，字形被画到窗口外（表现为纵向"溢出"的绿色条）
    wire       cnt_pixel_on = cnt_glyph_en && (cnt_char_sel != " ") && cnt_gl[3'd7 - cnt_col];

    // ==================== 右上角切图耗时 ====================
    // 紧贴图片计数器下方，同宽字号、同右边缘（x=572），视觉上属于同一块面板：
    //   T:x.xx   总耗时 —— 按键受理 → 新图完全可见，用户感知的就是这个数
    //   W:x.xx   等待   —— 按键受理 → 开始读卡，被"转场忙 + 600ms 静默窗"挡住
    //   L:x.xx   加载   —— 开始读卡 → 整帧写完，跟 SD 卡质量/簇碎片化有关
    // 拆三行是为了定位慢在哪一段：W 大是调度问题，L 大才是读卡问题。
    // 三行都 6 个字符格 x 16 像素 = 96 宽，每行高 32 像素。
    localparam [10:0] TM_X0 = 11'd476;
    localparam [10:0] TM_Y0 = 11'd46;    // 计数器底部在 y=42，留 4 像素间距
    localparam [10:0] TM_W  = 11'd96;    // 6 字符 x 16
    localparam [10:0] TM_H  = 11'd96;    // 3 行 x 32

    // osd_on 门控放在区域判定里，只影响最终叠加是否改写像素；
    // 下面的 tm_dy / tm_val 等坐标运算照常进行（纯组合、无副作用）。
    wire in_time = osd_on &&
                   (h_cnt >= TM_X0) && (h_cnt < TM_X0 + TM_W) &&
                   (v_cnt >= TM_Y0) && (v_cnt < TM_Y0 + TM_H);
    wire [10:0] tm_dy   = v_cnt - TM_Y0;       // 0~95
    wire [1:0]  tm_rowi = tm_dy[6:5];          // 第几行 0~2
    wire [10:0] tm_lrow = {6'd0, tm_dy[4:0]};  // 行内 0~31
    // 与计数器一致：字形 7 行放大 2 倍 = 14 行，在 32 行内居中（行 9~22）
    wire        tm_glyph_en = (tm_lrow >= 11'd9) && (tm_lrow < 11'd23);
    wire [2:0]  tm_row   = (tm_lrow - 11'd9) >> 1;   // /2，字形内行 0~6

    wire [11:0] tmx   = h_cnt - TM_X0;         // 0~95
    wire [3:0]  tm_ci = tmx[7:4];              // 第几个字符 0~5
    wire [3:0]  tm_cx = tmx[3:0];              // 字符内列 0~15

    // 按行选数：只有被选中那一行的值参与后面的十进制分解
    reg [9:0] tm_val;
    always @(*) begin
        case (tm_rowi)
            2'd0:    tm_val = sw_total_cs;
            2'd1:    tm_val = sw_wait_cs;
            default: tm_val = sw_load_cs;
        endcase
    end

    // ---- 0~999 拆成三位十进制 ----
    // 沿用本文件既有的"比较链"写法（见下面亮度/对比度），不用除法器：
    // TD 的组合除法器又大又慢，几个比较器铺开反而更省更稳。
    wire [9:0] tm_v = (tm_val > 10'd999) ? 10'd999 : tm_val;

    wire [3:0] tm_d100 = (tm_v >= 10'd900) ? 4'd9 :
                         (tm_v >= 10'd800) ? 4'd8 :
                         (tm_v >= 10'd700) ? 4'd7 :
                         (tm_v >= 10'd600) ? 4'd6 :
                         (tm_v >= 10'd500) ? 4'd5 :
                         (tm_v >= 10'd400) ? 4'd4 :
                         (tm_v >= 10'd300) ? 4'd3 :
                         (tm_v >= 10'd200) ? 4'd2 :
                         (tm_v >= 10'd100) ? 4'd1 : 4'd0;
    // 百位*100 = *64 + *32 + *4，移位相加，不占乘法器
    wire [9:0] tm_r100 = tm_v - ({tm_d100, 6'b0} + {tm_d100, 5'b0} + {tm_d100, 2'b0});

    wire [3:0] tm_d10 = (tm_r100 >= 10'd90) ? 4'd9 :
                        (tm_r100 >= 10'd80) ? 4'd8 :
                        (tm_r100 >= 10'd70) ? 4'd7 :
                        (tm_r100 >= 10'd60) ? 4'd6 :
                        (tm_r100 >= 10'd50) ? 4'd5 :
                        (tm_r100 >= 10'd40) ? 4'd4 :
                        (tm_r100 >= 10'd30) ? 4'd3 :
                        (tm_r100 >= 10'd20) ? 4'd2 :
                        (tm_r100 >= 10'd10) ? 4'd1 : 4'd0;
    // 十位*10 = *8 + *2
    wire [9:0] tm_r10 = tm_r100 - ({tm_d10, 3'b0} + {tm_d10, 1'b0});
    wire [3:0] tm_d1  = tm_r10[3:0];

    // ---- 字符选择：T:/W:/L: + 三位数字，小数点固定在字符 3 ----
    wire [7:0] tm_head = (tm_rowi == 2'd0) ? "T" :
                         (tm_rowi == 2'd1) ? "W" : "L";
    reg  [7:0] tm_char;
    always @(*) begin
        case (tm_ci)
            4'd0:    tm_char = tm_head;
            4'd1:    tm_char = ":";
            4'd2:    tm_char = 8'd48 + {4'd0, tm_d100};
            4'd3:    tm_char = ".";
            4'd4:    tm_char = 8'd48 + {4'd0, tm_d10};
            default: tm_char = 8'd48 + {4'd0, tm_d1};
        endcase
    end

    wire [4:0] tm_glyph = glyph_row(tm_char, tm_row);
    // 字符格 16 像素宽：5 位字模放大 2 倍占 10 列，其余为间隔（同计数器）
    wire [2:0] tm_col = tm_cx[3:1];
    wire [7:0] tm_gl   = {tm_glyph, 3'b000};
    wire       tm_pixel_on = tm_glyph_en && tm_gl[3'd7 - tm_col];

    // ==================== 左上角参数显示 ====================
    // 显示两行：亮度 "L:+xx" 与对比度 "C:1.00"（定点，1.00 = 1.0x）
    // 字符 8x8 点阵（char_h=8, char_w=8），底框已改为透明（2026-11-03），
    // 文字直接叠在画面上。
    // 数值全部用查找表（LUT）替代组合除法/取模，保证 TD 可综合且时序干净。

    // 显示区坐标：x 8~107，y 8~39（两行，每行 16 像素）
    // 字符格 16 像素：对比度行 6 个字符占 96 列，PM_W 取 100 留 4 像素右边距
    localparam [10:0] PM_X0 = 11'd8;
    localparam [10:0] PM_Y0 = 11'd8;
    localparam [10:0] PM_W  = 11'd100;   // 显示区宽：6 字符 x 16 = 96 + 4 像素右边距
    localparam [10:0] PM_H  = 11'd32;    // 显示区高（两行）

    // 同 in_time：osd_on 门控只挡显示，不挡数值计算。
    wire in_param = osd_on &&
                    (h_cnt >= PM_X0) && (h_cnt < PM_X0 + PM_W) &&
                    (v_cnt >= PM_Y0) && (v_cnt < PM_Y0 + PM_H);
    wire [10:0] pm_row_full = v_cnt - PM_Y0;   // 0~31
    // 参数区高 32 像素，分两行，每行 16 像素。字形 7 行放大 2 倍 = 14 行，
    // 每行 16 行内居中显示（行 1~14）。
    wire        pm_row_upper = (pm_row_full < 11'd16);
    wire [10:0] pm_line_row  = pm_row_upper ? pm_row_full : (pm_row_full - 11'd16); // 每行内 0~15
    wire        pm_glyph_en  = (pm_line_row >= 11'd1) && (pm_line_row < 11'd15);
    wire [10:0] pm_glyph_off = pm_line_row - 11'd1;
    wire [2:0]  pm_row       = pm_glyph_off[3:1]; // /2，字形内行号 0~6
    wire [11:0] px          = h_cnt - PM_X0;    // 局部列 0~99（PM_W 范围内）

    // ---- 亮度数值转十进制（比较链，不用除法器）----
    // bright 是 8bit 有符号（补码），显示绝对值两位 + 符号。
    // 步进改成 4 之后实际范围是 -96~+96，两位十进制一定放得下；
    // 这里再饱和到 99，防止异常值把十位算错。
    wire        bright_neg = bright[7];
    wire [7:0]  bright_abs = bright_neg ? (~bright + 8'd1) : bright;
    wire [7:0]  bright_cap = (bright_abs > 8'd99) ? 8'd99 : bright_abs;

    wire [3:0]  b_tens_v = (bright_cap >= 8'd90) ? 4'd9 :
                           (bright_cap >= 8'd80) ? 4'd8 :
                           (bright_cap >= 8'd70) ? 4'd7 :
                           (bright_cap >= 8'd60) ? 4'd6 :
                           (bright_cap >= 8'd50) ? 4'd5 :
                           (bright_cap >= 8'd40) ? 4'd4 :
                           (bright_cap >= 8'd30) ? 4'd3 :
                           (bright_cap >= 8'd20) ? 4'd2 :
                           (bright_cap >= 8'd10) ? 4'd1 : 4'd0;
    // 十位*10 = 十位*8 + 十位*2（移位相加，不占用乘法器）
    wire [7:0]  b_ten_val = {1'b0, b_tens_v, 3'b000} + {2'b0, b_tens_v, 1'b0};
    wire [7:0]  b_ones_v  = bright_cap - b_ten_val;
    wire [7:0]  b_tens    = 8'h30 + {4'd0, b_tens_v};
    wire [7:0]  b_ones    = 8'h30 + {4'd0, b_ones_v[3:0]};

    // ---- 对比度数值（比较链，不用除法器）----
    // contrast_q8/256 就是倍数，显示成 "x.xx"：
    //   disp100 = round(contrast_q8 * 100 / 256) = (contrast_q8*25 + 32) >> 6
    // contrast_q8 范围 128~512 → disp100 落在 50~200，14bit 足够。
    wire [13:0] c_x100   = {4'd0, contrast_q8} * 14'd25;
    wire [13:0] c_rnd    = c_x100 + 14'd32;        // 四舍五入
    wire [7:0]  c_disp   = c_rnd[13:6];            // /64 → 0~200
    wire [7:0]  c_int_v  = (c_disp >= 8'd200) ? 8'd2 :
                           (c_disp >= 8'd100) ? 8'd1 : 8'd0;
    wire [7:0]  c_base   = (c_int_v == 8'd2) ? 8'd200 :
                           (c_int_v == 8'd1) ? 8'd100 : 8'd0;
    wire [7:0]  c_frac   = c_disp - c_base;        // 0~99
    wire [3:0]  c_tens_v = (c_frac >= 8'd90) ? 4'd9 :
                           (c_frac >= 8'd80) ? 4'd8 :
                           (c_frac >= 8'd70) ? 4'd7 :
                           (c_frac >= 8'd60) ? 4'd6 :
                           (c_frac >= 8'd50) ? 4'd5 :
                           (c_frac >= 8'd40) ? 4'd4 :
                           (c_frac >= 8'd30) ? 4'd3 :
                           (c_frac >= 8'd20) ? 4'd2 :
                           (c_frac >= 8'd10) ? 4'd1 : 4'd0;
    wire [7:0]  c_ten_val = {1'b0, c_tens_v, 3'b000} + {2'b0, c_tens_v, 1'b0};
    wire [7:0]  c_ones_v  = c_frac - c_ten_val;
    wire [7:0]  c_int     = 8'h30 + c_int_v;
    wire [7:0]  c_frac0   = 8'h30 + {4'd0, c_tens_v};
    wire [7:0]  c_frac1   = 8'h30 + {4'd0, c_ones_v[3:0]};

    // ---- 逐字符选择（两行）----
    // 字符格 16 像素宽：字符索引 = px/16 = px[7:4]（px 只有 0~99，索引 0~5 有效）。
    // 名字带 pm_ 前缀是为了和上方滚动字幕的 ch_idx 区分，避免同一 module 内
    // 重名导致重复声明。
    wire [3:0] pm_ch_idx = px[7:4];
    reg [7:0] pm_char_sel;
    always @(*) begin
        if (pm_row_full < 11'd16) begin
            // 第一行：亮度  "L:±XX"
            case (pm_ch_idx)
                4'd0: pm_char_sel = "L";
                4'd1: pm_char_sel = ":";
                4'd2: pm_char_sel = bright_neg ? "-" : " ";
                4'd3: pm_char_sel = b_tens;
                4'd4: pm_char_sel = b_ones;
                default: pm_char_sel = " ";
            endcase
        end else begin
            // 第二行：对比度 "C:x.xx"
            case (pm_ch_idx)
                4'd0: pm_char_sel = "C";
                4'd1: pm_char_sel = ":";
                4'd2: pm_char_sel = c_int;
                4'd3: pm_char_sel = ".";
                4'd4: pm_char_sel = c_frac0;
                4'd5: pm_char_sel = c_frac1;
                default: pm_char_sel = " ";
            endcase
        end
    end

    wire [4:0] pm_glyph = glyph_row(pm_char_sel, pm_row);
    wire [3:0] pm_col_raw = px[3:0];
    wire [2:0] pm_col = pm_col_raw[3:1];
    wire [7:0] pm_gl = {pm_glyph, 3'b000};
    wire       pm_pixel_on = pm_glyph_en && (pm_char_sel != " ") && pm_gl[3'd7 - pm_col];
    // 原先这里由区域命中的整体判断铺一层深灰底框，2026-11-03 改为透明，
    // 显示区内非字形像素不再改写，透出底层画面。

    // ==================== 转场提示：透明边框 + 红色中文"转场提示" ====================
    // 16×16 点阵（宋体），4 个字 = 64×16 像素，画面正中：
    //   文字 x=288..351, y=232..247
    //   边框 x=280..359, y=224..255（2px 红框，框内不填充 → 透明）
    function [15:0] cn_row;
        input [1:0] ch;      // 0=转 1=场 2=提 3=示
        input [3:0] row;     // 0..15
        begin
            case ({ch, row})
                // 转
                {2'd0,4'd0}: cn_row = 16'h2020;  {2'd0,4'd1}:  cn_row = 16'h2020;
                {2'd0,4'd2}: cn_row = 16'h2020;  {2'd0,4'd3}:  cn_row = 16'hFDFC;
                {2'd0,4'd4}: cn_row = 16'h4020;  {2'd0,4'd5}:  cn_row = 16'h5040;
                {2'd0,4'd6}: cn_row = 16'h93FE;  {2'd0,4'd7}:  cn_row = 16'hFC40;
                {2'd0,4'd8}: cn_row = 16'h1080;  {2'd0,4'd9}:  cn_row = 16'h11FC;
                {2'd0,4'd10}: cn_row = 16'h1C04; {2'd0,4'd11}: cn_row = 16'hF088;
                {2'd0,4'd12}: cn_row = 16'h5050; {2'd0,4'd13}: cn_row = 16'h1020;
                {2'd0,4'd14}: cn_row = 16'h1010; {2'd0,4'd15}: cn_row = 16'h1010;
                // 场
                {2'd1,4'd0}: cn_row = 16'h2000;  {2'd1,4'd1}:  cn_row = 16'h23F0;
                {2'd1,4'd2}: cn_row = 16'h2020;  {2'd1,4'd3}:  cn_row = 16'h2040;
                {2'd1,4'd4}: cn_row = 16'h2080;  {2'd1,4'd5}:  cn_row = 16'hF900;
                {2'd1,4'd6}: cn_row = 16'h23FC;  {2'd1,4'd7}:  cn_row = 16'h2124;
                {2'd1,4'd8}: cn_row = 16'h2124;  {2'd1,4'd9}:  cn_row = 16'h2124;
                {2'd1,4'd10}: cn_row = 16'h2224; {2'd1,4'd11}: cn_row = 16'h3A44;
                {2'd1,4'd12}: cn_row = 16'hE444; {2'd1,4'd13}: cn_row = 16'h4884;
                {2'd1,4'd14}: cn_row = 16'h0128; {2'd1,4'd15}: cn_row = 16'h0210;
                // 提
                {2'd2,4'd0}: cn_row = 16'h1000;  {2'd2,4'd1}:  cn_row = 16'h11FC;
                {2'd2,4'd2}: cn_row = 16'h1104;  {2'd2,4'd3}:  cn_row = 16'h1104;
                {2'd2,4'd4}: cn_row = 16'hFDFC;  {2'd2,4'd5}:  cn_row = 16'h1104;
                {2'd2,4'd6}: cn_row = 16'h1104;  {2'd2,4'd7}:  cn_row = 16'h15FC;
                {2'd2,4'd8}: cn_row = 16'h1800;  {2'd2,4'd9}:  cn_row = 16'h33FE;
                {2'd2,4'd10}: cn_row = 16'hD020; {2'd2,4'd11}: cn_row = 16'h1120;
                {2'd2,4'd12}: cn_row = 16'h113C; {2'd2,4'd13}: cn_row = 16'h1120;
                {2'd2,4'd14}: cn_row = 16'h52A0; {2'd2,4'd15}: cn_row = 16'h247E;
                // 示
                {2'd3,4'd0}: cn_row = 16'h0000;  {2'd3,4'd1}:  cn_row = 16'h3FF8;
                {2'd3,4'd2}: cn_row = 16'h0000;  {2'd3,4'd3}:  cn_row = 16'h0000;
                {2'd3,4'd4}: cn_row = 16'h0000;  {2'd3,4'd5}:  cn_row = 16'h0000;
                {2'd3,4'd6}: cn_row = 16'hFFFE;  {2'd3,4'd7}:  cn_row = 16'h0100;
                {2'd3,4'd8}: cn_row = 16'h0100;  {2'd3,4'd9}:  cn_row = 16'h1110;
                {2'd3,4'd10}: cn_row = 16'h1108; {2'd3,4'd11}: cn_row = 16'h2104;
                {2'd3,4'd12}: cn_row = 16'h4102; {2'd3,4'd13}: cn_row = 16'h8102;
                {2'd3,4'd14}: cn_row = 16'h0500; {2'd3,4'd15}: cn_row = 16'h0200;
                default:     cn_row = 16'h0000;
            endcase
        end
    endfunction

    wire in_frame = trans_active &&
                    (h_cnt >= 11'd280) && (h_cnt < 11'd360) &&
                    (v_cnt >= 11'd224) && (v_cnt < 11'd256);
    // 2px 边框；框内不填充 → 透明
    wire frame_border = in_frame &&
                        ((h_cnt < 11'd282) || (h_cnt >= 11'd358) ||
                         (v_cnt < 11'd226) || (v_cnt >= 11'd254));

    wire in_text = trans_active &&
                   (h_cnt >= 11'd288) && (h_cnt < 11'd352) &&
                   (v_cnt >= 11'd232) && (v_cnt < 11'd248);
    wire [10:0] cn_dx = h_cnt - 11'd288;
    wire [10:0] cn_dy = v_cnt - 11'd232;
    wire [1:0]  cn_ci = cn_dx[5:4];          // 第几个字
    wire [3:0]  cn_cx = cn_dx[3:0];          // 字内列
    wire [3:0]  cn_cy = cn_dy[3:0];          // 字内行
    wire [15:0] cn_bits = cn_row(cn_ci, cn_cy);
    wire        cn_pix  = in_text && cn_bits[4'd15 - cn_cx];

    // ==================== 最终叠加 ====================
    // 优先级由 if 顺序决定（命中即停，不做 alpha 混合）：
    //   字幕条 > 计数绿字 > 耗时绿字 > 参数黄字 > 转场红框 > 转场中文。
    // 2026-11-03 起只有字幕条带底带，其后三层都只剩字形、无底框：
    // 未命中字形的像素统一走最后的 rgb_out_r <= rgb_in，即完全透明。
    // 注意 in_counter / in_time / in_param 这三个区域门控**不能删**：
    //   cnt_pixel_on / tm_pixel_on / pm_pixel_on 内部只做了行窗口判断，
    //   没有横向门控，直接放开会让字形在整幅宽度上重复出现。
    // 只有 de 有效时才改写 rgb_out_r；消隐期保持上一像素值不动，
    // 因为下游靠 de_out 屏蔽，不会把消隐期的颜色打进画面。
    // de_out 是 de 打一拍，与 rgb_out_r 的寄存延迟对齐。
    reg [23:0] rgb_out_r;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rgb_out_r <= 24'd0;
            de_out    <= 1'b0;
        end else begin
            de_out <= de;
            if (de) begin
                if (in_band)                        rgb_out_r <= band_pixel;    // 字幕白字 + 深灰底带
                else if (in_counter && cnt_pixel_on) rgb_out_r <= 24'h00FF00;   // 绿色计数（无底框）
                else if (in_time && tm_pixel_on)    rgb_out_r <= 24'h00FF00;    // 切图耗时文字（同绿色）
                else if (in_param && pm_pixel_on)   rgb_out_r <= 24'hFFFF00;    // 黄色参数文字
                else if (frame_border)              rgb_out_r <= 24'hFF0000;    // 转场提示：红框（内部透明）
                else if (cn_pix)                    rgb_out_r <= 24'hFF0000;    // 转场提示：红色中文
                else                                rgb_out_r <= rgb_in;
            end
        end
    end

    assign rgb_out = rgb_out_r;

endmodule
