`timescale 1ns/1ps
// ============================================================================
// img_scaler —— 任意分辨率 BMP 缩放到 DST_W×DST_H（写侧缩放）
// ----------------------------------------------------------------------------
// 位置
//   挂在 SD 卡读图通路里（sd_card_bmp 的写侧），不在显示通路上。
//   写侧读一张图约 1s，生成一帧约 9ms，时间余量极大；
//   显示侧是 25MHz 像素时钟 + SDRAM 突发喂数的硬实时通路，在那里做插值
//   会直接吃掉时序余量，风险高得多。
//
// 算法
//   可分离双线性：行内水平插值 + 行间垂直插值，效果等同 2×2 双线性，
//   硬件只需 8×8 乘法与移位，插值通路内没有除法器。
//     1) 水平：源像素到达时按输出列插值，写入 DST_W 宽的源行缓冲；
//     2) 垂直：两条行缓冲按垂直权重插值，逐行输出。
//
// 行缓冲（3 条轮转）
//   b_prev = 倒数第 2 条源行（已水平插值）
//   b_cur  = 最近一条源行
//   b_fill = 正在写入的那条（下一条源行）
//   垂直插值需要"上一条源行"仍然存在，所以用 3 条；每完成一条源行轮转一次，
//   (b_prev, b_cur) 就始终是最近两条完整行。
//
// 几何（Q16 定点）
//   step = 源长度 / 目标长度
//   xacc 初值 = step/2 - 0.5，中心对齐：首尾各偏半个目标像素，整体不偏移
//   KEEP_ASPECT=1：两轴取同一个（较大的）step，另一维居中留黑边，
//                  避免 16:9 的图被硬拉成 4:3；=0 则拉伸铺满。
//
// 速率约束（改动前先算）
//   填充一行 = src_w × 96 拍（25MHz SPI，24bpp）
//   生成一行 = 约 1920 拍（3 拍/像素 @100MHz）
//   条件：src_w × 96 > 1920 / step_y，即 src_w > 20 / step_y
//   本工程实际素材（≥427px 宽、step_y≈1.0）余量 20 倍以上；
//   源图小到约 100×75 以下时生成会追上填充，个别输出行读到已被覆盖的缓冲
//   （画面轻微横条抖动），但像素总数仍精确为 DST_W×DST_H，不会卡死。
// ============================================================================
module img_scaler #(
    parameter integer DST_W       = 640,     // 目标宽（= HDMI 输出宽）
    parameter integer DST_H       = 480,     // 目标高
    parameter integer KEEP_ASPECT = 1        // 1=保持宽高比（居中留黑边） 0=拉伸铺满
)(
    input  wire        clk,             // 100MHz（sd_card_clk，与写 FIFO 同域）
    input  wire        rst,

    // 一帧的开始：写请求被应答、写 FIFO 被清空的那一拍。
    // 必须与 FIFO 清空同拍，否则先吐的黑边行会被清掉 → 整帧错位。
    input  wire        frame_start,
    input  wire [15:0] dst_w,           // 目标宽（由 sd_card_bmp 传入 640）
    input  wire [15:0] dst_h,           // 目标高（480）

    // ---- 源像素流（bmp_read 输出：按 BMP 行序、已剔除行尾对齐填充）----
    input  wire        src_de,
    input  wire [23:0] src_pixel,       // {R,G,B}
    input  wire [15:0] src_x,           // 0 .. src_w-1
    input  wire [15:0] src_w,
    input  wire [15:0] src_h,

    // ---- 输出：固定 dst_w × dst_h 个像素，行优先，连续无空拍 ----
    output reg         out_de,
    output reg  [23:0] out_pixel,
    output reg         frame_done,      // 最后一个像素输出后拉高一拍
    output wire        params_ok        // 1 = 步进已算好，可以开始收像素
);

    localparam [15:0] DW    = DST_W;        // parameter 不能位选，先落成定宽常量
    localparam [15:0] DH    = DST_H;
    localparam [9:0]  DW_M1 = DST_W - 1;

    // ------------------------------------------------------------------
    // 0. 8 位线性插值 out = a + (b - a) * f / 256  —— 按通道分时复用
    //
    //    最早写成 function lerp8，填充侧和生成侧各调 3 次，一共 6 处。
    //    function 是展开而非实例化，综合器就老老实实做了 6 个 8×8 乘法器，
    //    占掉 6 个 DSP（全片 29 个里的两成）。可这三个通道用的是同一个
    //    分数 f、只差输入数据，本来就该共用一个乘法器。
    //
    //    现在改成"一次算一个通道"：ch 选出 a/b，做一次差值，一个 8×8
    //    乘法，再按原方向加回去。填充侧、生成侧各自一组（两条通路同时在
    //    跑，生成侧每 3 拍就要一次，硬凑一个反而要仲裁），合计 2 个。
    //
    //    算式本身没变，仍是无符号实现、分数 f 取 8 位、结果截断到高 8 位，
    //    误差小于 1 LSB。下面 f_* 属填充侧、g_* 属生成侧，声明在各自变量旁。
    // ------------------------------------------------------------------

    // ------------------------------------------------------------------
    // 1. 帧首算参数：step_x / step_y / 有效区尺寸 / 居中偏移
    //    3 次顺序除法（每次 32 拍）+ 1 次 16 拍移位乘法，合计约 150 拍，
    //    远小于一个源像素的间隔（24bpp SPI 下 96 拍/像素，整行更不用说）
    //    状态流：P_DX → P_DXW → P_DY → P_DYW → P_DF → P_DFM → P_DFW → P_RDY
    // ------------------------------------------------------------------
    localparam P_IDLE = 4'd0;
    localparam P_DX   = 4'd1;   // 发起 step_x = (src_w<<16)/dst_w
    localparam P_DXW  = 4'd2;
    localparam P_DY   = 4'd3;   // 发起 step_y = (src_h<<16)/dst_h
    localparam P_DYW  = 4'd4;
    localparam P_DF   = 4'd5;   // 装载拟合尺寸的乘数，发起 (另一维 × 目标)
    localparam P_DFM  = 4'd6;   // 16 拍移位加算乘积（原来是一条 * 语句）
    localparam P_DFW  = 4'd7;
    localparam P_RDY  = 4'd8;

    reg [3:0]  pstate;
    reg        d_start;
    reg [31:0] d_a;
    reg [15:0] d_b;
    wire [31:0] d_q;
    wire        d_done;

    // 拟合尺寸的乘法改成移位加：mcand 左移、mplier 右移，16 拍出结果。
    //   原来写 {16'd0,src_h} * {16'd0,dst_w}，两个 16 位操作数被综合成
    //   16×16 乘法器，吃掉 2 个 DSP —— 而这段每帧只在 frame_start 后
    //   跑一次。移位加只多花 16 拍（对比 3 次 32 拍除法，可忽略），
    //   代价是一个 32 位加法器。
    reg [31:0] mul_mcand;
    reg [15:0] mul_mplier;
    reg [31:0] mul_acc;
    reg [4:0]  mul_cnt;

    reg [31:0] step_x, step_y;
    reg [15:0] out_w, out_h, xoff, yoff;

    // 中心对齐的首个源坐标（Q16）。仅当 step >= 1.0 时才可能为负，钳到 0
    wire [31:0] x_init = (step_x >= 32'd65536) ? ((step_x >> 1) - 32'd32768) : 32'd0;
    wire [31:0] y_init = (step_y >= 32'd65536) ? ((step_y >> 1) - 32'd32768) : 32'd0;

    reg        p_ok;
    reg        width_limited;      // 1 = 宽度先到边界（out_w = dst_w）

    // 注意：yacc 只能由"生成侧"那一个 always 块驱动。
    //   参数状态机曾在 P_RDY 里直接写 yacc，与生成侧的复位 / G_NEXT 形成双驱动，
    //   TD 综合报 "net 'yacc[31]' is constantly driven from multiple places"。
    //   现在参数侧只把首行对应的源 y 存进 y_start，由生成侧在 p_ok 之后装一次。
    reg [31:0] y_start;            // 参数侧输出：首行对应的源 y（Q16）

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pstate   <= P_IDLE;
            d_start  <= 1'b0;
            d_a      <= 32'd0;
            d_b      <= 16'd1;
            step_x   <= 32'd65536;
            step_y   <= 32'd65536;
            y_start  <= 32'd0;
            out_w    <= DW;
            out_h    <= DH;
            xoff     <= 16'd0;
            yoff     <= 16'd0;
            p_ok     <= 1'b0;
            width_limited <= 1'b1;
            mul_mcand  <= 32'd0;
            mul_mplier <= 16'd0;
            mul_acc    <= 32'd0;
            mul_cnt    <= 5'd0;
        end else begin
            d_start <= 1'b0;
            if (frame_start) begin
                // 尺寸非法的图不启动，交给上层超时/跳过逻辑处理
                pstate <= ((src_w == 16'd0) || (src_h == 16'd0) ||
                           (dst_w == 16'd0) || (dst_h == 16'd0)) ? P_IDLE : P_DX;
                p_ok   <= 1'b0;
            end else begin
                case (pstate)
                    P_IDLE: ;
                    P_DX: begin
                        d_a     <= {src_w[15:0], 16'd0};
                        d_b     <= dst_w;
                        d_start <= 1'b1;
                        pstate  <= P_DXW;
                    end
                    P_DXW: begin
                        if (d_done) begin
                            step_x <= d_q;
                            pstate <= P_DY;
                        end
                    end
                    P_DY: begin
                        d_a     <= {src_h[15:0], 16'd0};
                        d_b     <= dst_h;
                        d_start <= 1'b1;
                        pstate  <= P_DYW;
                    end
                    P_DYW: begin
                        if (d_done) begin
                            step_y <= d_q;
                            pstate <= P_DF;
                        end
                    end
                    P_DF: begin
                        // 保持宽高比：取较大的 step 作为统一步进
                        //   宽度受限 → out_w = dst_w，out_h = src_h*dst_w/src_w
                        //   高度受限 → out_h = dst_h，out_w = src_w*dst_h/src_h
                        // 这里只装载乘数，乘积交给 P_DFM 用 16 拍移位加算。
                        if (KEEP_ASPECT != 0) begin
                            width_limited <= (step_x >= step_y);
                            // out_h = src_h*dst_w/src_w  （÷src_w，不用再除以 step）
                            // out_w = src_w*dst_h/src_h
                            if (step_x >= step_y) begin
                                mul_mcand  <= {16'd0, dst_w};
                                mul_mplier <= src_h;
                                d_b        <= src_w;
                            end else begin
                                mul_mcand  <= {16'd0, dst_h};
                                mul_mplier <= src_w;
                                d_b        <= src_h;
                            end
                        end else begin
                            width_limited <= 1'b1;
                            mul_mcand  <= 32'd0;
                            mul_mplier <= 16'd0;
                            d_b        <= 16'd1;
                        end
                        mul_acc <= 32'd0;
                        mul_cnt <= 5'd0;
                        pstate  <= P_DFM;
                    end
                    P_DFM: begin
                        // 16 位 × 16 位 = 移位加：被乘数左移一格、乘数右移一格，
                        // 每拍最多做一次 32 位加法。共 16 次迭代 + 1 拍出结果。
                        // 溢出行为与原 32 位乘法一致（都是取低 32 位）。
                        if (mul_cnt == 5'd16) begin
                            d_a     <= mul_acc;
                            d_start <= 1'b1;
                            pstate  <= P_DFW;
                        end else begin
                            if (mul_mplier[0])
                                mul_acc <= mul_acc + mul_mcand;
                            mul_mcand  <= {mul_mcand[30:0], 1'b0};
                            mul_mplier <= {1'b0, mul_mplier[15:1]};
                            mul_cnt    <= mul_cnt + 5'd1;
                        end
                    end
                    P_DFW: begin
                        if (d_done) begin
                            if (KEEP_ASPECT != 0) begin
                                if (width_limited) begin
                                    step_y <= step_x;               // 两轴统一
                                    out_w  <= dst_w;
                                    out_h  <= (d_q > {16'd0, dst_h}) ? dst_h : d_q[15:0];
                                end else begin
                                    step_x <= step_y;               // 两轴统一
                                    out_h  <= dst_h;
                                    out_w  <= (d_q > {16'd0, dst_w}) ? dst_w : d_q[15:0];
                                end
                            end else begin
                                out_w <= dst_w;
                                out_h <= dst_h;
                            end
                            pstate <= P_RDY;
                        end
                    end
                    P_RDY: begin
                        xoff   <= ({1'b0, dst_w} - {1'b0, out_w}) >> 1;
                        yoff   <= ({1'b0, dst_h} - {1'b0, out_h}) >> 1;
                        // 首行源 y 必须在这里给：step_y 到这一拍才稳定（保持宽高比
                        // 时它已被 step_x 覆盖）。漏掉会让整幅图垂直偏半个源像素，
                        // 且首行少插值一次。只写 y_start，不能直接写 yacc（多驱动）。
                        y_start <= y_init;
                        p_ok   <= 1'b1;
                        pstate <= P_IDLE;
                    end
                    default: pstate <= P_IDLE;
                endcase
            end
        end
    end

    assign params_ok = p_ok;

    // ------------------------------------------------------------------
    // 2. 三条行缓冲（每条 DST_W × 24bit）
    //    写只发生在 b_fill 指向的那条；读时用同一列地址同时读三条再选择，
    //    因此每条 RAM 只需 1 读口 + 1 写口（简单双口，可推成 BRAM）
    // ------------------------------------------------------------------
    reg [23:0] line0 [0:DST_W-1];
    reg [23:0] line1 [0:DST_W-1];
    reg [23:0] line2 [0:DST_W-1];

    reg [1:0]  b_fill, b_cur, b_prev;

    reg [9:0]  wr_addr;
    reg [23:0] wr_data;
    reg        wr_en;

    always @(posedge clk) begin
        if (wr_en) begin
            if (b_fill == 2'd0)      line0[wr_addr] <= wr_data;
            else if (b_fill == 2'd1) line1[wr_addr] <= wr_data;
            else                     line2[wr_addr] <= wr_data;
        end
    end

    // 行缓冲读地址：必须比要用它的那一级提前一拍给。
    //   RAM 输出是寄存器化的，G_RD 这一拍采到的是 rd_addr 上一拍的值。
    //   曾写成"在 G_RD 里 rd_addr <= gx"，每个像素都拿到上一列的数据，
    //   整行右移一个像素（直通测试表现为第 0 列重复、最后一列丢失）。
    reg [9:0]  rd_addr;
    reg [23:0] q0, q1, q2;

    always @(posedge clk) begin
        q0 <= line0[rd_addr];
        q1 <= line1[rd_addr];
        q2 <= line2[rd_addr];
    end

    // ------------------------------------------------------------------
    // 3. 填充侧：源像素 → 水平插值 → 行缓冲
    //    每个源像素展开为若干输出列（最多 ceil(1/step)+1 个），逐拍写入
    //    状态流：F_IDLE → F_LEAD(左黑边) → F_PIX(展开) → F_FLUSH(行尾补足)
    //            → F_TAIL(右黑边) → F_ROWEND(轮转缓冲)
    // ------------------------------------------------------------------
    localparam F_IDLE  = 3'd0;
    localparam F_LEAD  = 3'd1;   // 写左侧黑边
    localparam F_PIX   = 3'd2;   // 展开源像素
    localparam F_FLUSH = 3'd3;   // 行尾补足（用最后一个源像素）
    localparam F_TAIL  = 3'd4;   // 写右侧黑边
    localparam F_ROWEND= 3'd5;   // 行结束：轮转缓冲

    reg [2:0]  fstate;
    reg [23:0] f_prev, f_cur;
    reg [15:0] f_sx;
    reg        f_hold;
    reg        f_last;
    reg [31:0] xacc;
    reg [15:0] xo;               // 本行已写入行缓冲的像素数
    reg [15:0] done_rows;        // 已完成的源行数（0..src_h）

    // 三通道分时复用：一个输出像素拆成 3 拍算，每拍一个通道。
    //   f_busy 表示正在算；f_ch 是当前通道（0=R 1=G 2=B）；
    //   f_px 累积已算好的两个通道；f_frac 锁存本像素的插值分数
    //   （xacc[15:8]，因为 xacc 要到第 3 拍才加 step_x，中途不能让它变）。
    //   原来这三拍压成一拍、三个 lerp8 并行，才有 3 个乘法器。
    reg        f_busy;
    reg [1:0]  f_ch;
    reg [23:0] f_px;
    reg [7:0]  f_frac;

    // 填充侧唯一的 8×8 乘法器：按 f_ch 选出通道，做差值、乘、再加回
    wire [7:0]  f_a  = (f_ch == 2'd0) ? f_prev[23:16] :
                       (f_ch == 2'd1) ? f_prev[15:8]  : f_prev[7:0];
    wire [7:0]  f_b  = (f_ch == 2'd0) ? f_cur [23:16] :
                       (f_ch == 2'd1) ? f_cur [15:8]  : f_cur [7:0];
    wire        f_up = (f_b >= f_a);
    wire [7:0]  f_d  = f_up ? (f_b - f_a) : (f_a - f_b);
    wire [16:0] f_p  = {9'd0, f_d} * {9'd0, f_frac};
    wire [7:0]  f_r  = f_up ? (f_a + f_p[15:8]) : (f_a - f_p[15:8]);

    // 影子接收器：任何状态下到达的源像素都不能丢。
    //   曾在 F_PIX 里直接收 src_de，F_FLUSH→F_TAIL→F_ROWEND→F_LEAD 这 5 拍
    //   空窗里若来了新行首像素就会被丢掉，于是每行 x=0 用上一行残留的 f_prev
    //   去算，480 行里 479 行第 0 列错色。
    //   源字节间隔越短（如测试台 GAPB=2）越容易撞上，真机 SPI 每字节几十拍
    //   碰不到，但这是必须堵的功能性缺陷。
    reg        sh_v;
    reg [23:0] sh_pix;
    reg [15:0] sh_sx;
    reg        sh_last;
    // sh_pop 只能由填充状态机驱动，影子接收器只读它。
    //   曾叫 sh_take，接收器清零 + 填充侧置位两边都写，又是
    //   "constantly driven from multiple places"；现在所有权归填充侧，
    //   时序不变（置位后下一拍被接收器看见并清 sh_v）。
    reg        sh_pop;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sh_v    <= 1'b0;
            sh_pix  <= 24'd0;
            sh_sx   <= 16'd0;
            sh_last <= 1'b0;
        end else if (frame_start) begin
            sh_v    <= 1'b0;
        end else begin
            if (src_de) begin
                sh_v    <= 1'b1;
                sh_pix  <= src_pixel;
                sh_sx   <= src_x;
                sh_last <= (src_x == (src_w - 16'd1));
            end else if (sh_pop) begin
                sh_v    <= 1'b0;
            end
        end
    end

    wire [15:0] x_end = xoff + out_w;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            fstate <= F_IDLE;
            wr_en  <= 1'b0;
            wr_addr<= 10'd0;
            wr_data<= 24'd0;
            xo     <= 16'd0;
            xacc   <= 32'd0;
            f_hold <= 1'b0;
            f_last <= 1'b0;
            f_prev <= 24'd0;
            f_cur  <= 24'd0;
            f_sx   <= 16'd0;
            f_busy <= 1'b0;
            f_ch   <= 2'd0;
            f_px   <= 24'd0;
            f_frac <= 8'd0;
            b_fill <= 2'd0;
            b_cur  <= 2'd1;
            b_prev <= 2'd2;
            done_rows <= 16'd0;
            sh_pop <= 1'b0;
        end else if (frame_start) begin
            fstate <= F_IDLE;
            wr_en  <= 1'b0;
            sh_pop <= 1'b0;
            xo     <= 16'd0;
            f_hold <= 1'b0;
            f_last <= 1'b0;
            f_prev <= 24'd0;
            f_busy <= 1'b0;
            f_ch   <= 2'd0;
            f_px   <= 24'd0;
            b_fill <= 2'd0;
            b_cur  <= 2'd1;
            b_prev <= 2'd2;
            done_rows <= 16'd0;
        end else begin
            wr_en  <= 1'b0;
            sh_pop <= 1'b0;
            case (fstate)
                F_IDLE: begin
                    if (p_ok) begin
                        xo     <= 16'd0;
                        xacc   <= x_init;
                        f_hold <= 1'b0;
                        fstate <= F_LEAD;
                    end
                end

                // 左侧黑边：输出列 < xoff 的部分填黑
                F_LEAD: begin
                    if (xo < xoff) begin
                        wr_en   <= 1'b1;
                        wr_addr <= xo[9:0];
                        wr_data <= 24'd0;
                        xo      <= xo + 16'd1;
                    end else begin
                        fstate <= F_PIX;
                    end
                end

                F_PIX: begin
                    if (f_busy) begin
                        // 三通道串行：第 0/1 拍把算好的通道攒进 f_px，第 2 拍连同
                        // 刚算完的 B 一起写进行缓冲，然后才推进 xo / xacc。
                        //   xacc 必须等到第 2 拍才加，否则第 1 拍起分数就变了。
                        if (f_ch == 2'd2) begin
                            wr_en   <= 1'b1;
                            wr_addr <= xo[9:0];
                            wr_data <= {f_px[23:16], f_px[15:8], f_r};
                            xo      <= xo + 16'd1;
                            xacc    <= xacc + step_x;
                            f_ch    <= 2'd0;
                            f_busy  <= 1'b0;
                        end else begin
                            if (f_ch == 2'd0) f_px[23:16] <= f_r;
                            else              f_px[15:8]  <= f_r;
                            f_ch <= f_ch + 2'd1;
                        end
                    end else if (f_hold) begin
                        // 有像素待展开：只要下一个输出列还落在 [sx-1, sx) 内就吐一个
                        if ((f_sx >= 16'd1) && (xacc[31:16] == (f_sx - 16'd1)) &&
                            (xo < x_end)) begin
                            f_busy <= 1'b1;
                            f_ch   <= 2'd0;
                            f_frac <= xacc[15:8];    // 锁定本像素分数，3 拍内不变
                        end else begin
                            f_hold <= 1'b0;
                            f_prev <= f_cur;
                            if (f_last)
                                fstate <= F_FLUSH;
                        end
                    end else if (sh_v) begin
                        f_cur   <= sh_pix;
                        f_sx    <= sh_sx;
                        f_hold  <= 1'b1;
                        f_last  <= sh_last;
                        sh_pop  <= 1'b1;      // 下一拍让影子接收器放掉 sh_v
                    end
                end

                // 行尾补足：右边界最后一个源像素覆盖的剩余列
                F_FLUSH: begin
                    if (xo < x_end) begin
                        wr_en   <= 1'b1;
                        wr_addr <= xo[9:0];
                        wr_data <= f_prev;
                        xo      <= xo + 16'd1;
                    end else begin
                        fstate <= F_TAIL;
                    end
                end

                // 右侧黑边
                F_TAIL: begin
                    if (xo < dst_w) begin
                        wr_en   <= 1'b1;
                        wr_addr <= xo[9:0];
                        wr_data <= 24'd0;
                        xo      <= xo + 16'd1;
                    end else begin
                        fstate <= F_ROWEND;
                    end
                end

                // 一行结束：轮转 3 条行缓冲，源行计数 +1
                F_ROWEND: begin
                    b_prev <= b_cur;
                    b_cur  <= b_fill;
                    b_fill <= b_prev;
                    if (done_rows != src_h)
                        done_rows <= done_rows + 16'd1;
                    xo     <= 16'd0;
                    xacc   <= x_init;
                    // f_hold / f_last 必须在这里清掉：
                    //   行末从 F_FLUSH 退出时 f_hold 可能仍为 1，带进新行会让
                    //   F_PIX 在新行首像素到达前先走一遍 f_hold 分支，用上一行的
                    //   f_prev/f_cur 计算，x=0 被写成残值；
                    //   f_last 残留 1 则会让新行直接跳 F_FLUSH 整行补同色。
                    f_hold <= 1'b0;
                    f_last <= 1'b0;
                    fstate <= F_LEAD;
                end

                default: fstate <= F_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // 4. 生成侧：两条行缓冲 → 垂直插值 → 输出
    //    每像素 3 拍（三通道串行），行首多 2 拍（G_RD 等 + G_CALC 锁存），
    //    一行约 3×DST_W + 2 拍
    //    状态流：G_IDLE → G_BLACK(黑边行) | G_RD → G_CALC → G_OUT → G_NEXT
    //
    //    时序不变式（改动前必读）
    //      b_prev / b_cur 是"最近填好的两行"的物理缓冲编号，随填充侧每完成
    //      一行轮转一次。生成侧只在行首（G_IDLE）锁存 gsel_a / gsel_b，
    //      所以本行的 DST_W 个像素看到的一定是同一对缓冲。
    //      推论：填充侧每完成一行，生成侧必须已经输出过至少一行，
    //      否则本行需要的 sy / sy+1 会被挤掉，读到的是 sy+2 / sy+3。
    //
    //    折算成速率条件（sd_card_clk=100MHz，SPI 25MHz → 32 拍/字节）
    //      填充一行 = src_w × 96 拍（24bpp）
    //      生成一行 ≈ 3×DST_W + 1 拍，每个源行对应 1/step_y 个输出行
    //      要求 src_w × 96 > (3×DST_W+1) / step_y，即 src_w > 25 / step_y
    //      step_y=0.667 时要求 src_w > 38；本工程素材最窄 427，余量 10 倍以上
    //
    //      注：三通道串行并没有降低每像素的拍数 —— 原来 G_RD/G_CALC/G_OUT
    //      三个状态也是 3 拍/像素，现在把"等 RAM 输出"和"锁存"挪到上一个
    //      像素的第 3 拍里了，只有每行首个像素额外花 1 拍（G_RD）。
    //
    //    该条件只在"源数据比生成慢"时成立。ModelSim 用 +GAPB=2（1 字节/2 拍）
    //    喂数时填充快 16 倍，会打破条件，表现为顶部竖向拖影——那是测试台不真实，
    //    不是 RTL 错；测上板行为请用 +GAPB=32。
    //    若将来换 SDIO / 提速 SPI，必须改成"按行号打 tag 选缓冲 + 源侧 FIFO"。
    // ------------------------------------------------------------------
    localparam G_IDLE  = 3'd0;
    localparam G_RD    = 3'd1;   // 行首：等 RAM 吐出第 0 列（地址已在 G_IDLE 给出）
    localparam G_CALC  = 3'd2;   // 行首：锁存第 0 列，同时预取第 1 列
    localparam G_OUT   = 3'd3;   // 每像素 3 拍，三通道串行
    localparam G_NEXT  = 3'd4;
    localparam G_BLACK = 3'd5;

    reg [2:0]  gstate;
    reg [9:0]  gx;
    reg [23:0] ga, gb;
    reg [7:0]  gfy;
    reg [1:0]  gsel_a, gsel_b;
    reg [15:0] yo;
    reg        g_fin;
    // 当前输出行对应的源 y（Q16）。**只在本块里驱动**（TD 多驱动问题见第 1 节）。
    reg [31:0] yacc;
    reg        yacc_ld;          // 0 = 还没从 y_start 装过初值
    // 黑边行节流：黑边不需要源数据，1 像素/拍连发会让整行 640 个像素一口气
    // 灌进写 FIFO（深度 512），而 SDRAM 还要同时喂显示侧读，余量偏紧。
    // 隔拍输出后最坏突发降到 0.5 像素/拍，代价仅 640 拍/黑边行（可忽略）。
    reg        blast;

    // 三通道串行（同填充侧）：g_ch 当前通道，px_acc 攒已算好的两个通道
    reg [1:0]  g_ch;
    reg [23:0] px_acc;

    // 生成侧唯一的 8×8 乘法器
    wire [7:0]  g_a  = (g_ch == 2'd0) ? ga[23:16] :
                       (g_ch == 2'd1) ? ga[15:8]  : ga[7:0];
    wire [7:0]  g_b  = (g_ch == 2'd0) ? gb[23:16] :
                       (g_ch == 2'd1) ? gb[15:8]  : gb[7:0];
    wire        g_up = (g_b >= g_a);
    wire [7:0]  g_d  = g_up ? (g_b - g_a) : (g_a - g_b);
    wire [16:0] g_p  = {9'd0, g_d} * {9'd0, gfy};
    wire [7:0]  g_r  = g_up ? (g_a + g_p[15:8]) : (g_a - g_p[15:8]);

    // 当前输出行对应的源行号（钳位到 src_h-1）
    wire [15:0] yacc_i = yacc[31:16];
    wire [15:0] sy     = (yacc_i >= (src_h - 16'd1)) ? (src_h - 16'd1) : yacc_i;
    wire [15:0] sy_hi  = (sy >= (src_h - 16'd1)) ? (src_h - 16'd1) : (sy + 16'd1);
    // 两条源行都到齐了吗（done_rows = 已完成行数，行号 0..done_rows-1）
    wire        row_avail = (({1'b0, sy_hi} + 17'd1) <= {1'b0, done_rows});

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            gstate  <= G_IDLE;
            out_de  <= 1'b0;
            out_pixel <= 24'd0;
            frame_done <= 1'b0;
            gx      <= 10'd0;
            ga      <= 24'd0;
            gb      <= 24'd0;
            gfy     <= 8'd0;
            gsel_a  <= 2'd0;
            gsel_b  <= 2'd1;
            yacc    <= 32'd0;
            yacc_ld <= 1'b0;
            yo      <= 16'd0;
            g_fin   <= 1'b0;
            blast   <= 1'b0;
            rd_addr <= 10'd0;
            g_ch    <= 2'd0;
            px_acc  <= 24'd0;
        end else if (frame_start) begin
            gstate  <= G_IDLE;
            out_de  <= 1'b0;
            frame_done <= 1'b0;
            gx      <= 10'd0;
            gfy     <= 8'd0;
            yacc    <= 32'd0;
            yacc_ld <= 1'b0;
            yo      <= 16'd0;
            g_fin   <= 1'b0;
            blast   <= 1'b0;
            g_ch    <= 2'd0;
            px_acc  <= 24'd0;
        end else begin
            out_de     <= 1'b0;
            frame_done <= 1'b0;
            case (gstate)
                G_IDLE: begin
                    // 参数就绪后先花一拍把 y_start 装进 yacc。y_start 与 p_ok 在
                    // 同一个时钟沿更新，所以 p_ok 被采到的这一拍 y_start 已经是新值。
                    // 只装一次，之后每输出一行由 G_NEXT 累加 step_y。
                    if (p_ok && !yacc_ld) begin
                        yacc    <= y_start;
                        yacc_ld <= 1'b1;
                    end else if (!g_fin && p_ok && (yo >= dst_h)) begin
                        g_fin      <= 1'b1;
                        frame_done <= 1'b1;
                    end else if (!g_fin && p_ok && ((yo < yoff) || (yo >= (yoff + out_h)))) begin
                        // 上下黑边行：不需要源数据，直接整行填黑
                        gx     <= 10'd0;
                        gstate <= G_BLACK;
                    end else if (!g_fin && p_ok && row_avail) begin
                        // 锁存本行用的缓冲与垂直权重，避免中途被轮转改写
                        gfy    <= (sy_hi == sy) ? 8'd0 : yacc[15:8];
                        gsel_b <= b_cur;
                        gsel_a <= (sy_hi == sy) ? b_cur : b_prev;
                        gx      <= 10'd0;
                        rd_addr <= 10'd0;
                        gstate  <= G_RD;
                    end
                end

                G_BLACK: begin
                    if (blast) begin
                        blast <= 1'b0;               // 空一拍，把 SDRAM 写带宽让出去
                    end else begin
                        blast     <= 1'b1;
                        out_de    <= 1'b1;
                        out_pixel <= 24'd0;
                        if (gx == DW_M1) begin
                            gx     <= 10'd0;
                            yo     <= yo + 16'd1;
                            blast  <= 1'b0;          // 行末复位相位，下一行仍从"发"开始
                            gstate <= G_IDLE;
                        end else begin
                            gx <= gx + 10'd1;
                        end
                    end
                end

                G_RD: begin
                    // 行首：只等行缓冲吐出第 0 列（地址是 G_IDLE 给的）。
                    //   这里不能顺手锁存 —— RAM 是寄存器输出的，地址给出后要
                    //   隔一拍 q 才是新数据，这一拍读到的还是上一行最后一列。
                    gstate <= G_CALC;
                end

                G_CALC: begin
                    // 行首：锁存第 0 列，并把地址推到第 1 列。
                    //   之后每个像素的第 3 拍会顺手锁存下一像素（那时 q 早已
                    //   换成新地址的数据），不再回这两个状态。
                    ga      <= (gsel_a == 2'd0) ? q0 : (gsel_a == 2'd1) ? q1 : q2;
                    gb      <= (gsel_b == 2'd0) ? q0 : (gsel_b == 2'd1) ? q1 : q2;
                    rd_addr <= gx + 10'd1;
                    g_ch    <= 2'd0;
                    gstate  <= G_OUT;
                end

                G_OUT: begin
                    // 三通道串行。第 0/1 拍各算一个通道攒进 px_acc，第 2 拍算完 B
                    // 后整像素输出，同一拍把下一像素的 ga/gb 锁存掉 —— 这样每像素
                    // 仍是 3 拍，与原来的 G_RD/G_CALC/G_OUT 三态等价。
                    //
                    // 地址必须在第 0 拍就给：行缓冲是寄存器输出，给了地址要隔一拍
                    // q 才更新，第 2 拍才正好能采到下一列。写在第 2 拍的话采到的
                    // 还是本像素自己，整行会从第二个像素起重复上一像素。
                    if (g_ch == 2'd2) begin
                        out_de    <= 1'b1;
                        out_pixel <= {px_acc[23:16], px_acc[15:8], g_r};
                        ga <= (gsel_a == 2'd0) ? q0 : (gsel_a == 2'd1) ? q1 : q2;
                        gb <= (gsel_b == 2'd0) ? q0 : (gsel_b == 2'd1) ? q1 : q2;
                        g_ch <= 2'd0;
                        if (gx == DW_M1) begin
                            gstate <= G_NEXT;
                        end else begin
                            gx     <= gx + 10'd1;
                            gstate <= G_OUT;
                        end
                    end else begin
                        if (g_ch == 2'd0) begin
                            px_acc[23:16] <= g_r;
                            rd_addr       <= gx + 10'd1;   // 预取下一列
                        end else begin
                            px_acc[15:8]  <= g_r;
                        end
                        g_ch <= g_ch + 2'd1;
                    end
                end

                G_NEXT: begin
                    yo     <= yo + 16'd1;
                    yacc   <= yacc + step_y;
                    gstate <= G_IDLE;
                end

                default: gstate <= G_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // 5. 顺序除法器：32 位 / 16 位，32 拍出商（移位-减法的恢复余数法）
    //    busy 期间再来的 start 被忽略；done 为单拍脉冲，商在 d_q 上保持
    // ------------------------------------------------------------------
    div32 u_div (
        .clk    (clk),
        .rst    (rst),
        .start  (d_start),
        .a      (d_a),
        .b      (d_b),
        .q      (d_q),
        .done   (d_done)
    );

endmodule


// ============================================================================
// div32 —— 32 位无符号顺序除法器（恢复余数法）
//   a / b = q，32 拍出商，done 为单拍脉冲；b 必须非 0
//   （调用侧在 frame_start 已保证 dst_w/dst_h/src_w/src_h 均 > 0）
// ============================================================================
module div32 (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire [31:0] a,
    input  wire [15:0] b,
    output reg  [31:0] q,
    output reg         done
);
    reg [31:0] dd;
    reg [31:0] rr;
    reg [5:0]  cnt;
    reg        busy;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            busy <= 1'b0;
            done <= 1'b0;
            cnt  <= 6'd0;
            dd   <= 32'd0;
            rr   <= 32'd0;
            q    <= 32'd0;
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                dd   <= a;
                rr   <= 32'd0;
                cnt  <= 6'd0;
                q    <= 32'd0;
                busy <= 1'b1;
            end else if (busy) begin
                if (cnt == 6'd32) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end else begin
                    if ({rr[30:0], dd[31]} >= {16'd0, b}) begin
                        rr <= {rr[30:0], dd[31]} - {16'd0, b};
                        q  <= {q[30:0], 1'b1};
                    end else begin
                        rr <= {rr[30:0], dd[31]};
                        q  <= {q[30:0], 1'b0};
                    end
                    dd  <= {dd[30:0], 1'b0};
                    cnt <= cnt + 6'd1;
                end
            end
        end
    end
endmodule
