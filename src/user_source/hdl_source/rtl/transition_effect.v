// ============================================================================
// transition_effect —— 图片转场：旧图 → 白场 → 新图（单读通路）
// ----------------------------------------------------------------------------
// 效果
//   PH_A：读旧缓冲，out = 旧图*(1-a) + 白*a，a: 0→255，旧图逐渐被白场吃掉
//   PH_B：读新缓冲，out = 白*(1-b) + 新图*b，b: 0→255，新图从白场实体化
//   时长 = PHASE_FRAMES + WHITE_HOLD + PHASE_FRAMES = 30 + 4 + 30 = 64 帧
//        ≈ 1.08s @59.5Hz
//          0.00 ~ 0.50s  旧图褪成纯白
//          0.50 ~ 0.57s  停在纯白（短促一顿，看得出"已经变空了"）
//          0.57 ~ 1.08s  新图从纯白里逐渐实体化
//
// 为什么单读通路就够（不需要同时读新旧两张图）
//   sd_card_bmp 把新图写到"非当前显示"的另一块 buffer，整帧写完后才切
//   disp_buf_idx。所以：旧图在整个加载期间完好；前半段结束时画面已是全白，
//   此刻切缓冲看不到跳变；后半段新 buffer 已写完，读出来就是完整新图。
//   模块内部不区分 rgb_new 装的是旧图还是新图，只负责按段给权重；
//   读哪块 buffer 由顶层 rd_buf_sel 决定。
//
// 帧计数必须用 vs 上升沿（历史坑，勿改回 de）
//   de = h_active & v_active，每帧上升 V_ACTIVE 次（480 次）。早期用 de 数帧，
//   "60 帧/段"实际只有 120 行 ≈ 4ms，转场一闪而过，PHASE_FRAMES 调多大都
//   看不出来。vs 每帧只翻转一次，上升沿 = 场同步结束 = 新一帧开始，位置在
//   消隐区（v_cnt ≈ 第 11 行），权重在消隐区更新，不会造成场内撕裂。
//
// 调参
//   改 PHASE_FRAMES / WHITE_HOLD 时，lv_alpha 表长必须同步等于 PHASE_FRAMES；
//   top_signage.v 里的同名 localparam 也要同步（仅用于注释性校验）。
//
// 与 osd_overlay 的配合
//   osd_overlay 在 trans_active 期间于画面正中画红框 + "提示"。顶层目前把该
//   端口空连——1s 转场里红框会挡住整段淡化，看着像"画面坏了"；要恢复提示，
//   把 osd_overlay 的 .trans_active 接回本端口即可。
//
// 实现约束
//   安路 TD 对组合除法 / 取模支持不可靠，本模块只用预计算 LUT、移位与加减。
// ============================================================================

module transition_effect #(
    parameter integer H_ACTIVE     = 640,   // 保留：本版不再做空间类特效（滑动/擦除/百叶窗）
    parameter integer V_ACTIVE     = 480,   // 保留：同上
    parameter integer PHASE_FRAMES = 30,    // 每段帧数（淡出/淡入各自的时长）
    parameter integer WHITE_HOLD   = 4      // 中间全白停留帧数（≈0.07s，够看清但别拖太久）
)(
    input  wire        clk,        // video_clk
    input  wire        rst,

    input  wire        de,         // 数据有效（与 rgb_new 同拍）
    input  wire        vs,         // 场同步（本版用它数"帧"，见下方 frame_rise 说明）
    input  wire [23:0] rgb_new,    // 当前缓冲读出的像素（旧图或新图，由外部按缓冲切换）
    input  wire [23:0] rgb_old,    // 保留不用：本版两段都只读一路像素（见文件头说明）

    input  wire        trans_trig,    // 转场触发脉冲（顶层状态机产生，单拍）
    input  wire        start_phase_b, // 触发时从哪一段开始：0=先淡出（旧图→白），1=直接淡入（白→新图）
    input  wire [1:0]  trans_sel,     // 保留不用：本版统一白场淡化（原 KEY2 切换已删，顶层接 2'd0）

    output wire [23:0] rgb_out,    // 混合后像素
    output wire        de_out,     // 延迟后的 de（与 rgb_out 对齐，2 拍）
    output wire        trans_active,// 转场进行中（供 OSD 显示）
    output wire        phase_b,    // 1 = 已在后半段（应读新缓冲/新图片）
    output wire        phase_b_done, // 后半段"刚刚走完"的单拍脉冲（给顶层切换状态用）
    output wire        busy,       // 1 = 正在转场（顶层据此拒绝新的触发，防止重入）
    output wire [7:0]  prog        // 当前进度 0~255（本段内），仅供观察/调试
);

    // rgb_old / trans_sel / H_ACTIVE / V_ACTIVE 是本版保留的接口项，内部未使用，
    // TD 会报 unused input —— 属预期告警，不影响功能，也说明不是漏连。
    // 曾想用 _unused = &{...} 消告警，但整数参数不能位选（H_ACTIVE[10:0] 非法），
    // 且会把全部输入塞进一条多余的组合路径，保持原样更干净。

    // ---- 帧计数器 ----
    // 总帧数 = PHASE_FRAMES(淡出) + WHITE_HOLD(全白) + PHASE_FRAMES(淡入)
    //        = 30 + 4 + 30 = 64 帧 ≈ 1.08s @59.5Hz
    // 曾是 60+8+60 = 128 帧（≈2.15s），叠上读图时间后按键到新图近 3s，
    // 手感太迟钝，故整体减半。CNT_W=8 可计 0~255，够用；加长时注意位宽。
    localparam integer TOTAL_FRAMES = 2 * PHASE_FRAMES + WHITE_HOLD;
    localparam integer CNT_W        = 8;
    localparam [CNT_W-1:0] LAST_FRAME = TOTAL_FRAMES - 1;
    localparam [CNT_W-1:0] PB_START   = PHASE_FRAMES + WHITE_HOLD;  // 后半段起始帧号
    localparam [CNT_W-1:0] LAST_REL   = PHASE_FRAMES - 1;           // 段内最大相对帧号

    // ---- alpha 曲线（两段共用同一条）----
    // smoothstep：a(t) = 255 * (3t² - 2t³)，t = rel/(PHASE_FRAMES-1)
    //   t=0 与 t=1 处导数为 0 → 两头变化极慢，中点最快（约 1.5 倍线性斜率）。
    //
    // 两段共用一条的理由
    //   前半段 a 是白场权重（旧图→白），后半段 a 是新图权重（白→新图），
    //   都是 0→255 递增、方向一致，查同一张表即可；跨段那一拍两端的 a 分别
    //   落在 255 / 0 端点，画面严格连续，不会跳。
    //
    // 为什么不用线性
    //   线性淡化在"刚变白"和"刚出现"两个瞬间是突变的，看着像亮度一直在爬；
    //   smoothstep 两端变缓，纯白画面自然停留约 0.25s，肉眼能分清
    //   "旧图淡没了"和"新图开始浮现"两件事。
    //
    // 表长 = PHASE_FRAMES = 30，索引为段内相对帧号 rel（0~29），
    // 单帧最大跳变 7/255，看不出台阶。
    function [7:0] lv_alpha;
        input [CNT_W-1:0] f;
        begin
            case (f)
                8'd0: lv_alpha = 8'd0;
                8'd1: lv_alpha = 8'd1;
                8'd2: lv_alpha = 8'd3;
                8'd3: lv_alpha = 8'd8;
                8'd4: lv_alpha = 8'd13;
                8'd5: lv_alpha = 8'd20;
                8'd6: lv_alpha = 8'd28;
                8'd7: lv_alpha = 8'd37;
                8'd8: lv_alpha = 8'd48;
                8'd9: lv_alpha = 8'd58;
                8'd10: lv_alpha = 8'd70;
                8'd11: lv_alpha = 8'd82;
                8'd12: lv_alpha = 8'd95;
                8'd13: lv_alpha = 8'd108;
                8'd14: lv_alpha = 8'd121;
                8'd15: lv_alpha = 8'd134;
                8'd16: lv_alpha = 8'd147;
                8'd17: lv_alpha = 8'd160;
                8'd18: lv_alpha = 8'd173;
                8'd19: lv_alpha = 8'd185;
                8'd20: lv_alpha = 8'd197;
                8'd21: lv_alpha = 8'd207;
                8'd22: lv_alpha = 8'd218;
                8'd23: lv_alpha = 8'd227;
                8'd24: lv_alpha = 8'd235;
                8'd25: lv_alpha = 8'd242;
                8'd26: lv_alpha = 8'd247;
                8'd27: lv_alpha = 8'd252;
                8'd28: lv_alpha = 8'd254;
                8'd29: lv_alpha = 8'd255;   // PHASE_FRAMES=30 时最后一帧
                default: lv_alpha = 8'd255;
            endcase
        end
    endfunction

    // 后半段无需单独一张表：两段 alpha 方向一致，共用上面这条 smoothstep 曲线。

    // ---- 帧起始：vs 上升沿（每帧恰好一次，见文件头说明） ----
    reg  vs_d1;
    wire frame_rise = vs & ~vs_d1;

    reg [CNT_W-1:0] frame_cnt;
    reg             active;
    reg             phase_b_r;
    reg             pb_done_r;    // 后半段刚走完的单拍事件（active 落下那一拍）

    // 当前这一拍是否已在转场中（用于 trans_trig 的重入判断）
    wire            already_active = active;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            vs_d1      <= 1'b0;
            frame_cnt  <= {CNT_W{1'b0}};
            active     <= 1'b0;
            phase_b_r  <= 1'b0;
            pb_done_r  <= 1'b0;
        end else begin
            vs_d1     <= vs;
            pb_done_r <= 1'b0;     // 默认清掉，只在走完那一拍置 1

            if (trans_trig) begin
                // 从哪一段起由顶层决定
                //   start_phase_b=0：从 frame_cnt=0 做淡出（旧图→白），
                //                    走到 PB_START 自动转后半段；
                //   start_phase_b=1：直接从后半段起（白→新图），用于首图显示，
                //                    以及"淡出中途新图才就绪"这类没有旧图可淡的情况。
                //
                // 重入保护：already_active=1 表示上一轮未走完又来了触发。
                //   旧代码无条件重装 frame_cnt，后果是转场被打断、画面闪一下；
                //   且 phase_b 被拉回 0，顶层在 FADE_IN 里收不到上升沿，
                //   phase_b_done 永远不来 → 卡在 FADE_IN，不再接受新转场。
                //   现在转场中的触发直接忽略；"再切一张"的请求由 sd_card_bmp 的
                //   next_req_pending 锁着，转场结束后补发，不会丢按键。
                if (!already_active) begin
                    active    <= 1'b1;
                    phase_b_r <= start_phase_b;
                    frame_cnt <= start_phase_b ? PB_START : {CNT_W{1'b0}};
                end
            end else if (active && frame_rise) begin
                if (frame_cnt >= LAST_FRAME) begin
                    active    <= 1'b0;
                    frame_cnt <= {CNT_W{1'b0}};
                    // 只有"真的走完后半段"才算结束；
                    // 若这一轮根本没经历过后半段（极短触发），不报完成。
                    if (phase_b_r) begin
                        pb_done_r <= 1'b1;
                        // 必须把 phase_b_r 清回 0：旧代码不清，转场结束后
                        // phase_b 一直为高，顶层等它的下降沿判断"后半段走完"就
                        // 永远等不到，只能靠 3s 超时兜底，每次切图多卡 3 秒。
                        phase_b_r <= 1'b0;
                    end
                end else begin
                    frame_cnt <= frame_cnt + {{(CNT_W-1){1'b0}}, 1'b1};
                    // 跨过半段边界 → 进入后半段。
                    // 顶层用 phase_b 的上升沿判断这次跨段（见 top_signage.v）。
                    if ((frame_cnt + {{(CNT_W-1){1'b0}}, 1'b1}) == PB_START)
                        phase_b_r <= 1'b1;
                end
            end
        end
    end

    assign trans_active  = active;
    assign phase_b       = phase_b_r;
    assign phase_b_done  = pb_done_r;
    assign busy          = active;
    // 顶层用 phase_b 的上升沿判断"淡出段已经结束、该切缓冲了"
    // （在顶层检测，避免再加一个端口）

    // ---- 段内相对帧号（带钳位，用于中间的"全白停留"） ----
    // 前半段：rel = frame_cnt，超过 PHASE_FRAMES-1 后钳在 29（= LAST_REL）
    //         → alpha 恒为 255 = 纯白，这就是 WHITE_HOLD 停留段
    // 后半段：rel = frame_cnt - PB_START，同样钳在 29
    wire [CNT_W-1:0] rel_a = (frame_cnt >= PHASE_FRAMES) ? LAST_REL : frame_cnt;
    wire [CNT_W-1:0] rel_b = (frame_cnt >= (PB_START + PHASE_FRAMES))
                             ? LAST_REL
                             : (frame_cnt - PB_START[CNT_W-1:0]);
    wire [CNT_W-1:0] rel   = phase_b_r ? rel_b : rel_a;

    // ---- alpha 曲线 ----
    // 两段共用 lv_alpha，只是"这个 a 代表谁"不同，由下面的权重分配区分：
    //   前半段 lv_alpha(rel)：返回值 = 白场权重（0→255，旧图被吃掉）
    //   后半段 lv_alpha(rel)：返回值 = 新图权重（0→255，新图实体化）
    wire [7:0] a_val = lv_alpha(rel);
    wire [7:0] a_inv = 8'd255 - a_val;      // 与 a_val 互补的权重

    // prog 对外统一表示"本段进度"：前半段是白场权重，后半段是新图权重，
    // 两者都是从 0 涨到 255，所以状态机用同一个阈值判断"本段走完"即可。
    assign prog = a_val;

    // -------------------------------------------------------------------
    // 混合器（2 拍延迟）：out = rgb_new*gain + 255*white
    //   两权重之和恒为 255，结果不超过 255*255，取高 8 位即可。
    //
    // 为什么只有 gain 和 white、没有 keep 通道
    //   本模块只有一路像素输入，旧图和新图都从 rgb_new 进来（读哪块 buffer
    //   由顶层 rd_buf_sel 决定），所以两个阶段的"图像权重"都必须挂在 gain 上：
    //     前半段（读旧 buffer）：gain = 255-a（旧图 255→0），white = a（0→255）
    //     后半段（读新 buffer）：gain = a（新图 0→255），white = 255-a（255→0）
    //   曾把前半段的旧图权重写在 keep 上，而 keep 乘的是 rgb_old（顶层接
    //   24'd0），gain=0 使旧图整段丢失，输出退化成 255*white = "纯黑→纯白"，
    //   看起来是闪一下黑，不像"旧图慢慢褪成白"。
    //   rgb_old 端口保留但不使用：单读架构下它永远是 0。
    // -------------------------------------------------------------------
    reg [7:0]  gain_r;      // 图像权重：前半段=旧图，后半段=新图
    reg [7:0]  white_r;     // 白场权重，恒与 gain_r 互补（和为 255）

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            gain_r  <= 8'd255;
            white_r <= 8'd0;
        end else begin
            if (phase_b_r) begin
                // 后半段（白→新图）
                gain_r  <= a_val;       // 新图 0 → 255，实体化
                white_r <= a_inv;       // 白场 255 → 0，退去
            end else begin
                // 前半段（旧图→白）
                gain_r  <= a_inv;       // 旧图 255 → 0，褪色
                white_r <= a_val;       // 白场 0 → 255，覆盖上来
            end
        end
    end

    // 转场结束后直接把图像原样送出
    wire       run_now = active;
    wire [7:0] g_e = run_now ? gain_r  : 8'd255;
    wire [7:0] w_e = run_now ? white_r : 8'd0;

    // 两个 8x8 乘 + 加法。权重和为 255，最大值 255*255 = 65025，16 位都够；
    // 乘积用显式 16 位 wire 承接，避免"乘法结果宽度按操作数取 8 位被截断"
    // 这类工具相关的坑（有些综合器在上下文推断上不按预期扩展）。
    wire [15:0] r_p = rgb_new[23:16] * g_e;
    wire [15:0] g_p = rgb_new[15:8]  * g_e;
    wire [15:0] b_p = rgb_new[7:0]   * g_e;
    wire [15:0] w_p = 8'd255         * w_e;

    reg [16:0] r_acc, g_acc, b_acc;
    reg [23:0] rgb_mix;
    reg [1:0]  de_shf;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            r_acc   <= 17'd0;
            g_acc   <= 17'd0;
            b_acc   <= 17'd0;
            rgb_mix <= 24'd0;
            de_shf  <= 2'd0;
        end else begin
            r_acc <= {1'b0, r_p} + {1'b0, w_p};
            g_acc <= {1'b0, g_p} + {1'b0, w_p};
            b_acc <= {1'b0, b_p} + {1'b0, w_p};

            rgb_mix <= { r_acc[15:8], g_acc[15:8], b_acc[15:8] };

            de_shf <= { de_shf[0], de };
        end
    end

    assign rgb_out = rgb_mix;
    assign de_out  = de_shf[1];   // 延迟 2 拍，与 rgb_out 对齐

endmodule
