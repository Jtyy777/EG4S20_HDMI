// ============================================================================
// audio_spectrum —— 音频频谱分析（8 带 Goertzel 单频点 DFT）
// ----------------------------------------------------------------------------
// 一句话：对 PCM 流做 8 个频点的 Goertzel 递推，每个窗口给出 8 个频带的
//         能量，换算成 8 根柱子的高度。这是真正的频域分解，不是数周期。
//
// 与上一版的根本区别
//   上一版在窗口内数方波上升沿的个数，等于测基频：只要输入是单音方波，
//   8 根柱会随音调阶梯变化，看着像频谱；但换真实音乐（多频叠加）就失效，
//   因为过零率不等于频谱能量。本版改成对每个频带单独做单频点 DFT，
//   各带互相独立，宽带信号也能正确分配能量。
//
// 一、为什么用 Goertzel 而不是 FFT
//   只需要 8 个频点（不是整条谱）。Goertzel 每带每采样只做 1 次乘加，
//   单频点代价 O(N)；8 点 FFT 要 N logN 且需要蝶形旋转因子表。
//   在 EG4S20 这种 DSP 只有 29 个的器件上，Goertzel 是唯一省资源的做法。
//
//   递推式（标准形式）：
//       s[n] = x[n] + coeff·s[n-1] - s[n-2]
//       coeff = 2·cos(2π·f/fs)
//   窗口末 |X(f)|² = s1² + s2² - coeff·s1·s2。
//
// 二、8 个频带选在哪儿
//   有意对齐 hdmi_audio_tone_i2s_64fs 的 8 个测试音（C4~C5 一个八度）：
//       261.63 / 293.66 / 329.63 / 349.23 / 392.00 / 440.00 / 493.88 / 523.25 Hz
//   这样一个音符点亮一根柱，演示时"音阶走到哪、白帽就跟到哪"，直观。
//   如果要给真实音乐做频谱，把 coef_lut 换成倍频程中心频率
//   （125/250/500/1k/2k/4k/8k/16k）即可，系数按同一公式重算，别手改。
//
// 三、定点与位宽（这一段决定了会不会溢出，改参数前先读）
//   · 输入取 audio_data[23:12]，即 12bit 有符号。测试音 AMP = 2e6，
//     右移 12 位后约 ±488，没有浪费也没有削顶；
//   · 系数用 Q14（coeff×16384）。为什么不是 Q15：2cos 最大接近 2，
//     ×32768 会顶到 65504，16bit 装不下，必须留一位；
//   · 递推状态 s1/s2 用 32bit 有符号。窗口内 |s| 的稳态幅度
//     ≈ |X|/sin(ω)，261Hz 时 sin(ω)=0.0342，放大 29 倍，实测峰值约 2^25，
//     32bit 还剩 7 位余量；再加一级饱和钳位兜住极端情况；
//   · 每窗口结束必须把 s1/s2 清零。Goertzel 的极点就在单位圆上，
//     系数量化会让极点略微跑到圆外（Q14 下约每 4096 点增长 3%~6%），
//     不清零就会跨窗口累积，几十个窗口后状态饱和，柱子全部贴顶。
//     这一条踩过坑，别省。
//
// 四、时域复用（为什么只用 1 个乘法器）
//   48kHz 采样、25MHz 时钟 → 每个采样间隔 520 拍。8 个频带串行处理，
//   每带「1 拍算乘、1 拍写回」共 16 拍，占不到 4%。窗口末还要算 8 次
//   幅度和 8 次柱高，另有 ~30 拍。所以全程只实例化 1 个 32×16 乘法器，
//   它在 18×18 的 DSP 上按 2 块拼接，故本模块共占 2 个 DSP。
//   窗口末的 ×1.5 也刻意写成移位加法（见第九节），否则 TD 会再吃掉
//   1 个 DSP；本模块实测从 3 个降到 2 个靠的就是这一手。
//   EG4S20 只有 29 个 DSP，这里每省一个都是净赚。
//
// 五、从能量到柱高（为什么不需要标定）
//   · 幅度代理用 max(|s1|,|s2|)。严格式是 sqrt(s1²+s2²-coeff·s1·s2)，
//     但平方会多出 24 次乘法、48bit 中间量；而 |X| = B·sin(ω)、B 就是
//     递推状态的幅度，max 与 B 的偏差只在 1~1.41 倍之间（≤3dB，取决于
//     窗口末相位），对柱状图完全够用。这是有意的简化，不是笔误。
//   · 幅度转 logq（1/4 倍频程分辨率，1 单位 ≈ 1.5dB）：
//       找最高有效位 k，再把 k 以下 3 位当尾数 → logq = k×4 + 尾数
//   · 自适应归一化：本窗口最强的带记为 logq_max，其余按差值往下压。
//     这样不依赖信号绝对幅度，换音量不用重新标定：
//       h = MAX_H - (logq_max - logq_i) × 1.5   （1.5 用 ((x<<1)+x)>>1 实现）
//     主带必得 MAX_H，比主带低 20 档（30dB）的带落到 28 像素。
//   · 静音判定：logq_max < LOG_MIN 时认为没有信号，柱高按峰值保持衰减，
//     否则 8 个全零状态会让 diff=0、8 根柱一起顶到最高，变成假谱。
//
// 六、峰值保持
//   每窗口新值只升不降，下降时按 DECAY 像素/窗口衰减，避免柱子闪烁。
//   85ms × 6 ≈ 0.5s 落回地面，与测试音 0.5s 换一个音符的节奏吻合。
//
// 七、输出语义（与 spectrum_overlay 的约定）
//   bar_h       每带 8bit，0 ~ BAR_MAX_H，扁平化 [NUM_BANDS*8-1:0]
//   active_col  本窗口能量最强的那一带 —— overlay 在它顶上画白帽。
//               注意语义已变：上一版是"轮询指针"，本版是"主导频带"。
//   frame_tick  每窗口一个单拍脉冲（顶层当前未使用，留作调试）
//
// 八、已知边界
//   · 频率分辨率 = fs/WIN = 11.7Hz，主瓣 23.4Hz。E4(329.63) 与
//     F4(349.23) 只差 19.6Hz，是全音阶里最挤的一对，两带会互相泄漏
//     （实测主带仍能正确胜出，只是差值偏小）。要更干净就把 WIN 提到
//     8192，代价是窗口 170ms、一音符只能出 3 次结果；
//   · 窗口跨越音符切换点时，前后两个音的频率都会进窗，柱高有一次
//     过渡态（约 1/6 的窗口会碰上）。不处理是刻意的：做窗函数会让
//     资源翻倍，而视觉上一窗口的过渡完全看不出来。
// ============================================================================

module audio_spectrum #(
    parameter integer NUM_BANDS   = 8,          // 频带数（coef_lut 固定 8 项）
    parameter integer BAR_MAX_H   = 60,         // 柱高上限（像素）
    parameter integer WIN_SAMPLES = 4096,       // 每窗口采样数：85.3ms @48kHz
    parameter integer DECAY       = 6           // 峰值保持的每窗口衰减（像素）
)(
    input  wire        clk,          // video_clk 25MHz（audio_valid 在该域）
    input  wire        rst,
    input  wire        audio_valid,  // 每个 PCM 样本一拍的使能（48kHz）
    input  wire [23:0] audio_data,   // 左声道 PCM（24bit 补码）

    // 每根柱的高度（0 ~ BAR_MAX_H），每根 8bit，扁平化为 [NUM_BANDS*8-1:0]
    output wire [NUM_BANDS*8-1:0] bar_h,
    // 能量最强的频带（0 ~ NUM_BANDS-1），供 overlay 画白色顶帽
    output reg  [2:0]  active_col,
    // 每窗口一拍的刷新脉冲（顶层当前未使用，保留给调试/示波器观察）
    output reg         frame_tick
);

    // ========================================================================
    // 0. 常量
    // ========================================================================
    localparam integer S_BITS      = 32;    // 递推状态字长（见头注三）
    localparam integer COEF_FRAC   = 14;    // 系数小数位（Q14）
    localparam [7:0]   MAX_H       = BAR_MAX_H - 2;  // 58：给 overlay 的 60 留余量
    localparam [7:0]   DECAY_V     = DECAY;          // 落成定宽，避免 8bit 比 32bit 整数
    localparam [7:0]   LOG_MIN     = 8'd32;          // 静音门限（约 |s|<2^8）
    localparam [31:0]  LAST_SAMP   = WIN_SAMPLES - 1;

    // 状态机
    localparam [2:0] ST_IDLE   = 3'd0;   // 等下一个音频采样
    localparam [2:0] ST_CALC   = 3'd1;   // 发乘法
    localparam [2:0] ST_WR     = 3'd2;   // 用乘法结果更新 s1/s2
    localparam [2:0] ST_LOGP   = 3'd3;   // 窗口末：逐带求 logq
    localparam [2:0] ST_HEIGHT = 3'd4;   // 窗口末：逐带算柱高
    localparam [2:0] ST_END    = 3'd5;   // 收尾：打拍、清状态

    // ========================================================================
    // 1. Goertzel 系数表（Q14）
    //    重算方法：coef = round(2·cos(2π·f/48kHz) × 16384)
    //    这 8 个值对应 8 个测试音，改频带必须整表重算，不要单独改一个。
    // ========================================================================
    function [15:0] coef_lut;
        input [2:0] idx;
        begin
            case (idx)
                3'd0:    coef_lut = 16'd32749;   // 261.63 Hz  C4
                3'd1:    coef_lut = 16'd32744;   // 293.66 Hz  D4
                3'd2:    coef_lut = 16'd32738;   // 329.63 Hz  E4
                3'd3:    coef_lut = 16'd32734;   // 349.23 Hz  F4
                3'd4:    coef_lut = 16'd32725;   // 392.00 Hz  G4
                3'd5:    coef_lut = 16'd32714;   // 440.00 Hz  A4
                3'd6:    coef_lut = 16'd32700;   // 493.88 Hz  B4
                default: coef_lut = 16'd32691;   // 523.25 Hz  C5
            endcase
        end
    endfunction

    // ========================================================================
    // 2. 幅度 → logq（1/4 倍频程量化）
    //    先用优先编码器找最高有效位 k，再把 k 以下 3 位当尾数，
    //    归一化到 4bit（8~15）后取低 3 位作小数。
    //    输入 0 返回 0 —— 调用方用 LOG_MIN 判静音，不要在这里特殊处理。
    // ========================================================================
    function [7:0] logq_of;
        input [31:0] v;
        integer  i;
        reg [5:0] k;
        reg [31:0] m;
        begin
            k = 6'd0;
            for (i = 0; i < 32; i = i + 1)
                if (v[i])
                    k = i;                   // i ≤ 31，赋给 6bit 自然截断；
                                                 // 不写 i[5:0]，部分工具会对 integer
                                                 // 位选报 warning

            // 归一化：右移到 4bit 宽（m ∈ 8~15），移位量 = k-3
            m = (k > 6'd3) ? (v >> (k - 6'd3)) : v;
            logq_of = {k, 2'b00} + {5'd0, m[2:0]};   // k*4 + 尾数(0~7)
        end
    endfunction

    // ========================================================================
    // 3. 递推状态：8 组 s1/s2
    //    刻意写成 8 个独立寄存器而不是数组：数组变量索引在部分综合器上
    //    会被推断成 RAM（需要读端口竞争），这里要的是 8 个并行寄存器堆。
    // ========================================================================
    reg signed [S_BITS-1:0] s1_0, s1_1, s1_2, s1_3, s1_4, s1_5, s1_6, s1_7;
    reg signed [S_BITS-1:0] s2_0, s2_1, s2_2, s2_3, s2_4, s2_5, s2_6, s2_7;

    // 每带的 logq 暂存（窗口末两遍计算用）
    reg [7:0] logq_0, logq_1, logq_2, logq_3, logq_4, logq_5, logq_6, logq_7;

    reg [2:0]  st;
    reg [2:0]  bi;                  // 当前处理的频带
    reg [12:0] samp_cnt;            // 0 ~ WIN_SAMPLES-1
    reg signed [11:0] x_lat;        // 锁存的 12bit 样本
    reg        av_d;                // audio_valid 上一拍，用来取上升沿

    wire av_rise = audio_valid & ~av_d;

    // ========================================================================
    // 4. 当前带的状态读出（case 展开，避免变量索引）
    // ========================================================================
    reg signed [S_BITS-1:0] s1_cur;
    reg signed [S_BITS-1:0] s2_cur;
    reg [15:0]              cf_cur;

    always @(*) begin
        case (bi)
            3'd0: begin s1_cur = s1_0; s2_cur = s2_0; end
            3'd1: begin s1_cur = s1_1; s2_cur = s2_1; end
            3'd2: begin s1_cur = s1_2; s2_cur = s2_2; end
            3'd3: begin s1_cur = s1_3; s2_cur = s2_3; end
            3'd4: begin s1_cur = s1_4; s2_cur = s2_4; end
            3'd5: begin s1_cur = s1_5; s2_cur = s2_5; end
            3'd6: begin s1_cur = s1_6; s2_cur = s2_6; end
            default: begin s1_cur = s1_7; s2_cur = s2_7; end
        endcase
        cf_cur = coef_lut(bi);
    end

    // ========================================================================
    // 5. 唯一的一个乘法器 + 饱和加法
    //    位宽意图：32bit 有符号 × 16bit（系数恒正）→ 48bit
    //    综合器会拆成 2 个 18×18 的 DSP；只用 1 组，因为是时间复用的。
    //    乘法结果在 ST_CALC 拍寄存一次再用：ST_CALC → ST_WR 正好两拍，
    //    这样"寄存器 → 乘法器 → 寄存器"是标准 DSP 时序，不会把一条长
    //    组合路径（乘法 + 加法 + 饱和判断）串起来。
    // ========================================================================
    wire signed [47:0] mul_p = $signed(s1_cur) * $signed({1'b0, cf_cur});

    reg signed [47:0] mul_r;
    always @(posedge clk or posedge rst) begin
        if (rst)
            mul_r <= 48'sd0;
        else if (st == ST_CALC)
            mul_r <= mul_p;
    end

    wire signed [35:0] mul_sh = mul_r >>> COEF_FRAC;          // Q14 还原

    // s[n] = x[n] + coeff·s[n-1] - s[n-2]，先算到 36bit 再判断溢出
    wire signed [35:0] acc = {{24{x_lat[11]}}, x_lat}
                           + mul_sh
                           - {{4{s2_cur[31]}}, s2_cur};

    // 饱和到 32bit：正常情况永远不触发（峰值约 2^25），是兜底
    wire signed [S_BITS-1:0] s_new =
        (acc >  36'sd2147483647) ?  32'sh7FFFFFFF :
        (acc < -36'sd2147483648) ?  32'sh80000000 : acc[S_BITS-1:0];

    // ========================================================================
    // 6. 主状态机
    // ========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st       <= ST_IDLE;
            bi       <= 3'd0;
            samp_cnt <= 13'd0;
            x_lat    <= 12'sd0;
            av_d     <= 1'b0;
        end else begin
            av_d <= audio_valid;

            case (st)
                // ------------------------------------------------------
                // 等下一个有效样本。用上升沿触发，避免 audio_valid
                // 若为电平（多拍有效）时被重复处理成多个样本。
                // ------------------------------------------------------
                ST_IDLE: begin
                    if (av_rise) begin
                        x_lat <= audio_data[23:12];   // 24bit → 12bit 有符号
                        bi    <= 3'd0;
                        st    <= ST_CALC;
                    end
                end

                // 乘法结果需要寄存一拍才能用（DSP 输出寄存器）
                ST_CALC: st <= ST_WR;

                // 写回本带状态，处理下一带
                ST_WR: begin
                    if (bi == 3'd7) begin
                        bi <= 3'd0;
                        // 本采样 8 带都走完了：推进窗口计数
                        if (samp_cnt == LAST_SAMP[12:0]) begin
                            samp_cnt <= 13'd0;
                            st       <= ST_LOGP;     // 进入窗口末计算
                        end else begin
                            samp_cnt <= samp_cnt + 13'd1;
                            st       <= ST_IDLE;
                        end
                    end else begin
                        bi <= bi + 3'd1;
                        st <= ST_CALC;
                    end
                end

                // ------------------------------------------------------
                // 窗口末第一遍：逐带求 logq，同时找最大值和 argmax
                // 注意 bi 必须同时归零：只改 st 不改 bi，第二遍会从 7 开始，
                // 一拍就判结束，只有最后一根柱子被更新。
                // ------------------------------------------------------
                ST_LOGP: begin
                    if (bi == 3'd7) begin
                        bi <= 3'd0;
                        st <= ST_HEIGHT;
                    end else begin
                        bi <= bi + 3'd1;
                    end
                end

                // ------------------------------------------------------
                // 窗口末第二遍：逐带算柱高（含峰值保持），写完收尾
                // ------------------------------------------------------
                ST_HEIGHT: begin
                    if (bi == 3'd7) begin
                        bi <= 3'd0;
                        st <= ST_END;
                    end else begin
                        bi <= bi + 3'd1;
                    end
                end

                ST_END: st <= ST_IDLE;

                default: st <= ST_IDLE;
            endcase
        end
    end

    // ========================================================================
    // 7. 递推状态写回
    //    只有 ST_WR 这一拍写，且只写 bi 指向的那一带，其余保持。
    //    窗口结束（进入 ST_END）时 8 带一起清零 —— 见头注三，不清会累积。
    // ========================================================================
    wire clr_state = (st == ST_END);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            s1_0 <= 32'sd0; s1_1 <= 32'sd0; s1_2 <= 32'sd0; s1_3 <= 32'sd0;
            s1_4 <= 32'sd0; s1_5 <= 32'sd0; s1_6 <= 32'sd0; s1_7 <= 32'sd0;
            s2_0 <= 32'sd0; s2_1 <= 32'sd0; s2_2 <= 32'sd0; s2_3 <= 32'sd0;
            s2_4 <= 32'sd0; s2_5 <= 32'sd0; s2_6 <= 32'sd0; s2_7 <= 32'sd0;
        end else if (clr_state) begin
            s1_0 <= 32'sd0; s1_1 <= 32'sd0; s1_2 <= 32'sd0; s1_3 <= 32'sd0;
            s1_4 <= 32'sd0; s1_5 <= 32'sd0; s1_6 <= 32'sd0; s1_7 <= 32'sd0;
            s2_0 <= 32'sd0; s2_1 <= 32'sd0; s2_2 <= 32'sd0; s2_3 <= 32'sd0;
            s2_4 <= 32'sd0; s2_5 <= 32'sd0; s2_6 <= 32'sd0; s2_7 <= 32'sd0;
        end else if (st == ST_WR) begin
            case (bi)
                3'd0: begin s1_0 <= s_new; s2_0 <= s1_0; end
                3'd1: begin s1_1 <= s_new; s2_1 <= s1_1; end
                3'd2: begin s1_2 <= s_new; s2_2 <= s1_2; end
                3'd3: begin s1_3 <= s_new; s2_3 <= s1_3; end
                3'd4: begin s1_4 <= s_new; s2_4 <= s1_4; end
                3'd5: begin s1_5 <= s_new; s2_5 <= s1_5; end
                3'd6: begin s1_6 <= s_new; s2_6 <= s1_6; end
                default: begin s1_7 <= s_new; s2_7 <= s1_7; end
            endcase
        end
    end

    // ========================================================================
    // 8. 窗口末第一遍：算 logq / 找最大值 / 找 argmax
    //    幅度取 max(|s1|,|s2|)（见头注五，这是有意简化）
    // ========================================================================
    // 当前带的两个状态绝对值
    wire signed [S_BITS:0] s1x = {s1_cur[S_BITS-1], s1_cur};
    wire signed [S_BITS:0] s2x = {s2_cur[S_BITS-1], s2_cur};
    wire [S_BITS:0] abs1 = s1x[S_BITS] ? (~s1x + 1'b1) : s1x;
    wire [S_BITS:0] abs2 = s2x[S_BITS] ? (~s2x + 1'b1) : s2x;
    wire [S_BITS:0] mag  = (abs1 > abs2) ? abs1 : abs2;
    wire [7:0]      lq   = logq_of(mag[S_BITS-1:0]);

    reg [7:0] logq_max;
    reg [2:0] col_max;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            logq_0 <= 8'd0; logq_1 <= 8'd0; logq_2 <= 8'd0; logq_3 <= 8'd0;
            logq_4 <= 8'd0; logq_5 <= 8'd0; logq_6 <= 8'd0; logq_7 <= 8'd0;
            logq_max <= 8'd0;
            col_max  <= 3'd0;
        end else if (st == ST_LOGP) begin
            // 每一拍写一个带，同时更新最大值
            case (bi)
                3'd0: logq_0 <= lq;
                3'd1: logq_1 <= lq;
                3'd2: logq_2 <= lq;
                3'd3: logq_3 <= lq;
                3'd4: logq_4 <= lq;
                3'd5: logq_5 <= lq;
                3'd6: logq_6 <= lq;
                default: logq_7 <= lq;
            endcase

            // 注意用 bi==0 复位比较器，保证每窗口重新选最大
            if (bi == 3'd0 || lq > logq_max) begin
                logq_max <= lq;
                col_max  <= bi;
            end
        end
    end

    // 静音：本窗口最强带都没过门限
    wire muted = (logq_max < LOG_MIN);

    // ========================================================================
    // 9. 窗口末第二遍：柱高 = MAX_H - (减量)×1.5，再叠加峰值保持
    //    减量 = logq_max - logq_i（钳到 0~127，避免乘法溢出）
    //    ×1.5 = ((x<<1)+x)>>1，纯移位加法，不用除法器（也不占 DSP）
    // ========================================================================
    reg [7:0] lq_cur;
    always @(*) begin
        case (bi)
            3'd0:    lq_cur = logq_0;
            3'd1:    lq_cur = logq_1;
            3'd2:    lq_cur = logq_2;
            3'd3:    lq_cur = logq_3;
            3'd4:    lq_cur = logq_4;
            3'd5:    lq_cur = logq_5;
            3'd6:    lq_cur = logq_6;
            default: lq_cur = logq_7;
        endcase
    end

    wire [7:0]  dlog  = (logq_max > lq_cur) ? (logq_max - lq_cur) : 8'd0;
    wire [7:0]  dclip = (dlog > 8'd127) ? 8'd127 : dlog;
    // 不要写成 "dclip * 3"：TD 会为这个常数乘单独分配 1 个 DSP，
    // 本模块实测因此从 2 个 DSP 涨到 3 个。移位加法的结果完全等价
    // （已穷举 0~255 验证），且综合器不会把它还原成乘法器。
    wire [9:0]  dd    = ({1'b0, dclip, 1'b0}) + {2'b00, dclip};
    wire [9:0]  drop  = dd >> 1;                       // 3x/2 ≤ 190
    wire [7:0]  h_raw = (drop >= {2'b00, MAX_H}) ? 8'd0 : (MAX_H - drop[7:0]);

    // ========================================================================
    // 10. 8 根柱子寄存器 + 峰值保持
    //     新值更高就直接跟；更低则按 DECAY_V 缓慢下落；静音时不接受新值
    //     （只做衰减），否则全零状态会被归一化成满高。
    // ========================================================================
    genvar g;
    generate
        for (g = 0; g < NUM_BANDS; g = g + 1) begin : gen_band
            localparam [2:0] COL_IDX = g;

            reg [7:0] bar_r;

            // 峰值保持：上升即时，下降限速
            wire [7:0] fall   = (bar_r > DECAY_V) ? (bar_r - DECAY_V) : 8'd0;
            wire [7:0] next_h = muted ? fall
                                      : ((h_raw >= bar_r) ? h_raw : fall);

            always @(posedge clk or posedge rst) begin
                if (rst)
                    bar_r <= 8'd0;
                else if (st == ST_HEIGHT && bi == COL_IDX)
                    bar_r <= next_h;
            end

            assign bar_h[g*8 +: 8] = bar_r;
        end
    endgenerate

    // ========================================================================
    // 11. 收尾：argmax 送出、frame_tick 单拍
    // ========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            active_col <= 3'd0;
            frame_tick <= 1'b0;
        end else begin
            frame_tick <= 1'b0;
            if (st == ST_END) begin
                frame_tick <= 1'b1;
                active_col <= col_max;
            end
        end
    end

endmodule
