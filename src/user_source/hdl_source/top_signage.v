// ============================================================================
// top_signage —— 数字标牌信息发布终端（赛道一：HDMI 多媒体播放系统）
// ----------------------------------------------------------------------------
//   本版以 lab_ex4_tf 和 lab_ex5_i2s 为原型进行增添
//   完成了基础要求
//   1. 对存入 SCSD 卡中的 4 张 24 位无压缩 640*480 像素的 BMP 图片进行手动和自动切换
//   2. 4×4 矩阵键盘替代样例的 2 个独立按键
//   完成了扩展要求
//   1. OSD：下方滚动字幕 + 图片计数 + 参数实时显示
//   2. 在手动切图和自动切图进行图片转场：旧图 → 白场 → 新图
//   3. 
//   4. 参数实时调节：对亮度(L)和对比度(C)进行按键 + OSD 数值反馈
//   5. 音频可视化：8 段频谱柱状图叠加
//
// 视频流水线
//   写侧（sd_card_clk 100MHz）：
//   TF 读图 → bmp_read → img_scaler → SDRAM
//   显示侧（video_clk 25MHz）：
//   SDRAM 双缓冲 → 转场 → 亮度/对比度 →
//   频谱叠加 → OSD 叠加 → RGB→AXIS → HDMI
//
// 音频链路
//   PLL_HDMI_AUDIO(12.29MHz MCLK) → I2S 测试音 → I2S_receiver →
//   audio_arc_calculate(ARC) → HDMI 发射核；audio_spectrum 同时算频谱
//
// 复用的平台模块
//   sys_pll / video_pll / PLL_HDMI_AUDIO / sdram / sd_card_bmp / bmp_read /
//   sd_card_top / frame_read_write / video_timing_data / video_delay /
//   hdmi_1_4b_transmitter_core_wrapper / hdmi_phy_wrapper / I2S_receiver /
//   audio_arc_calculate / video_rgb_to_axis_640x480 / startup_pulse
// ============================================================================

module top_signage (
    input                       clk,            // 50MHz 晶振（核心板 R7）
    input                       rst_n,          // 复位键（核心板 E3，低有效）

    // 4x4 矩阵键盘
    output [3:0]                key_row,        // Key_Row[3:0]（输出扫描）
    input  [3:0]                key_col,        // Key_Col[3:0]（输入读取）

    // 数码管（核心板，显示状态码/参数）
    output [7:0]                seg_data,
    output [3:0]                seg_sel,

    // 拨码开关（核心板，可选扩展输入）
    input  [7:0]                sw_in,          // SW[7:0]

    // HDMI TMDS 输出
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // TF 卡 SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso
);

    // ==================== 参数定义 ====================
    parameter MEM_DATA_BITS = 32;
    parameter ADDR_BITS     = 21;
    parameter BUSRT_BITS    = 10;
    parameter FRAME_PIXELS  = 24'd307200;   // 307200=640*480
    parameter BUF0_ADDR     = 24'd0;
    parameter BUF1_ADDR     = FRAME_PIXELS;
    // 第三块缓冲：预读（prefetch）专用。
    //   SDRAM 是 2M words（8MB），三块占 43.9%，余 4.48MB，容量无压力。
    //   frame_read_write 的 read_addr_2 / write_addr_2 通道本来就有，只是过去
    //   被接成 24'd0，所以 2 位索引其实只用到 0/1 两档。
    //   有 BUF2 之后，写的目标永远不等于"当前显示"和"转场正在淡出的旧图"，
    //   于是读卡可以提前在后台做完，切图时零等待。
    parameter BUF2_ADDR     = FRAME_PIXELS * 2;

    // ==================== 时钟与复位 ====================
    wire sd_card_clk;
    wire ext_mem_clk;
    wire ext_mem_clk_sft;
    wire video_clk;
    wire hdmi_5x_clk;

    wire audio_pll_lock;
    wire audio_mclk;

    wire Sdr_init_done;
    wire Sdr_init_ref_vld;
    wire Sdr_busy;

    // 统一复位：TF 图像链路 + HDMI 音频链路
    wire rst_all;
    assign rst_all = ~rst_n | ~audio_pll_lock;

    // ==================== PLL ====================
    sys_pll sys_pll_m0(
        .refclk     (clk),
        .clk0_out   (sd_card_clk),
        .clk1_out   (ext_mem_clk),
        .clk2_out   (ext_mem_clk_sft),
        .reset      (1'b0)
    );

    video_pll video_pll_m0(
        .refclk     (clk),
        .clk0_out   (video_clk),
        .clk1_out   (hdmi_5x_clk),
        .reset      (1'b0)
    );

    PLL_HDMI_AUDIO u_audio_pll(
        .refclk     (clk),
        .reset      (1'b0),
        .extlock    (audio_pll_lock),
        .clk0_out   (),
        .clk1_out   (),
        .clk2_out   (audio_mclk)
    );

    // ==================== 交互控制 ====================
    // 矩阵键盘扫描
    wire [15:0] key_press;
    wire [15:0] key_hold;       // 按住电平（供长按连发使用）
    keypad_scanner #(
        .CLK_FREQ_HZ (50_000_000)
    ) u_keypad (
        .clk            (clk),
        .rst            (rst_all),
        .key_row        (key_row),
        .key_col        (key_col),
        .key_press_pulse(key_press),
        .key_hold       (key_hold)
    );

    // 参数控制器（按键 -> 参数）
    wire        ctrl_key_next;
    wire        ctrl_key_auto;
    wire        ctrl_osd_show;      // KEY7 翻转的 OSD 信息层显隐（clk 域电平）
    wire [7:0]  ctrl_bright;
    wire [9:0]  ctrl_contrast_q8;
    param_controller u_param_ctrl (
        .clk         (clk),
        .rst         (rst_all),
        .key_press   (key_press),
        .key_hold    (key_hold),
        .key_next    (ctrl_key_next),
        .key_auto    (ctrl_key_auto),
        .osd_show    (ctrl_osd_show),
        .bright      (ctrl_bright),
        .contrast_q8 (ctrl_contrast_q8)
    );

    // ---- 参数跨到 video_clk 域（clk → video_clk，翻转脉冲 + 整组锁存）----
    // 下游两个消费者（brightness_contrast / osd_overlay）都在 video_clk 域，
    // 而参数在 clk 域产生。原来这两条总线是直连的，等于 18 位裸跨异步时钟：
    // 既没有同步器，又让 TD 把它当同步数据通路来卡 —— 预算只有 20ns，
    // 而 OSD 那边算对比度十进制要 24 级组合，于是常年挂在 -5.7 ~ -7.6ns。
    // 那本来是「按键才变一次」的配置总线，不该按数据通路算，但直连的话
    // 工具只能这么判。这里补上握手，把这条路径变成真正的准静态跨域。
    // 时序约束见 timing.sdc 第 5 节（对这对时钟声明 false path）。
    wire [7:0]  ctrl_bright_v;
    wire [9:0]  ctrl_contrast_v;
    param_cdc u_param_cdc (
        .clk_src      (clk),
        .rst_src      (rst_all),
        .bright_src   (ctrl_bright),
        .contrast_src (ctrl_contrast_q8),
        .clk_dst      (video_clk),
        .rst_dst      (rst_all),
        .bright_dst   (ctrl_bright_v),
        .contrast_dst (ctrl_contrast_v)
    );

    // ---- OSD 信息层显隐：clk → video_clk，单 bit 两级同步 ----
    // 为什么不用 param_cdc：那套翻转脉冲 + 整组锁存是给 18 位参数总线的，
    // 保证"亮度/对比度同一批生效"。这里只有 1 bit 慢变电平（按键才翻一次），
    // 两级同步就够，而且不需要 false path 之外的特殊约束。
    // timing.sdc 第 5b 节已声明 clk → video_clk 的 false path，这条路径不新增违例。
    reg [1:0] osd_show_v;
    always @(posedge video_clk or posedge rst_all) begin
        if (rst_all)
            osd_show_v <= 2'b00;
        else
            osd_show_v <= { osd_show_v[0], ctrl_osd_show };
    end
    wire osd_on = osd_show_v[1];    // 1 = 显示 L:/C: 与 T/W/L，0 = 隐藏

    // ==================== KEY0 直连 ====================
    wire key_next_rep;
    assign key_next_rep = ctrl_key_next;

    // ==================== TF 读图与 SDRAM 双缓冲 ====================
    wire [3:0]  state_code;
    wire [2:0]  img_count;       // 已发现 BMP 图片数
    wire [1:0]  img_index;       // 当前显示图片编号
    wire [3:0]  load_diag;       // 首图加载诊断
    wire        display_valid;
    wire [1:0]  write_buf_idx;
    wire [1:0]  disp_buf_idx;
    wire        sd_card_write_req;
    wire        sd_card_write_req_ack;
    wire        sd_card_write_en;
    wire [31:0] sd_card_write_data;
    wire        frame_write_finish;

    reg         frame_write_toggle_mem;
    always @(posedge ext_mem_clk or posedge rst_all) begin
        if (rst_all)
            frame_write_toggle_mem <= 1'b0;
        else if (frame_write_finish)
            frame_write_toggle_mem <= ~frame_write_toggle_mem;
    end

    // ---- 转场进行中标志（video_clk → sd_card_clk，两级同步）----
    // 为什么要送给 sd_card_bmp
    // 写图目标永远是"非当前显示"的那块 buffer，而转场 FADE_OUT 段读的恰好
    // 就是这块（旧图）。若在淡出的 1 秒里开始写新图，会边淡边被覆盖，出现
    // 一条推进的撕裂带。所以要告诉 sd_card_bmp："现在别开新一轮写。"
    // 请求不会丢——next_req_pending 是锁存位，转场结束后自动补做。
    wire        trans_busy;         // 1 = 转场进行中（transition_effect.busy）
    reg  [1:0]  trans_busy_sd;      // sd_card_clk 域同步链
    always @(posedge sd_card_clk or posedge rst_all) begin
        if (rst_all)
            trans_busy_sd <= 2'd0;
        else
            trans_busy_sd <= { trans_busy_sd[0], trans_busy };
    end
    wire        disp_busy_sd = trans_busy_sd[1];

    // ---- 切图耗时（sd_card_clk → video_clk：翻转脉冲 + 整组锁存）----
    // sd 域那边只在一次切换彻底走完时才更新三个数，并翻转 sw_upd_tgl。
    // 这里检出翻转边沿后整组锁存，保证 OSD 上的"等待 / 加载 / 总计"是同一批
    // 数据，不会新旧混搭。三个数在翻转前已经稳定了几百毫秒，直接采不会亚稳。
    wire [9:0]  sw_wait_cs;
    wire [9:0]  sw_load_cs;
    wire [9:0]  sw_total_cs;
    wire        sw_upd_tgl;

    reg  [2:0]  sw_tgl_v;
    reg  [9:0]  sw_wait_v;
    reg  [9:0]  sw_load_v;
    reg  [9:0]  sw_total_v;
    wire        sw_upd_pulse = sw_tgl_v[2] ^ sw_tgl_v[1];

    always @(posedge video_clk or posedge rst_all) begin
        if (rst_all) begin
            sw_tgl_v   <= 3'b000;
            sw_wait_v  <= 10'd0;
            sw_load_v  <= 10'd0;
            sw_total_v <= 10'd0;
        end else begin
            sw_tgl_v <= { sw_tgl_v[1:0], sw_upd_tgl };
            if (sw_upd_pulse) begin
                sw_wait_v  <= sw_wait_cs;
                sw_load_v  <= sw_load_cs;
                sw_total_v <= sw_total_cs;
            end
        end
    end

    sd_card_bmp #(
        .CLK_FREQ_HZ       (100_000_000),
        .AUTO_DWELL_SEC    (3),            // 自动轮播：每张图停留 3 秒
        .SCAN_START_SECTOR (32'd0),
        .SCAN_MAX_SECTOR   (32'd131071),
        .SCAN_TARGET_COUNT (3'd4)
    ) sd_card_bmp_m0(
        .clk               (sd_card_clk),
        .rst               (rst_all),
        .key_next          (key_next_rep),    
        .key_auto          (ctrl_key_auto),
        .disp_busy         (disp_busy_sd),    // 转场期间禁止开新一轮写图（防撕裂）
        .state_code        (state_code),
        .img_count_out     (img_count),
        .img_idx_out       (img_index),
        .load_diag         (load_diag),
        // 切图耗时三件套（sd 域，单位 10ms），跨到 video 域的锁存在上面
        .sw_wait_cs        (sw_wait_cs),
        .sw_load_cs        (sw_load_cs),
        .sw_total_cs       (sw_total_cs),
        .sw_upd_tgl        (sw_upd_tgl),
        // 这两个端口语义已从"源图必须是 640*480"变成"缩放目标尺寸"：
        // 任意分辨率的源图由内部 img_scaler 双线性缩放到这个尺寸。
        .bmp_width         (16'd640),
        .bmp_height        (16'd480),
        .display_valid     (display_valid),

        .write_finish_toggle(frame_write_toggle_mem),
        .write_buf_idx     (write_buf_idx),
        .disp_buf_idx      (disp_buf_idx),

        .write_req         (sd_card_write_req),
        .write_req_ack     (sd_card_write_req_ack),
        .write_en          (sd_card_write_en),
        .write_data        (sd_card_write_data),
        .SD_nCS            (sd_ncs),
        .SD_DCLK           (sd_dclk),
        .SD_MOSI           (sd_mosi),
        .SD_MISO           (sd_miso)
    );

    // ==================== 数码管显示（4 位） ====================
    // 【当前：诊断模式 DIG_SEL = 3】
    //
    // 稳定 "4030" 的旧读法（DIG_SEL=0，DIG3..DIG0）：
    //   4 = img_count（只扫到 4 张，正常）  0 = last_key（没按过键）
    //   3 = load_diag  但 **3 在旧代码里只表示"加载失败/超时"**
    //   0 = img_index
    // 也就是说 4030 = "键没按时 4 张图 + 首图加载失败"，画面其实是空白的。
    //
    // DIG_SEL = 3 时的读法：
    //   DIG3 = 最近按下的键号（F = 上电后没按过任何一个键）
    //   DIG2 = bmp_read 状态码 state_code
    //          0=SD 未初始化  1=IDLE 空闲  2=扫描/读 BMP 头  3=等写 SDRAM 4=读像素 5=查 FAT
    //   DIG1 = load_diag（0 未开始 / 1 加载中 / 2 已提交显示 /
    //                     3 = 8s 兜底超时（读链路卡死）
    //                     4 = 源侧读完但整帧写完成始终不来（含 FAT 簇链提前终止）
    //                     5 = BMP 头非法，读引擎从未发出写请求）
    //   DIG0 = img_count（扫到的图片数）
    //   加载进行中（load_diag==1）时，DIG0~DIG2 整排显示 "ddd"，
    //   一眼就能区分"卡住了"和"正在加载"。
    //
    // 可选值：0=正常  1=纯按键自检  2=加载链路诊断  3=按键自检+加载中显 ddd
    // 定位完成后把 DIG_SEL 改回 0 即可恢复（见 case 的 default 分支）。
    localparam integer DIG_SEL = 3;
    reg [3:0] last_key;
    always @(posedge clk or posedge rst_all) begin
        if (rst_all)
            last_key <= 4'hF;
        else if (key_press[0]) last_key <= 4'd0;
        else if (key_press[1]) last_key <= 4'd1;
        else if (key_press[2]) last_key <= 4'd2;
        else if (key_press[3]) last_key <= 4'd3;
        else if (key_press[4]) last_key <= 4'd4;
        else if (key_press[5]) last_key <= 4'd5;
        else if (key_press[6]) last_key <= 4'd6;
        // KEY7 只翻 OSD 信息层显隐，不改变画面内容，故不参与键号记录
        // （复位参数键已前移到 KEY6；KEY8~KEY15 当前无功能）。
    end

    // 四位要显示的数字，按 DIG_SEL 选择内容
    reg [3:0] seg_d0, seg_d1, seg_d2, seg_d3;

    always @(*) begin
        case (DIG_SEL)
            1: begin   // 按键自检：按 KEYn → 最右位变 n（F=未按过）
                seg_d0 = last_key;
                seg_d1 = load_diag;
                seg_d2 = 4'd7;                // 版本标记：7 = 本次修复版
                seg_d3 = {1'b0, img_count};
            end
            2: begin   // 加载链路诊断
                seg_d0 = {2'd0, img_index};
                seg_d1 = load_diag;
                seg_d2 = state_code;
                seg_d3 = 4'd2;
            end
            3: begin   // 按键自检 + 加载中整排显 d
                // 加载期间 DIG0~DIG2 = d，DIG3 仍是最近按下的键号，
                // 这样同时能看到"键被识别了"和"确实在加载"。
                if (load_diag == 4'd1) begin
                    seg_d0 = 4'hd;
                    seg_d1 = 4'hd;
                    seg_d2 = 4'hd;
                    seg_d3 = last_key;            // 键号一直可见
                end else begin
                    seg_d0 = {1'b0, img_count};
                    seg_d1 = load_diag;
                    seg_d2 = state_code;
                    seg_d3 = last_key;
                end
            end
            default: begin  // 0 = 正常显示
                seg_d0 = {2'd0, img_index};   // 当前第几张（0~3）
                seg_d1 = load_diag;           // 首图加载诊断
                seg_d2 = last_key;            // 最近按下的键号
                seg_d3 = {1'b0, img_count};   // 扫到的图片数
            end
        endcase
    end

    seg4_driver u_seg4_driver (
        .clk    (clk),
        .rst_n  (rst_n),
        .digit0 (seg_d0),
        .digit1 (seg_d1),
        .digit2 (seg_d2),
        .digit3 (seg_d3),
        .seg_sel(seg_sel),
        .seg_data(seg_data)
    );

    // ==================== 图像时序与帧缓存 ====================
    wire        hs;
    wire        vs;
    wire        de;
    wire [23:0] vout_data_raw;
    wire [23:0] vout_data;
    wire        video_read_req;
    wire        video_read_req_ack;
    wire        video_read_en;
    wire [31:0] video_read_data;

    wire        hs_0;
    wire        vs_0;
    wire        de_0;

    // 读缓冲调度（由下面的转场状态机驱动，见"两段式白场转场"一节）
    wire [1:0]  read_addr_index;
    wire        read_rebase;

    wire App_rd_en;
    wire [ADDR_BITS-1:0] App_rd_addr;
    wire Sdr_rd_en;
    wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
    wire App_wr_en;
    wire [ADDR_BITS-1:0] App_wr_addr;
    wire [MEM_DATA_BITS-1:0] App_wr_din;
    wire [3:0] App_wr_dm;

    video_timing_data video_timing_data_m0(
        .video_clk         (video_clk),
        .rst               (rst_all),
        .read_req          (video_read_req),
        .read_req_ack      (video_read_req_ack),
        .hs                (hs_0),
        .vs                (vs_0),
        .de                (de_0)
    );

    video_delay video_delay_m0(
        .video_clk         (video_clk),
        .rst               (rst_all),
        .read_en           (video_read_en),
        .read_data         (video_read_data[31:8]),
        .hs                (hs_0),
        .vs                (vs_0),
        .de                (de_0),
        .hs_r              (hs),
        .vs_r              (vs),
        .de_r              (de),
        .vout_data         (vout_data_raw)
    );

    assign vout_data = display_valid ? vout_data_raw : 24'd0;

    frame_read_write #(
        .WRITE_V_FLIP     (1),
        .FRAME_WIDTH      (640),
        .FRAME_HEIGHT     (480)
    ) frame_read_write_m0(
        .mem_clk           (ext_mem_clk),
        .rst               (rst_all),
        .Sdr_init_done     (Sdr_init_done),
        .Sdr_init_ref_vld  (Sdr_init_ref_vld),
        .Sdr_busy          (Sdr_busy),

        .App_rd_en         (App_rd_en),
        .App_rd_addr       (App_rd_addr),
        .Sdr_rd_en         (Sdr_rd_en),
        .Sdr_rd_dout       (Sdr_rd_dout),

        .read_clk          (video_clk),
        .read_req          (video_read_req),
        .read_req_ack      (video_read_req_ack),
        .read_finish       (),
        .read_addr_0       (BUF0_ADDR),
        .read_addr_1       (BUF1_ADDR),
        .read_addr_2       (BUF2_ADDR),
        .read_addr_3       (24'd0),
        .read_addr_index   (read_addr_index),
        .read_len          (FRAME_PIXELS),
        .read_en           (video_read_en),
        .read_data         (video_read_data),
        .read_rebase       (read_rebase),

        .App_wr_en         (App_wr_en),
        .App_wr_addr       (App_wr_addr),
        .App_wr_din        (App_wr_din),
        .App_wr_dm         (App_wr_dm),

        .write_clk         (sd_card_clk),
        .write_req         (sd_card_write_req),
        .write_req_ack     (sd_card_write_req_ack),
        .write_finish      (frame_write_finish),
        .write_addr_0      (BUF0_ADDR),
        .write_addr_1      (BUF1_ADDR),
        .write_addr_2      (BUF2_ADDR),
        .write_addr_3      (24'd0),
        .write_addr_index  (write_buf_idx),
        .write_len         (FRAME_PIXELS),
        .write_en          (sd_card_write_en),
        .write_data        (sd_card_write_data)
    );

    sdram U3(
        .Clk               (ext_mem_clk),
        .Clk_sft           (ext_mem_clk_sft),
        .Rst               (rst_all),
        .Sdr_init_done     (Sdr_init_done),
        .Sdr_init_ref_vld  (Sdr_init_ref_vld),
        .Sdr_busy          (Sdr_busy),
        .App_wr_en         (App_wr_en),
        .App_wr_addr       (App_wr_addr),
        .App_wr_dm         (App_wr_dm),
        .App_wr_din        (App_wr_din),
        .App_rd_en         (App_rd_en),
        .App_rd_addr       (App_rd_addr),
        .Sdr_rd_en         (Sdr_rd_en),
        .Sdr_rd_dout       (Sdr_rd_dout)
    );

    // ==================== 差异化图像处理流水线 ====================
    // 1. 像素坐标生成（与 vout_data 对齐）
    wire [10:0] px_x;
    wire [10:0] px_y;
    pixel_coord_gen #(
        .H_ACTIVE (640),
        .V_ACTIVE (480)
    ) u_pixel_coord (
        .clk  (video_clk),
        .rst  (rst_all),
        .de   (de),
        .vs   (vs),
        .x    (px_x),
        .y    (px_y)
    );

    // 2. 转场触发：在 sd_card_clk 域检测 disp_buf_idx 变化产生脉冲，
    //    再两级同步到 video_clk 域（标准 CDC 同步）
    //    注意：sd_card_clk=100MHz、video_clk=25MHz。1 拍 10ns 的脉冲
    //    在 40ns 的 video_clk 里大概率采不到，转场会"时有时无"。
    //    这里把脉冲展宽到 12 拍（120ns），保证 25MHz 域稳定采到 3 次以上。
    reg [1:0] disp_buf_d1;
    // 切屏前那一刻的缓冲号 = 本轮转场要淡出的"旧图"。
    // 双缓冲时代旧图就是"另一块"（0/1 互换即可），三缓冲下 0→1→2→0 轮转，
    // "另一块"有两个候选，只能靠锁存得到真正的上一块。
    // 锁存点选在 disp_buf_idx 变化的那一拍：此拍 disp_buf_d1 尚未更新，
    // 仍是变化前的值，正好就是旧缓冲。
    reg [1:0] disp_buf_prev;
    reg       trans_trig_raw;
    reg [3:0] trig_hold;
    reg [1:0] trans_trig_sync;
    wire      trans_trig;

    always @(posedge sd_card_clk or posedge rst_all) begin
        if (rst_all) begin
            disp_buf_d1    <= 2'd0;
            disp_buf_prev  <= 2'd0;
            trans_trig_raw <= 1'b0;
            trig_hold      <= 4'd0;
        end else begin
            disp_buf_d1 <= disp_buf_idx;
            if (disp_buf_idx != disp_buf_d1) begin
                trig_hold     <= 4'd12;             // 重新装载展宽计数
                disp_buf_prev <= disp_buf_d1;       // 记下"切换前的旧缓冲"
            end else if (trig_hold != 4'd0) begin
                trig_hold <= trig_hold - 4'd1;
            end
            trans_trig_raw <= (trig_hold != 4'd0) || (disp_buf_idx != disp_buf_d1);
        end
    end

    always @(posedge video_clk or posedge rst_all) begin
        if (rst_all)
            trans_trig_sync <= 2'd0;
        else
            trans_trig_sync <= { trans_trig_sync[0], trans_trig_raw };
    end
    assign trans_trig = trans_trig_sync[0] & ~trans_trig_sync[1];

    // ========================================================================
    // 两段式白场转场 —— 读缓冲调度的核心
    //
    // 需求：第一张图逐渐淡化变成白屏 → 第二张图从白屏里逐渐实体化。
    // 约束：只有一条 SDRAM 读通路（不做双读），所以**任何时刻只能读一块 buffer**。
    //
    // 借力 sd_card_bmp 已有的双缓冲：
    //   · 新图永远写到"非当前显示"的另一块（next_buf_lut）；
    //   · disp_buf_idx 直到整帧写完才切换（见 sd_card_bmp 的提交逻辑）。
    //   于是整个加载期间，旧 buffer 完好无损，可以放心读。
    //
    // 调度分三段（rd_sel 是"这一刻读哪块"，独立于 sd_card_bmp 的 disp_buf_idx）：
    //
    //   S_NORMAL  rd_sel = disp_buf_idx            正常显示当前图
    //   S_FADE_OUT rd_sel = 旧 buffer（锁存）      旧图淡到白（alpha 0→255）
    //   S_FADE_IN  rd_sel = 新 buffer（=disp_buf_idx） 白→新图（beta 0→255）
    //
    // 为什么 FADE_OUT 期间可以慢慢淡：旧图数据在写新图时不受影响。
    // 为什么 FADE_IN 必须等 disp_buf_idx 已经切过来才做：新图必须写完。
    //   FADE_OUT 有个超时看门狗，万一新图没来（加载失败），淡出结束后
    //   自动回到 S_NORMAL，不会永远停在白屏上。
    // ========================================================================
    localparam [1:0] TSN_IDLE     = 2'd0;
    localparam [1:0] TSN_FADE_OUT = 2'd1;   // 旧图 → 白
    localparam [1:0] TSN_FADE_IN  = 2'd2;   // 白 → 新图
    localparam [1:0] TSN_NORMAL   = 2'd3;   // 稳态

    // 与 transition_effect 的 PHASE_FRAMES 保持一致（两处必须同步改）
    // 30 帧/段 = 0.50s（59.5Hz）。中间另有 WHITE_HOLD=4 帧全白停留，
    // 所以一轮转场总计 64 帧 ≈ 1.08s：
    //   0.00~0.50s 旧图褪成白 | 0.50~0.57s 停白 | 0.57~1.08s 新图浮现
    // （原为 60+8+60=128 帧 ≈ 2.15s，按键到换完图近 3s，故整体减半）
    // 以前写 15/30/60 都"看不出效果"，根因不在帧数，
    //    而在帧计数用了 de 上升沿（每帧 480 次）→ 整段只跑 4ms。
    localparam integer PHASE_FRAMES = 30;

    reg [1:0]  tsn_state;
    reg [1:0]  buf_old_lock;     // 淡出期间锁存的旧 buffer 号
    reg        tsn_go;           // 交给 transition_effect 的触发脉冲（单拍）
    reg        tsn_start_pb;     // 本次触发从后半段（白→新图）开始

    // transition_effect 输出的当前段与进度，回读用于状态推进
    wire [7:0]  trans_prog;
    wire        trans_phase_b;
    wire        trans_phase_b_done;  // 后半段走完的单拍脉冲
    // 注：trans_busy 已在上面（TF 读图那一节）提前声明，因为要同步给 sd_card_bmp

    // 读缓冲选择：稳态跟 disp_buf_idx，淡出锁旧 buffer，淡入读新 buffer
    reg [1:0] rd_buf_sel;
    reg       rd_rebase;         // 换缓冲时给 frame_read_write 的"重对齐"脉冲

    // ========================================================================
    // 三段式状态机（TSN_xxx）
    //
    //   上电 ──► TSN_IDLE ──(第一张图已显示)──► TSN_FADE_IN ──► TSN_NORMAL
    //              │                                              │
    //              │                            (新图写入完成，disp_buf_idx 切换)
    //              │                                              ▼
    //              └─────────────────────────────────────► TSN_FADE_OUT
    //                                                             │
    //                                        (淡出到全白)         ▼
    //                                                        TSN_FADE_IN ──► TSN_NORMAL
    //
    // 关键点：TSN_FADE_OUT 期间 read_addr_index = buf_old_lock（旧图那块），
    //         因为 disp_buf_idx 这时已指向新图，直接跟它读就会读到新图，
    //         "旧图逐渐淡化"就看不到了。
    //
    // 段切换判据：
    //   FADE_OUT → FADE_IN 用 trans_phase_b 的上升沿（transition_effect 跨段那一拍）
    //   FADE_IN  → NORMAL   用 trans_phase_b_done（后半段走完的那一拍）
    //   **不能**用 prog 判断：prog 在跨段瞬间会跳回 0，用它判会漏掉段边界，
    //   导致"已经进后半段了却还指着旧 buffer"，白场里显示出旧图的残影。
    // ========================================================================

    reg trans_pb_d1;
    wire trans_pb_rise = trans_phase_b & ~trans_pb_d1;
    wire trans_pb_fall = ~trans_phase_b & trans_pb_d1;   // 后半段结束（电平下降沿）

    // ---- 卡死自愈：有新图提交了，但转场没被触发 ----
    // 原来这里用 trans_trig_sync[0] 当"待处理"电平，可那个电平是被展宽的
    // 12 拍 sd_card_clk（≈120ns），根本撑不到 1.5s，计数器每拍都被清零 ——
    // 加上 tsn_wait_cnt 只有 24 位（上限 16777215 < 37500000），
    // 两个原因叠在一起，这个看门狗从来不会响，等于死代码。
    //
    // 改成用"已呈现缓冲号"来判断：TSN_NORMAL 下记住上次接受转场那一刻的
    // disp_buf_idx，一旦它变了就说明有新图提交却没走转场。此时若
    // transition_effect 空闲，持续 0.5s 就补一次；繁忙说明正常流程在跑，
    // 打断它反而会把 phase_b 拉回 0 造成永久卡死，所以必须等 busy 落下。
    reg  [1:0]  disp_seen;                                    // 已呈现的缓冲号
    wire        change_pending = (disp_buf_idx != disp_seen);
    reg  [25:0] tsn_wait_cnt;
    wire        tsn_wait_ovf = (tsn_wait_cnt >= 26'd12_500_000);   // 0.5s @25MHz

    // ---- FADE_IN 超时保护 ----
    // trans_phase_b_done 是单拍脉冲。顶层和 transition_effect 在同一个
    // posedge video_clk 上，虽然这一拍的值要到下一拍才被采到，但脉冲本身
    // 有一整个周期的宽度，下一拍一定能看到，所以它是可用的。
    // 真正的隐患在 transition_effect 内部：旧版转场结束时没有把 phase_b_r
    // 清回 0，phase_b 会一直保持高电平，顶层就永远等不到它的下降沿。
    // 那一处已在 transition_effect 里修好（结束时清 phase_b_r）。
    // 这里保留一路 3s 超时兜底，任何异常都不会永久停在 FADE_IN。
    reg [26:0] fade_in_wait;
    wire       fade_in_ovf = (fade_in_wait >= 27'd75_000_000);    // 3s @25MHz

    always @(posedge video_clk or posedge rst_all) begin
        if (rst_all) begin
            tsn_state    <= TSN_IDLE;
            buf_old_lock <= 2'd0;
            tsn_go       <= 1'b0;
            tsn_start_pb <= 1'b1;
            rd_buf_sel   <= 2'd0;
            rd_rebase    <= 1'b0;      // 复位期不需要 rebase：frame_fifo_read 内部已强制停 S_IDLE
            trans_pb_d1  <= 1'b0;
            tsn_wait_cnt <= 26'd0;
            fade_in_wait <= 27'd0;
            disp_seen    <= 2'd0;
        end else begin
            tsn_go      <= 1'b0;
            rd_rebase   <= 1'b0;
            trans_pb_d1 <= trans_phase_b;

            // FADE_IN 计时
            if (tsn_state != TSN_FADE_IN)
                fade_in_wait <= 27'd0;
            else if (fade_in_wait < 27'd75_000_000)
                fade_in_wait <= fade_in_wait + 27'd1;

            // 不在 NORMAL、或没有"未呈现的变化" → 清计时
            if (tsn_state != TSN_NORMAL || !change_pending)
                tsn_wait_cnt <= 26'd0;
            else if (tsn_wait_cnt < 26'd12_500_000)
                tsn_wait_cnt <= tsn_wait_cnt + 26'd1;

            case (tsn_state)
                // 上电后等第一张图提交显示
                TSN_IDLE: begin
                    rd_buf_sel <= disp_buf_idx;
                    if (display_valid) begin
                        // 首图没有"旧图"可淡，直接从白场显出第一张图
                        tsn_state    <= TSN_FADE_IN;
                        tsn_go       <= 1'b1;
                        tsn_start_pb <= 1'b1;
                        disp_seen    <= disp_buf_idx;   // 记录"已经呈现的是哪一块"
                    end
                end

                // 旧图 → 白：读锁存的旧 buffer
                TSN_FADE_OUT: begin
                    rd_buf_sel <= buf_old_lock;
                    if (trans_pb_rise) begin
                        // transition_effect 刚跨入后半段 = 屏幕此刻正好全白。
                        // 同一个脉冲里切到新 buffer 并让读侧重新对齐，
                        // 后半段第一帧就能读到新图（此时新图权重为 0，仍是纯白，
                        // 所以看不到任何跳变）。
                        tsn_state <= TSN_FADE_IN;
                        rd_rebase <= 1'b1;
                    end
                end

                // 白 → 新图：读新 buffer（= disp_buf_idx）
                TSN_FADE_IN: begin
                    rd_buf_sel <= disp_buf_idx;
                    // 主判据：trans_phase_b 从 1 落回 0 = 后半段刚走完（可靠的下降沿）。
                    // 冗余判据：trans_phase_b_done 单拍脉冲（正常情况下已含在下降沿里）。
                    // 兜底：3s 超时，防止任何异常导致永久停在 FADE_IN。
                    if (trans_pb_fall || trans_phase_b_done || fade_in_ovf) begin
                        tsn_state <= TSN_NORMAL;
                    end
                end

                // 稳态：跟着 disp_buf_idx 走；检测到新图提交就启动一轮淡出
                TSN_NORMAL: begin
                    rd_buf_sel <= disp_buf_idx;
                    // !trans_busy 是必要的：trans_trig 只是"disp_buf_idx 变了"，
                    // 万一上一轮转场还没走完，此时硬启动会把 phase_b 打回 0，
                    // 转场模块的结束信号就再也发不出来。忙就等它，交给下面的
                    // 0.5s 看门狗兜底。
                    if (trans_trig && !trans_busy) begin
                        // 旧缓冲 = 切换前显示的那一块，由 sd_card_clk 域锁存而来。
                        //
                        // 这里原来是 (disp_buf_idx == 0) ? 1 : 0 —— 双缓冲的
                        // "另一块就是旧图"。三缓冲 0→1→2→0 轮转后这个式子不再
                        // 成立：disp 从 0 切到 1 时旧图确实是 2？不是，旧图是 0，
                        // 而式子给出 1（= 新图本身），淡出的就成了新图。
                        // 所以改成直接用锁存值 disp_buf_prev，与缓冲块数无关。
                        buf_old_lock <= disp_buf_prev;
                        tsn_state    <= TSN_FADE_OUT;
                        tsn_go       <= 1'b1;
                        tsn_start_pb <= 1'b0;      // 先淡出（旧图→白）
                        disp_seen    <= disp_buf_idx;   // 这一块已被接受，不再算"待处理"
                    end else if (tsn_wait_ovf) begin
                        // 兜底：有新图提交却一直没走进转场（例如触发脉冲被别的
                        // 请求挤掉了）。仅在 transition_effect 确实空闲时才重启，
                        // 否则会打断正在进行的转场，把 phase_b 拉回 0，
                        // 反而导致转场结束信号收不到、永久卡死。
                        if (!trans_busy) begin
                            buf_old_lock <= disp_buf_prev;   // 同主路径：用锁存值，不能靠 0/1 互换
                            tsn_state    <= TSN_FADE_OUT;
                            tsn_go       <= 1'b1;
                            tsn_start_pb <= 1'b0;
                            rd_rebase    <= 1'b1;
                            disp_seen    <= disp_buf_idx;
                        end
                    end
                end

                default: begin
                    rd_buf_sel <= disp_buf_idx;
                    tsn_state  <= TSN_NORMAL;
                end
            endcase
        end
    end

    // 3. 转场特效
    wire [23:0] trans_rgb;
    wire        de_trans;
    transition_effect #(
        .H_ACTIVE     (640),
        .V_ACTIVE     (480),
        .PHASE_FRAMES (PHASE_FRAMES)
    ) u_transition (
        .clk           (video_clk),
        .rst           (rst_all),
        .de            (de),
        .vs            (vs),        // 用它数"帧"：de 每帧上升 480 次，不能用来计帧
        .trans_trig    (tsn_go),
        .start_phase_b (tsn_start_pb),
        .trans_sel     (2'd0),      // KEY2 转场类型切换已删除，恒 0（模块内部本来也不引用）
        .rgb_new       (vout_data),
        .rgb_old       (24'd0),     // 本版不使用：淡出段直接读旧 buffer，走 rgb_new 通道
        .rgb_out       (trans_rgb),
        .de_out          (de_trans),
        .trans_active    (),         // 不接：OSD 的红框会在转场那 2.2s 里盖住画面中心
        .phase_b         (trans_phase_b),
        .phase_b_done    (trans_phase_b_done),
        .busy            (trans_busy),
        .prog            (trans_prog)
    );

    // OSD 的"红框 + 提示"已停用（见上面 .trans_active() 空连）。
    // 它会在画面正中画一个 80x32 的红框，转场正好要 2 秒，
    // 等于每次淡入淡出都被红框挡掉中间一大块，看着就像"画面坏了"。
    // 若以后还想要这个提示，把 osd_overlay 的 .trans_active 接回
    // transition_effect 的 trans_active 端口即可。

    // 读缓冲选择给 frame_read_write
    assign read_addr_index = rd_buf_sel;
    assign read_rebase     = rd_rebase;

    // 4. 亮度/对比度调节
    wire [23:0] bc_rgb;
    wire        de_bc;
    brightness_contrast u_brightness_contrast (
        .clk         (video_clk),
        .rst         (rst_all),
        .de          (de_trans),
        .rgb_in      (trans_rgb),
        .bright      (ctrl_bright_v),
        .contrast_q8 (ctrl_contrast_v),
        .rgb_out     (bc_rgb),
        .de_out      (de_bc)
    );

    // ==================== 音频链路 ====================
    wire        audio_i2s_bclk;
    wire        audio_i2s_lrck;
    wire        audio_i2s_dout;
    wire        audio_valid;
    wire [23:0] audio_left_data;
    wire [23:0] audio_right_data;
    wire        acr_valid;
    wire [19:0] acr_cts;
    wire [19:0] acr_n;

    // 内部 I2S 测试音（do/re/mi 旋律，后续可替换为真实音频）
    hdmi_audio_tone_i2s_64fs #(
        .PHASE_INC (32'd39370534),
        .AMP       (24'd2000000)
    ) u_hdmi_audio_tone_i2s_64fs (
        .I_mclk      (audio_mclk),
        .I_rst       (rst_all),
        .O_i2s_BCLK  (audio_i2s_bclk),
        .O_i2s_LRCK  (audio_i2s_lrck),
        .O_i2s_DOUT  (audio_i2s_dout)
    );

    I2S_receiver u_I2S_receiver(
        .I_clk              (video_clk),
        .I_rst              (rst_all),
        .I_i2s_BCLK         (audio_i2s_bclk),
        .I_i2s_LRCK         (audio_i2s_lrck),
        .I_i2s_DOUT         (audio_i2s_dout),
        .O_audio_valid      (audio_valid),
        .O_audio_left_data  (audio_left_data),
        .O_audio_right_data (audio_right_data)
    );

    audio_arc_calculate #(
        .ACR_N         (6144)
    ) u_audio_arc_calculate (
        .I_clk         (video_clk),
        .I_rst         (rst_all),
        .I_audio_valid (audio_valid),
        .O_acr_valid   (acr_valid),
        .O_acr_cts     (acr_cts),
        .O_acr_n       (acr_n)
    );

    // ==================== 音频可视化频谱 ====================
    // 扁平化柱高向量 [63:0]：band0 在 [7:0]，band7 在 [63:56]（每根 8bit）
    // 8 带 Goertzel 频谱：每 85ms（4096 采样 @48kHz）全体刷新一次，
    // active_col 是能量最强的那一带，送给 overlay 画白色顶帽。
    // WIN_SAMPLES 不要设成 24000（=一个音符 0.5s）：窗口与音符起点无对齐关系，
    // 窗口一跨音符就会把前后两个音混在一起，柱高乱跳。4096（85ms）足够短，
    // 又能保证 11.7Hz 的频率分辨率把相邻半音分开。
    wire [63:0] spec_bar_h;
    wire        spec_frame_tick;
    wire [2:0]  spec_active_col;
    audio_spectrum #(
        .NUM_BANDS   (8),
        .BAR_MAX_H   (60),        // 必须与 spectrum_overlay 的 TRACK_H 一致
        .WIN_SAMPLES (4096)       // 85.3ms @48kHz；上限 8191（内部计数 13 位）
    ) u_audio_spectrum (
        .clk         (video_clk),
        .rst         (rst_all),
        .audio_valid (audio_valid),
        .audio_data  (audio_left_data),
        .bar_h       (spec_bar_h),
        .active_col  (spec_active_col),
        .frame_tick  (spec_frame_tick)
    );

    // 5. 频谱柱状图叠加（含当前更新柱的白色顶帽）
    // 布局（v4：缩小 + 左下角 + 去掉黑框）
    //   8 根 × PITCH 24 = 192 宽，START_X=16 → x 16~207；
    //   柱身 16、间隔 8；柱底 y=428；柱高上限 60 → 顶 y=368；
    //   柱底画 2 行基线（y 428~429），距 OSD 字幕条（y>=440）还有 10 行空隙。
    //   PANEL_TOP 背景板与暗色轨道已删除（那就是原来的黑框），现在除柱身和
    //   一条细基线外完全不遮挡原图。
    // 坐标由 spectrum_overlay 内部用自己的 de/vs 现算，不接 px_x/px_y，
    // 避免"数据经过 trans/bc 多拍延迟、坐标却是未经延迟的"造成横向漂移。
    wire [23:0] spec_rgb;
    wire        de_spec;
    spectrum_overlay #(
        .H_ACTIVE   (640),
        .V_ACTIVE   (480),
        .NUM_BANDS  (8),
        .START_X    (16),
        .PITCH      (24),
        .BODY_W     (16),
        .BASE_Y     (428),
        .TRACK_H    (60),
        .HAT_H      (4)
    ) u_spectrum_overlay (
        .clk        (video_clk),
        .rst        (rst_all),
        .de         (de_bc),
        .vs         (vs),
        .rgb_in     (bc_rgb),
        .bar_h      (spec_bar_h),
        .active_col (spec_active_col),
        .rgb_out    (spec_rgb),
        .de_out     (de_spec)
    );

    // 6. OSD 字幕叠加
    wire [23:0] osd_rgb;
    wire        de_osd;
    osd_overlay #(
        .H_ACTIVE (640),
        .V_ACTIVE (480)
    ) u_osd_overlay (
        .clk         (video_clk),
        .rst         (rst_all),
        .de          (de_spec),
        .vs          (vs),
        .h_cnt       (px_x),
        .v_cnt       (px_y),
        .rgb_in      (spec_rgb),
        .img_count   (img_count),
        .img_index   (img_index),
        .bright      (ctrl_bright_v),
        .contrast_q8 (ctrl_contrast_v),
        // 切图耗时（已锁存到本域，单位 10ms）
        .sw_wait_cs  (sw_wait_v),
        .sw_load_cs  (sw_load_v),
        .sw_total_cs (sw_total_v),
        .trans_active(1'b0),        // 停用转场红框（原因见上面 u_transition 处的注释）
        .osd_on      (osd_on),      // KEY7 控制：0 隐藏 L:/C: 与 T:/W:/L:，1 显示
        .rgb_out     (osd_rgb),
        .de_out      (de_osd)
    );

    // ==================== RGB/DE 转 AXIS ====================
    // 流水线共引入 5 拍延迟（trans 2 + bc 1 + spec 1 + osd 1），
    // 需将 vs 同步延迟 5 拍与 osd_rgb 对齐
    reg [4:0] vs_shf;
    wire      vs_delayed;
    always @(posedge video_clk or posedge rst_all) begin
        if (rst_all)
            vs_shf <= 5'd0;
        else
            vs_shf <= { vs_shf[3:0], vs };
    end
    assign vs_delayed = vs_shf[4];

    wire        axis_s_user;
    wire        axis_s_valid;
    wire        axis_s_last;
    wire [23:0] axis_s_data;

    video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
        .I_clk         (video_clk),
        .I_rst         (rst_all),
        .I_vs          (vs_delayed),
        .I_de          (de_osd),
        .I_rgb         (osd_rgb),
        .O_video_user  (axis_s_user),
        .O_video_valid (axis_s_valid),
        .O_video_last  (axis_s_last),
        .O_video_data  (axis_s_data)
    );

    // ==================== EDID 触发 ====================
    wire edid_trig;
    wire edid_valid;
    wire [7:0] edid_data;
    startup_pulse #(
        .CNT_MAX(20'd100000)
    ) u_startup_pulse (
        .I_clk   (video_clk),
        .I_rst   (rst_all),
        .O_pulse (edid_trig)
    );

    // ==================== HDMI 发射（视频+音频） ====================
    wire [9:0] tmds_ch0_data;
    wire [9:0] tmds_ch1_data;
    wire [9:0] tmds_ch2_data;
    wire [9:0] tmds_clk_data;

    hdmi_1_4b_transmitter_core_wrapper #(
        .DEVICE                 ( "EG"       ),
        .HTOTAL                 ( 800        ),
        .HSA                    ( 96         ),
        .HFP                    ( 16         ),
        .HBP                    ( 48         ),
        .HACTIVE                ( 640        ),
        .VTOTAL                 ( 525        ),
        .VSA                    ( 2          ),
        .VFP                    ( 10         ),
        .VBP                    ( 33         ),
        .VACTIVE                ( 480        ),
        .VIDEO_VIC              ( 1          ),
        .VIDEO_TPG              ( "Disable"  ),
        .VIDEO_FORMAT           ( "RGB"      ),
        .AUDIO_SAMPLE_RATE      ( "48K"      ),
        .IIC_SCL_DIV            ( 250        )
    ) u_hdmi_1_4b_transmitter_core_wrapper(
        .I_pixel_clk        (video_clk),
        .I_rst              (rst_all),
        .I_edid_read_trig   (edid_trig),
        .O_edid_read_valid  (edid_valid),
        .O_edid_read_data   (edid_data),

        .I_axis_s_user      (axis_s_user),
        .I_axis_s_valid     (axis_s_valid),
        .I_axis_s_last      (axis_s_last),
        .I_axis_s_data      (axis_s_data),
        .O_axis_s_ready     (),

        .I_audio_valid      (audio_valid),
        .I_audio_left_data  (audio_left_data),
        .I_audio_right_data (audio_right_data),
        .I_acr_valid        (acr_valid),
        .I_acr_cts          (acr_cts),
        .I_acr_n            (acr_n),

        .O_video_locked     (),
        .O_ddc_scl          (),
        .IO_ddc_sda         (),

        .O_ch0_tmds_data    (tmds_ch0_data),
        .O_ch1_tmds_data    (tmds_ch1_data),
        .O_ch2_tmds_data    (tmds_ch2_data),
        .O_clk_tmds_data    (tmds_clk_data)
    );

    hdmi_phy_wrapper #(
        .DEVICE ( "EG" )
    ) u_hdmi2phy_wrapper(
        .I_pixel_clk        (video_clk),
        .I_serial_clk       (hdmi_5x_clk),
        .I_rst              (rst_all),
        .I_tmds_channel_0   (tmds_ch0_data),
        .I_tmds_channel_1   (tmds_ch1_data),
        .I_tmds_channel_2   (tmds_ch2_data),
        .I_tmds_channel_clk (tmds_clk_data),
        .O_tmds_ch0_p       (HDMI_D0_P),
        .O_tmds_ch1_p       (HDMI_D1_P),
        .O_tmds_ch2_p       (HDMI_D2_P),
        .O_tmds_clk_p       (HDMI_CLK_P)
    );

endmodule
