
// ============================================================================
// sd_card_bmp —— 图片轮播调度：扫描 / 加载 / 三缓冲切换
// ----------------------------------------------------------------------------
// 职责
//   上电后让 bmp_read 扫根目录找 BMP，按 KEY0（下一张）/ KEY1（自动播放）
//   的请求加载图片，经 img_scaler 缩放后写入 SDRAM，整帧写完后切换 disp_buf_idx，
//   由显示侧在转场中完成画面更替。
//
// 三缓冲约定（2026-10-28 由双缓冲升级）
//   write_buf_idx    本次写入的目标缓冲，三块按 0→1→2→0 轮转（next_buf_lut）
//   disp_buf_idx     当前显示缓冲
//   pre_buf/pre_ready 已经读完、但还没上屏的那一帧
//
//   为什么要第三块：
//     双缓冲下"加载"必然占用另一块，所以只能"先读完再上屏"，读 1080P 那 2 秒
//     期间屏幕只能干等旧图。三缓冲把"读"和"上屏"解耦 → 读卡在后台完成，
//     到点只做一次指针切换，观感是严格的 AUTO_DWELL_SEC 一张，与图多大无关。
//     顺带的好处：加载失败的重试也发生在后台，屏幕上纹丝不动，
//     不会再出现"卡很久然后跳到第一张"。
//
//   块号分配必须满足：写目标 ≠ 当前显示块 ≠ 转场正在淡出的旧块。
//   0→1→2→0 轮转天然满足（disp=1 时写 2，而转场只读 0 和 1）。
//   SDRAM 8MB，三块占 43.9%，容量与带宽都远未饱和。
//
// 加载握手
//   load_start_pulse → bmp_read 读文件头 → write_req/write_req_ack 清空写 FIFO
//   → 缩放开始吐像素（sc_frame_start = ack 的上升沿）→ sc_done 收尾
//
// 失败处理（三级兜底，避免一张坏图把播放卡死）
//   1. 早退检测：源侧结束但整帧迟迟写不完（500ms）→ 判失败
//   2. 兜底超时：加载全程超过 8s → 判失败
//   3. 失败位图 + 原地重试 2 次 → 仍失败才拉黑该图，全部失败则整机重扫
//
// 关键约束
//   加载期间不能写"正在显示"的那块缓冲，也不能写"转场正在淡出"的那块，
//   否则会把旧图逐行覆盖，出现一条推进的撕裂带。
//   三缓冲（0→1→2→0）从分配上就保证了这两条：写目标永远是第三块。
//
// 切图耗时测量（sw_wait_cs / sw_load_cs / sw_total_cs）
//   内部实例化 switch_latency，观测点四个：load_start_pulse / load_frame_done /
//   (swap_now | load_commit_now) / disp_busy，算出"等待 / 加载 / 总计"，
//   单位 10ms，结果随 sw_upd_tgl 翻转送给 video_clk 域显示到 OSD 上。
//   发布时刻挂"这一张上屏"，不挂"整帧写完"：三缓冲预读会在上一张的停留期里
//   就读完下一张，用写完时刻发布会让屏幕上的数字永远超前一张。
//   只做观测，不参与任何调度判断。
// ============================================================================
module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    // 自动轮播：每张图在屏上停留的秒数。
    // 三缓冲预读之后这就是**真实观感间隔**：读卡在后台跑，到点即切，
    // 与源图分辨率无关（1920×1080 也是 3 秒一张）。
    // 唯一的例外：某张图读得比这个时间还久（超慢卡 / 失败重试），
    // 那一次会顺延到读完为止，之后自动恢复。
    parameter integer AUTO_DWELL_SEC    = 3,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4
)(
    input                       clk,
    input                       rst,
    input                       key_next,
    input                       key_auto,
    input                       disp_busy,      // 显示侧转场中。三缓冲之后**不再用于门控**
                                                 // 加载（写目标与转场读的两块必然不同），
                                                 // 只透传给 switch_latency 做观测。
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output reg                  display_valid,
    output      [3:0]           state_code,
    output      [2:0]           img_count_out,   // 已发现的 BMP 图片数（供 OSD 显示）
    output      [1:0]           img_idx_out,     // 当前显示中的图片编号 0-based（供 OSD）
    output reg  [3:0]           load_diag,       // 首图加载诊断：0=未开始 1=加载中 2=已显示 3=加载失败

    // 切图耗时测量（送 OSD 显示），单位 10ms
    //   三个数一一对应 switch_latency 里的"等待 / 加载 / 总计"，
    //   走翻转脉冲跨到 video_clk 域后再显示。
    output wire [9:0]           sw_wait_cs,
    output wire [9:0]           sw_load_cs,
    output wire [9:0]           sw_total_cs,
    output wire                 sw_upd_tgl,

    input                       write_finish_toggle,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO
);

wire key_next_press;
wire key_auto_press;

wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;
wire [3:0]       state_code_i;
// bmp_read 输出的源像素流（任意分辨率）
wire             pix_de;
wire [23:0]      pix_data;
wire [15:0]      pix_x;
wire [15:0]      src_w;
wire [15:0]      src_h;
// img_scaler 输出（固定 bmp_width × bmp_height）
wire             sc_de;
wire [23:0]      sc_pixel;
wire             sc_done;
wire             sc_params_ok;
wire             sd_init_done;
wire             bmp_ready;
wire             scan_done;
wire             scan_found_valid;
wire [31:0]      scan_found_sector;
wire [2:0]       scan_found_total;

reg              scan_start_pulse;
reg              load_start_pulse;
reg [31:0]       load_sector;
reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
reg [31:0]       auto_cnt;
reg [2:0]        img_found_count;
reg [1:0]        img_idx;              // 当前真正显示中的图片编号
reg [1:0]        load_idx;             // 当前正在写入的图片编号
reg [1:0]        pending_buf_idx;      // 当前正在写入的目标缓冲区
reg [31:0]       img_sector0;
reg [31:0]       img_sector1;
reg [31:0]       img_sector2;
reg [31:0]       img_sector3;
reg              next_req_pending;
reg [31:0]       pending_wd;        // 切图请求看门狗：2s 未被消费则丢弃
reg              load_busy;
reg [31:0]       load_timeout_cnt;

// 加载失败位图：bit[n]=1 表示第 n 张图加载失败过，切换时跳过它。
// 用 4 位位图而不是"只记最后一个失败编号"，是为了避免两张坏图
// 轮流占位、导致播放永远在这两张之间打转（表现为只轮播两张）。
reg [3:0]        fail_map;
reg              load_fail_pulse;    // 本次加载失败的 1 拍标记
reg [3:0]        fail_streak;        // 连续失败次数，达到阈值后触发整机重扫
reg              retry_pending;      // 上一张失败、下一次仍加载同一张（重试）
reg [1:0]        retry_cnt;          // 已重试次数，便宜的失败最多重试 1 次
// 本次失败是否来自 8s 兜底超时。超时说明读链路已经卡死（不是一个字节抖一下），
// 原地重读同一张只会在同一条死路上再撞 8s，所以这种失败不重试、直接跳过。
reg              fail_was_timeout;
reg [31:0]       early_fail_cnt;     // "源图送完但整帧写完成迟迟不来"的计时
reg [31:0]       scan_wd_cnt;        // 首次扫描迟迟不完成时的看门狗

reg              load_abort;

// ---- 预读（三缓冲）----
// 问题的由来：双缓冲下"加载"必然占用另一块，所以只能"先读完再上屏"，
// 读 1080P 那 2 秒期间屏幕只能干等旧图 —— 观感就是某张图停留特别久。
//
// 三缓冲把"读"和"上屏"彻底解耦：
//   pre_ready = 1  下一张已经整帧写进 pre_buf，屏幕上仍是当前图，完全无感；
//   swap_now = 1   dwell 到点（或按键），disp 瞬间从 pre_buf 起，零等待。
// 于是读卡耗时被藏进停留时间里，无论图多大，每张都严格停 AUTO_DWELL_SEC。
//
// 失败重试也一并在后台完成：读错了就重试，屏幕上纹丝不动，
// 不会再出现"卡住很久然后跳到第一张"。
reg              pre_ready;
reg [1:0]        pre_buf;          // 预读好的那块缓冲
reg [1:0]        pre_idx;          // 预读好的图号
reg              swap_on_commit;   // 本次加载完成后立即上屏（手动/首图），不进预读等待

// 缩放通路握手：
//   load_armed   = 本次加载已经把写请求送到 SDRAM（写 FIFO 已清空）
//   sc_frame_start 上升沿 = FIFO 清空那一拍 → 缩放模块从第 0 行开始吐像素
reg              load_armed;
reg              sc_ack_d;
wire             sc_frame_start = write_req_ack & ~sc_ack_d;

// bmp_ready 先表示“源文件读取/送 FIFO 完成”；真正切显示要等 write_finish_toggle 同步后
reg              source_done_seen;
reg              write_done_seen;

// 同步 mem_clk 域的 write_finish_toggle
reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;

// 停留计时的计数终点：3s @100MHz = 300_000_000 拍，仍在 32 位内（上限 4.29e9）。
// 写成 localparam 是因为 parameter 不能位选，auto_cnt 是 32 位，直接比较会
// 因位宽不一致产生告警。
localparam [31:0] AUTO_TICK_LAST = (CLK_FREQ_HZ * AUTO_DWELL_SEC) - 1;

assign write_en   = sc_de;
assign write_data = {sc_pixel, 8'b0};

// auto_due = "停留时间已到、并且还没上屏"的电平，一直保持到真正 swap 为止。
//
// 必须是电平，不能写成 (auto_cnt == AUTO_TICK_LAST) 那种单拍脉冲。原因：
//   读 1080P 那张要 3s 上下，预读常常比 3s 停留窗口晚一点才完成。若 swap 条件
//   取单拍脉冲，到点那一拍 pre_ready 还是 0，脉冲就永远消失了，只能等计数器
//   重新数满一整圈 3s —— 表现正是"第 4 张（1080P）转第 1 张要停 6 秒以上，
//   其它三张都正常"。改成电平 + 计数器饱和后，预读一完成就立刻补切，
//   迟到的只有读卡尾巴，不再多赔一个完整 dwell 周期。
//
// 用 >= 而非 == ：饱和后二者等价，但 >= 对"计数器被意外写大"这类情况更稳。
wire auto_due = auto_play_en && (auto_cnt >= AUTO_TICK_LAST);

// 从当前图往后找第一张"没失败过"的图；一张可用的都没有就停在当前图。
wire [1:0] next_from_current = next_avail(img_idx, img_found_count, fail_map);

// 上屏请求：手动按键，或自动轮播到点。
// 两者都只有在"下一张已经预读好"时才能瞬间生效，否则退回原来的加载路径。
// next_req_pending 本身就是电平（保持到被消费），与 auto_due 语义一致。
wire swap_want = pre_ready && (next_req_pending || auto_due);
wire swap_now  = swap_want && !load_busy;

// 首图不能走 next_avail：img_idx 上电初值是 0，而 next_avail 返回的是"下一张"，
// 会让 0 号图永远得不到尝试机会。首图优先选 0 号，只有 0 号确实失败过才跳。
wire [1:0] first_load_idx = fail_map[0] ? next_from_current : 2'd0;

// 整帧写完（预读成功也走这里，此时还不一定上屏）
wire load_frame_done = load_busy && source_done_seen && write_done_seen;
// 真正把新图推上屏幕的那一拍：首图、手动、或预读好的图被 dwell 到点触发
wire load_commit_now = load_frame_done && (!display_valid || swap_on_commit);

// ---- 上屏后短静默窗（50ms）----
// 双缓冲时代这里要静默 600ms：因为写图目标恰好就是转场 FADE_OUT 正在读的
// 那块旧图，抢着写会撕出一条推进的横带。
// 三缓冲之后写目标（next_buf_lut）永远不等于"当前显示"和"刚被换下的旧图"，
// 那条冲突从根上不存在了，于是可以把窗口缩到 50ms —— 它现在只是给
// disp_buf_idx 跨到 video_clk 域留一点余量，不再承担防撕裂职责。
// 这一点很关键：预读要赶在 3s 停留期内读完 1080P（约 2.1s），
// 留在里面的 600ms 会直接吃掉三分之一预算。
reg  [25:0] settle_cnt;
wire        settle_ok = (settle_cnt >= 26'd5_000_000);    // 50ms @100MHz
always @(posedge clk or posedge rst) begin
    if (rst)
        settle_cnt <= 26'd5_000_000;        // 复位后视为"早已静默完毕"，首图不被拖慢
    else if (load_commit_now || swap_now)
        settle_cnt <= 26'd0;                // 刚上屏 → 重新开始计静默
    else if (settle_cnt < 26'd5_000_000)
        settle_cnt <= settle_cnt + 26'd1;   // 饱和，不会回绕
end

// 本拍要拉黑的图号 / 本拍要清除的位（都由时序逻辑产生，见下方 retry 逻辑）
reg  [3:0] fail_set_r;
reg  [3:0] fail_clr_r;
// 只要有一张加载成功，就清空整个失败位图 —— SD 读失败绝大多数是偶发时序
// 问题（卡忙、token 抖动），不该永久记住，否则好图会被越拉越少，
// 最后只剩一张，画面就再也不动了（"卡死在第一张"）。
wire [3:0] fail_map_n = (fail_map & ~fail_clr_r) | fail_set_r;

// 本次要加载的目标图号：失败重试期间锁定在同一张，否则取下一张可用图。
wire [1:0] target_idx = retry_pending ? load_idx : next_from_current;

assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
assign state_code = state_code_i;
assign img_count_out = img_found_count;
assign img_idx_out   = img_idx;

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_next),
    .press_pulse(key_next_press)
);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_auto (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_auto),
    .press_pulse(key_auto_press)
);

function [1:0] next_index_limited;
    input [1:0] cur;
    input [2:0] count;
    begin
        case (count)
            3'd0: next_index_limited = 2'd0;
            3'd1: next_index_limited = 2'd0;
            3'd2: next_index_limited = (cur == 2'd1) ? 2'd0 : (cur + 2'd1);
            3'd3: next_index_limited = (cur == 2'd2) ? 2'd0 : (cur + 2'd1);
            default: next_index_limited = (cur == 2'd3) ? 2'd0 : (cur + 2'd1);
        endcase
    end
endfunction

// 从 cur 的下一张开始，沿 count 限定的环找第一张"没失败过"的图。
// 转满一圈都失败过，就返回 cur（当前正在显示的图）：
// 绝不能返回一个已知失败的编号，否则会反复重读同一张坏图，画面永久卡住。
function [1:0] next_avail;
    input [1:0] cur;
    input [2:0] count;
    input [3:0] fmap;
    reg [1:0] cand;
    reg [1:0] result;
    reg       ok;
    integer   k;
    begin
        cand   = next_index_limited(cur, count);
        result = cand;
        ok     = 1'b0;
        for (k = 0; k < 4; k = k + 1) begin
            if (!ok && ({1'b0, cand} < count) && !fmap[cand]) begin
                result = cand;
                ok     = 1'b1;
            end
            cand = next_index_limited(cand, count);
        end
        if (ok)
            next_avail = result;
        else
            next_avail = cur;
    end
endfunction

function [31:0] sector_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: sector_lut = img_sector0;
            2'd1: sector_lut = img_sector1;
            2'd2: sector_lut = img_sector2;
            2'd3: sector_lut = img_sector3;
            default: sector_lut = img_sector0;
        endcase
    end
endfunction

function [1:0] next_buf_lut;
    input [1:0] cur_disp_buf;
    input       valid_now;
    begin
        if (!valid_now)
            next_buf_lut = 2'd0;                 // 首图固定写 buffer0
        else
            // 三缓冲轮转 0→1→2→0。这样写目标同时满足两条：
            //   · ≠ 当前显示块（屏幕上那张不能被覆盖）
            //   · ≠ 刚被换下去的那块（转场 FADE_OUT 还在读它做淡出）
            // 例如 disp=1 时写 2，转场正读 0 和 1，互不相干。
            case (cur_disp_buf)
                2'd0:    next_buf_lut = 2'd1;
                2'd1:    next_buf_lut = 2'd2;
                default: next_buf_lut = 2'd0;
            endcase
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync        <= 3'b000;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        load_sector           <= 32'd0;
        scan_kicked           <= 1'b0;
        first_image_committed <= 1'b0;
        auto_play_en          <= 1'b0;
        auto_cnt              <= 32'd0;
        img_found_count       <= 3'd0;
        img_idx               <= 2'd0;
        load_idx              <= 2'd0;
        pending_buf_idx       <= 2'd0;
        write_buf_idx         <= 2'd0;
        disp_buf_idx          <= 2'd0;
        img_sector0           <= 32'd0;
        img_sector1           <= 32'd0;
        img_sector2           <= 32'd0;
        img_sector3           <= 32'd0;
        next_req_pending      <= 1'b0;
        pending_wd            <= 32'd0;
        load_busy             <= 1'b0;
        load_timeout_cnt      <= 32'd0;
        fail_map              <= 4'b0000;
        fail_streak           <= 4'd0;
        fail_set_r            <= 4'b0000;
        fail_clr_r            <= 4'b0000;
        retry_pending         <= 1'b0;
        retry_cnt             <= 2'd0;
        load_fail_pulse       <= 1'b0;
        fail_was_timeout      <= 1'b0;
        early_fail_cnt        <= 32'd0;
        scan_wd_cnt           <= 32'd0;
        load_abort            <= 1'b0;
        source_done_seen      <= 1'b0;
        write_done_seen       <= 1'b0;
        display_valid         <= 1'b0;
        load_diag             <= 4'd0;
        load_armed            <= 1'b0;
        sc_ack_d              <= 1'b0;
        pre_ready             <= 1'b0;
        pre_buf               <= 2'd0;
        pre_idx               <= 2'd0;
        swap_on_commit        <= 1'b0;
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        sc_ack_d         <= write_req_ack;
        if (load_start_pulse)
            load_armed <= 1'b0;
        else if (write_req_ack)
            load_armed <= 1'b1;
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;
        load_abort       <= 1'b0;
        load_fail_pulse  <= 1'b0;
        // 失败位图每拍刷新：置位本次失败、清掉本次成功，全失败时自动清零重来
        fail_map         <= fail_map_n;

        if (!sd_init_done) begin
            scan_kicked           <= 1'b0;
            first_image_committed <= 1'b0;
            auto_play_en          <= 1'b0;
            auto_cnt              <= 32'd0;
            img_found_count       <= 3'd0;
            img_idx               <= 2'd0;
            load_idx              <= 2'd0;
            pending_buf_idx       <= 2'd0;
            write_buf_idx         <= 2'd0;
            disp_buf_idx          <= 2'd0;
            img_sector0           <= 32'd0;
            img_sector1           <= 32'd0;
            img_sector2           <= 32'd0;
            img_sector3           <= 32'd0;
            next_req_pending      <= 1'b0;
            pending_wd            <= 32'd0;
            load_busy             <= 1'b0;
            load_timeout_cnt      <= 32'd0;
            load_abort            <= 1'b0;
            fail_was_timeout      <= 1'b0;
            source_done_seen      <= 1'b0;
            write_done_seen       <= 1'b0;
            display_valid         <= 1'b0;
            load_diag             <= 4'd0;
            load_armed            <= 1'b0;
            pre_ready             <= 1'b0;
            pre_buf               <= 2'd0;
            pre_idx               <= 2'd0;
            swap_on_commit        <= 1'b0;
        end else begin
            // 扫描阶段缓存前 4 张图的起始 sector
            if (scan_found_valid) begin
                case (img_found_count)
                    3'd0: img_sector0 <= scan_found_sector;
                    3'd1: img_sector1 <= scan_found_sector;
                    3'd2: img_sector2 <= scan_found_sector;
                    3'd3: img_sector3 <= scan_found_sector;
                    default: ;
                endcase

                if (img_found_count < 3'd4)
                    img_found_count <= img_found_count + 3'd1;
            end

            // 记住"源侧已结束"，但此时还不能切显示，得等整帧写完。
            // 源图读完 ≠ 缩放映完：缩放模块把 DST_W×DST_H 全部吐完（sc_done）
            // 才算源侧完成。另外两条也算"源侧不用再等了"：
            //   · bmp_ready && load_armed：已启动过写请求、读引擎又回到 IDLE
            //     = 这一轮读结束了（正常读到 file_len 末，或 FAT 簇链提前终止）；
            //   · bmp_ready && !load_armed && 已过 2µs：BMP 头不合法，write_req
            //     从未发出、bmp_read 直接回 IDLE。
            //
            //   ⚠ 这两个条件必须用 load_armed 的正负形式区分，不能写成"或"：
            //     加载发起那一拍 load_busy 已置 1，但 bmp_read 还在 ST_IDLE、
            //     bmp_ready 仍为 1，且 load_armed 才刚被清 0（要等头读完才 arm）。
            //     若写成 (!load_armed || 已过2µs)，这一拍就会把 source_done_seen
            //     误置 1，早退计时从加载开头起算 —— 640×480 要读 0.3s 以上、
            //     1080P 要 2s 以上，必然在阈值处被判失败，数码管显示
            //     load_diag=4（"源侧结束但整帧写完成不来"），且图越大越必然。
            if (load_busy && (sc_done || (bmp_ready && (load_armed || (load_timeout_cnt > 32'd200)))))
                source_done_seen <= 1'b1;

            // 记住写帧完成脉冲，避免“先写完后源结束”导致脉冲丢失而卡死
            if (load_busy && write_finish_pulse)
                write_done_seen <= 1'b1;

            // 只有"源图送完 + 整帧写完"都满足，才认为这一帧真的可用
            if (load_frame_done) begin
                load_busy             <= 1'b0;
                load_timeout_cnt      <= 32'd0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
                first_image_committed <= 1'b1;
                load_diag             <= 4'd2;   // 整帧写好（预读完成时屏幕上还没换）
                fail_was_timeout      <= 1'b0;
                // 关键分岔：
                //   有画面在显示 且 这次是后台预读 → 只挂起，等 dwell 到点再上屏。
                //     这一行就是"读卡时间被隐藏"的全部秘密：帧已经躺在 pre_buf 里，
                //     屏幕还停在当前图上，用户完全感知不到 SD 卡在忙。
                //   首图（display_valid=0）/ 手动切图（swap_on_commit=1）→ 立即上屏，
                //     因为没有"可以继续留着看的旧图"。
                if (display_valid && !swap_on_commit) begin
                    pre_ready <= 1'b1;
                    pre_buf   <= pending_buf_idx;
                    pre_idx   <= load_idx;
                end else begin
                    disp_buf_idx  <= pending_buf_idx;
                    img_idx       <= load_idx;
                    display_valid <= 1'b1;
                    auto_cnt      <= 32'd0;
                end
            end else if (load_busy) begin
                // 兜底超时 8s。演进过程：1s → 3s → 8s。
                // 640×480×24 的 BMP 有 1801 个扇区，25MHz SPI 理论 0.3s，但 SD 卡
                // 每隔一阵有几十毫秒内部忙等，叠上 SDRAM 写争用很容易摸到 1s 边缘
                // → 偶发"加载失败"、数码管跳 load_diag=3、画面停在空白。
                // 支持任意分辨率后，1920×1080×24 有 12440 个扇区（6.2MB），理论
                // 就要 2.1s，故放宽到 8s（覆盖 4096×2048 量级）；更大的图会走超时
                // 分支，属预期行为。
                if (load_timeout_cnt > 32'd800_000_000) begin
                    // 强制释放 busy，并把这张图标记为失败，
                    // 后续切换会自动跳过它，不会一直卡在同一张图上。
                    load_busy             <= 1'b0;
                    load_timeout_cnt      <= 32'd0;
                    source_done_seen      <= 1'b0;
                    write_done_seen       <= 1'b0;
                    load_abort            <= 1'b1;
                    load_diag             <= 4'd3;   // 3 = 8s 兜底超时（读链路卡死）
                    load_fail_pulse       <= 1'b1;
                    fail_was_timeout      <= 1'b1;   // 超时不重试，直接跳过这张
                    // 注意：这里**不能**清 next_req_pending。
                    // 原来清掉会把"加载期间按下的 KEY0"一起丢弃，
                    // 表现为按键偶尔没反应。请求保留，等 busy 释放后自动补切。
                end else begin
                    load_timeout_cnt <= load_timeout_cnt + 32'd1;
                end
            end else begin
                load_timeout_cnt <= 32'd0;
            end

            // ---- 加载"早退"检测 ----
            // bmp_read 遇到 BMP 头不合法、扇区无效、FAT 簇链提前终止等情况会直接
            // 回 IDLE，既不产生 write_finish 也不通知外面。只靠兜底超时会让按键
            // 卡满 8s；这里判"源侧已结束、整帧写完成却一直不来"，提前判失败。
            // 阈值 500ms：帧写完成本来就会比源侧结束晚一点（FIFO 里残留的几百个
            // 像素要写进 SDRAM），三缓冲后写与显示读并发争用 SDRAM，这个尾巴会
            // 比双缓冲时更长。500ms 对"最后几个像素落盘"是很宽的余量，同时把
            // 真失败的恢复时间控制在 0.5s（原来是 8s）。
            if (load_busy && bmp_ready && source_done_seen && !write_done_seen) begin
                if (early_fail_cnt > 32'd50_000_000) begin    // 500ms @100MHz
                    load_busy        <= 1'b0;
                    early_fail_cnt   <= 32'd0;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    // 4 = 源侧读完（或提前结束）但整帧写完成始终不来
                    // 5 = BMP 头就没过（bmp_read 从未 armed）
                    load_diag        <= load_armed ? 4'd4 : 4'd5;
                    load_fail_pulse  <= 1'b1;
                    // 同上：保留按键请求，不在这里丢弃
                    load_abort       <= 1'b1;
                end else begin
                    early_fail_cnt <= early_fail_cnt + 32'd1;
                end
            end else begin
                early_fail_cnt <= 32'd0;
            end

            // ---- 失败重试 / 拉黑 / 整机重扫 ----
            // 一次加载失败多半是 SD 读抖动（卡忙、token 抖一下），立刻跳过
            // 会把本来能显示的图也丢掉，越跳越少，最后只剩一张就不动了。
            // 所以便宜的失败（500ms 早退）允许原地重试 1 次。
            //
            // 8s 兜底超时是另一回事：它意味着读链路已经卡死，原地重读同一张
            // 只会在同一条死路上再撞 8s —— 3 次就是 24s 无响应，正是用户看到的
            // "长延迟后直接跳到第一张"。这类失败不重试，直接跳过这张，
            // 靠"成功一次就清空 fail_map"的机制自然恢复。
            fail_set_r <= 4'b0000;
            fail_clr_r <= 4'b0000;

            if (load_frame_done) begin
                fail_clr_r    <= 4'b1111;     // 成功一次 → 全部解除拉黑
                retry_pending <= 1'b0;
                retry_cnt     <= 2'd0;
                fail_streak   <= 4'd0;
            end else if (load_fail_pulse) begin
                if (!fail_was_timeout && (retry_cnt < 2'd1)) begin
                    retry_cnt     <= retry_cnt + 2'd1;
                    retry_pending <= 1'b1;    // 下次仍加载同一张
                end else begin
                    retry_cnt     <= 2'd0;
                    retry_pending <= 1'b0;
                    fail_set_r    <= 4'b0001 << load_idx;
                end

                // 连续 8 次都失败，说明不是"某张图坏了"，而是读通道失去字节
                // 对齐（一次超时后卡还在发旧扇区数据，后续 CMD17 收不到正确
                // 的 R1，会无限重试）。整机重扫一轮相当于软复位读链路。
                if (fail_streak >= 4'd7) begin
                    fail_streak   <= 4'd0;
                    fail_map      <= 4'b0000;
                    retry_pending <= 1'b0;
                    retry_cnt     <= 2'd0;
                    scan_kicked   <= 1'b0;
                end else begin
                    fail_streak <= fail_streak + 4'd1;
                end
            end

            // ---- 首次扫描看门狗 ----
            // 上电后若因 SD 读卡死导致扫描迟迟不完成（scan_done 恒 0），
            // 所有按键都会被 scan_done 条件挡住。这里自动重新发起扫描。
            if (scan_kicked && !scan_done && (img_found_count == 3'd0)) begin
                if (scan_wd_cnt > 32'd100_000_000) begin       // 1s @100MHz
                    scan_wd_cnt      <= 32'd0;
                    scan_start_pulse <= 1'b1;
                end else begin
                    scan_wd_cnt <= scan_wd_cnt + 32'd1;
                end
            end else begin
                scan_wd_cnt <= 32'd0;
            end

            // 上电后自动发起一次“扫描前 4 张 BMP”
            if (!scan_kicked && bmp_ready) begin
                scan_start_pulse      <= 1'b1;
                scan_kicked           <= 1'b1;
                first_image_committed <= 1'b0;
                auto_play_en          <= 1'b0;
                auto_cnt              <= 32'd0;
                img_found_count       <= 3'd0;
                img_idx               <= 2'd0;
                load_idx              <= 2'd0;
                pending_buf_idx       <= 2'd0;
                write_buf_idx         <= 2'd0;
                disp_buf_idx          <= 2'd0;
                next_req_pending      <= 1'b0;
                display_valid         <= 1'b0;
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
                fail_map              <= 4'b0000;
                fail_streak           <= 4'd0;
                retry_pending         <= 1'b0;
                retry_cnt             <= 2'd0;
                // 整机重扫：预读出来的那一帧也一并作废，否则会带着上一轮的
                // 旧 pre_ready 上屏，screen 上跳出一张和当前图号对不上的画面。
                pre_ready             <= 1'b0;
                pre_buf               <= 2'd0;
                pre_idx               <= 2'd0;
                swap_on_commit        <= 1'b0;
            end else begin
                // ---- KEY0（手动下一张）请求锁存 ----
                // 改成"只要按下就记住"，把 scan_done / img_found_count / !load_busy
                // 这些条件全部挪到下面的消费端判断。效果：
                //   · 正在加载某张图时按下，不会被吞，忙完自动补切；
                //   · SD 还在扫描时按下，也会被记住，扫描完成后执行一次。
                // 只锁 1 位，不会累积成"连跳两张"。
                // 看门狗：2s 仍未被执行（例如卡彻底读不出来）就自动丢弃，
                // 避免请求永久挂着、之后突然自己切一张。
                if (!next_req_pending)
                    pending_wd <= 32'd0;
                else if (pending_wd > 32'd800_000_000)     // 8s @100MHz
                    next_req_pending <= 1'b0;
                else
                    pending_wd <= pending_wd + 32'd1;

                // 放在看门狗之后：同一拍按键时，置位优先，不会被看门狗盖掉
                if (key_next_press)
                    next_req_pending <= 1'b1;

                // 自动播放开/关
                if (key_auto_press && scan_done && (img_found_count > 3'd1)) begin
                    auto_play_en <= ~auto_play_en;
                    auto_cnt     <= 32'd0;
                end

                // 自动播放停留计数。
                //
                // 与旧实现的区别：**去掉了 !load_busy 门控**。
                // 旧实现要求"加载结束后才开始数 3 秒"，于是真实切换周期是
                // "3s + 读卡耗时"——读 1080P 要 2s，那张图就会在屏幕上停 5s。
                // 现在预读在后台跑，屏幕上一整段时间都是这张图，所以整段都算
                // 停留；到点若预读已完成就瞬间切走，观感是严格的 3 秒一张。
                //
                // 到点但预读没好时，计数器**必须饱和**（停在终点），不能清零重数：
                // 清零重数会让"迟到 0.4s 的预读"多赔一整圈 3 秒，这正是上一轮
                // "第 4 张转第 1 张特别慢"的直接原因（1080P 读卡约 3s，刚好吃不满
                // 窗口）。饱和之后，auto_due 一直为 1，pre_ready 落地的下一拍就切。
                if (scan_done && auto_play_en && display_valid && first_image_committed && (img_found_count > 3'd1)) begin
                    if (swap_now)
                        auto_cnt <= 32'd0;                 // 上屏 → 停留重新计时
                    else if (auto_cnt < AUTO_TICK_LAST)
                        auto_cnt <= auto_cnt + 32'd1;      // 到顶即停，等 pre_ready 补切
                end else begin
                    auto_cnt <= 32'd0;
                end

                // ============================================================
                // 加载调度（三缓冲）
                //
                // ① swap_now        预读好的帧已经躺在 pre_buf 里 → 只改 disp 指针，
                //                   一个时钟周期完成，屏幕上"啪"地换掉，零等待。
                //                   这是自动轮播能严格 3 秒一张的原因，也是
                //                   KEY0 能立即响应的原因。
                // ② 手动 / 首图     没有现成帧 → 读卡，读完立即上屏（swap_on_commit）。
                // ③ 预读            上屏之后立刻在后台读下一张，写进第三块缓冲。
                //                   读完就停在 pre_ready，不再空转读卡。
                // ============================================================

                // ---- ① 上屏：dwell 到点 或 手动按键，且预读已完成 ----
                if (swap_now) begin
                    disp_buf_idx     <= pre_buf;
                    img_idx          <= pre_idx;
                    display_valid    <= 1'b1;
                    pre_ready        <= 1'b0;
                    auto_cnt         <= 32'd0;      // 停留计时从这一次上屏重新开始
                    next_req_pending <= 1'b0;
                    load_diag        <= 4'd2;
                end
                // ---- ② 手动下一张：没有预读好的帧，只能现读，读完立即上屏 ----
                // 不要求 display_valid：首图失败时 display_valid 恒 0，
                // 加了这个条件 KEY0 就永远没反应（历史踩坑）。
                // 也不再门控 !disp_busy：三缓冲下写目标既不等于当前显示块、
                // 也不等于转场正在淡出的旧块，转场期间读卡是安全的。
                else if (scan_done && bmp_ready && !load_busy && !pre_ready && settle_ok && next_req_pending && (img_found_count != 3'd0)) begin
                    load_idx         <= target_idx;
                    load_sector      <= sector_lut(target_idx);
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    fail_was_timeout <= 1'b0;   // 新一轮加载，失败原因重新判定
                    swap_on_commit   <= 1'b1;   // 手动：读完马上上屏
                    next_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                    if (!display_valid)
                        load_diag    <= 4'd1;   // 还没有画面 → 这一张相当于首图，标记加载中
                end
                // ---- ② 首图自动加载到 buffer0（0 号失败过才跳到下一张好图）----
                else if (scan_done && !first_image_committed && bmp_ready && !load_busy && (img_found_count != 3'd0)) begin
                    load_idx         <= first_load_idx;
                    load_sector      <= sector_lut(first_load_idx);
                    pending_buf_idx  <= 2'd0;
                    write_buf_idx    <= 2'd0;
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    fail_was_timeout <= 1'b0;
                    swap_on_commit   <= 1'b1;   // 首图没有旧图可留，立即上屏
                    next_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                    load_diag        <= 4'd1;   // 首图加载中
                end
                // ---- ③ 后台预读下一张 ----
                // 这是本次改造的核心：上屏后立刻开始读下一张，不等 dwell 到点。
                // 3 秒停留期内把它读完，到点就只做一次指针切换 → 观感严格 3s 一张。
                //
                // 不门控 auto_play_en：手动模式下也预读，于是 KEY0 同样是零延迟。
                // 读完即停在 pre_ready（第 ③ 条被 !pre_ready 挡住），不会反复读卡。
                else if (scan_done && bmp_ready && display_valid && !load_busy && !pre_ready
                         && settle_ok && (img_found_count > 3'd1)) begin
                    load_idx         <= target_idx;
                    load_sector      <= sector_lut(target_idx);
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    fail_was_timeout <= 1'b0;
                    swap_on_commit   <= 1'b0;   // 后台预读：读完只挂起，等 dwell
                    // 注意：这里**不能**清 auto_cnt。
                    // 停留计时只在上屏那一刻归零；若预读启动也清一次，那么每次
                    // 失败重试都会把 3 秒重新数一遍，屏幕上这张图就会被无限延长。
                end
            end
        end
    end
end

bmp_read bmp_read_m0(
    .clk                    (clk),
    .rst                    (rst),
    .ready                  (bmp_ready),

    .scan_start             (scan_start_pulse),
    .scan_start_sector      (SCAN_START_SECTOR),
    .scan_max_sector        (SCAN_MAX_SECTOR),
    .scan_target_count      (SCAN_TARGET_COUNT),
    .scan_done              (scan_done),
    .scan_found_valid       (scan_found_valid),
    .scan_found_sector      (scan_found_sector),
    .scan_found_total       (scan_found_total),

    .load_start             (load_start_pulse),
    .load_abort             (load_abort),
    .load_sector            (load_sector),

    .sd_init_done           (sd_init_done),
    .state_code             (state_code_i),
    .img_w                  (src_w),
    .img_h                  (src_h),
    .write_req              (write_req),
    .write_req_ack          (write_req_ack),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .pix_de                 (pix_de),
    .pix_data               (pix_data),
    .pix_x                  (pix_x)
);

// ============================================================================
// 任意分辨率 → bmp_width × bmp_height 的双线性缩放
// 放在写侧：SD 读一张图要 1 秒量级，而生成 307200 个像素只要 3ms 量级，
// 时间余量极大，不会影响显示通路的实时性。
// KEEP_ASPECT=1：保持宽高比，多出来的边留黑，避免 16:9 被拉成 4:3。
// ============================================================================
img_scaler #(
    .DST_W       (640),
    .DST_H       (480),
    .KEEP_ASPECT (1)
) u_img_scaler (
    .clk         (clk),
    .rst         (rst),
    .frame_start (sc_frame_start),
    .dst_w       (bmp_width),
    .dst_h       (bmp_height),
    .src_de      (pix_de),
    .src_pixel   (pix_data),
    .src_x       (pix_x),
    .src_w       (src_w),
    .src_h       (src_h),
    .out_de      (sc_de),
    .out_pixel   (sc_pixel),
    .frame_done  (sc_done),
    .params_ok   (sc_params_ok)
);

sd_card_top sd_card_top_m0(
    .clk                    (clk),
    .rst                    (rst),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

// ============================================================================
// 切图耗时测量
// ----------------------------------------------------------------------------
// 四个观测点全部取本模块内已有的信号，不额外增加对 bmp_read / SDRAM 的探针：
//   load_start_pulse   开始读卡（预读模式下这一拍屏幕上还在显示旧图）
//   load_frame_done    整帧写好 —— "读卡花了多久"就是这一段
//   swap_now |         **这一张正式上屏**的那一拍。三缓冲之后读卡与上屏
//   load_commit_now    彻底解耦（预读会在上一张的停留期里就读完下一张），
//                      所以"发布"必须挂在上屏，不能挂在写完 —— 否则第 N 张
//                      还在屏幕上时，右上角显示的已经是第 N+1 张的读卡耗时。
//   disp_busy          转场忙（video_clk 侧同步过来的，落下即"新图完全可见"）
// 按键受理时刻则要在 switch_latency 内部用 key_next_press 记录，这里只透传。
// ============================================================================
switch_latency #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ)
) u_switch_latency (
    .clk          (clk),
    .rst          (rst),
    .key_press    (key_next_press),
    .req_pending  (next_req_pending),
    .load_start   (load_start_pulse),
    .commit       (load_frame_done),                 // 整帧写好（只用于算 L）
    .swap         (swap_now || load_commit_now),     // 这一张上屏（只有它触发发布）
    .disp_busy    (disp_busy),
    .wait_cs      (sw_wait_cs),
    .load_cs      (sw_load_cs),
    .total_cs     (sw_total_cs),
    .upd_tgl      (sw_upd_tgl)
);

endmodule

// ============================================================================
// switch_latency —— 切图耗时测量（SD 卡时钟域，10ms 时基）
// ----------------------------------------------------------------------------
// 为什么这个模块写在本文件里，而不是 rtl/switch_latency.v：
//   TD 打开工程时会重写 .al，手工追加的源码条目会被丢掉，综合时本模块就成了
//   黑盒（报 "switch_latency ... is a black box"）。本文件底部本来就有第二个
//   module（key_press_debounce），说明"一个文件放多个 module"是本工程既有写法，
//   并进来就不依赖 .al 登记了。
//
// 量什么（全部在 sd_card_clk 域）
//   t_req   这张图"开始处理"的时刻：
//             手动切图（req_pending=1）→ 按键受理时刻；
//             自动轮播（req_pending=0）→ 开始读卡时刻，于是 W 自然为 0。
//   t_load  真正开始读卡（load_start_pulse）
//   t_cmt   整帧写完（load_frame_done）
//   t_sw    这一张上屏（swap_now 或 load_commit_now）
//   t_end   转场忙落下（新图完全可见）
//   wait  = t_load - t_req   开始处理 → 开始读卡（被静默窗 / 调度挡住的那段）
//   load  = t_cmt  - t_load  SD 读扇区 + 缩放 + 写 SDRAM
//   total = t_end  - t_req   开始处理 → 新图完全可见
//
// 为什么发布挂在"上屏"而不是"写完"（2026-11-02 重写本模块的原因）
//   双缓冲时代从"按键"到"上屏"是一条串行链，写完那一刻基本就是上屏那一刻，
//   用 load_frame_done 发布没有问题。三缓冲预读把这根链子拆成了两段：
//   第 N 张上屏后 50ms 就开始读第 N+1 张，2.4s 就读完，而屏幕上还在放第 N 张。
//   若仍在"写完"时发布，第 N 张的停留期里 OSD 数字已被第 N+1 张覆盖，
//   看到的就是"这张图配着下一张的耗时"。
//   现在：写完只把结果挂起（p_*），等这一张真正上屏时再搬进发布影子（q_*），
//   然后等这张图的转场结束才翻转 upd_tgl。
//
// 为什么测量和发布要拆成两级寄存器
//   预读让同一张图的两个动作在时间上重叠：第 N 张的淡入（约 1.07s）还没走完，
//   第 N+1 张的读卡（load_start）就已经开始了。若共用一个状态机，
//   load_start 会把还没发布完的第 N 张结果冲掉。拆级之后：
//     · m_* / p_*  只描述"读卡"这一段，由 load_start 重置；
//     · q_*        只在 swap 时装载，此后与读卡阶段彻底无关。
//
// 跨域：三个值定稿后 upd_tgl 翻转一次，video_clk 侧两级同步 + 边沿检出整组锁存。
// 只在 total 定稿时翻转，保证三个数同批、互相自洽。
//
// 四个有意为之的行为
//   · 9s 内没写完视为加载失败，结果不更新，屏幕上保留上一次成功的数字；
//   · 上屏后 300ms 没等到 disp_busy，认为这次没有转场，total 取到上屏时刻
//     （首图走 start_phase_b=1，转场只做淡入，busy 照样会起，这条只是兜底）；
//   · 转场超过 6s 仍未落下，按当前时刻强制发布，避免发布状态机永久挂住；
//   · swap 到来时若手上没有挂起结果（理论上不该发生），本次不发布，
//     屏幕上保留旧数字 —— 宁可少更新一次，也不发出错批的数据。
//
// 口径提醒（保持旧定义，2026-11-02 与用户确认）
//   三种触发方式下 t_req 的取法不同，但都落在"这张图的请求时刻"上：
//     · 自动轮播        = 开始读卡时刻（预读在上一张上屏后立刻启动），
//                         所以 T 会包含"等停留到点"：1080P 上 T ≈ 4.1s
//                         （≈0.05 静默窗 + 2.4 读卡 + 0.6 停留余量 + 1.07 转场）；
//     · 按键 + 预读命中 = 按键时刻（帧早就读好了，不产生新的 load_start），
//                         W 记 0，T ≈ 转场 1.07s；
//     · 按键 + 无预读   = 按键时刻，W/L 照常（这条是唯一"读卡真的在等你"的路径）。
//   要判断"3s 停留期内读不读得完"，看的是 L，不是 T。
//
// 量程：cs_cnt 16bit（655s），输出饱和到 999 即 9.99s，正常一次切换 1~4s。
// ============================================================================
module switch_latency #(
    parameter integer CLK_FREQ_HZ = 100_000_000
)(
    input  wire        clk,           // sd_card_clk
    input  wire        rst,

    input  wire        key_press,     // KEY0 去抖后的单拍脉冲
    input  wire        req_pending,   // next_req_pending 当前值（判断本次是否按键触发）
    input  wire        load_start,    // load_start_pulse 单拍
    input  wire        commit,        // load_frame_done 单拍：整帧写完
    input  wire        swap,          // 这一张上屏单拍：swap_now | load_commit_now
    input  wire        disp_busy,     // 转场忙（已同步到本域）

    output reg  [9:0]  wait_cs,       // 等待：开始处理 → 开始读卡
    output reg  [9:0]  load_cs,       // 加载：开始读卡 → 整帧写完
    output reg  [9:0]  total_cs,      // 总计：开始处理 → 新图完全可见
    output reg         upd_tgl        // 结果更新翻转脉冲（给 video 域）
);

    // ------------------------------------------------------------------
    // 10ms 时基
    // ------------------------------------------------------------------
    localparam integer CS_DIV = (CLK_FREQ_HZ >= 100) ? (CLK_FREQ_HZ / 100) : 1;

    reg  [19:0] pre;
    wire        cs_tick = (pre == (CS_DIV - 1));

    always @(posedge clk or posedge rst) begin
        if (rst)
            pre <= 20'd0;
        else
            pre <= cs_tick ? 20'd0 : (pre + 20'd1);
    end

    // 自由运行的 10ms 计数。用"差值"而不是"清零重计"，是为了让 t_req / t_load /
    // t_cmt / t_sw / t_end 五个时刻可以同时有效，不必为每个时刻各开一个计数器。
    reg  [15:0] cs_cnt;

    always @(posedge clk or posedge rst) begin
        if (rst)
            cs_cnt <= 16'd0;
        else if (cs_tick)
            cs_cnt <= cs_cnt + 16'd1;
    end

    // 差值可能超过 999，统一在这里饱和，免得 OSD 那边还要再判一次
    function [9:0] sat10;
        input [15:0] v;
        begin
            sat10 = (v > 16'd999) ? 10'd999 : v[9:0];
        end
    endfunction

    // ------------------------------------------------------------------
    // t_req：按键时刻。单独一个 always，避免和状态机争同一个寄存器
    // ------------------------------------------------------------------
    reg [15:0] t_req;

    always @(posedge clk or posedge rst) begin
        if (rst)
            t_req <= 16'd0;
        else if (key_press)
            t_req <= cs_cnt;
    end

    // 说明为什么用 req_pending_d 而不是 req_pending：
    //   load_start_pulse 是在分支里"下一拍才为 1"，而同一拍 next_req_pending
    //   已经被清成 0。等 load_start 为 1 时再去读 req_pending，读到的必然是 0，
    //   按键触发的这一路就被误判成自动轮播、等待时间恒为 0。所以先打一拍。
    reg req_pending_d;

    always @(posedge clk or posedge rst) begin
        if (rst)
            req_pending_d <= 1'b0;
        else
            req_pending_d <= req_pending;
    end

    // ==================================================================
    // 第一级：读卡测量 —— 这张图从"开始处理"到"整帧写完"
    // ==================================================================
    localparam [15:0] TO_LOAD_CS = 16'd900;   // 9s 没写完 = 加载失败
    localparam [15:0] TO_RISE_CS = 16'd30;    // 上屏后 300ms 等不到转场
    localparam [15:0] TO_FALL_CS = 16'd600;   // 转场 6s 还没落下 = 强制发布

    reg        m_act;          // 正在测量读卡
    reg [15:0] t_start;        // 这张图"开始处理"的时刻
    reg [15:0] t_load;         // 开始读卡的时刻
    reg [15:0] m_to;           // 读卡超时计数（10ms）

    reg [9:0]  p_load;         // 挂起：这张图的读卡耗时
    reg [9:0]  p_wait;         // 挂起：这张图的等待耗时
    reg [15:0] p_treq;         // 挂起：这张图开始处理的时刻
    reg        p_valid;        // 挂起结果有效（等 swap 消费）

    wire commit_now = m_act && commit;

    // 手动 / 首图路径下 commit 与 swap 同一拍到来，此时 p_* 还是旧值，
    // 用组合旁路给出"本拍应有"的读数，否则会把上一张的数字发布出去。
    wire [9:0]  p_load_n = commit_now ? sat10(cs_cnt - t_load)  : p_load;
    wire [9:0]  p_wait_n = commit_now ? sat10(t_load  - t_start) : p_wait;
    wire [15:0] p_treq_n = commit_now ? t_start : p_treq;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            m_act   <= 1'b0;
            t_start <= 16'd0;
            t_load  <= 16'd0;
            m_to    <= 16'd0;
            p_load  <= 10'd0;
            p_wait  <= 10'd0;
            p_treq  <= 16'd0;
            p_valid <= 1'b0;
        end else if (load_start) begin
            // 新一轮读卡开始：无论上一轮是否收尾都重新起算
            t_start <= req_pending_d ? t_req : cs_cnt;
            t_load  <= cs_cnt;
            m_to    <= 16'd0;
            m_act   <= 1'b1;
            // 上一份挂起结果作废（正常情况下它已在 swap 时被消费掉；
            // 走到这里还留着，说明那张图根本没上屏，数字不该再发出去）
            p_valid <= 1'b0;
        end else if (commit_now) begin
            p_load  <= sat10(cs_cnt - t_load);
            p_wait  <= sat10(t_load  - t_start);
            p_treq  <= t_start;
            p_valid <= 1'b1;
            m_act   <= 1'b0;
        end else if (m_act && cs_tick) begin
            // 9s 还没写完 = 加载失败（sd_card_bmp 那边 8s 超时兜底，这里晚 1s
            // 收尾，避免抢在它前面误判）。结果不发布，屏幕上保留上一次数字。
            if (m_to >= TO_LOAD_CS)
                m_act <= 1'b0;
            else
                m_to <= m_to + 16'd1;
        end
    end

    // ==================================================================
    // 第二级：发布 —— 这一张上屏后，等它的转场走完再整批送出去
    //   与第一级完全解耦：下一张的 load_start 随时可以进来改写 m_*/p_*，
    //   不会影响这里已经装进 q_* 的这一张的数字。
    // ==================================================================
    localparam [1:0] Q_IDLE = 2'd0,   // 没在发布
                     Q_RISE = 2'd1,   // 已上屏，等转场起来
                     Q_FALL = 2'd2;   // 转场中，等它落下（新图完全可见）

    reg  [1:0]  q_st;
    reg  [9:0]  q_load;        // 本次要发布的读卡耗时
    reg  [9:0]  q_wait;        // 本次要发布的等待耗时
    reg  [15:0] q_treq;        // 本次这张图"开始处理"的时刻
    reg  [15:0] q_swap_t;      // 上屏时刻（无转场时 total 取到这里）
    reg  [15:0] q_to;          // 发布阶段超时计数（10ms）

    // 按键直接上屏（预读命中）：这一拍 next_req_pending 还没被清掉，所以 req_pending
    // 仍为 1。这种切图**不产生新的 load_start**（帧早读好了），若还沿用 p_treq
    // （= 这张图开始读卡的时刻，可能是十几秒前），T 会显示成"从上一张上屏到现在"，
    // 按了 10s 之后还会撞上 9.99 的饱和值。所以按"这张图的请求时刻 = 按键时刻"
    // 重新取基准，等待则记为 0 —— 这张图确实是零等待就在手里。
    wire        swap_key  = swap && req_pending;
    wire [15:0] q_treq_n  = swap_key ? t_req  : p_treq_n;
    wire [9:0]  q_wait_n  = swap_key ? 10'd0  : p_wait_n;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            q_st      <= Q_IDLE;
            q_load    <= 10'd0;
            q_wait    <= 10'd0;
            q_treq    <= 16'd0;
            q_swap_t  <= 16'd0;
            q_to      <= 16'd0;
            wait_cs   <= 10'd0;
            load_cs   <= 10'd0;
            total_cs  <= 10'd0;
            upd_tgl   <= 1'b0;
        end else if (swap) begin
            // 上屏：把这一张的读数搬进发布影子，开始等转场
            if (p_valid || commit_now) begin
                q_load   <= p_load_n;
                q_wait   <= q_wait_n;
                q_treq   <= q_treq_n;
                q_swap_t <= cs_cnt;
                q_to     <= 16'd0;
                q_st     <= Q_RISE;
            end
        end else begin
            case (q_st)
                // ----------------------------------------------
                Q_IDLE: ;
                // ----------------------------------------------
                Q_RISE: begin
                    if (disp_busy) begin
                        q_to <= 16'd0;
                        q_st <= Q_FALL;
                    end else if (cs_tick) begin
                        // 300ms 内转场没起来：按"本次没有转场"处理，
                        // total 取到上屏时刻
                        if (q_to >= TO_RISE_CS) begin
                            wait_cs  <= q_wait;
                            load_cs  <= q_load;
                            total_cs <= sat10(q_swap_t - q_treq);
                            upd_tgl  <= ~upd_tgl;
                            q_st     <= Q_IDLE;
                        end else begin
                            q_to <= q_to + 16'd1;
                        end
                    end
                end
                // ----------------------------------------------
                Q_FALL: begin
                    if (!disp_busy) begin
                        // 转场落下 = 新图完全可见，三个数定稿，整批发布
                        wait_cs  <= q_wait;
                        load_cs  <= q_load;
                        total_cs <= sat10(cs_cnt - q_treq);
                        upd_tgl  <= ~upd_tgl;
                        q_st     <= Q_IDLE;
                    end else if (cs_tick) begin
                        // 6s 兜底：转场正常 1.07s，卡这么久说明转场那边出问题，
                        // 按当前时刻强制发布，不让状态机永久挂住
                        if (q_to >= TO_FALL_CS) begin
                            wait_cs  <= q_wait;
                            load_cs  <= q_load;
                            total_cs <= sat10(cs_cnt - q_treq);
                            upd_tgl  <= ~upd_tgl;
                            q_st     <= Q_IDLE;
                        end else begin
                            q_to <= q_to + 16'd1;
                        end
                    end
                end
                // ----------------------------------------------
                default: q_st <= Q_IDLE;
            endcase
        end
    end

endmodule


module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,    // 默认：松开=1，按下=0
    output reg  press_pulse
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg button_sync0;
reg button_sync1;
reg button_stable;
reg [31:0] cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        press_pulse  <= 1'b0;

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else begin
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;
                button_stable <= button_sync1;
                cnt           <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule
