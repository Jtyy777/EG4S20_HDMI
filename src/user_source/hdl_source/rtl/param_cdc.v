// ============================================================================
// param_cdc —— 亮度 / 对比度参数：clk(50MHz) → video_clk(25MHz) 准静态跨域
// ----------------------------------------------------------------------------
// 为什么需要这个模块
//   bright / contrast_q8 由 param_controller 在 clk 域产生（按键触发），而
//   brightness_contrast 和 osd_overlay 都跑在 video_clk 域。原来这两条总线
//   是直连的 —— 18 位裸跨异步时钟，没有同步器，也没有任何时序保障。
//
//   更麻烦的是 OSD 那一侧：对比度显示值每个像素都重算一遍
//   （×25 → 四舍五入 → /64 → 十进制比较链 → 字符选择），一共 24 级组合。
//   于是 TD 把「clk 发射、video_clk 捕获」当同步数据通路来卡，预算只有 20ns
//   （video_clk 上升沿 40ns − clk 上升沿 20ns），这条路径常年挂在
//   -5.7 ~ -7.6ns。可它根本不是数据通路，只是一条按键才变一次的配置总线。
//
// 做法：翻转脉冲 + 整组锁存（与顶层 sw_wait_cs 那套同构）
//   源侧：一旦发现参数与影子寄存器不同，就把新值锁进影子寄存器，同时翻转
//         tag。两条总线在同一拍更新，所以影子里永远是一致的一组快照。
//   目的侧：tag 过三级同步，检出翻转边沿后把影子寄存器整组锁存过来。检出时
//         影子已经稳定两拍以上，采到的一定是完整的一组值，不会新旧混搭
//         —— 这一点比「每条总线各自两级同步」严格：那样可能采到某一位还是
//         旧值的撕裂码（例如对比度 0x64 变成 0x66）。
//
// 时序上为什么可以不管
//   参数只在按键时变化，两次变化间隔是 100ms 量级，而三级同步延迟约 120ns。
//   即使连按到硬件极限，也不会漏掉变化：万一 tag 在一次翻转窗口内又翻了
//   一次，目的侧锁存的仍是最新快照，结果一样正确。
//   所以顶层对这对时钟只声明 set_false_path，不按同步通路卡。
//
// 复位
//   两侧都是异步复位。源侧影子寄存器初值为 0，复位释放后只要参数不为 0，
//   就会立刻触发一次 tag 翻转，目的侧随即锁存到正确初值 —— 不需要额外的
//   初始化握手。即使参数初值恰好全 0，锁存结果也与复位值相同，无副作用。
// ============================================================================
module param_cdc (
    input  wire        clk_src,         // 参数域时钟（clk，50MHz）
    input  wire        rst_src,
    input  wire [7:0]  bright_src,      // 有符号亮度偏移，-96~+96
    input  wire [9:0]  contrast_src,    // 对比度定点，256 = 1.0x

    input  wire        clk_dst,         // 使用域时钟（video_clk，25MHz）
    input  wire        rst_dst,
    output reg  [7:0]  bright_dst,      // 已同步到 video_clk 域
    output reg  [9:0]  contrast_dst
);

    // ------------------------------------------------------------------
    // 源侧：影子寄存器 + tag
    //   用「与影子比较」而不是让 param_controller 额外输出一个变化标志，
    //   这样不用改动 param_controller 的端口，职责也更清楚：
    //   谁跨域谁负责打包。
    // ------------------------------------------------------------------
    reg [7:0]  b_hold;
    reg [9:0]  c_hold;
    reg        tag;

    always @(posedge clk_src or posedge rst_src) begin
        if (rst_src) begin
            b_hold <= 8'd0;
            c_hold <= 10'd0;
            tag    <= 1'b0;
        end else if ((bright_src != b_hold) || (contrast_src != c_hold)) begin
            b_hold <= bright_src;
            c_hold <= contrast_src;
            tag    <= ~tag;
        end
    end

    // ------------------------------------------------------------------
    // 目的侧：三级同步链 + 边沿检出 + 整组锁存
    //   为什么是三级不是两级：tag 同步过来的第 2 级照理会带亚稳态衰减，
    //   用第 2、3 级的异或检出边沿，锁存动作发生在第 3 级更新之后，此时
    //   影子寄存器已稳定 ≥2 个 video_clk 周期，取数窗口最宽。
    // ------------------------------------------------------------------
    reg [2:0] tgl_v;
    wire      upd_pulse = tgl_v[2] ^ tgl_v[1];

    always @(posedge clk_dst or posedge rst_dst) begin
        if (rst_dst) begin
            tgl_v        <= 3'b000;
            bright_dst   <= 8'd0;
            contrast_dst <= 10'd0;
        end else begin
            tgl_v <= { tgl_v[1:0], tag };
            if (upd_pulse) begin
                bright_dst   <= b_hold;
                contrast_dst <= c_hold;
            end
        end
    end

endmodule
