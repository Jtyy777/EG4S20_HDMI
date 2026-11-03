
# ============================================================
# timing.sdc —— 工程时序约束
# 适用：数字标牌信息发布终端（TF 图片轮播 + HDMI 视频/音频）
#
# 约束分四步，顺序不能乱：
#   1) 先声明板级输入主时钟（clk = 50MHz）；
#   2) 用 derive_clocks 让工具自动把 PLL 输出都推出来；
#   3) 给 PLL 输出改回代码里的名字，报告才看得懂；
#   4) 声明互斥时钟组 —— 只对「设计上确实不同时用」的时钟这样做。
#
# 关键的取舍：clock group 不是越多越好。每加一组 -asynchronous，
# 就等于告诉工具「这两个时钟之间的路径不用分析」，真有问题也看不出来了。
# 所以本文件只对像素时钟和 5 倍串行时钟做了 exclusive（理由见第 4 节），
# 其余全部保持默认的同步分析。
# ============================================================

# ------------------------------------------------------------
# 1. 板级输入主时钟
#    clk 来自核心板 50MHz 晶振（R7），周期 20ns，占空比 50%。
#    这个时钟必须第一个声明 —— 后面所有 rename_clock 都要把它当 master。
# ------------------------------------------------------------
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]

# ------------------------------------------------------------
# 2. 自动推导 PLL 输出时钟
#    三个 PLL（sys_pll / video_pll / PLL_HDMI_AUDIO）的输出都由这一句
#    按 PLL 的分频倍频参数自动算出周期，不需要手写。
# ------------------------------------------------------------
derive_clocks

# ------------------------------------------------------------
# 3. 给关键 PLL 输出命名
#    推导出来的时钟默认是引脚路径名，报告里读起来很痛苦，这里改成代码里
#    的线网名。实例名与顶层一致：
#      sys_pll   → sys_pll_m0     ：clk0 = sd_card_clk，clk1 = ext_mem_clk
#      video_pll → video_pll_m0   ：clk0 = video_clk， clk1 = hdmi_5x_clk
#      PLL_HDMI_AUDIO → u_audio_pll：clk2 = audio_mclk
#
#    若综合后实例层级和这里对不上（改名、被优化掉），要以 Timing Analyzer
#    里实际的 pin 路径为准改这几行，否则 rename 会静默失效。
# ------------------------------------------------------------
rename_clock -name {sd_card_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {ext_mem_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {video_clk}   -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {hdmi_5x_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[1]}]


# ------------------------------------------------------------
# 4. 像素时钟与 5 倍串行时钟互斥
#    这两个时钟同出 video_pll：clk0 = 25.175MHz（像素），clk1 = 125.875MHz
#    （5 倍，供 HDMI 串行化用）。它们之间确实存在数据交接（lane_lvds_10_1
#    里用 odd_even 翻转做装载握手），但那是设计上有意的「相位握手」，
#    并非一拍一拍的同步逻辑，用同步时序去卡它没有物理意义，只会刷出大量
#    假违例。所以这里声明成 exclusive：两者之间的路径不参与时序分析。
#
#    前提是：跨这两域的信号只有那套握手（odd_even / load_en），没有别的
#    单比特直通信号。以后若新增 pixel_clk → hdmi_5x_clk 的直接路径，
#    这条约束会把真问题一起藏掉，必须同步检查。
# ------------------------------------------------------------
set_clock_groups -exclusive \
    -group [get_clocks {video_clk}] \
    -group [get_clocks {hdmi_5x_clk}]



# ------------------------------------------------------------
# 5. 其余跨域分组：保持关闭（默认做同步分析）
#    目前 sd_card_clk / ext_mem_clk / video_clk / clk 之间的跨域全部靠
#    异步 FIFO 或两级同步器处理，理论上都可以开成 -asynchronous。但一旦
#    开了，这几条域之间的路径就完全不报了，包括写错的那些。
#
#    建议流程：先按现在这样看报告，确认跨域只经过 FIFO / 同步器之后，
#    再逐组打开，而不是一次性全开。
# ------------------------------------------------------------
# set_clock_groups -asynchronous \
#     -group [get_clocks {sd_card_clk}] \
#     -group [get_clocks {ext_mem_clk}] \
#     -group [get_clocks {video_clk}] \
#     -group [get_clocks {hdmi_5x_clk}]

# ------------------------------------------------------------
# 5b. clk → video_clk：只走 param_cdc 握手，声明为 false path
#
#     这对时钟的参考源是同一个 50MHz，TD 会当成同步关系分析：clk 上升沿
#     (20ns) 发射、video_clk 上升沿 (40ns) 捕获，预算只有 20ns。
#
#     历史上这条跨域是 18 位裸线直连（param_ctrl → brightness_contrast /
#     osd_overlay），既没同步器也谈不上时序保障，于是 OSD 那边算对比度
#     十进制的那条 24 级组合链常年挂在 -5.7 ~ -7.6ns，还带着 GP 最大违例
#     的帽子。那本来只是一条「按键才变一次」的配置总线，按数据通路卡它
#     没有物理意义。
#
#     现在硬件上已经改成 param_cdc 的「翻转脉冲 + 整组锁存」：数据在源侧
#     影子寄存器里稳定两拍以上才被目的侧采走，tag 走三级同步。所以这里
#     对整对时钟声明 false path。
#
#     前提（每次看报告都要复查）：clk → video_clk 除 param_cdc 的握手外
#     不能有别的直通信号。判据是「该组的终点只能是 param_cdc 里的
#     bright_dst / contrast_dst / tgl_v[0]」。
#     改前那一轮（10-29 21:25）该组 38 个端点全在 brightness_contrast 的
#     *_adj_reg 和 osd_overlay 的 rgb_out_r_reg 上，正是裸总线直连的证据；
#     补上 param_cdc 之后这些端点应当整体消失。
#     以后若新增 clk 域信号直接进 video_clk，必须删掉这条约束或改成
#     -through 精确限定，否则会把真问题一起藏掉。
# ------------------------------------------------------------
set_false_path -from [get_clocks {clk}] -to [get_clocks {video_clk}]

# ------------------------------------------------------------
# 6. 输入/输出延时
#    本工程对外只有按键、拨码、数码管、SD 卡 SPI 和 HDMI 串行输出，
#    没有需要约束建立/保持关系的并行总线（SD 卡走 SPI，速率由软件分频
#    决定，远低于 IO 时序边界），所以 set_input_delay / set_output_delay
#    全部留空。若以后接并行接口（如摄像头、并口屏），这里必须补上。
# ------------------------------------------------------------
