# -*- coding: utf-8 -*-
"""
throughput.py —— 写侧吞吐与 sd_card_clk 降频可行性核算

回答一个具体问题：
    sd_card_clk 100MHz → 50MHz 之后，
    写侧（img_scaler → 跨域 FIFO → SDRAM burst 写）够不够 480p60？

所有常数都从 RTL 里读出来的，不是估的：
    · img_scaler 每像素 3 拍        src/user_source/hdl_source/rtl/img_scaler.v (G_OUT)
    · 写 FIFO 512 深 × 32bit        rfifo_32_32_512 / wfifo_32_32_512
    · burst 阈值 128 字             frame_fifo_write.v  BURST_SIZE
    · SPI DCLK = sys_clk / 4        spi_master.v  半周期 = (clk_div + 2)，clk_div=0
    · 加载超时 800_000_000 拍       sd_card_bmp.v

用法：python doc/power/throughput.py
"""

# ---------------------------------------------------------------------------
# 设计常数（改 RTL 时同步改这里，或者以后接解析器自动读）
# ---------------------------------------------------------------------------
DST_W, DST_H = 640, 480              # SDRAM 帧缓存尺寸
CYCLES_PER_PIXEL = 3                 # img_scaler G_OUT：三通道串行
LINE_OVERHEAD = 2                    # 每行额外 G_RD + G_CALC 两拍
FIFO_DEPTH = 512                     # wfifo_32_32_512
BURST_SIZE = 128                     # frame_fifo_write
MEM_CLK_MHZ = 125.0                  # ext_mem_clk
LOAD_TIMEOUT_CYCLES = 800_000_000    # sd_card_bmp 兜底超时
AUTO_DWELL_SEC = 3                   # 自动轮播间隔

# 480p60（VGA 标准 800×525 @ 25.175MHz）
VGA_TOTAL_W, VGA_TOTAL_H = 800, 525
VGA_PIXEL_CLK_MHZ = 25.175

# 素材：分辨率 + 24bpp BMP 的扇区数
# 扇区数 = ceil(宽×高×3 / 512) + FAT/目录开销（约 8~16 个扇区）
MATERIALS = [
    ("640×480",    640,  480,  1801),
    ("1024×768",  1024,  768,  4616),
    ("1440×1080", 1440, 1080,  9113),
    ("1920×1080", 1920, 1080, 12440),
]


def hz(mhz):
    return mhz * 1e6


# ---------------------------------------------------------------------------
# 1. 写侧生成能力 vs 480p60 实时需求
# ---------------------------------------------------------------------------
def write_side_rate(clk_mhz):
    """img_scaler 连续输出时的像素吞吐（Mpixel/s）。"""
    line_cycles = CYCLES_PER_PIXEL * DST_W + LINE_OVERHEAD
    frame_cycles = line_cycles * DST_H
    return hz(clk_mhz) / frame_cycles * DST_W * DST_H / 1e6, frame_cycles


def sec1_write_vs_480p():
    print("=" * 74)
    print("1. 写侧生成速率 vs 480p60 实时需求")
    print("=" * 74)

    live = hz(VGA_PIXEL_CLK_MHZ) * (DST_W * DST_H) / (VGA_TOTAL_W * VGA_TOTAL_H) / 1e6
    print("  每像素 %d 拍，每行 %d 拍（含行首 %d 拍开销）"
          % (CYCLES_PER_PIXEL, CYCLES_PER_PIXEL * DST_W + LINE_OVERHEAD, LINE_OVERHEAD))
    print("  480p60 有效像素率 = %.3f Mpixel/s（800×525@25.175MHz 折算）" % live)
    print()
    print("  %-10s %-12s %-12s %-10s %s" % ("sd_card_clk", "一帧拍数", "一帧耗时", "像素率", "能否撑 480p60"))
    print("  " + "-" * 68)

    for mhz in (100, 50):
        rate, frame_cycles = write_side_rate(mhz)
        frame_ms = frame_cycles / hz(mhz) * 1e3
        ok = "够，余量 %.2fx" % (rate / live) if rate >= live else "不够，缺 %.1f%%" % (100 * (1 - rate / live))
        print("  %-10s %-12d %-12s %-10s %s"
              % ("%dMHz" % mhz, frame_cycles, "%.2fms" % frame_ms, "%.2f MP/s" % rate, ok))

    print()
    print("  注：这是「img_scaler 一刻不停地生成」的上限速率。")
    print("      本工程的写侧不是视频流，只在加载一张静态图时工作，")
    print("      所以即使 50MHz 下低于 480p60，也不影响正常显示。")
    print()


# ---------------------------------------------------------------------------
# 2. 真正的硬约束：SPI DCLK 被 sd_card_clk 硬绑
# ---------------------------------------------------------------------------
def sec2_spi_binding():
    print("=" * 74)
    print("2. 真正的硬约束：SPI DCLK = sd_card_clk / 4（clk_div=0 已是最快档）")
    print("=" * 74)
    print("  spi_master 半周期 = (clk_div + 2) 拍，clk_div=0 → 2 拍半周期 → 4 拍全周期。")
    print("  clk_div 是 16 位无符号，最小 0，**没有负档可以补**。")
    print()
    print("  %-10s %-12s %-14s %-12s" % ("sd_card_clk", "DCLK", "理论读速", "相对 100MHz"))
    print("  " + "-" * 60)
    for mhz in (100, 50):
        dclk = mhz / 4.0
        mbs = dclk / 8.0
        print("  %-10s %-12s %-14s %-12s"
              % ("%dMHz" % mhz, "%.2fMHz" % dclk, "%.2f MB/s" % mbs, "1.00x" if mhz == 100 else "0.50x"))
    print()


def sec3_load_time():
    print("=" * 74)
    print("3. 读卡时间翻倍后，预读还能不能藏住（AUTO_DWELL_SEC = %ds）" % AUTO_DWELL_SEC)
    print("=" * 74)
    print("  %-12s %-8s %-11s %-11s %-13s %s"
          % ("素材", "扇区", "100MHz理论", "50MHz理论", "50MHz实测估", "能否藏在 3s 内"))
    print("  " + "-" * 78)

    dclk_25 = hz(25.0)
    for name, w, h, sectors in MATERIALS:
        t100 = sectors * 512 / (dclk_25 / 8)
        t50 = t100 * 2
        # 实测普遍是理论的 1.2~1.5 倍（卡内部忙等），取 1.35 做中位估计
        t50_real = t50 * 1.35
        verdict = "能" if t50_real <= AUTO_DWELL_SEC else "不能，会明显等图"
        print("  %-12s %-8d %-11s %-11s %-13s %s"
              % (name, sectors, "%.2fs" % t100, "%.2fs" % t50, "%.2fs" % t50_real, verdict))

    print()
    print("  「藏住」的机制：sd_card_bmp 是三缓冲 + 后台预读，")
    print("  只要一张图在 dwell 时间内读完，用户看到的就是到点即切。")
    print("  一旦读卡时间 > dwell，就退化成「切过去先看白屏，等卡读完」。")
    print()
    print("  兜底超时是 800_000_000 拍，50MHz 下等于 %.1fs（不是 8s）。"
          % (LOAD_TIMEOUT_CYCLES / hz(50.0)))
    print("  超时本身不会提前误触发，但 CLK_FREQ_HZ 参数若不同步改，")
    print("  数码管上的切换耗时显示会全部翻倍（switch_latency 的 10ms 时基）。")
    print()


# ---------------------------------------------------------------------------
# 4. 写 FIFO 水位余量
# ---------------------------------------------------------------------------
def sec4_fifo():
    print("=" * 74)
    print("4. 跨域写 FIFO 水位余量（%d 深 × 32bit）" % FIFO_DEPTH)
    print("=" * 74)

    # 读侧：一个 burst 128 字，125MHz 下约 128 + 10 拍开销
    burst_cycles = BURST_SIZE + 10
    burst_us = burst_cycles / hz(MEM_CLK_MHZ) * 1e6
    drain_rate = BURST_SIZE / burst_us   # 字/us

    print("  读侧：%d 字一个 burst，%.2f us，排水能力 %.1f 字/us" % (BURST_SIZE, burst_us, drain_rate))
    print()
    print("  %-10s %-16s %-14s %s" % ("sd_card_clk", "写侧最坏瞬时", "占排水能力", "结论"))
    print("  " + "-" * 62)
    for mhz in (100, 50):
        # 最坏瞬时：黑边行隔拍输出，0.5 字/拍
        worst = 0.5 * hz(mhz) / 1e6
        print("  %-10s %-16s %-14s %s"
              % ("%dMHz" % mhz, "%.1f 字/us" % worst,
                 "%.0f%%" % (100 * worst / drain_rate),
                 "安全" if worst < drain_rate else "会溢出"))

    # 关键：净排水为正，所以不会积压。真正需要 FIFO 兜的只是
    # 读侧被 App_rd_busy 挡住的那些拍。
    print()
    print("  净速率：写侧最坏 50 字/us，读侧排水 115.9 字/us → 净排水为正，不积压。")
    print("  FIFO 只在「读侧正在突发、写侧继续灌」时临时存料：")
    print("    一次 burst 128 字耗 1.10us，期间写侧最多灌 55 字，占 512 深的 11%。")
    print("  结论：FIFO 深度不是瓶颈；降频后写侧更慢，只会更宽松。")
    print()


# ---------------------------------------------------------------------------
# 5. SDRAM 带宽重新核算
# ---------------------------------------------------------------------------
def sec5_sdram():
    print("=" * 74)
    print("5. SDRAM 带宽核算（32bit × MEM_CLK）")
    print("=" * 74)

    live = hz(VGA_PIXEL_CLK_MHZ) * (DST_W * DST_H) / (VGA_TOTAL_W * VGA_TOTAL_H) / 1e6
    read_bw = live * 4          # 每像素 1 个 32bit 字
    # 写侧受 SPI 限制：一帧 307200 字 / 读卡耗时
    t100 = 1801 * 512 / (hz(25.0) / 8)
    write_bw = DST_W * DST_H * 4 / t100 / 1e6

    print("  需求：")
    print("    显示读      %6.1f MB/s（%.2f Mpixel/s × 4B）" % (read_bw, live))
    print("    加载写      %6.1f MB/s（一帧 1.23MB / %.2fs，被 SPI 限速）" % (write_bw, t100))
    print("    峰值合计    %6.1f MB/s" % (read_bw + write_bw))
    print()
    print("  供给（SDR SDRAM 顺序访问，效率按 60%~75% 取）：")
    for mhz in (125.0, 62.5):
        peak = mhz * 4
        print("    %-8s 峰值 %5.0f MB/s → 有效 %3.0f~%3.0f MB/s，余量 %.1f~%.1fx"
              % ("%.1fMHz" % mhz, peak, peak * 0.60, peak * 0.75,
                 peak * 0.60 / (read_bw + write_bw), peak * 0.75 / (read_bw + write_bw)))
    print()
    print("  注意：之前 budget.py 里写的「250→125MB/s」是按 16bit 算的，")
    print("  本板 SDRAM 是 EG_PHY_SDRAM_2M_32，32bit 宽，实际是 500→250MB/s。")
    print()


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    sec1_write_vs_480p()
    sec2_spi_binding()
    sec3_load_time()
    sec4_fifo()
    sec5_sdram()
    print("=" * 74)
    print("结论：50MHz 卡在 SPI 上，不在写侧吞吐上")
    print("=" * 74)
    print("  写侧 16.67 Mpixel/s 确实低于 480p60 的 18.41，但静态图不受影响；")
    print("  真正的问题是 DCLK 25→12.5MHz，1080P 类素材读卡时间翻到 4s 以上，")
    print("  超过 AUTO_DWELL_SEC=3s，三缓冲预读就藏不住了。")
