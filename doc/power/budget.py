#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
power/budget.py —— 功耗"翻转预算"计算器（TD 6.2.1 无功耗分析时的替代手段）

背景
----
TD 这个版本没有功耗分析（用户手册、TCL 命令手册、GUI 菜单三处都查过，零入口），
所以拿不到 mW。但功耗不是凭空来的，动态功耗只有四个自变量：

    P_dyn = Σ  C · V² · f · α · N

C（每节点电容）和 V（核压 1.2V）在整个芯片内是常数，动不了也测不到；
真正能改的只有 f（频率）、N（参与翻转的寄存器数）、α（翻转率）。
所以本工具的度量单位是"翻转预算" B = Σ N × f，量纲 FF·MHz：

    * 它是**上界**（隐含 α = 1），不是功耗值，不能当 mW 报。
    * 它的用途是排序和算比例：某项措施把 B 砍掉多少，功耗就大致砍掉同样的比例
      （同一域内 C、V 相同，α 相同时严格成立）。
    * 绝对 mW 必须靠板上实测标定，见 doc/power_log.md 的"标定"一节。

数据来源（全部是 TD 自己生成的报告，每次 P&R 自动刷新）
------------------------------------------------------
    phy_1/info_signage_v1.0_phy.area   每个实例的 seq（触发器）数
    phy_1/final_timing.rpt             时钟频率；并用 End Point 反查实例所属时钟域

用法
----
    python doc/power/budget.py                    # 打印基线预算表
    python doc/power/budget.py --scenario         # 再打印各候选措施的 Δ
    python doc/power/budget.py --run phy_1        # 指定 run 目录
"""

import argparse
import collections
import os
import re
import sys

# TD 生成的报告用 GBK 更容易读全，但这两份是纯 ASCII，用 utf-8 + ignore 最稳
ENC = dict(encoding="utf-8", errors="ignore")

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, "..", ".."))
RUNS = os.path.join(PROJ, "src", "td_project", "info_signage_v1.0_Runs")

# ---------------------------------------------------------------------------
# 实例 → 时钟域 的映射
#
# 这份映射不是猜的，两条独立证据交叉确认：
#   1) 顶层/子模块里的端口连接（代码是最终依据）；
#   2) final_timing.rpt 的 End Point 行会写明 "triggered by clock xxx"，
#      把 248 个端点按实例归类，就能反查出该实例实际被哪个时钟打。
# 两者一致才写进来；只覆盖 seq 占比大的实例，剩余归入 OTHER。
# ---------------------------------------------------------------------------
CLOCK_OF = {
    # ---- sd_card_clk 100MHz：SD 读卡 + 缩放 + 写 SDRAM 的整条写侧链路 ----
    # 证据：sd_card_bmp.v:829  img_scaler .clk(clk)，clk 由顶层接 sd_card_clk；
    #       报告里 sd_card_bmp_m0 的 39 个端点全落在 sd_card_clk。
    "sd_card_bmp_m0":                        "sd_card_clk",
    # ---- ext_mem_clk 125MHz：SDRAM 控制器本体 ----
    "U3":                                    "ext_mem_clk",
    # ---- 跨域搬运：主体在 ext_mem 侧，另有 132 个跨域握手寄存器在 sd_card 侧；
    #      按 199:132 拆，见 SPLIT 表 ----
    "frame_read_write_m0":                   "ext_mem_clk",
    # ---- video_clk 25MHz：显示主通路 ----
    "u_hdmi_1_4b_transmitter_core_wrapper":  "video_clk",
    "u_audio_spectrum":                      "video_clk",
    "u_I2S_receiver":                        "video_clk",
    "u_osd_overlay":                         "video_clk",
    "u_transition":                          "video_clk",
    "u_spectrum_overlay":                    "video_clk",
    "u_brightness_contrast":                 "video_clk",
    "u_video_rgb_to_axis_640x480":           "video_clk",
    "video_timing_data_m0":                  "video_clk",
    "video_delay_m0":                        "video_clk",
    "u_pixel_coord":                         "video_clk",
    # ---- clk 50MHz：按键、参数、数码管扫描 ----
    "u_param_ctrl":                          "clk",
    "u_keypad":                              "clk",
    "u_seg4_driver":                         "clk",
    "u_param_cdc":                           "clk",
    "u_audio_arc_calculate":                 "video_clk",
    # ---- hdmi_5x_clk 125MHz：并串转换（只有 34 个触发器） ----
    "u_hdmi2phy_wrapper":                    "hdmi_5x_clk",
    # ---- audio 12.295MHz：只有测试音发生器落在 PLL 的音频输出上；
    #      I2S_receiver / audio_arc_calculate 都在 video_clk（见 top 842/855 行） ----
    "u_hdmi_audio_tone_i2s_64fs":            "u_audio_pll/pll_inst.clkc[2]",
}

# frame_read_write_m0 的 331 个触发器里，跨域 FIFO 的地址握手那部分是另一侧时钟域的。
# 数字来自 area 报告的子实例：read_buf/write_buf 各 95/91（ext_mem 侧），
# rd_to_wr/wr_to_rd_cross 合计 132（sd_card 侧）。
SPLIT = {
    "frame_read_write_m0": {"ext_mem_clk": 199, "sd_card_clk": 132},
}

# 时钟频率直接取 Clock Summary 的 C-Freq。注意 C-Freq 是"约束频率"，
# audio PLL 的实际输出 12.295MHz 报告里就是按实际值列的，不需要另做覆盖。
# 报告里时钟全名太长会撑坏表格，显示时换个别名。
DISPLAY = {"u_audio_pll/pll_inst.clkc[2]": "audio_mclk",
           "sys_pll_m0/pll_inst.clkc[2]": "sys_pll_m0"}


def disp(clk):
    return DISPLAY.get(clk, clk)


def find_run(name):
    d = os.path.join(RUNS, name)
    if not os.path.isdir(d):
        sys.exit("找不到 run 目录: %s" % d)
    return d


def parse_clock_summary(path):
    """Clock Summary：Clock-Id / C-Freq / Fanout / Clock-Name → {name: (freq, fanout)}"""
    out = {}
    started = False
    for line in open(path, **ENC):
        if "Clock-Id" in line and "C-Freq" in line:
            started = True
            continue
        if not started:
            continue
        if line.startswith("---"):
            if out:
                break
            continue
        m = re.match(r"\s*clk\d+:\s+([\d.]+)\s+(\d+)\s+(\S+)", line)
        if m:
            out[m.group(3)] = (float(m.group(1)), int(m.group(2)))
    return out


def parse_hierarchy(path):
    """Report Hierarchy Area：实例 → seq（触发器数）。

    行格式：
      |top                                      |top_signage  |12211 |9244 |...|6452|
      |  U3                                     |sdram        |378   |231  |...|363 |
      |    u2_ram                               |sdr_as_ram   |378   |231  |...|363 |
    缩进 2 格一层。必须按缩进分层，否则子实例会被重复计数
    （top 已含全部子模块，再加一遍就翻倍）。
    """
    rows = []
    collecting = False
    for line in open(path, **ENC):
        if "Report Hierarchy Area" in line:
            collecting = True
            continue
        if not collecting:
            continue
        if line.startswith("+"):
            if rows:
                break
            continue
        if not line.startswith("|"):
            continue
        first = line[1:].split("|")[0]                  # 保留前导空格以判断层级
        depth = (len(first) - len(first.lstrip())) // 2
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) < 6 or cells[0] == "Instance":
            continue
        try:
            seq = int(cells[5])
        except ValueError:
            continue
        rows.append((depth, cells[0], cells[1], seq))
    return rows


def parse_endpoint_clocks(path):
    """End Point 行 → {顶层实例: Counter(时钟名)}，用来交叉验证 CLOCK_OF。"""
    pat = re.compile(r"End Point\s*:\s*(\S+?)/\S*\s*\(\S+ edge triggered by clock (\S+)")
    txt = open(path, **ENC).read()
    d = collections.defaultdict(collections.Counter)
    for inst, clk in pat.findall(txt):
        d[inst][clk] += 1
    return d


def build_budget(run_dir):
    freq = parse_clock_summary(os.path.join(run_dir, "final_timing.rpt"))
    rows = parse_hierarchy(os.path.join(run_dir, "info_signage_v1.0_phy.area"))

    # 只取顶层子实例（depth 1）。depth 0 是 top 自己，它的 seq 是总和，再加就翻倍。
    top = [(name, mod, seq) for depth, name, mod, seq in rows if depth == 1]

    items = []
    for name, mod, seq in top:
        if name in SPLIT:
            for clk, n in SPLIT[name].items():
                items.append((name, clk, n))
            # 剩下没分配的挂到主域
            rest = seq - sum(SPLIT[name].values())
            if rest > 0:
                items.append((name, CLOCK_OF[name], rest))
        elif name in CLOCK_OF:
            items.append((name, CLOCK_OF[name], seq))
        else:
            items.append((name, "OTHER", seq))
    return freq, items


def show_budget(freq, items):
    total = sum(n * freq.get(c, (0, 0))[0] for _, c, n in items)
    agg = collections.Counter()
    cnt = collections.Counter()
    for name, clk, n in items:
        f = freq.get(clk, (0.0, 0))[0]
        agg[clk] += n * f
        cnt[clk] += n

    print("\n=== 按时钟域的翻转预算（基线） ===")
    print("%-14s %8s %8s %12s %8s" % ("时钟域", "f/MHz", "触发器", "B=FF·MHz", "占比"))
    print("-" * 56)
    for clk, b in agg.most_common():
        f = freq.get(clk, (0.0, 0))[0]
        print("%-14s %8.3f %8d %12.0f %7.1f%%" % (disp(clk), f, cnt[clk], b, 100.0 * b / total))
    print("-" * 56)
    print("%-14s %8s %8d %12.0f" % ("合计", "", sum(cnt.values()), total))

    print("\n=== 单实例 Top 12（谁在烧翻转） ===")
    per = collections.Counter()
    for name, clk, n in items:
        per[(name, clk)] += n * freq.get(clk, (0.0, 0))[0]
    print("%-36s %-14s %12s %8s" % ("实例", "时钟域", "B=FF·MHz", "占比"))
    print("-" * 74)
    for (name, clk), b in per.most_common(12):
        print("%-36s %-14s %12.0f %7.1f%%" % (name, disp(clk), b, 100.0 * b / total))
    return total


def show_scenarios(freq, items, total, seqmap):
    """每个候选措施 → B 的绝对变化与占比变化。

    每条都注明它改的是哪个自变量（f / N / α / IO），以及假设。
    """
    by = collections.Counter()
    for name, clk, n in items:
        by[name] += n * freq.get(clk, (0.0, 0))[0]

    f_sd = freq.get("sd_card_clk", (100.0, 0))[0]
    f_em = freq.get("ext_mem_clk", (125.0, 0))[0]

    # sd_card_clk 域里的总触发器（含 frame_rw 挂过来的 132 个）
    n_sd = sum(n for _, c, n in items if c == "sd_card_clk")
    n_em = sum(n for _, c, n in items if c == "ext_mem_clk")
    # 这两个数每次 P&R 会变，从 area 报告里查，别写死
    n_scaler = seqmap.get("u_img_scaler", 0)
    n_seg = seqmap.get("u_seg4_driver", 0)

    scen = []
    scen.append(("sd_card_clk 100→50MHz",       "f", -0.5 * n_sd * f_sd,
                 "已否决：SPI DCLK 硬绑 clk/4，降频=读卡时间翻倍，1080P 素材超 3s dwell"))
    scen.append(("img_scaler 加读使能（已做）",   "α", -n_scaler * f_sd,
                 "此数不成立：真实收益是停掉 96%% 空闲期的 9 块 BRAM 空读，不是省 %d 个 FF 的翻转" % n_scaler))
    scen.append(("ext_mem_clk 125→62.5MHz",     "f", -0.5 * n_em * f_em,
                 "SDRAM 32bit：500→250MB/s 峰值；需求仅 78MB/s，余量 1.9~2.4x"))
    scen.append(("数码管 1kHz→200Hz",           "IO", -0.8 * n_seg * freq.get("clk", (50.0, 0))[0],
                 "只减寄存器翻转；LED 占空比不变，平均电流不变，别按这个数报"))

    print("\n=== 候选措施对翻转预算的影响（上界估计） ===")
    print("%-26s %-4s %12s %10s %s" % ("措施", "改的量", "ΔB", "占基线", "假设/说明"))
    print("-" * 100)
    for name, var, db, note in scen:
        print("%-26s %-4s %12.0f %9.1f%% %s" % (name, var, db, 100.0 * db / total, note))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="phy_1")
    ap.add_argument("--scenario", action="store_true")
    args = ap.parse_args()

    run_dir = find_run(args.run)
    freq, items = build_budget(run_dir)

    print("run 目录: %s" % run_dir)
    print("时钟频率来源: final_timing.rpt / Clock Summary")
    print("触发器数来源: info_signage_v1.0_phy.area / Report Hierarchy Area")

    # 交叉验证：报告里能查到的实例，看看 CLOCK_OF 有没有写错。
    # 注意报告只列"最差路径"，端点数是抽样而不是全集：某个 video_clk 模块
    # 冒出两三个 hdmi_5x_clk 端点是正常的（跨域路径的捕获端）。
    # 所以只对端点样本够多（>=5）且主域完全没有命中的情况提示。
    ep = parse_endpoint_clocks(os.path.join(run_dir, "final_timing.rpt"))
    bad = []
    for inst, counter in ep.items():
        want = CLOCK_OF.get(inst)
        if not want or want == "OTHER":
            continue
        if sum(counter.values()) < 5 or counter[want] > 0:
            continue
        bad.append((inst, want, counter.most_common(1)[0]))
    if bad:
        print("\n[提示] 以下实例的时钟域只有代码依据、报告端点对不上，复核一下：")
        for inst, want, (got, n) in bad:
            print("  %-36s 脚本按代码判 %-14s 报告 %d 个端点落在 %s" % (inst, want, n, got))

    total = show_budget(freq, items)
    if args.scenario:
        rows = parse_hierarchy(os.path.join(run_dir, "info_signage_v1.0_phy.area"))
        seqmap = {name: seq for _, name, _, seq in rows}
        show_scenarios(freq, items, total, seqmap)
    print()


if __name__ == "__main__":
    main()
