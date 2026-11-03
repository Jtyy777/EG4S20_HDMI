#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 TD final_timing.rpt 提取各时钟域的最差路径，按 slack 升序。"""
import re, sys, collections

path = sys.argv[1] if len(sys.argv) > 1 else "final_timing.rpt"
txt = open(path, encoding="utf-8", errors="ignore").read()

# 路径块以 "Slack               : x.xxxns" 开头
blocks = re.split(r"\n(?=Slack\s+:)", txt)
recs = []
for b in blocks:
    m = re.search(r"Slack\s+:\s*(-?[\d.]+)ns", b)
    if not m:
        continue
    slack = float(m.group(1))
    bp = re.search(r"Begin Point\s+:\s*(\S+).*?clock\s+(\S+)", b)
    ep = re.search(r"End Point\s+:\s*(\S+).*?clock\s+(\S+)", b)
    lvl = re.search(r"Logic Level\s+:\s*(\d+)", b)
    dly = re.search(r"Data Path Delay\s+:\s*(-?[\d.]+)ns", b)
    recs.append({
        "slack": slack,
        "begin": bp.group(1) if bp else "?",
        "bclk": bp.group(2) if bp else "?",
        "end": ep.group(1) if ep else "?",
        "eclk": ep.group(2) if ep else "?",
        "lvl": lvl.group(1) if lvl else "?",
        "delay": dly.group(1) if dly else "?",
    })

if not recs:
    print("未解析到路径块")
    sys.exit(1)

# 按域分组：以 End Point 的时钟作为所属域（setup 检查关注捕获端）
by = collections.defaultdict(list)
for r in recs:
    by[r["eclk"]].append(r)

print(f"共解析 {len(recs)} 条路径\n")
for clk in sorted(by, key=lambda c: min(r["slack"] for r in by[c])):
    grp = sorted(by[clk], key=lambda r: r["slack"])
    print(f"===== {clk}  共 {len(grp)} 条 =====")
    for r in grp[:6]:
        print(f"  {r['slack']:8.3f}ns  L{r['lvl']:>2}  delay={r['delay']}ns")
        print(f"      {r['begin']}")
        print(f"   -> {r['end']}")
    print()

# 重点关注 sd_card_clk 自域
print("=" * 70)
print("sd_card_clk 相关（起点或终点在 sd_card_clk）")
print("=" * 70)
cand = [r for r in recs if "sd_card_clk" in (r["bclk"], r["eclk"])]
for r in sorted(cand, key=lambda r: r["slack"])[:10]:
    print(f"{r['slack']:8.3f}ns  L{r['lvl']:>2}  {r['bclk']} -> {r['eclk']}")
    print(f"    {r['begin']}")
    print(f" -> {r['end']}")
