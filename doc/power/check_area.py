#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""检查 P-1c（img_scaler 行缓冲读使能）在 TD 报告里是否按预期落地。

判据只有两条：
  1. u_img_scaler 的 bram 仍是 9     —— 说明读使能走了 BRAM 原生的 CE 端口
  2. u_img_scaler 的 lut 基本不涨    —— 若涨约 72（3 组 × 24 位），
                                        说明 CE 被落到了触发器上，写法在 TD 上不生效

用法：
    python doc/power/check_area.py                # 读 phy_1 的面积报告
    python doc/power/check_area.py --run syn_1    # 读综合的报告（gate/rtl）
    python doc/power/check_area.py --save         # 顺便把报告存到 doc/timing_rpt/

为什么要有 --save：TD 每次重新 P&R 都会覆盖 phy_1 下的报告，
上一次的基线就是这么没的。留证必须当次拷走。
"""

from __future__ import print_function

import argparse
import io
import json
import os
import re
import shutil
import sys
import time

ENC = dict(encoding="utf-8", errors="ignore")

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RUNS = os.path.join(ROOT, "src", "td_project", "info_signage_v1.0_Runs")
ARCHIVE = os.path.join(ROOT, "doc", "timing_rpt")
BASELINE_JSON = os.path.join(HERE, "area_baseline.json")


# ---------------------------------------------------------------- 解析

def find_report(run):
    """在一个 run 目录里找面积报告，返回 (路径, 类型)。

    TD 会产出三种面积报告，内容层次不同：
      *_phy.area   —— P&R 后，最准，最关心
      *_gate.area  —— 综合映射后（K=5 LUT），次准
      *_rtl.area   —— 综合读入后的门级统计，只能看趋势
    """
    d = os.path.join(RUNS, run)
    if not os.path.isdir(d):
        return None, None
    for kind in ("phy", "gate", "rtl"):
        p = os.path.join(d, "info_signage_v1.0_%s.area" % kind)
        if os.path.isfile(p):
            return p, kind
    return None, None


def newest_file(run):
    """run 目录里最新的文件及其时间，用来说明"跑到哪一步了"。"""
    d = os.path.join(RUNS, run)
    best, t = None, 0
    if not os.path.isdir(d):
        return None, 0
    for name in os.listdir(d):
        p = os.path.join(d, name)
        if os.path.isfile(p):
            m = os.path.getmtime(p)
            if m > t:
                best, t = name, m
    return best, t


def parse_util(path):
    """Utilization Statistics：取 #lut / #reg / #le / #bram / #dsp。"""
    out = {}
    keymap = {"#lut": "lut", "#reg": "reg", "#le": "le",
              "#bram": "bram", "#dsp": "dsp", "#pll": "pll"}
    with io.open(path, **ENC) as f:
        for line in f:
            s = line.strip()
            if not s.startswith("#"):
                continue
            parts = s.split()
            key = parts[0]
            if key in keymap and key not in out:
                try:
                    out[keymap[key]] = int(parts[1])
                except (IndexError, ValueError):
                    pass
    return out


KNOWN_COLS = ("le", "lut", "ripple", "seq", "bram", "dsp", "dff", "io", "reg")


def parse_hierarchy(path):
    """Report Hierarchy Area：实例名 → {le, lut, ripple, seq, bram, dsp}。

    行格式： |    u_img_scaler  |img_scaler  |1565 |1264 |283 |798 |9 |2 |
    实例名带缩进，缩进层级 = 层次深度，这里只按名字索引，不关心层级。

    列集合随报告类型变化，必须按表头对齐：
      phy.area   —— Instance | Module | le | lut | ripple | seq | bram | dsp
      gate.area  —— Instance | Module | lut | ripple | seq | bram | dsp   （无 le）
    早先按固定偏移取值，读 gate 报告时整体串了一位。
    """
    rows = {}
    colidx = None
    collecting = False
    with io.open(path, **ENC) as f:
        for line in f:
            if "Report Hierarchy Area" in line:
                collecting = True
                continue
            if not collecting:
                continue
            if not line.startswith("|"):
                if rows:
                    break
                continue
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            if cells and cells[0] == "Instance":
                colidx = {}
                for i, name in enumerate(cells):
                    if name in KNOWN_COLS:
                        colidx[name] = i
                continue
            if len(cells) < 3 or not colidx:
                continue
            name = cells[0].split()[-1] if cells[0] else ""
            if not name:
                continue
            rec = {}
            for col, i in colidx.items():
                if i < len(cells):
                    try:
                        rec[col] = int(cells[i])
                    except ValueError:
                        pass
            if rec:
                rows[name] = rec
    return rows


# ---------------------------------------------------------------- 判定

def delta_str(after, before):
    if after is None or before is None:
        return "—"
    d = after - before
    if d == 0:
        return "0"
    return "%+d" % d


def judge(before, after, comparable=True):
    """两条判据 + 结论文字。

    comparable=False 表示当前报告与基线不是同一级（比如基线是 P&R 后的 phy，
    现在拿到的是综合的 gate）：bram 数仍可比，lut/seq 不可比，跳过判据2。
    """
    if not before or not after:
        return ["[无法判定] 缺基线或缺本次数据"]

    lines = []
    b_bram = before.get("bram")
    a_bram = after.get("bram")
    b_lut = before.get("lut")
    a_lut = after.get("lut")

    ok_bram = (a_bram == b_bram == 9)
    lines.append("判据1  bram = %s（基线 %s）：%s" % (
        a_bram, b_bram, "通过" if ok_bram else "不通过"))

    if not comparable:
        lines.append("判据2  跳过：当前报告级别与基线不同，lut 不可比")
        if ok_bram:
            lines.append("结论  bram 数对得上；lut 判定等 phy_1 出报告后再看")
        else:
            lines.append("结论  bram 数已经不对，先回看代码")
        return lines

    if a_lut is None or b_lut is None:
        dl = None
    else:
        dl = a_lut - b_lut
    if dl is None:
        lines.append("判据2  lut 差值：无法计算")
        ok_lut = False
    else:
        # ±40 以内算工具波动（同版本重跑也会差几十）
        ok_lut = dl <= 40
        lines.append("判据2  lut = %d（基线 %d，%+d）：%s" % (
            a_lut, b_lut, dl, "通过" if ok_lut else "不通过"))
        if dl >= 50:
            lines.append("       差值接近 3×24=%d，疑似 CE 落到触发器上" % (3 * 24))

    if ok_bram and ok_lut:
        lines.append("结论  P-1c 落地正常：读使能走了 BRAM 原生 CE")
    elif ok_bram and not ok_lut:
        lines.append("结论  BRAM 数量对，但 LUT 涨了 —— TD 把 CE 实现成了外部选择器，写法要换")
    else:
        lines.append("结论  BRAM 数量变了 —— 读端口结构被改动，需回看代码")
    return lines


# ---------------------------------------------------------------- 主流程

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="phy_1", help="run 目录名，默认 phy_1")
    ap.add_argument("--save", action="store_true", help="把报告归档到 doc/timing_rpt/")
    ap.add_argument("--baseline", default=None, help="直接指定基线 json")
    args = ap.parse_args()

    path, kind = find_report(args.run)
    if not path:
        name, t = newest_file(args.run)
        print("!! %s 里还没有面积报告，流程可能没跑到那一步。" % args.run)
        if name:
            print("   最新文件：%s  (%s)" % (name, time.strftime("%H:%M:%S", time.localtime(t))))
        print("   P&R 完成后会生成 info_signage_v1.0_phy.area，再跑一次本脚本即可。")
        return 2

    util = parse_util(path)
    rows = parse_hierarchy(path)

    print("报告：%s" % os.path.relpath(path, ROOT).replace("\\", "/"))
    print("类型：%s    修改时间：%s" % (
        kind, time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(os.path.getmtime(path)))))
    print()
    print("整片资源（%s）：" % kind)
    for k, label in (("lut", "LUT"), ("reg", "REG"), ("le", "LE"),
                     ("bram", "BRAM"), ("dsp", "DSP")):
        if k in util:
            print("  %-5s %6d" % (label, util[k]))
    print()

    target = "u_img_scaler"
    after = rows.get(target)
    if not after:
        print("!! 层次报告里没有 %s，可能被优化掉了（看报告里的实例名）" % target)
        return 3

    # 基线
    bl_path = args.baseline or BASELINE_JSON
    before = None
    meta = {}
    if os.path.isfile(bl_path):
        data = json.load(io.open(bl_path, **ENC))
        before = data.get(target)
        meta = data.get("_meta", {})

    print("目标实例：%s" % target)
    cols = ["le", "lut", "ripple", "seq", "bram", "dsp"]
    head = "  %-8s" % "项"
    for c in cols:
        head += "%10s" % c
    print(head)
    print("  %-8s" % "本次" + "".join("%10s" % (after.get(c, "—")) for c in cols))
    comparable = True
    if before:
        print("  %-8s" % "基线" + "".join("%10s" % (before.get(c, "—")) for c in cols))
        print("  %-8s" % "差值" + "".join("%10s" % delta_str(after.get(c), before.get(c))
                                          for c in cols))
        if meta:
            print("  （基线来源：%s）" % meta.get("source", "未注明"))
            if meta.get("kind") and meta["kind"] != kind:
                comparable = False
                print("  （注意：基线是 %s 级报告，本次是 %s 级，lut/seq 不可比，"
                      "只判 bram）" % (meta["kind"], kind))
    else:
        print("  未找到基线文件 %s" % os.path.relpath(bl_path, ROOT))
    print()

    for line in judge(before, after, comparable):
        print(line)

    # 顺带看一眼 param_cdc 还在不在（A 方案是否还在生效）
    cdc = rows.get("u_param_cdc")
    if cdc:
        print()
        print("附带：u_param_cdc 存在（le=%s lut=%s seq=%s），A 方案仍生效"
              % (cdc.get("le", "—"), cdc.get("lut", "—"), cdc.get("seq", "—")))

    if args.save:
        if kind != "phy":
            print("\n[跳过归档] 只归档 phy 报告，当前是 %s" % kind)
        else:
            if not os.path.isdir(ARCHIVE):
                os.makedirs(ARCHIVE)
            stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(os.path.getmtime(path)))
            dst = os.path.join(ARCHIVE, "phy_%s.area" % stamp)
            shutil.copy2(path, dst)
            print("\n已归档：%s" % os.path.relpath(dst, ROOT).replace("\\", "/"))

    return 0


if __name__ == "__main__":
    sys.exit(main())
