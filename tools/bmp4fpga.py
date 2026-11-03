#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bmp4fpga.py —— 把任意图片转成 info_signage FPGA 工程能吃的 BMP

针对 D:\\FPGA\\project\\src\\user_source\\hdl_source\\SD\\bmp_read.v + rtl\\img_scaler.v
的硬性要求做了强校验，转出来的文件放进去就能显示。

RTL 侧的硬约束（本脚本逐条保证）
------------------------------------------------------------------------------
1. 只认【短文件名 8.3】+ 扩展名必须是大写 .BMP
   bmp_read.v:307  wire bmp_ext_match = (dir_ext0=="B") && (dir_ext1=="M") && (dir_ext2=="P");
        即目录项偏移 8..10 必须是 'B','M','P' 三个大写字母。
        长文件名（LFN）项会被 !dir_lfn 跳过，但短名项仍然生效，
        所以用小写 bmp / 长文件名也能被认出来 —— 但目录扫描顺序会变得不可预期，
        本脚本一律输出 IMGn.BMP 这种规范名，顺序 = 文件名数字顺序。
2. 位深只能是 24 或 32，压缩方式必须是 BI_RGB(0)
   bmp_read.v:136  hdr_ok = ... (bit_count==24 || bit_count==32) && compression==0 ...
   32bpp 的第 4 字节是 alpha，RTL 直接丢弃。
3. 宽高都在 1..4096 之间
4. 必须是【自底向上】的正 height。自上而下（负 height）会上下颠倒，
   因为 frame_fifo_write 的 VFLIP_FIRST_ADDR_OFFSET 是 localparam 写死的。
5. 文件长度字段（偏移 2，4 字节）必须准确 —— RTL 靠 bmp_len_cnt>=file_len 收工。
6. 只扫【根目录】前 SCAN_MAX_SECTOR 扇区，找到【前 4 个】.BMP 就停
   sd_card_bmp.v:  SCAN_TARGET_COUNT = 3'd4
   所以文件必须放 SD 卡根目录（不能放子文件夹），多于 4 个的不会显示。
7. 单个文件别超过 ~8MB：sd_card_bmp 的加载超时是 8s（25MHz SPI 约 3MB/s）

显示侧本来就固定 640x480（SDRAM 双缓冲各 307200 像素），所以：
  --mode fit      (默认) 在这里就把图缩到 640x480 并留黑边 → 进 FPGA 后 step=1 直通，
                  缩放质量由 Pillow 的 LANCZOS 决定，比 FPGA 双线性好得多
  --mode native   保留原分辨率，让 FPGA 的 img_scaler 自己缩放（KEEP_ASPECT=1 同样留黑边），
                  好处是源图信息全保，代价是 FPGA 只有双线性、缩小倍数大时有混叠

用法
------------------------------------------------------------------------------
  # 最常用：一张图铺满 640x480 画布（等比 + 黑边）
  python bmp4fpga.py poster.jpg -o E:\\sd

  # 整个文件夹按文件名顺序转成 IMG0..IMG3.BMP
  python bmp4fpga.py D:\\pics -o E:\\sd

  # 保留原分辨率交给 FPGA 缩放
  python bmp4fpga.py big.png -o E:\\sd --mode native

  # 检查 SD 卡上已有文件是否符合 RTL 要求
  python bmp4fpga.py --check E:\\sd

清理 SD 卡（不依赖 Pillow）
------------------------------------------------------------------------------
  # 1) 防物理残留：把卡上所有 BMP 的 'BM' 魔数抹成 0x00
  #    这样即使 FAT 数据块还留在 NAND 里，FPGA 也会因为 hdr_ok 不成立而跳过
  python bmp4fpga.py --scrub E:\\

  # 2) 彻底清空：先预览要删什么（默认不删）
  python bmp4fpga.py --wipe E:\\

  # 3) 确认后真删（FAT32 无回收站，删了不回来）
  python bmp4fpga.py --wipe E:\\ --yes

  # 4) 一步到位：先毁魔数、再删光（最推荐，顺序很重要）
  python bmp4fpga.py --scrub --wipe E:\\ --yes

  顺序说明：scrub 必须在前。先删再 scrub 的话，文件已经不在目录里了，
  脚本就找不到它们，物理残留的数据块仍然带着完整 'BM' 头。

依赖：pip install Pillow（仅转换 / 检查功能需要；--scrub / --wipe 不需要）
"""

from __future__ import annotations

import argparse
import os
import stat as stat_mod
import struct
import sys
import traceback
import unicodedata
from pathlib import Path

# ---------------------------------------------------------------------------
# RTL 限制常量（改 RTL 的话同步改这里）
# ---------------------------------------------------------------------------
DST_W_DEFAULT = 640
DST_H_DEFAULT = 480
SCAN_TARGET_COUNT = 4          # sd_card_bmp.v 的 SCAN_TARGET_COUNT
MAX_DIM = 4096                 # bmp_read.v 的 hdr_w/hdr_h 上限
MAX_FILE_BYTES = 8 * 1024 * 1024   # 8s 加载超时的安全线
ALLOWED_BPP = (24, 32)

# FAT 短文件名非法字符
FAT_ILLEGAL = set('"*+,/:;<=>?[\\]|')

# --wipe 绝对不碰的顶层目录名（盘符根下的第一层）
DANGER_TOP = {
    "windows", "program files", "program files (x86)", "programdata",
    "system volume information", "$recycle.bin", "appdata", "perflogs",
    "recovery", "boot", "efi", "$windows.~bt", "$windows.~ws",
    "users", "documents and settings",
}

# 破坏魔数时默认写多少个 0x00
SCRUB_BYTES_DEFAULT = 2

# wipe 时直接跳过的顶层目录（Windows 自己维护的，删不掉也会重建）
SKIP_TOP = {"system volume information", "$recycle.bin", "recycler"}

try:                                                    # Pillow >= 9.1
    _R = None
    from PIL import Image, ImageOps
    _R = Image.Resampling
except ImportError:                                     # pragma: no cover
    Image = None
    ImageOps = None
    _R = None

RESAMPLE_NAMES = {
    "lanczos": ("LANCZOS", "LANCZOS"),
    "bicubic": ("BICUBIC", "BICUBIC"),
    "bilinear": ("BILINEAR", "BILINEAR"),
    "nearest": ("NEAREST", "NEAREST"),
}


def resample_of(name: str):
    new_name, old_name = RESAMPLE_NAMES[name]
    if _R is not None and hasattr(_R, new_name):
        return getattr(_R, new_name)
    return getattr(Image, old_name)


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def die(msg: str, code: int = 1):
    print("错误: " + msg, file=sys.stderr)
    raise SystemExit(code)


def warn(msg: str):
    print("  ! " + msg)


def human(n: float) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return f"{n:.1f}{unit}" if unit != "B" else f"{int(n)}B"
        n /= 1024.0
    return f"{n:.1f}GB"


def dwidth(s: str) -> int:
    """终端显示宽度（中文算 2 列）。"""
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in str(s))


def pad(s, width: int) -> str:
    s = str(s)
    return s + " " * max(0, width - dwidth(s))


def is_short_name(name: str) -> bool:
    """名字是否符合 FAT 8.3 短名规范。"""
    if not name:
        return False
    base, dot, ext = name.rpartition(".")
    if not dot:
        base, ext = name, ""
    if not (1 <= len(base) <= 8):
        return False
    if ext and not (1 <= len(ext) <= 3):
        return False
    if any(c in FAT_ILLEGAL or c in " \t" or ord(c) > 127 for c in name):
        return False
    return True


def parse_bg(text: str):
    parts = [p.strip() for p in text.replace("#", "").split(",")]
    if len(parts) != 3:
        die(f"--bg 要写成 R,G,B 三个 0..255 的数，收到: {text}")
    try:
        vals = [int(p) for p in parts]
    except ValueError:
        die(f"--bg 里有非整数: {text}")
    if any(not (0 <= v <= 255) for v in vals):
        die(f"--bg 分量要在 0..255: {text}")
    return tuple(vals)


# ---------------------------------------------------------------------------
# 图片加载：统一成 RGB，丢弃透明，纠正 EXIF 方向
# ---------------------------------------------------------------------------
def load_rgb(path: Path, bg):
    try:
        im = Image.open(path)
        im.load()
    except Exception as exc:
        die(f"打不开 {path.name}: {exc}")

    # 手机照片的 EXIF 方向，不处理会躺倒
    try:
        im = ImageOps.exif_transpose(im)
    except Exception:
        pass

    # 带透明通道的：先铺一层背景色再合成，否则透明区会变全黑/乱码
    if im.mode in ("RGBA", "LA", "PA") or (im.mode == "P" and "transparency" in im.info):
        rgba = im.convert("RGBA")
        base = Image.new("RGBA", rgba.size, tuple(bg) + (255,))
        base.alpha_composite(rgba)
        im = base.convert("RGB")
    elif im.mode != "RGB":
        im = im.convert("RGB")          # 灰度 / CMYK / 调色板 一律转 RGB
    return im


# ---------------------------------------------------------------------------
# 目标尺寸计算
# ---------------------------------------------------------------------------
def plan_resize(sw: int, sh: int, W: int, H: int, mode: str, upscale: bool):
    """返回 (缩放后尺寸, 说明文字)。mode = fit / fill / stretch / native"""
    if mode == "native":
        if sw > MAX_DIM or sh > MAX_DIM:
            s = MAX_DIM / max(sw, sh)
            return (max(1, round(sw * s)), max(1, round(sh * s))), \
                   f"源图超过 {MAX_DIM}，强制缩到上限"
        return (sw, sh), "保持原尺寸（交给 FPGA 缩放）"

    if mode == "stretch":
        if not upscale and (sw > W or sh > H):
            s = min(W / sw, H / sh)
            return (max(1, round(sw * s)), max(1, round(sh * s))), \
                   f"只缩不放，{s:.4f}x（会有留白）"
        return (W, H), "直接拉伸到目标尺寸（可能变形）"

    if mode == "fit":
        s = min(W / sw, H / sh)
        note = "等比缩放 + 居中留边"
    else:  # fill
        s = max(W / sw, H / sh)
        note = "等比缩放 + 居中裁剪"

    if not upscale and s > 1.0:
        s = 1.0
        note = "只缩不放，原尺寸居中" if mode == "fill" else "只缩不放，原尺寸居中留边"

    return (max(1, round(sw * s)), max(1, round(sh * s))), f"{s:.4f}x {note}"


def render(im, W: int, H: int, mode: str, upscale: bool, bg, rsp):
    """把 RGB 图渲染成目标画布（fit/fill/stretch）或原尺寸（native）。"""
    sw, sh = im.size
    (nw, nh), note = plan_resize(sw, sh, W, H, mode, upscale)

    if (nw, nh) != (sw, sh):
        im = im.resize((nw, nh), rsp, reducing_gap=3.0 if nw < sw else None)

    if mode == "native":
        return im, note

    canvas = Image.new("RGB", (W, H), tuple(bg))

    if mode == "stretch":
        if (nw, nh) == (W, H):
            return im, note
        # 只缩不放时 nw/nh 可能小于画布 → 走居中粘贴
        canvas.paste(im, ((W - nw) // 2, (H - nh) // 2))
        return canvas, note

    if nw >= W and nh >= H and mode == "fill":
        left = (nw - W) // 2
        top = (nh - H) // 2
        return im.crop((left, top, left + W, top + H)), note

    canvas.paste(im, ((W - nw) // 2, (H - nh) // 2))
    return canvas, note


# ---------------------------------------------------------------------------
# 保存 + 复检
# ---------------------------------------------------------------------------
def save_bmp(im, out: Path, bpp: int) -> int:
    if bpp == 32:
        out_im = im.convert("RGBA")
        # RTL 丢弃 alpha，填 255 只是为了别的看图软件不把图显示成全透明
        out_im.putalpha(255)
    else:
        out_im = im.convert("RGB")

    out_im.save(out, format="BMP")
    return out.stat().st_size


def read_bmp_header(path: Path) -> dict:
    """按 bmp_read.v 的字段偏移解析 BMP 头。"""
    data = path.read_bytes()[:54]
    if len(data) < 54:
        raise ValueError("文件不足 54 字节，连 BMP 头都不完整")
    file_len = struct.unpack_from("<I", data, 2)[0]
    pix_off = struct.unpack_from("<I", data, 10)[0]
    hdr_size = struct.unpack_from("<I", data, 14)[0]
    w = struct.unpack_from("<i", data, 18)[0]
    h = struct.unpack_from("<i", data, 22)[0]
    bpp = struct.unpack_from("<H", data, 28)[0]
    comp = struct.unpack_from("<I", data, 30)[0]
    return dict(
        magic=data[0:2].decode("latin-1"),
        file_len=file_len, pix_off=pix_off, hdr_size=hdr_size,
        w=w, h=h, bpp=bpp, comp=comp,
        disk_len=path.stat().st_size,
    )


def verify(path: Path, W: int, H: int, verbose: bool = True):
    """按 RTL 的判据检查一个 BMP。返回 (是否合格, 问题列表)。"""
    problems = []
    try:
        hd = read_bmp_header(path)
    except Exception as exc:
        return False, [f"读头失败: {exc}"]

    if hd["magic"] != "BM":
        problems.append(f"magic 不是 'BM'（是 {hd['magic']!r}）")
    if hd["bpp"] not in ALLOWED_BPP:
        problems.append(f"位深 {hd['bpp']} 不支持，只接受 24/32")
    if hd["comp"] != 0:
        problems.append(f"压缩方式 {hd['comp']} 不是 BI_RGB(0)")
    if not (1 <= hd["w"] <= MAX_DIM):
        problems.append(f"宽 {hd['w']} 超出 1..{MAX_DIM}")
    if not (1 <= hd["h"] <= MAX_DIM):
        problems.append(f"高 {hd['h']} 超出 1..{MAX_DIM}（负数表示自上而下，会上下颠倒）")
    if hd["file_len"] != hd["disk_len"]:
        problems.append(f"头里 file_len={hd['file_len']} 与实际 {hd['disk_len']} 不符")
    if hd["pix_off"] < 54:
        problems.append(f"像素数据偏移 {hd['pix_off']} 不合理")
    if hd["disk_len"] > MAX_FILE_BYTES:
        problems.append(f"文件 {human(hd['disk_len'])} 超过 8MB 加载超时安全线")
    name = path.name
    if not is_short_name(name):
        problems.append(f"文件名 {name!r} 不是 FAT 8.3 短名")
    if not name.upper().endswith(".BMP") or path.suffix != ".BMP":
        problems.append(f"扩展名必须是【大写】.BMP，当前 {path.suffix!r}")

    if verbose:
        tag = "OK  " if not problems else "FAIL"
        shown = name if dwidth(name) <= 24 else name[:21] + "..."
        print(f"  [{tag}] {pad(shown, 26)} {hd['w']}x{hd['h']} {hd['bpp']}bpp "
              f"comp={hd['comp']} {human(hd['disk_len'])}")
        for p in problems:
            print(f"         - {p}")
    return not problems, problems


# ---------------------------------------------------------------------------
# 收集输入
# ---------------------------------------------------------------------------
IMG_EXTS = {".bmp", ".jpg", ".jpeg", ".png", ".gif", ".webp", ".tif", ".tiff",
            ".ppm", ".pgm", ".tga", ".ico", ".jfif", ".avif"}


def natural_key(p: Path):
    """文件名里的数字按数值排序（IMG2 排在 IMG10 前面）。"""
    import re
    return [int(t) if t.isdigit() else t.lower()
            for t in re.split(r"(\d+)", p.name)]


def collect(inputs, recursive: bool):
    files = []
    for raw in inputs:
        p = Path(raw)
        if p.is_dir():
            it = p.rglob("*") if recursive else p.glob("*")
            got = [q for q in it if q.is_file() and q.suffix.lower() in IMG_EXTS]
            got.sort(key=natural_key)
            if not got:
                warn(f"目录 {p} 里没找到图片")
            files.extend(got)
        elif p.is_file():
            files.append(p)
        else:
            warn(f"找不到 {p}")
    return files


def make_out_name(template: str, n: int, start: int, src: Path) -> str:
    if "{n}" in template:
        name = template.format(n=n, stem=src.stem)
    elif "{stem}" in template:
        name = template.format(n=n, stem=src.stem)
    else:
        name = template
    # 扩展名强制大写 .BMP
    base, dot, ext = name.rpartition(".")
    if not dot:
        base, ext = name, "BMP"
    return base + "." + ext.upper()


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def do_convert(args):
    files = collect(args.inputs, args.recursive)
    if not files:
        die("没有可转换的图片")

    outdir = Path(args.outdir)
    if not args.dry_run:
        outdir.mkdir(parents=True, exist_ok=True)

    bg = parse_bg(args.bg)
    rsp = resample_of(args.resample)
    W, H = args.width, args.height

    if len(files) > SCAN_TARGET_COUNT:
        print(f"\n!! 注意：共 {len(files)} 张图，FPGA 只会显示根目录里前 "
              f"{SCAN_TARGET_COUNT} 个 .BMP（SCAN_TARGET_COUNT={SCAN_TARGET_COUNT}），"
              f"第 {SCAN_TARGET_COUNT + 1} 张起不会出现在屏幕上。\n")

    print(f"目标 {W}x{H}  mode={args.mode}  bpp={args.bpp}  "
          f"resample={args.resample}  bg={bg}")
    print(f"输出目录 {outdir}\n")

    rows = []
    ok_cnt = 0
    for i, src in enumerate(files):
        n = args.start + i
        name = make_out_name(args.name, n, args.start, src)
        if not is_short_name(name):
            die(f"生成的名字 {name!r} 不是合法 FAT 8.3 短名，"
                f"请改 --name（主名 <=8 字符，扩展名 <=3）")

        try:
            im = load_rgb(src, bg)
        except SystemExit:
            raise
        except Exception:
            warn(f"{src.name}: {traceback.format_exc(limit=1).strip()}")
            continue

        sw, sh = im.size
        if sw > MAX_DIM or sh > MAX_DIM:
            warn(f"{src.name} 源尺寸 {sw}x{sh} 超过 {MAX_DIM}，"
                 f"FPGA 头校验会拒绝，已自动缩到上限")

        out_im, note = render(im, W, H, args.mode, not args.no_upscale, bg, rsp)
        out = outdir / name

        if args.dry_run:
            # 只算尺寸不落盘：24bpp 的 BMP 是 54 字节头 + 每行 4 字节对齐
            row = (out_im.width * 3 + 3) & ~3
            size = 54 + row * out_im.height
            rows.append((name, f"{sw}x{sh}", f"{out_im.width}x{out_im.height}",
                         f"{args.bpp}", "~" + human(size), note, "plan"))
            continue

        try:
            size = save_bmp(out_im, out, args.bpp)
        except Exception as exc:
            warn(f"保存 {out} 失败: {exc}")
            continue

        good, probs = verify(out, W, H, verbose=False)
        if good:
            ok_cnt += 1
        else:
            for p in probs:
                warn(f"{name}: {p}")
            if size > MAX_FILE_BYTES and args.mode == "native":
                warn(f"{name}: native 模式保留了大图 → 换 --mode fit 会先缩到 {W}x{H}，"
                     f"体积降到 ~{human(54 + ((W * 3 + 3) & ~3) * H)}，也更快")

        rows.append((name, f"{sw}x{sh}", f"{out_im.width}x{out_im.height}",
                     f"{args.bpp}", human(size), note, "OK" if good else "FAIL"))

    # 表格输出
    if rows:
        hdr = ("文件", "源尺寸", "输出尺寸", "bpp", "大小", "处理", "校验")
        widths = [max(dwidth(r[i]) for r in rows + [hdr]) for i in range(len(hdr))]
        line = "  ".join(pad(hdr[i], widths[i]) for i in range(len(hdr)))
        print("\n" + line)
        print("-" * dwidth(line))
        for r in rows:
            print("  ".join(pad(r[i], widths[i]) for i in range(len(r))))

    done_cnt = len(rows) if args.dry_run else ok_cnt
    print(f"\n完成 {done_cnt}/{len(rows)} 张，输出到 {outdir}")

    if args.dry_run:
        print("（--dry-run：以上只是计划，没有写入任何文件）")
        return 0

    if ok_cnt > SCAN_TARGET_COUNT:
        print(f"\n!! 生成了 {ok_cnt} 张，但 RTL 只取根目录前 {SCAN_TARGET_COUNT} 个 .BMP，"
              f"第 {SCAN_TARGET_COUNT + 1} 张起不会显示。")
        print(f"   只想显示前 {SCAN_TARGET_COUNT} 张：只拷 IMG0..IMG{SCAN_TARGET_COUNT - 1}.BMP 过去；")
        print(f"   想换一批：先 run.bat --wipe <盘符>:\\ --yes 清空再拷。")

    if ok_cnt:
        print("\n下一步：")
        print("  1) 把这几个 .BMP 直接拷到 SD 卡【根目录】（不要放进子文件夹）")
        print(f"  2) RTL 只认根目录里【前 {SCAN_TARGET_COUNT} 个】.BMP"
              f"（扫描顺序 = 目录项的物理写入顺序，不是资源管理器里的排序）")
        print("     → 所以请按 IMG0、IMG1、IMG2、IMG3 的顺序逐个拷贝；"
              "一起多选拖拽通常也是按名字顺序")
        print(f"  3) 上板前想再确认一次：python {Path(sys.argv[0]).name} --check {outdir}")
    return 0


def do_check(args):
    targets = []
    for raw in args.inputs:
        p = Path(raw)
        if p.is_dir():
            got = [q for q in p.glob("*") if q.is_file()]
            got.sort(key=natural_key)
            targets.extend([q for q in got if q.suffix.lower() == ".bmp"])
            extra = [q.name for q in got if q.suffix.lower() != ".bmp"]
            if extra:
                print(f"（{p} 里非 .BMP 文件 {len(extra)} 个，RTL 不认，已跳过）")
        elif p.is_file():
            targets.append(p)
        else:
            warn(f"找不到 {p}")

    if not targets:
        die("没有可检查的 BMP")

    print(f"按 bmp_read.v 的判据检查 {len(targets)} 个文件：\n")
    good = 0
    for t in targets:
        ok, _ = verify(t, DST_W_DEFAULT, DST_H_DEFAULT, verbose=True)
        good += bool(ok)

    print(f"\n合格 {good}/{len(targets)}")
    if len(targets) > SCAN_TARGET_COUNT:
        print(f"!! 共 {len(targets)} 个 .BMP，但 SCAN_TARGET_COUNT={SCAN_TARGET_COUNT}，"
              f"只有前 {SCAN_TARGET_COUNT} 个会被加载")
    return 0 if good == len(targets) else 2


# ===========================================================================
# SD 卡清理
#   --scrub  把 BMP 开头的 'BM' 魔数抹成 0x00
#   --wipe   删掉目录下所有可见文件和文件夹
#   两个同时给 = 先毁魔数再删，物理残留的数据块也已经认不出来了
# ===========================================================================

def drive_type(p: Path) -> str:
    """removable / fixed / network / cdrom / ramdisk / unknown / posix"""
    if os.name != "nt":
        return "posix"
    try:
        import ctypes
        k32 = ctypes.WinDLL("kernel32", use_last_error=True)
        k32.GetDriveTypeW.argtypes = [ctypes.c_wchar_p]
        k32.GetDriveTypeW.restype = ctypes.c_uint
        drive = os.path.splitdrive(str(p.resolve()))[0] + "\\"
        t = k32.GetDriveTypeW(drive)
    except Exception:
        return "unknown"
    return {2: "removable", 3: "fixed", 4: "network",
            5: "cdrom", 6: "ramdisk"}.get(t, "unknown")


def guard_target(root: Path, force: bool, action: str) -> Path:
    """路径安全检查。任何会碰到系统 / 用户关键目录的请求一律拒绝。"""
    if not root.exists():
        die(f"找不到 {root}")
    if not root.is_dir():
        die(f"{root} 不是目录（本功能只接受目录）")

    r = root.resolve()
    parts = list(r.parts)

    # 盘符根下的第一层目录名
    if len(parts) >= 2:
        top = parts[1].lower()
        if top in DANGER_TOP:
            # Windows / Program Files 这类任何深度都拦；
            # Users 只拦浅层（C:\Users、C:\Users\张三），
            # 更深的（C:\Users\张三\Desktop\sd_test）交给下面的 home 检查
            if top not in ("users", "documents and settings") or len(parts) <= 3:
                die(f"拒绝{action}：{r} 位于系统目录（{parts[1]}），脚本不会碰这里")

    # 用户主目录及其浅层子目录
    try:
        home = Path.home().resolve()
        if r == home or home in r.parents:
            if len(r.parts) < len(home.parts) + 2:
                die(f"拒绝{action}：{r} 在用户主目录里且层级太浅\n"
                    f"      至少要指到 {home}\\<某目录>\\<某目录> 这种深度")
    except Exception:
        pass

    # 盘符根本身
    if len(parts) <= 1:
        dt = drive_type(r)
        if dt in ("network", "cdrom", "unknown"):
            die(f"拒绝{action}：{r} 是 {dt} 设备，不是可写磁盘")
        if dt == "fixed" and not force:
            die(f"拒绝{action}：{r} 是【固定硬盘】的根目录。\n"
                f"      这个功能是给 SD 卡 / U 盘用的。\n"
                f"      真的确定要清整块硬盘的根目录，再加 --force")
    return r


def walk_entries(root: Path, skip_sys: bool = True):
    """返回 (所有文件, 所有子目录, 读取错误, 被跳过的系统目录)。

    os.walk 默认不跟符号链接。skip_sys=True 时会跳过顶层的
    System Volume Information / $RECYCLE.BIN —— 那是 Windows 自己建的，
    普通权限删不掉，删了下次插卡还会重建，列进清单只会制造困惑。
    """
    files, dirs, errs, skipped = [], [], [], []

    def onerr(e):
        errs.append(str(e))

    for dp, dns, fns in os.walk(root, topdown=True, onerror=onerr, followlinks=False):
        d = Path(dp)
        if skip_sys and d == root:
            keep = []
            for dn in dns:
                if dn.lower() in SKIP_TOP:
                    skipped.append(d / dn)
                else:
                    keep.append(dn)
            dns[:] = keep               # 改 dns 才能真正阻止 os.walk 进去
        for fn in fns:
            files.append(d / fn)
        for dn in dns:
            dirs.append(d / dn)

    files.sort(key=lambda p: str(p).lower())
    dirs.sort(key=lambda p: (len(p.parts), str(p).lower()))
    return files, dirs, errs, skipped


def rtl_would_accept(path: Path):
    """按 bmp_read.v 的 hdr_ok 判定：这个文件现在会不会被 FPGA 当成图片。

    hdr_ok = header_0=='B' && header_1=='M' && (bpp==24||32)
             && compression==0 && w/h 在 1..4096
    """
    try:
        with open(path, "rb") as f:
            head = f.read(54)
    except OSError:
        return False, "读不了"
    if len(head) < 54:
        return False, "不足 54 字节"
    if head[0:2] != b"BM":
        return False, "魔数不是 'BM'"
    bpp = struct.unpack_from("<H", head, 28)[0]
    comp = struct.unpack_from("<I", head, 30)[0]
    w = struct.unpack_from("<i", head, 18)[0]
    h = struct.unpack_from("<i", head, 22)[0]
    if bpp not in ALLOWED_BPP:
        return False, f"bpp={bpp}"
    if comp != 0:
        return False, f"comp={comp}"
    if not (1 <= w <= MAX_DIM and 1 <= h <= MAX_DIM):
        return False, f"尺寸 {w}x{h}"
    return True, ""


def scrub_file(path: Path, nbytes: int, dry: bool) -> str:
    """破坏一个文件的 'BM' 魔数。返回状态字符串。"""
    try:
        size = path.stat().st_size
    except OSError as exc:
        return "err:" + str(exc)
    if size < 2:
        return "not-bmp"

    try:
        with open(path, "rb") as f:
            if f.read(2) != b"BM":
                return "not-bmp"
    except OSError as exc:
        return "err:" + str(exc)

    if dry:
        return "would"

    try:                                    # 去掉只读属性，否则写不进去
        os.chmod(path, stat_mod.S_IWRITE | stat_mod.S_IREAD)
    except OSError:
        pass

    try:
        with open(path, "r+b") as f:
            f.write(b"\x00" * min(nbytes, size))
            f.flush()
            os.fsync(f.fileno())            # 可移动盘务必真刷下去
    except OSError as exc:
        return "err:" + str(exc)

    try:                                    # 复检
        with open(path, "rb") as f:
            if f.read(2) == b"BM":
                return "err:回读仍是 'BM'，可能写保护"
    except OSError as exc:
        return "err:" + str(exc)
    return "ok"


def do_scrub(roots, args) -> int:
    nbytes = max(2, args.scrub_bytes)
    print("=" * 68)
    print("扫描 BMP 并破坏魔数 'BM' → 往文件开头写 0x00")
    print(f"判据：bmp_read.v 的 hdr_ok 要求 header_0=='B' && header_1=='M'")
    print(f"      魔数被抹掉后，即使 FAT 数据块还在，FPGA 也会直接跳过")
    print(f"写入字节数：{nbytes}")
    print("=" * 68)

    st = {"ok": 0, "would": 0, "bmp_ext_no_hdr": 0, "err": 0, "scanned": 0}
    errs = []

    for root in roots:
        r = guard_target(root, args.force, "扫描")
        files, _dirs, werrs, _sk = walk_entries(r)
        st["scanned"] += len(files)
        print(f"\n[{r}]  设备 {drive_type(r)}  {len(files)} 个文件")

        for f in files:
            res = scrub_file(f, nbytes, args.dry_run)
            rel = f.relative_to(r)
            if res == "ok":
                st["ok"] += 1
                print(f"  [已破坏] {rel}")
            elif res == "would":
                st["would"] += 1
                print(f"  [将破坏] {rel}")
            elif res == "not-bmp":
                if f.suffix.lower() in (".bmp", ".dib"):
                    ok, why = rtl_would_accept(f)
                    if not ok:
                        st["bmp_ext_no_hdr"] += 1
                        print(f"  [扩展名像 BMP，但{why} → FPGA 本来就会跳过] {rel}")
            elif res.startswith("err:"):
                st["err"] += 1
                errs.append((rel, res[4:]))
                print(f"  [失败] {rel}: {res[4:]}")

        for e in werrs[:5]:
            warn(e)

    print("\n" + "-" * 68)
    print(f"扫描 {st['scanned']} 个文件")
    if args.dry_run:
        print(f"待破坏魔数：{st['would']} 个")
        print("\n这只是预览，没有改动任何文件。去掉 --dry-run 即可执行。")
    else:
        print(f"魔数已破坏：{st['ok']} 个")
    print(f"跳过的非 BMP：{st['scanned'] - st['ok'] - st['would'] - st['err']} 个"
          f"（其中 {st['bmp_ext_no_hdr']} 个扩展名是 .bmp 但头已无效）")
    if st["err"]:
        print(f"失败：{st['err']} 个")
        for rel, e in errs[:10]:
            print(f"  - {rel}: {e}")
        print("  常见原因：文件被占用 / SD 卡写保护开关没关 / 无写权限")

    if not args.dry_run and st["ok"]:
        print("\n注意：被破坏的文件还占着空间，但 FPGA 已经认不出来了。")
        if not args.wipe:
            print("      想连空间一起腾出来：加 --wipe --yes 或直接格式化。")
        print("      （此后跑 --check 会报 \"magic 不是 'BM'\"，这是预期的）")
    return 2 if st["err"] else 0


def do_wipe(roots, args) -> int:
    plans = []
    for root in roots:
        r = guard_target(root, args.force, "清空")
        files, dirs, werrs, skipped = walk_entries(r)
        plans.append((r, files, dirs, werrs, skipped))

    total_files = sum(len(p[1]) for p in plans)
    total_dirs = sum(len(p[2]) for p in plans)
    total_bytes = 0
    for _r, files, _d, _e, _sk in plans:
        for f in files:
            try:
                total_bytes += f.stat().st_size
            except OSError:
                pass

    print("=" * 68)
    print(f"准备删除 {total_files} 个文件 + {total_dirs} 个文件夹，共 {human(total_bytes)}")
    print("=" * 68)

    limit = args.list_limit
    for r, files, dirs, werrs, skipped in plans:
        print(f"\n[{r}]  设备 {drive_type(r)}")
        if skipped:
            print(f"  跳过 {len(skipped)} 个 Windows 系统目录（删不掉也会重建）："
                  + "、".join(s.name for s in skipped))
        if not files and not dirs:
            print("  （已经空了）")
            continue
        shown = files if limit <= 0 else files[:limit]
        for f in shown:
            try:
                sz = human(f.stat().st_size)
            except OSError:
                sz = "?"
            print(f"  - {f.relative_to(r)}   ({sz})")
        if len(files) > len(shown):
            print(f"  ... 还有 {len(files) - len(shown)} 个文件未显示"
                  f"（--list-limit 0 可显示全部）")
        if dirs:
            print(f"  另有 {len(dirs)} 个文件夹会被递归删除")
        for e in werrs[:5]:
            warn(e)

    if total_files == 0 and total_dirs == 0:
        print("\n没什么可删的。")
        return 0

    if not args.yes:
        print("\n" + "!" * 68)
        print("!! 以上只是预览，一个文件都没删。")
        print("!! SD 卡 / FAT32 没有回收站，删掉就找不回来了。")
        print("!! 确认无误后加 --yes 才真正执行：")
        print(f"!!   python {Path(sys.argv[0]).name} --wipe \"{plans[0][0]}\" --yes")
        if not args.scrub:
            print("!!")
            print("!! 想连物理残留一起处理，请用：--scrub --wipe ... --yes")
            print("!! （只删文件的话，NAND 里的数据块仍带着完整 'BM' 头）")
        print("!" * 68)
        return 0

    print("\n开始删除 ...")
    ok_f = ok_d = 0
    fails = []
    for r, files, dirs, _werrs, _sk in plans:
        for f in files:
            try:
                try:
                    os.chmod(f, stat_mod.S_IWRITE | stat_mod.S_IREAD)
                except OSError:
                    pass
                os.remove(f)
                ok_f += 1
            except OSError as exc:
                fails.append((f, str(exc)))
        for d in sorted(dirs, key=lambda p: len(p.parts), reverse=True):
            try:
                try:
                    os.chmod(d, stat_mod.S_IWRITE | stat_mod.S_IREAD)
                except OSError:
                    pass
                os.rmdir(d)
                ok_d += 1
            except OSError as exc:
                fails.append((d, str(exc)))

    freed = 0
    for _r, files, _d, _e, _sk in plans:
        for f in files:
            try:
                freed += f.stat().st_size
            except OSError:
                pass

    print(f"已删除 {ok_f} 个文件 / {ok_d} 个文件夹，腾出约 {human(total_bytes)}")
    if fails:
        print(f"\n!! {len(fails)} 项删不掉：")
        for p, e in fails[:10]:
            print(f"   - {p}: {e}")
        print("   常见原因：")
        print("     · 文件正被别的程序打开（资源管理器缩略图、看图软件、杀毒扫描）")
        print("     · System Volume Information / RECYCLE.BIN 是 Windows 建的，")
        print("       普通权限删不掉 → 用【管理员】CMD：rmdir /s /q \"<路径>\"")
        print("     · SD 卡的写保护开关（卡套侧面那个小滑块）打开了")
        return 2

    print("\n清理完成。现在可以重新拷入 IMG0.BMP ~ IMG3.BMP 了。")
    return 0


def do_list_drives(args) -> int:
    """列出所有盘符 + 类型 + 容量，用来确认 SD 卡是哪个盘。"""
    if os.name != "nt":
        die("--drives 只在 Windows 上有意义")
    import ctypes
    import shutil as _sh

    k32 = ctypes.WinDLL("kernel32", use_last_error=True)
    k32.GetLogicalDrives.restype = ctypes.c_uint
    mask = k32.GetLogicalDrives()

    kind = {"removable": "可移动 SD卡/U盘", "fixed": "固定硬盘",
            "network": "网络盘", "cdrom": "光驱",
            "ramdisk": "内存盘", "unknown": "未知"}

    print("盘符   类型              总容量      可用空间")
    print("-" * 60)
    found = []
    for i in range(26):
        if not (mask >> i) & 1:
            continue
        root = f"{chr(65 + i)}:\\"
        t = drive_type(Path(root))
        try:
            u = _sh.disk_usage(root)
            tot, free = human(u.total), human(u.free)
        except OSError:
            tot, free = "?", "?"
        tip = "   <<< 多半是这个" if t == "removable" else ""
        print(f"  {chr(65 + i)}:   {pad(kind.get(t, t), 18)} {tot:>9} {free:>9}{tip}")
        if t == "removable":
            found.append(f"{chr(65 + i)}:\\")

    print()
    if found:
        print("可移动盘：" + "、".join(found))
        print(f"下一步试试：python bmp4fpga.py --check {found[0]}")
    else:
        print("没检测到可移动盘 —— SD 卡没插好，或者读卡器被识别成了固定硬盘。")
    return 0


def do_clean(args) -> int:
    roots = []
    for raw in args.inputs:
        p = Path(raw)
        if not p.exists():
            warn(f"找不到 {p}")
        elif p.is_file():
            warn(f"{p} 是文件，--scrub / --wipe 只接受目录，已忽略")
        else:
            roots.append(p)
    if not roots:
        die("没有可处理的目录")
    if len(roots) > 1:
        print(f"（一次处理 {len(roots)} 个目录）\n")

    rc = 0
    if args.scrub:
        rc |= do_scrub(roots, args)
        if args.wipe:
            print("\n")
    if args.wipe:
        rc |= do_wipe(roots, args)
    return rc


def build_parser():
    p = argparse.ArgumentParser(
        description="把任意图片转成 info_signage FPGA 工程可直接读取的 BMP",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例:\n"
               "  python bmp4fpga.py poster.jpg -o E:\\sd\n"
               "  python bmp4fpga.py D:\\pics -o E:\\sd --mode native\n"
               "  python bmp4fpga.py --check E:\\sd\n"
               "  python bmp4fpga.py --scrub E:\\\n"
               "  python bmp4fpga.py --wipe E:\\            (只预览)\n"
               "  python bmp4fpga.py --scrub --wipe E:\\ --yes\n"
               "\n"
               "清理流程建议：先 --scrub 再 --wipe（顺序反了残留数据仍带完整 'BM' 头）\n",
    )
    p.add_argument("inputs", nargs="*", help="图片文件或目录（可多个）")
    p.add_argument("-o", "--outdir", default="sd_bmp",
                   help="输出目录（默认 ./sd_bmp）")
    p.add_argument("-W", "--width", type=int, default=DST_W_DEFAULT,
                   help=f"目标宽（默认 {DST_W_DEFAULT}）")
    p.add_argument("-H", "--height", type=int, default=DST_H_DEFAULT,
                   help=f"目标高（默认 {DST_H_DEFAULT}）")
    p.add_argument("-m", "--mode", default="fit",
                   choices=["fit", "fill", "stretch", "native"],
                   help="fit=等比+留边(默认) | fill=等比+裁剪 | stretch=拉伸 | native=原尺寸")
    p.add_argument("--resample", default="lanczos",
                   choices=list(RESAMPLE_NAMES), help="预缩放算法（默认 lanczos）")
    p.add_argument("--bg", default="0,0,0", help='fit 留边颜色 "R,G,B"（默认 0,0,0）')
    p.add_argument("--bpp", type=int, default=24, choices=list(ALLOWED_BPP),
                   help="位深 24(默认) 或 32（32 的 alpha 会被 RTL 丢弃）")
    p.add_argument("--name", default="IMG{n}.BMP",
                   help='命名模板，占位 {n} / {stem}（默认 IMG{n}.BMP）')
    p.add_argument("--start", type=int, default=0, help="起始编号（默认 0）")
    p.add_argument("--no-upscale", action="store_true", help="小图不放大，只缩不放")
    p.add_argument("-r", "--recursive", action="store_true", help="目录递归查找")
    p.add_argument("--dry-run", action="store_true", help="只打印计划，不写文件")
    p.add_argument("--check", action="store_true",
                   help="只检查已有 BMP 是否符合 RTL 要求，不做转换")
    g = p.add_argument_group("清理 SD 卡（不依赖 Pillow）")
    g.add_argument("--scrub", action="store_true",
                   help="扫描目录，把每个 BMP 开头的魔数 'BM' 抹成 0x00")
    g.add_argument("--scrub-bytes", type=int, default=SCRUB_BYTES_DEFAULT,
                   help=f"魔数区要写多少个 0x00（默认 {SCRUB_BYTES_DEFAULT}，"
                        f"给 54 可连整个 BMP 信息头一起清掉）")
    g.add_argument("--wipe", action="store_true",
                   help="删除目录下所有可见文件和文件夹（默认只预览，要 --yes 才真删）")
    g.add_argument("--yes", action="store_true",
                   help="确认执行 --wipe 的删除（不加就只打印清单）")
    g.add_argument("--force", action="store_true",
                   help="允许对【固定硬盘根目录】操作（危险，通常不需要）")
    g.add_argument("--list-limit", type=int, default=50,
                   help="--wipe 预览时最多列多少个文件（0 = 全部，默认 50）")
    p.add_argument("--drives", action="store_true",
                   help="列出所有盘符和类型，用来确认 SD 卡是哪个盘")
    return p


def main():
    try:
        # 写到控制台时交给 PEP 528（Windows 会用 UTF-16 直写，中文本来就不会乱）；
        # 只有重定向到文件 / 管道时才强制 UTF-8，方便 grep 和比对脚本。
        if not sys.stdout.isatty():
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
            sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

    args = build_parser().parse_args()

    if args.drives:
        return do_list_drives(args)

    if not args.inputs:
        build_parser().print_help()
        return 0

    # 清理功能不需要 Pillow，放在最前面
    if args.scrub or args.wipe:
        return do_clean(args)

    if not args.check and (Image is None):
        die("缺少 Pillow，请先安装：pip install Pillow")
    if not args.check:
        if not (1 <= args.width <= MAX_DIM and 1 <= args.height <= MAX_DIM):
            die(f"目标尺寸要在 1..{MAX_DIM} 之间")

    if args.check:
        return do_check(args)

    if args.dry_run:
        # dry-run 时不落盘，但仍要加载图片拿到真实尺寸
        print("（--dry-run 模式）\n")
    return do_convert(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n已中断", file=sys.stderr)
        raise SystemExit(130)
