# -*- coding: utf-8 -*-
"""
把 _src 下四张 AI 底图排成公益广告版式，输出 24 位未压缩 BMP（自底向上），
文件名沿用 IMG0~IMG3，直接覆盖 SD 卡素材。

版式（四张完全一致，轮播时结构不跳）：
    左侧 56% 宽白色渐变蒙版 → 放两行主标题 + 一行副标题
    底部 7.5% 高深绿色带 → 盖住右下角水印，写栏目名与「公益宣传」
"""
import os
from PIL import Image, ImageDraw, ImageFont, ImageOps

SRC = r'D:\FPGA\project\doc\sd_images\_src'
OUT = r'D:\FPGA\project\doc\sd_images'
FONT_B = r'C:\Windows\Fonts\msyhbd.ttc'
FONT_R = r'C:\Windows\Fonts\msyh.ttc'

TITLE_COLOR = (18, 40, 26)
SUB_COLOR = (86, 104, 92)
BAND_COLOR = (20, 104, 58)

# (源图, 目标尺寸, 主标题上行, 主标题下行, 副标题)
ITEMS = [
    ('src_img0.png', (640, 480),
     '四色分类', '从我做起', '蓝 · 红 · 绿 · 灰 各归其位'),
    ('src_img1.png', (800, 600),
     '一个瓶子', '一份资源', '请投入蓝色可回收桶'),
    ('src_img2.png', (1024, 768),
     '变废为宝', '城市更美', '厨余垃圾也是资源'),
    ('src_img3.png', (1280, 720),
     '分类投放', '定时定点', '让每件垃圾去对的地方'),
]


def left_mask(w, h, ratio=0.56):
    """左浓右淡的白色渐变，保证标题压在任何底色上都清楚。"""
    band = int(w * ratio)
    col = Image.new('L', (band, 1))
    for x in range(band):
        t = x / (band - 1)
        col.putpixel((x, 0), int(238 * (1 - t) ** 1.25))
    grad = col.resize((band, h))
    full = Image.new('L', (w, h), 0)
    full.paste(grad, (0, 0))
    layer = Image.new('RGBA', (w, h), (255, 255, 255, 255))
    layer.putalpha(full)
    return layer


def compose(src, size, t1, t2, sub):
    w, h = size
    img = Image.open(src).convert('RGB')
    img = ImageOps.fit(img, size, method=Image.LANCZOS, centering=(0.5, 0.5))
    img = Image.alpha_composite(img.convert('RGBA'), left_mask(w, h))

    d = ImageDraw.Draw(img)
    fs1 = round(0.088 * w)
    fs2 = round(0.034 * w)
    f1 = ImageFont.truetype(FONT_B, fs1)
    f2 = ImageFont.truetype(FONT_R, fs2)

    band_h = round(0.075 * h)
    line_h = round(fs1 * 1.30)
    gap = round(fs1 * 0.34)

    def tb(text, font):
        b = d.textbbox((0, 0), text, font=font)
        return b[2] - b[0], b[3] - b[1], b[1]

    _, h1, off1 = tb(t1, f1)
    _, h2, off2 = tb(t2, f1)
    _, h3, off3 = tb(sub, f2)
    block = line_h + line_h + gap + h3
    y = round((h - band_h - block) / 2)
    x = round(0.055 * w)

    d.text((x, y - off1), t1, font=f1, fill=TITLE_COLOR)
    d.text((x, y + line_h - off2), t2, font=f1, fill=TITLE_COLOR)
    d.text((x, y + line_h * 2 + gap - off3), sub, font=f2, fill=SUB_COLOR)

    # 底部色带：遮住右下角水印，同时充当页脚
    d.rectangle([0, h - band_h, w, h], fill=BAND_COLOR)
    fb = ImageFont.truetype(FONT_B, round(0.030 * w))
    fy = h - band_h + (band_h - round(0.030 * w)) // 2 - round(0.006 * w)
    d.text((x, fy), '垃圾分类 · 文明生活', font=fb, fill=(255, 255, 255))
    tw = d.textbbox((0, 0), '公益宣传', font=fb)[2]
    d.text((w - x - tw, fy), '公益宣传', font=fb, fill=(255, 255, 255))

    return img.convert('RGB')


def main():
    os.makedirs(os.path.join(OUT, '_orig'), exist_ok=True)
    for i, (src, size, t1, t2, sub) in enumerate(ITEMS):
        dst = os.path.join(OUT, 'IMG%d.BMP' % i)
        if os.path.isfile(dst):
            bak = os.path.join(OUT, '_orig', 'IMG%d.BMP' % i)
            if not os.path.isfile(bak):
                Image.open(dst).save(bak)
        out = compose(os.path.join(SRC, src), size, t1, t2, sub)
        out.save(dst, 'BMP')
        chk = Image.open(dst)
        print('IMG%d.BMP  %dx%d  %s  %d bytes' % (i, chk.size[0], chk.size[1], chk.mode, os.path.getsize(dst)))
        out.save(os.path.join(SRC, 'preview_IMG%d.png' % i))


if __name__ == '__main__':
    main()
