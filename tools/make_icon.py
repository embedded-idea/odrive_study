"""生成 OdrvStudio 图标 (.ico)。

设计:FOC 磁场定向控制主题 —— 深色圆角底板上,三相电流正弦波
(U/V/W 黄绿蓝,互差 120°) 为底,叠加 SVPWM 六边形与内切圆,
中央一支橙色电压矢量箭头(带拖尾弧线)指向右上。

用法: .venv\\Scripts\\python tools\\make_icon.py
输出: tools\\OdrvStudio\\assets\\odrvstudio.ico
"""
import math
import os

from PIL import Image, ImageDraw

SIZE = 1024  # 高分辨率绘制,再缩小生成各尺寸
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "OdrvStudio", "assets", "odrvstudio.ico")

PHASE_COLORS = ["#f1c40f", "#2ecc71", "#3498db"]  # U / V / W


def lerp(a, b, t):
    return tuple(int(a[i] + (b[i] - a[i]) * t) for i in range(3))


def rounded_gradient_bg(size, top, bottom, radius):
    """圆角矩形 + 垂直渐变底板。"""
    img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    grad = Image.new("RGBA", (size, size))
    gd = ImageDraw.Draw(grad)
    for y in range(size):
        gd.line([(0, y), (size, y)], fill=lerp(top, bottom, y / size))
    mask = Image.new("L", (size, size), 0)
    md = ImageDraw.Draw(mask)
    md.rounded_rectangle([0, 0, size - 1, size - 1], radius=radius, fill=255)
    img.paste(grad, (0, 0), mask)
    return img


def polar(cx, cy, r, deg):
    a = math.radians(deg)
    return cx + r * math.cos(a), cy + r * math.sin(a)


def draw_foc(d, cx, cy, s):
    # ---- 三相正弦波(底层,横贯画面) ----
    amp = 140 * s
    x0, x1 = 70 * s, (1024 - 70) * s
    width = int(13 * s)
    for k, color in enumerate(PHASE_COLORS):
        phase = k * 120
        pts = []
        x = x0
        while x <= x1:
            t = (x - cx) / (430 * s)          # 归一化相位(约 1 个周期)
            y = cy + amp * math.sin(2 * math.pi * t - math.radians(phase))
            pts.append((x, y))
            x += 4 * s
        # 波形发光底衬
        d.line(pts, fill=color + "40", width=width + int(9 * s), joint="curve")
        d.line(pts, fill=color, width=width, joint="curve")

    # ---- SVPWM 六边形(扇区边界) ----
    r_hex = 320 * s
    hex_pts = [polar(cx, cy, r_hex, 60 * k - 30) for k in range(6)]
    d.polygon(hex_pts, outline="#6b7c93", width=int(9 * s))
    for p in hex_pts:   # 顶点小圆点
        r = 13 * s
        d.ellipse([p[0] - r, p[1] - r, p[0] + r, p[1] + r], fill="#6b7c93")

    # ---- 内切圆(线性调制边界) ----
    r_circ = r_hex * math.cos(math.pi / 6)
    d.ellipse([cx - r_circ, cy - r_circ, cx + r_circ, cy + r_circ],
              outline="#8fa2b8", width=int(5 * s))

    # ---- α/β 轴(细十字) ----
    d.line([cx - r_hex, cy, cx + r_hex, cy], fill="#46566a",
           width=int(4 * s))
    d.line([cx, cy - r_hex, cx, cy + r_hex], fill="#46566a",
           width=int(4 * s))

    # ---- 电压矢量(拖尾弧 + 箭头) ----
    ang = -52        # 指向右上
    r_vec = r_circ
    # 拖尾弧:从箭头后方顺时针扫过约 130°
    d.arc([cx - r_vec, cy - r_vec, cx + r_vec, cy + r_vec],
          ang + 16, ang + 150, fill="#e67e2266", width=int(13 * s))
    p_end = polar(cx, cy, r_vec, ang)
    # 矢量杆(发光底衬 + 主体)
    d.line([cx, cy, p_end[0], p_end[1]], fill="#f39c1255",
           width=int(34 * s))
    d.line([cx, cy, p_end[0], p_end[1]], fill="#e67e22", width=int(22 * s))
    # 箭头(底边垂直于杆的三角形)
    ah = 70 * s       # 箭头长度
    aw = 56 * s       # 底边宽
    base = polar(cx, cy, r_vec - ah, ang)
    perp = polar(0, 0, 1, ang + 90)   # 垂直方向单位向量
    pa = (base[0] + perp[0] * aw / 2, base[1] + perp[1] * aw / 2)
    pb = (base[0] - perp[0] * aw / 2, base[1] - perp[1] * aw / 2)
    pc = polar(cx, cy, r_vec + 12 * s, ang)
    d.polygon([pa, pb, pc], fill="#e67e22")
    # 矢量起点圆
    r0 = 26 * s
    d.ellipse([cx - r0, cy - r0, cx + r0, cy + r0], fill="#f5f7fa",
              outline="#2c3e50", width=int(5 * s))


def main():
    s = SIZE / 1024.0
    img = rounded_gradient_bg(SIZE, (44, 62, 80), (26, 32, 44), radius=180)
    d = ImageDraw.Draw(img)
    draw_foc(d, SIZE / 2, SIZE / 2, s)

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    sizes = [(16, 16), (24, 24), (32, 32), (48, 48), (64, 64),
             (128, 128), (256, 256)]
    img.resize((256, 256), Image.LANCZOS).save(
        OUT, format="ICO", sizes=sizes)
    print("saved:", OUT)
    # 预览图
    preview = OUT.replace(".ico", "_preview.png")
    img.resize((256, 256), Image.LANCZOS).save(preview)
    print("preview:", preview)


if __name__ == "__main__":
    main()
