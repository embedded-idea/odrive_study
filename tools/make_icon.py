"""生成 OdrvStudio 图标 (.ico)。

设计:BLDC 电机横截面 —— 深色圆角底板 + 定子铁芯(12 槽铜线圈)
+ 转子 4 极磁钢(N/S 红蓝交替)+ 中心转轴,右下角三根相线(U/V/W)。

用法: .venv\\Scripts\\python tools\\make_icon.py
输出: tools\\OdrvStudio\\assets\\odrvstudio.ico
"""
import math
import os

from PIL import Image, ImageDraw

SIZE = 1024  # 高分辨率绘制,再缩小生成各尺寸
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "OdrvStudio", "assets", "odrvstudio.ico")


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


def draw_motor(d, cx, cy, s):
    """s = 缩放系数(相对 1024 基准)。"""
    # ---- 定子外圈(铁芯) ----
    r_out = 300 * s
    r_stator_in = 205 * s
    d.ellipse([cx - r_out, cy - r_out, cx + r_out, cy + r_out],
              fill="#3d4854", outline="#55637a", width=int(6 * s))
    d.ellipse([cx - r_stator_in, cy - r_stator_in,
               cx + r_stator_in, cy + r_stator_in], fill="#243040")

    # ---- 12 个定子齿 + 铜线圈 ----
    n_slots = 12
    for k in range(n_slots):
        ang = k * 360 / n_slots
        a0, a1 = ang - 13, ang + 13
        # 齿(从定子内缘向外)
        d.pieslice([cx - r_out, cy - r_out, cx + r_out, cy + r_out],
                   a0, a1, fill="#3d4854")
        # 铜线圈(饱满的梯形绕组,靠近内缘)
        r1, r2 = r_stator_in + 6 * s, r_stator_in + 66 * s
        p1 = polar(cx, cy, r1, a0)
        p2 = polar(cx, cy, r1, a1)
        p3 = polar(cx, cy, r2, a1)
        p4 = polar(cx, cy, r2, a0)
        d.polygon([p1, p2, p3, p4], fill="#e08a36", outline="#8a4d16",
                  width=max(1, int(4 * s)))

    # ---- 转子:4 极磁钢(N/S 红蓝交替) ----
    r_rotor = 168 * s
    d.ellipse([cx - r_rotor, cy - r_rotor, cx + r_rotor, cy + r_rotor],
              fill="#1a222e", outline="#0f141c", width=int(4 * s))
    n_poles = 4
    for k in range(n_poles):
        a0 = k * 360 / n_poles + 2
        a1 = (k + 1) * 360 / n_poles - 2
        color = "#c0392b" if k % 2 == 0 else "#2980b9"
        d.pieslice([cx - r_rotor, cy - r_rotor, cx + r_rotor, cy + r_rotor],
                   a0, a1, fill=color)
    # 转子边缘环
    d.ellipse([cx - r_rotor, cy - r_rotor, cx + r_rotor, cy + r_rotor],
              outline="#0d1118", width=int(10 * s))

    # ---- 转轴 ----
    r_shaft = 52 * s
    d.ellipse([cx - r_shaft, cy - r_shaft, cx + r_shaft, cy + r_shaft],
              fill="#aeb9c6", outline="#7d8894", width=int(4 * s))
    d.ellipse([cx - r_shaft * 0.45, cy - r_shaft * 0.45,
               cx + r_shaft * 0.45, cy + r_shaft * 0.45],
              fill="#8a95a3")

    # ---- 三根相线 U/V/W(右下角引出) ----
    wire_r0 = r_out * 0.82
    colors = ["#f1c40f", "#2ecc71", "#3498db"]
    for k, c in enumerate(colors):
        a = 38 + k * 17  # 右下角度范围
        p_start = polar(cx, cy, wire_r0, a)
        p_mid = polar(cx, cy, r_out + 26 * s, a + 6)
        p_end = polar(cx, cy, r_out + 92 * s, a + 14)
        d.line([p_start, p_mid, p_end], fill=c,
               width=max(2, int(16 * s)), joint="curve")

    # ---- 高光弧(玻璃质感) ----
    d.arc([cx - r_out, cy - r_out, cx + r_out, cy + r_out],
          200, 320, fill="#ffffff40", width=int(10 * s))


def main():
    s = SIZE / 1024.0
    img = rounded_gradient_bg(SIZE, (44, 62, 80), (26, 32, 44), radius=180)
    d = ImageDraw.Draw(img)
    draw_motor(d, SIZE / 2, SIZE / 2, s)

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
