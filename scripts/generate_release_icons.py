#!/usr/bin/env python3
"""Deterministic release icon generator for ChipCutter.

Generates:
  - packaging/icons/chipcutter.svg
  - packaging/icons/chipcutter-256.png
  - packaging/icons/hicolor/{64x64,128x128,256x256,512x512}/apps/chipcutter.png
  - packaging/windows/chipcutter.ico (16, 32, 48, 64, 128, 256 px)

Uses only the Python 3 standard library so it runs reproducibly on Linux,
Windows, and macOS with zero external dependencies.
"""

from __future__ import annotations

import math
import os
import struct
import zlib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

SVG_SOURCE = """\
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512" width="512" height="512">
  <defs>
    <linearGradient id="bg" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0%" stop-color="#1E2738"/>
      <stop offset="100%" stop-color="#111722"/>
    </linearGradient>
    <linearGradient id="rim" x1="0" y1="0" x2="1" y2="1">
      <stop offset="0%" stop-color="#4A5D7A"/>
      <stop offset="100%" stop-color="#222C3D"/>
    </linearGradient>
    <linearGradient id="steel" x1="0" y1="0" x2="1" y2="0">
      <stop offset="0%" stop-color="#94A3B8"/>
      <stop offset="35%" stop-color="#F1F5F9"/>
      <stop offset="70%" stop-color="#CBD5E1"/>
      <stop offset="100%" stop-color="#64748B"/>
    </linearGradient>
    <linearGradient id="amber" x1="0" y1="0" x2="1" y2="1">
      <stop offset="0%" stop-color="#FFB84D"/>
      <stop offset="55%" stop-color="#F97316"/>
      <stop offset="100%" stop-color="#EA580C"/>
    </linearGradient>
    <linearGradient id="cyan" x1="0" y1="0" x2="1" y2="0">
      <stop offset="0%" stop-color="#0EA5E9"/>
      <stop offset="50%" stop-color="#38BDF8"/>
      <stop offset="100%" stop-color="#0284C7"/>
    </linearGradient>
  </defs>
  <!-- Rounded badge background -->
  <rect x="24" y="24" width="464" height="464" rx="108" fill="url(#bg)" stroke="url(#rim)" stroke-width="10"/>
  <!-- Machined cyan stock contour arc at base -->
  <path d="M 116 372 A 164 62 0 0 0 396 372" fill="none" stroke="url(#cyan)" stroke-width="20" stroke-linecap="round" opacity="0.92"/>
  <path d="M 156 396 A 120 42 0 0 0 356 396" fill="none" stroke="#38BDF8" stroke-width="10" stroke-linecap="round" opacity="0.45"/>
  <!-- Dynamic amber-orange cutting chip & toolpath sweep -->
  <path d="M 134 268 A 126 126 0 1 0 368 220" fill="none" stroke="url(#amber)" stroke-width="26" stroke-linecap="round"/>
  <!-- Flying chip sparks -->
  <circle cx="386" cy="186" r="14" fill="#FFB84D"/>
  <circle cx="412" cy="150" r="8" fill="#FDE68A"/>
  <!-- CNC Endmill body -->
  <g>
    <!-- Shank & collar -->
    <rect x="216" y="76" width="80" height="72" rx="12" fill="url(#steel)"/>
    <rect x="206" y="138" width="100" height="22" rx="8" fill="#475569"/>
    <!-- Fluted body -->
    <path d="M 218 160 L 294 160 L 294 316 L 256 356 L 218 316 Z" fill="url(#steel)"/>
    <!-- Helical flute cutouts -->
    <path d="M 218 192 L 294 168 L 294 196 L 218 220 Z" fill="#1E293B"/>
    <path d="M 218 248 L 294 224 L 294 252 L 218 276 Z" fill="#1E293B"/>
    <path d="M 218 304 L 294 280 L 294 308 L 256 322 L 218 316 Z" fill="#1E293B"/>
  </g>
</svg>
"""


def clamp(v: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return lo if v < lo else (hi if v > hi else v)


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def mix_rgba(
    dst: tuple[float, float, float, float],
    src: tuple[float, float, float, float],
    alpha: float,
) -> tuple[float, float, float, float]:
    a = clamp(alpha) * src[3]
    if a <= 0.0:
        return dst
    out_a = a + dst[3] * (1.0 - a)
    if out_a <= 1e-6:
        return (0.0, 0.0, 0.0, 0.0)
    inv = 1.0 / out_a
    return (
        (src[0] * a + dst[0] * dst[3] * (1.0 - a)) * inv,
        (src[1] * a + dst[1] * dst[3] * (1.0 - a)) * inv,
        (src[2] * a + dst[2] * dst[3] * (1.0 - a)) * inv,
        out_a,
    )


def sdf_rounded_box(px: float, py: float, cx: float, cy: float, hx: float, hy: float, r: float) -> float:
    qx = abs(px - cx) - hx + r
    qy = abs(py - cy) - hy + r
    return math.hypot(max(qx, 0.0), max(qy, 0.0)) + min(max(qx, qy), 0.0) - r


def sdf_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    pax, pay = px - ax, py - ay
    bax, bay = bx - ax, by - ay
    denom = bax * bax + bay * bay
    h = clamp((pax * bax + pay * bay) / denom) if denom > 1e-9 else 0.0
    return math.hypot(pax - bax * h, pay - bay * h)


def sdf_ellipse_arc(
    px: float,
    py: float,
    cx: float,
    cy: float,
    rx: float,
    ry: float,
    a0: float,
    a1: float,
    samples: int = 48,
) -> float:
    best = 1e9
    prev_x = cx + rx * math.cos(a0)
    prev_y = cy + ry * math.sin(a0)
    for i in range(1, samples + 1):
        t = a0 + (a1 - a0) * (i / samples)
        nx = cx + rx * math.cos(t)
        ny = cy + ry * math.sin(t)
        d = sdf_segment(px, py, prev_x, prev_y, nx, ny)
        if d < best:
            best = d
        prev_x, prev_y = nx, ny
    return best


def sdf_circle_arc(
    px: float, py: float, cx: float, cy: float, radius: float, a0: float, a1: float
) -> float:
    dx, dy = px - cx, py - cy
    ang = math.atan2(dy, dx)
    # Normalize angle into [a0, a0 + 2*pi)
    while ang < a0:
        ang += 2.0 * math.pi
    while ang >= a0 + 2.0 * math.pi:
        ang -= 2.0 * math.pi
    if ang <= a1:
        return abs(math.hypot(dx, dy) - radius)
    x0, y0 = cx + radius * math.cos(a0), cy + radius * math.sin(a0)
    x1, y1 = cx + radius * math.cos(a1), cy + radius * math.sin(a1)
    return min(math.hypot(px - x0, py - y0), math.hypot(px - x1, py - y1))


def steel_gradient(px: float) -> tuple[float, float, float, float]:
    t = clamp((px - 216.0) / 80.0)
    if t < 0.35:
        u = t / 0.35
        return (lerp(0.58, 0.95, u), lerp(0.64, 0.96, u), lerp(0.72, 0.98, u), 1.0)
    if t < 0.70:
        u = (t - 0.35) / 0.35
        return (lerp(0.95, 0.80, u), lerp(0.96, 0.84, u), lerp(0.98, 0.88, u), 1.0)
    u = (t - 0.70) / 0.30
    return (lerp(0.80, 0.39, u), lerp(0.84, 0.45, u), lerp(0.88, 0.55, u), 1.0)


def sample_icon(x: float, y: float, aa: float) -> tuple[float, float, float, float]:
    """Evaluate icon RGBA at canonical 512x512 coordinates (x, y)."""
    col = (0.0, 0.0, 0.0, 0.0)

    # 1. Rounded square badge background + rim
    d_badge = sdf_rounded_box(x, y, 256.0, 256.0, 232.0, 232.0, 108.0)
    if d_badge < aa * 1.5:
        t_y = clamp((y - 24.0) / 464.0)
        bg = (lerp(0.12, 0.067, t_y), lerp(0.153, 0.090, t_y), lerp(0.220, 0.133, t_y), 1.0)
        col = mix_rgba(col, bg, clamp(0.5 - d_badge / aa))

        d_rim = abs(d_badge) - 5.0
        if d_rim < aa:
            t_d = clamp((x + y - 48.0) / 928.0)
            rim = (lerp(0.29, 0.13, t_d), lerp(0.36, 0.17, t_d), lerp(0.48, 0.24, t_d), 1.0)
            col = mix_rgba(col, rim, clamp(0.5 - d_rim / aa))

    if d_badge > aa:
        return col

    # 2. Machined cyan contour arcs at base
    d_cyan1 = sdf_ellipse_arc(x, y, 256.0, 366.0, 146.0, 56.0, 0.12, math.pi - 0.12) - 10.0
    if d_cyan1 < aa:
        t_x = clamp((x - 110.0) / 292.0)
        cyan = (lerp(0.055, 0.22, 1.0 - abs(2.0 * t_x - 1.0)), lerp(0.65, 0.74, 1.0 - abs(2.0 * t_x - 1.0)), 0.96, 0.92)
        col = mix_rgba(col, cyan, clamp(0.5 - d_cyan1 / aa))

    d_cyan2 = sdf_ellipse_arc(x, y, 256.0, 392.0, 106.0, 36.0, 0.18, math.pi - 0.18) - 5.0
    if d_cyan2 < aa:
        col = mix_rgba(col, (0.22, 0.74, 0.97, 0.45), clamp(0.5 - d_cyan2 / aa))

    # 3. Dynamic amber-orange toolpath / chip curl sweep (from ~165 deg around bottom to ~-18 deg)
    # In screen coords (+Y down), 0 is right (+X), pi/2 is down (+Y), pi is left (-X)
    a_start = math.radians(-18.0)
    a_end = math.radians(176.0)
    d_amber = sdf_circle_arc(x, y, 256.0, 268.0, 126.0, a_start, a_end) - 13.5
    if d_amber < aa:
        t_am = clamp((x - 120.0) / 260.0)
        amber = (lerp(0.92, 1.0, t_am), lerp(0.36, 0.72, t_am), lerp(0.05, 0.30, t_am), 1.0)
        col = mix_rgba(col, amber, clamp(0.5 - d_amber / aa))

    # Flying chip sparks
    d_sp1 = math.hypot(x - 386.0, y - 186.0) - 14.0
    if d_sp1 < aa:
        col = mix_rgba(col, (1.0, 0.72, 0.30, 1.0), clamp(0.5 - d_sp1 / aa))
    d_sp2 = math.hypot(x - 412.0, y - 150.0) - 8.5
    if d_sp2 < aa:
        col = mix_rgba(col, (0.99, 0.90, 0.54, 1.0), clamp(0.5 - d_sp2 / aa))

    # 4. CNC Endmill Cutter (centered at x = 256)
    # 4a. Shank
    d_shank = sdf_rounded_box(x, y, 256.0, 112.0, 40.0, 36.0, 12.0)
    if d_shank < aa:
        col = mix_rgba(col, steel_gradient(x), clamp(0.5 - d_shank / aa))

    # 4b. Fluted cutter body (x in [218, 294], y in [160, 316], tapered V-tip to (256, 356))
    dx_body = abs(x - 256.0) - 38.0
    d_top = 160.0 - y
    # Two diagonal tip planes from (218,316)->(256,356) and (294,316)->(256,356)
    # Unit normal pointing outward down-left/down-right: (40, 38) / hypot(40,38)
    inv_tip = 1.0 / math.hypot(40.0, 38.0)
    d_tip = ((abs(x - 256.0) * 40.0 + (y - 316.0) * 38.0) - 38.0 * 40.0) * inv_tip
    d_body = max(dx_body, d_top, d_tip)
    if d_body < aa:
        body_col = steel_gradient(x)
        # Helical flute bands: slanted stripes y + 0.315*(x - 256) in specific intervals
        s = y + 0.315 * (x - 256.0)
        flute_hit = 0.0
        for lo, hi in ((180.0, 206.0), (236.0, 262.0), (292.0, 318.0)):
            d_band = max(lo - s, s - hi)
            if d_band < aa:
                flute_hit = max(flute_hit, clamp(0.5 - d_band / aa))
        if flute_hit > 0.0:
            flute_col = (0.11, 0.16, 0.23, 1.0)
            body_col = (
                lerp(body_col[0], flute_col[0], flute_hit),
                lerp(body_col[1], flute_col[1], flute_hit),
                lerp(body_col[2], flute_col[2], flute_hit),
                1.0,
            )
        col = mix_rgba(col, body_col, clamp(0.5 - d_body / aa))

    # 4c. Tool holder collar
    d_collar = sdf_rounded_box(x, y, 256.0, 149.0, 50.0, 11.0, 7.0)
    if d_collar < aa:
        t_c = clamp((x - 206.0) / 100.0)
        collar = (lerp(0.28, 0.42, 1.0 - abs(2.0 * t_c - 1.0)), lerp(0.34, 0.48, 1.0 - abs(2.0 * t_c - 1.0)), lerp(0.43, 0.58, 1.0 - abs(2.0 * t_c - 1.0)), 1.0)
        col = mix_rgba(col, collar, clamp(0.5 - d_collar / aa))

    return col


def render_rgba(size: int) -> bytes:
    """Render size x size RGBA8 image with 2x2 supersampling."""
    scale = 512.0 / float(size)
    aa = max(1.0, scale * 1.15)
    out = bytearray(size * size * 4)
    offsets = (0.25, 0.75)
    idx = 0
    for py in range(size):
        for px in range(size):
            r = g = b = a = 0.0
            for oy in offsets:
                sy = (py + oy) * scale
                for ox in offsets:
                    sx = (px + ox) * scale
                    sr, sg, sb, sa = sample_icon(sx, sy, aa)
                    r += sr * sa
                    g += sg * sa
                    b += sb * sa
                    a += sa
            a *= 0.25
            if a > 1e-6:
                inv_a = 0.25 / a
                r = clamp(r * inv_a)
                g = clamp(g * inv_a)
                b = clamp(b * inv_a)
            else:
                r = g = b = 0.0
            out[idx] = int(round(r * 255.0))
            out[idx + 1] = int(round(g * 255.0))
            out[idx + 2] = int(round(b * 255.0))
            out[idx + 3] = int(round(clamp(a) * 255.0))
            idx += 4
    return bytes(out)


def encode_png(size: int, rgba: bytes) -> bytes:
    def chunk(tag: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

    stride = size * 4
    raw = bytearray()
    for row in range(size):
        raw.append(0)  # filter type 0 (None)
        start = row * stride
        raw.extend(rgba[start : start + stride])

    ihdr = struct.pack(">IIBBBBB", size, size, 8, 6, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", zlib.compress(bytes(raw), level=9))
        + chunk(b"IEND", b"")
    )


def encode_ico_bmp_frame(size: int, rgba: bytes) -> bytes:
    """Encode a 32-bit BGRA bottom-up DIB + 1-bit AND mask for Windows ICO."""
    # BITMAPINFOHEADER (40 bytes): biHeight is 2 * size (XOR + AND masks)
    header = struct.pack(
        "<IiiHHIIiiII",
        40,
        size,
        size * 2,
        1,
        32,
        0,
        size * size * 4,
        0,
        0,
        0,
        0,
    )
    xor_buf = bytearray(size * size * 4)
    dst = 0
    for row in range(size - 1, -1, -1):
        base = row * size * 4
        for col in range(size):
            p = base + col * 4
            xor_buf[dst] = rgba[p + 2]      # B
            xor_buf[dst + 1] = rgba[p + 1]  # G
            xor_buf[dst + 2] = rgba[p]      # R
            xor_buf[dst + 3] = rgba[p + 3]  # A
            dst += 4

    # 1-bit AND mask rows padded to 32-bit boundaries (all zeros; alpha channel controls transparency)
    and_row_bytes = ((size + 31) // 32) * 4
    and_mask = bytes(and_row_bytes * size)
    return header + bytes(xor_buf) + and_mask


def encode_ico(frames: list[tuple[int, bytes]]) -> bytes:
    """Pack multiple icon resolutions into a single Windows .ico file."""
    count = len(frames)
    ico_header = struct.pack("<HHH", 0, 1, count)
    entries = bytearray()
    payloads: list[bytes] = []
    offset = 6 + count * 16

    for size, rgba in frames:
        if size >= 256:
            data = encode_png(size, rgba)
        else:
            data = encode_ico_bmp_frame(size, rgba)
        dim_byte = 0 if size >= 256 else size
        entries.extend(
            struct.pack(
                "<BBBBHHII",
                dim_byte,
                dim_byte,
                0,
                0,
                1,
                32,
                len(data),
                offset,
            )
        )
        payloads.append(data)
        offset += len(data)

    return ico_header + bytes(entries) + b"".join(payloads)


def main() -> None:
    icons_dir = REPO_ROOT / "packaging" / "icons"
    win_dir = REPO_ROOT / "packaging" / "windows"
    icons_dir.mkdir(parents=True, exist_ok=True)
    win_dir.mkdir(parents=True, exist_ok=True)

    svg_path = icons_dir / "chipcutter.svg"
    svg_path.write_text(SVG_SOURCE, encoding="utf-8")
    print(f"Wrote {svg_path.relative_to(REPO_ROOT)}")

    rendered: dict[int, bytes] = {}
    for size in (16, 32, 48, 64, 128, 256, 512):
        rendered[size] = render_rgba(size)

    for size in (64, 128, 256, 512):
        out_dir = icons_dir / "hicolor" / f"{size}x{size}" / "apps"
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / "chipcutter.png"
        out_path.write_bytes(encode_png(size, rendered[size]))
        print(f"Wrote {out_path.relative_to(REPO_ROOT)} ({out_path.stat().st_size} bytes)")

    embedded_256 = icons_dir / "chipcutter-256.png"
    embedded_256.write_bytes(encode_png(256, rendered[256]))
    print(f"Wrote {embedded_256.relative_to(REPO_ROOT)} ({embedded_256.stat().st_size} bytes)")

    ico_frames = [(s, rendered[s]) for s in (16, 32, 48, 64, 128, 256)]
    ico_path = win_dir / "chipcutter.ico"
    ico_path.write_bytes(encode_ico(ico_frames))
    print(f"Wrote {ico_path.relative_to(REPO_ROOT)} ({ico_path.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
