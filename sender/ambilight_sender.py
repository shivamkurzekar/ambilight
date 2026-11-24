"""
Ambilight sender - screen capture -> edge sampling -> UDP multicast

What it does:
- Captures the full monitor frame once per cycle (MSS) and slices bands/center.
- Computes LED colors from bands, applies center blend, blur, tuning, smoothing.
- Sends UDP multicast packets to the LED controller.
"""

import socket, struct, time, ctypes
from ctypes import wintypes
import numpy as np
import mss

# ============================== CONFIG ==============================
# Network settings live in config_local.py (gitignored).
# Copy config_local.example.py -> config_local.py and edit it.
try:
    import config_local as _local
except ImportError:
    _local = None
    print("[config] config_local.py not found - using defaults "
          "(copy config_local.example.py to config_local.py)")

def _cfg(name, default):
    return getattr(_local, name, default)

MCAST_GRP = _cfg("MCAST_GRP", "239.1.1.1")
PORT = _cfg("PORT", 7777)
LED_COUNT = 107

TARGET_FPS = 30
TIMING_EVERY_SEQ = 5
LOG_EVERY = 30

# Monitor selection policy:
#   "external_largest" -> largest NON-primary (usually external in extend)
#   "primary"          -> primary monitor
#   "largest"          -> largest monitor regardless of primary
MONITOR_PICK_MODE = "external_largest"

# Sampling
DOWNSCALE = 4                 # applied via slicing [::DOWNSCALE]
BAND_THICKNESS_PX = 60        # thickness in FINAL (downscaled) pixels
BOTTOM_CENTER_SKIP_PX = 200   # skip center chunk on bottom edge (FINAL pixels)
BOTTOM_SAMPLE_Y_OFFSET_PX = 60  # lift bottom band upward (FINAL pixels)

# Pixel masks
DARK_THRESH = 10
IGNORE_NEAR_WHITE = True
WHITE_BRIGHT_THR = 250
WHITE_CHROMA_THR = 10

# Center blend
ENABLE_CENTER_BLEND = False
EDGE_WEIGHT = 0.95
CENTER_WEIGHT = 0.50
CENTER_CROP_X = 0.60
CENTER_CROP_Y = 0.55
CENTER_EXCLUDE_BOTTOM = 0.18

# Center extra downscale (performance win; usually negligible visually)
CENTER_DOWNSCALE = 8
# Compute center every N frames (1 = every frame). Reuses last value between updates.
CENTER_EVERY_N = 2

# Color tuning
BRIGHTNESS_KNOB = 0.2
SAT_BOOST = 1.15
WARMTH = 0.10
CHANNEL_CLIP = 255

# Temporal smoothing
SMOOTH_ALPHA = 0.08

# Spatial blur
SPATIAL_BLUR_RADIUS = 0
SPATIAL_BLUR_PASSES = 1

# DEBUG = True
DEBUG = False
# ====================================================================

# LED layout
N_BOTTOM_LEFT  = 16
N_LEFT         = 20
N_TOP          = 35
N_RIGHT        = 20
N_BOTTOM_RIGHT = 16
assert N_BOTTOM_LEFT + N_LEFT + N_TOP + N_RIGHT + N_BOTTOM_RIGHT == LED_COUNT

MAGIC = 0xA1B2
VERSION = 2

# -------------------- misc helpers --------------------
def dbg(*a):
    if DEBUG:
        print(*a)

def now_us32() -> int:
    return (time.perf_counter_ns() // 1000) & 0xFFFFFFFF

def pack_frame_v2(seq, rgb_u8):
    t_send_us = now_us32()
    hdr = struct.pack("<HBBHI", MAGIC, VERSION, LED_COUNT, seq & 0xFFFF, t_send_us)
    return hdr + rgb_u8.tobytes(order="C"), t_send_us

# -------------------- Windows monitor enumeration (ctypes) --------------------
class RECT(ctypes.Structure):
    _fields_ = [("left", wintypes.LONG),
                ("top", wintypes.LONG),
                ("right", wintypes.LONG),
                ("bottom", wintypes.LONG)]

class MONITORINFO(ctypes.Structure):
    _fields_ = [("cbSize", wintypes.DWORD),
                ("rcMonitor", RECT),
                ("rcWork", RECT),
                ("dwFlags", wintypes.DWORD)]

MONITORINFOF_PRIMARY = 0x00000001
user32 = ctypes.windll.user32

def list_monitors():
    monitors = []

    @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HMONITOR, wintypes.HDC, ctypes.POINTER(RECT), wintypes.LPARAM)
    def enum_proc(hMon, hdc, lprc, lparam):
        mi = MONITORINFO()
        mi.cbSize = ctypes.sizeof(MONITORINFO)
        user32.GetMonitorInfoW(hMon, ctypes.byref(mi))
        r = mi.rcMonitor
        monitors.append({
            "hmon": hMon,
            "left": int(r.left),
            "top": int(r.top),
            "right": int(r.right),
            "bottom": int(r.bottom),
            "primary": bool(mi.dwFlags & MONITORINFOF_PRIMARY),
        })
        return True

    user32.EnumDisplayMonitors(0, 0, enum_proc, 0)
    return monitors

def pick_target_monitor(mode="external_largest"):
    mons = list_monitors()
    if not mons:
        return None

    def area(m):
        return max(1, (m["right"] - m["left"])) * max(1, (m["bottom"] - m["top"]))

    if len(mons) == 1:
        return mons[0]

    if mode == "primary":
        prim = [m for m in mons if m["primary"]]
        return prim[0] if prim else max(mons, key=area)

    if mode == "external_largest":
        non_primary = [m for m in mons if not m["primary"]]
        if non_primary:
            return max(non_primary, key=area)
        return max(mons, key=area)

    return max(mons, key=area)

# -------------------- pooling helpers --------------------
def split_edges(length, n):
    e = np.linspace(0, length, n + 1).astype(np.int32)
    for i in range(n):
        if e[i + 1] <= e[i]:
            e[i + 1] = e[i] + 1
    if e[-1] != length:
        e[-1] = length
    return e

def precompute_rgb_weight(rgb_u8):
    rgb_f = rgb_u8.astype(np.float32, copy=False)
    mx = rgb_f.max(axis=2)
    mn = rgb_f.min(axis=2)
    chroma = mx - mn

    valid = mx >= DARK_THRESH
    if IGNORE_NEAR_WHITE:
        near_white = (mx >= WHITE_BRIGHT_THR) & (chroma <= WHITE_CHROMA_THR)
        valid &= ~near_white

    w = np.maximum(chroma, 1.0)
    w *= valid.astype(np.float32, copy=False)
    return rgb_f, w

def pool_bins_horizontal(rgb_f_band, w_band, edges):
    col_w = w_band.sum(axis=0)
    col_rgbw = (rgb_f_band * w_band[..., None]).sum(axis=0)

    starts = edges[:-1]
    sum_rgb = np.add.reduceat(col_rgbw, starts, axis=0)
    sum_w = np.add.reduceat(col_w, starts, axis=0)

    sum_w = np.maximum(sum_w, 1.0)[:, None]
    return sum_rgb / sum_w

def pool_bins_vertical(rgb_f_band, w_band, edges):
    row_w = w_band.sum(axis=1)
    row_rgbw = (rgb_f_band * w_band[..., None]).sum(axis=1)

    starts = edges[:-1]
    sum_rgb = np.add.reduceat(row_rgbw, starts, axis=0)
    sum_w = np.add.reduceat(row_w, starts, axis=0)

    sum_w = np.maximum(sum_w, 1.0)[:, None]
    return sum_rgb / sum_w

def compute_center_weighted(center_rgb_u8):
    rgb_f, w = precompute_rgb_weight(center_rgb_u8)
    sw = float(w.sum())
    if sw <= 0.0:
        return rgb_f.reshape(-1, 3).mean(axis=0, dtype=np.float32)
    return (rgb_f * w[..., None]).sum(axis=(0, 1)) / sw

# -------------------- color helpers --------------------
def chroma_boost(rgb01, sat):
    if abs(sat - 1.0) < 1e-6:
        return rgb01
    gray = (0.299 * rgb01[..., 0] + 0.587 * rgb01[..., 1] + 0.114 * rgb01[..., 2])[..., None]
    return np.clip(gray + (rgb01 - gray) * sat, 0.0, 1.0)

def apply_warmth(rgb01, warmth):
    if abs(warmth) < 1e-6:
        return rgb01
    out = rgb01.copy()
    out[..., 0] *= (1.0 + 0.60 * warmth)
    out[..., 1] *= (1.0 + 0.10 * warmth)
    out[..., 2] *= (1.0 - 0.60 * warmth)
    return np.clip(out, 0.0, 1.0)

# -------------------- spatial blur --------------------
def spatial_blur_segment(arr, start, count):
    if SPATIAL_BLUR_RADIUS <= 0 or count < 3:
        return

    if SPATIAL_BLUR_RADIUS == 1:
        w = np.array([1, 2, 1], np.float32); r = 1
    elif SPATIAL_BLUR_RADIUS == 2:
        w = np.array([1, 2, 3, 2, 1], np.float32); r = 2
    else:
        w = np.array([1, 2, 3, 4, 3, 2, 1], np.float32); r = 3
    w /= w.sum()

    seg = arr[start:start + count].copy()
    for _ in range(SPATIAL_BLUR_PASSES):
        tmp = seg.copy()
        for i in range(count):
            acc = np.zeros(3, np.float32)
            for k, wk in enumerate(w):
                j = min(max(i + k - r, 0), count - 1)
                acc += tmp[j] * wk
            seg[i] = acc
    arr[start:start + count] = seg

def apply_spatial_blur_per_segment(out):
    s = 0
    spatial_blur_segment(out, s, N_BOTTOM_LEFT);  s += N_BOTTOM_LEFT
    spatial_blur_segment(out, s, N_LEFT);         s += N_LEFT
    spatial_blur_segment(out, s, N_TOP);          s += N_TOP
    spatial_blur_segment(out, s, N_RIGHT);        s += N_RIGHT
    spatial_blur_segment(out, s, N_BOTTOM_RIGHT)

# ------------------- MSS capture helpers -------------------
def mss_grab_bgra(sct: mss.mss, rect: dict):
    return sct.grab(rect)

# ------------------- main -------------------
def main():
    # pick monitor (virtual coords, can be negative)
    m = pick_target_monitor(MONITOR_PICK_MODE)
    if m is None:
        raise RuntimeError("Could not enumerate monitors via Win32 API.")

    mon_left, mon_top, mon_right, mon_bottom = m["left"], m["top"], m["right"], m["bottom"]
    mon_w = max(1, mon_right - mon_left)
    mon_h = max(1, mon_bottom - mon_top)

    dbg(f"[strip] target monitor: primary={m['primary']} virtual=({mon_left},{mon_top},{mon_right},{mon_bottom}) size={mon_w}x{mon_h}")

    # convert "FINAL pixel sizes" -> capture pixel sizes BEFORE downscale
    ds = max(1, int(DOWNSCALE))
    band_px = max(2, int(BAND_THICKNESS_PX) * ds)
    yoff_px = max(0, int(BOTTOM_SAMPLE_Y_OFFSET_PX) * ds)
    skip_px = max(0, int(BOTTOM_CENTER_SKIP_PX) * ds)

    # Derived effective dims after downscale
    W_eff = mon_w // ds
    H_eff = mon_h // ds

    band_eff = max(2, BAND_THICKNESS_PX)
    cx_eff = W_eff // 2
    skip_eff = (skip_px // ds) if ds > 0 else 0
    yoff_eff = (yoff_px // ds) if ds > 0 else 0

    # Precompute LED bin edges in EFFECTIVE coords
    edges_top = split_edges(W_eff, N_TOP)
    edges_left = split_edges(H_eff, N_LEFT)
    edges_right = split_edges(H_eff, N_RIGHT)
    edges_bl = split_edges(max(1, cx_eff - skip_eff), N_BOTTOM_LEFT)
    edges_br = split_edges(max(1, W_eff - (cx_eff + skip_eff)), N_BOTTOM_RIGHT)

    dbg(f"[strip] sampling(eff)={W_eff}x{H_eff} band_eff={band_eff} skip_eff={skip_eff} yoff_eff={yoff_eff} hdr=v{VERSION}")

    # UDP sockets
    sock_strip = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock_strip.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)

    # Buffers
    out = np.empty((LED_COUNT, 3), np.float32)
    prev_strip = np.zeros((LED_COUNT, 3), np.float32)
    center_color = None

    seq = 0
    fi = 0
    dt = 1.0 / TARGET_FPS if TARGET_FPS > 0 else 0.0

    mon = {"left": mon_left, "top": mon_top, "width": mon_w, "height": mon_h}

    try:
        with mss.mss() as sct:
            while True:
                t0 = time.perf_counter_ns()

                # ---------------- CAPTURE (single full grab) ----------------
                t_cap0 = time.perf_counter_ns()
                shot = mss_grab_bgra(sct, mon)
                img = np.frombuffer(shot.raw, dtype=np.uint8).reshape((shot.height, shot.width, 4))
                t_cap1 = time.perf_counter_ns()

                bgr = img[..., :3]
                rgb_full = bgr[..., ::-1]
                t_cap = t_cap1

                # ---------------- DOWNSCALE (cheap slicing) ----------------
                if ds > 1:
                    rgb = rgb_full[::ds, ::ds, :]
                else:
                    rgb = rgb_full
                t_ds = time.perf_counter_ns()

                # ---------------- SLICE bands from full frame ----------------
                H_now, W_now = rgb.shape[:2]
                y1 = max(1, H_now - yoff_eff)
                y0 = max(0, y1 - band_eff)
                if y1 <= y0:
                    y0 = max(0, H_now - band_eff)
                    y1 = H_now

                top_rgb = rgb[0:band_eff, :, :]
                left_rgb = rgb[:, 0:band_eff, :]
                right_rgb = rgb[:, max(0, W_now - band_eff):W_now, :]
                bottom_rgb = rgb[y0:y1, :, :]
                t_slice = time.perf_counter_ns()

                # ---------------- PRECOMPUTE (bands only) ----------------
                top_f, top_w = precompute_rgb_weight(top_rgb)
                left_f, left_w = precompute_rgb_weight(left_rgb)
                right_f, right_w = precompute_rgb_weight(right_rgb)
                bot_f, bot_w = precompute_rgb_weight(bottom_rgb)
                t_pre = time.perf_counter_ns()

                if (W_now != W_eff) or (H_now != H_eff):
                    W_eff = W_now
                    H_eff = H_now
                    cx_eff = W_eff // 2
                    edges_top = split_edges(W_eff, N_TOP)
                    edges_left = split_edges(H_eff, N_LEFT)
                    edges_right = split_edges(H_eff, N_RIGHT)
                    edges_bl = split_edges(max(1, cx_eff - skip_eff), N_BOTTOM_LEFT)
                    edges_br = split_edges(max(1, W_eff - (cx_eff + skip_eff)), N_BOTTOM_RIGHT)

                top_bins_rgb = pool_bins_horizontal(top_f, top_w, edges_top)
                left_bins_rgb = pool_bins_vertical(left_f, left_w, edges_left)
                right_bins_rgb = pool_bins_vertical(right_f, right_w, edges_right)

                bl_w_end = max(1, cx_eff - skip_eff)
                br_x0 = min(W_eff - 1, cx_eff + skip_eff)

                bl_bins_rgb = pool_bins_horizontal(bot_f[:, :bl_w_end, :], bot_w[:, :bl_w_end], edges_bl)
                br_bins_rgb = pool_bins_horizontal(bot_f[:, br_x0:, :], bot_w[:, br_x0:], edges_br)
                t_pool = time.perf_counter_ns()

                # ---------------- ASSEMBLE ----------------
                i = 0
                out[i:i + N_BOTTOM_LEFT] = bl_bins_rgb[::-1]; i += N_BOTTOM_LEFT
                out[i:i + N_LEFT] = left_bins_rgb[::-1]; i += N_LEFT
                out[i:i + N_TOP] = top_bins_rgb; i += N_TOP
                out[i:i + N_RIGHT] = right_bins_rgb; i += N_RIGHT
                out[i:i + N_BOTTOM_RIGHT] = br_bins_rgb[::-1]
                t_asm = time.perf_counter_ns()

                # ---------------- CENTER (blend and/or wiz) ----------------
                need_center = (ENABLE_CENTER_BLEND and CENTER_WEIGHT > 0.0)
                if need_center and (CENTER_EVERY_N <= 1 or (fi % CENTER_EVERY_N) == 0 or center_color is None):
                    cw = max(2, int(W_now * CENTER_CROP_X))
                    ch = max(2, int(H_now * CENTER_CROP_Y))
                    x0c = (W_now - cw) // 2
                    x1c = x0c + cw
                    usable_h = max(2, H_now - int(H_now * CENTER_EXCLUDE_BOTTOM))
                    y0c = max(0, (usable_h - ch) // 2)
                    y1c = min(usable_h, y0c + ch)

                    center_rgb = rgb[y0c:y1c, x0c:x1c, :]
                    if CENTER_DOWNSCALE > 1:
                        center_rgb = center_rgb[::CENTER_DOWNSCALE, ::CENTER_DOWNSCALE, :]

                    center_color = compute_center_weighted(center_rgb)

                if need_center and (center_color is not None) and ENABLE_CENTER_BLEND and CENTER_WEIGHT > 0.0:
                    out[:] = (EDGE_WEIGHT * out) + (CENTER_WEIGHT * center_color[None, :])
                t_ctr = time.perf_counter_ns()

                # ---------------- BLUR ----------------
                apply_spatial_blur_per_segment(out)
                t_blur = time.perf_counter_ns()

                # ---------------- TUNE ----------------
                rgb01 = np.clip(out / 255.0, 0.0, 1.0)
                rgb01 = np.clip(rgb01 * BRIGHTNESS_KNOB, 0.0, 1.0)
                rgb01 = chroma_boost(rgb01, SAT_BOOST)
                rgb01 = apply_warmth(rgb01, WARMTH)
                out[:] = rgb01 * 255.0
                if CHANNEL_CLIP < 255:
                    np.minimum(out, float(CHANNEL_CLIP), out=out)
                t_tune = time.perf_counter_ns()

                # ---------------- SMOOTH ----------------
                if SMOOTH_ALPHA > 0:
                    prev_strip[:] = (1.0 - SMOOTH_ALPHA) * prev_strip + SMOOTH_ALPHA * out
                    out_s = prev_strip
                else:
                    out_s = out

                rgb_u8 = np.clip(out_s + 0.5, 0, 255).astype(np.uint8)
                t_sm = time.perf_counter_ns()

                # ---------------- SEND STRIP ----------------
                pkt, t_send_us = pack_frame_v2(seq, rgb_u8)
                sock_strip.sendto(pkt, (MCAST_GRP, PORT))
                t_sent = time.perf_counter_ns()

                # ---------------- LOGS ----------------
                if DEBUG and (seq % TIMING_EVERY_SEQ == 0):
                    loop_ms = (t_sent - t0) / 1e6
                    cap_ms  = (t_cap1 - t_cap0) / 1e6
                    ds_ms   = (t_ds - t_cap1) / 1e6
                    slice_ms = (t_slice - t_ds) / 1e6
                    pre_ms  = (t_pre - t_slice) / 1e6
                    pool_ms = (t_pool - t_pre) / 1e6
                    asm_ms  = (t_asm - t_pool) / 1e6
                    ctr_ms  = (t_ctr - t_asm) / 1e6
                    blur_ms = (t_blur - t_ctr) / 1e6
                    tune_ms = (t_tune - t_blur) / 1e6
                    sm_ms   = (t_sm - t_tune) / 1e6
                    send_ms = (t_sent - t_sm) / 1e6

                    dbg(f"[timing] seq={seq:05d} t_send_us={t_send_us:10d} loop={loop_ms:7.2f}ms")
                    dbg(f"[step]  cap={cap_ms:6.2f} ds={ds_ms:6.2f} slice={slice_ms:6.2f} | "
                        f"pre={pre_ms:6.2f} pool={pool_ms:6.2f} asm={asm_ms:6.2f} ctr={ctr_ms:6.2f} | "
                        f"blur={blur_ms:6.2f} tune={tune_ms:6.2f} smth={sm_ms:6.2f} send={send_ms:6.2f}")

                if DEBUG and (fi % LOG_EVERY == 0):
                    led0 = tuple(int(x) for x in rgb_u8[0])
                    top_mid = tuple(int(x) for x in rgb_u8[N_BOTTOM_LEFT + N_LEFT + (N_TOP // 2)])
                    last = tuple(int(x) for x in rgb_u8[-1])
                    dbg(f"[strip] f={fi:06d} seq={seq:05d} led0={led0} topMid={top_mid} last={last}")

                seq = (seq + 1) & 0xFFFF
                fi += 1

                # FPS cap
                if dt > 0:
                    sleep_s = dt - (time.perf_counter_ns() - t0) / 1e9
                    if sleep_s > 0:
                        time.sleep(sleep_s)

    except KeyboardInterrupt:
        dbg("\n[main] Stopped")

if __name__ == "__main__":
    main()
