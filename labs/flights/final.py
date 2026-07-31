import drone_core
import drone_utils as uav_utils
import cv2
import numpy as np
import time


# -- Course setup: makes the shared `neo_lab` helper importable.
#    You don't need to read or change this block. --
import os as _os, sys as _sys
_d = _os.path.dirname(_os.path.realpath(__file__))
while _os.path.basename(_d) != "labs" and _os.path.dirname(_d) != _d:
    _d = _os.path.dirname(_d)
if _d not in _sys.path:
    _sys.path.insert(0, _d)
import neo_lab

# -- Constants --------------------------------------------------------------
V_MIN         = 220
MIN_PIXELS    = 200
FORWARD_PITCH = 0.3          # forward stick on a straight
MAX_ROLL      = 0.5
FOLLOW_TIME   = 345.0
IMAGE_CENTER  = 320
YAW_GAIN = 1
MAX_YAW = 1

KP   = 0.5
KI   = 0
KD   = 0.1
imax = 1.0
MAX_LAT = 0.15

PITCH_TURN      = 0.1
SLOPE_FULL_SLOW = 0.35
ARUCO_DICT    = cv2.aruco.DICT_5X5_100
GATE_IDS      = None
ROW_CENTER    = 240
KP_V          = 0.4
KD_V          = 0.15
MAX_THROTTLE  = 0.15
ALT_EVERY     = 2
ALT_COAST     = 6
_timer = 0.0
_done  = False
slopef = 0.0
_frame = 0
xm   = 0.25
mfill   = 0.25
mresid  = 150.0
intg = 0.0
pe   = 0.0
df = 0.0
alt_center = None
alt_radius = None
alt_offsets = {}
alt_pe = 0.0
alt_df = 0.0
alt_lost = 999
alt_thr = 0.0
alt_dy = None
alt_ntags = 0


_adet = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT),
                                cv2.aruco.DetectorParameters())
def _detect_tags(gray):
    return _adet.detectMarkers(gray)
"""
def _detect_tags(gray):
    p = cv2.aruco.DetectorParameters_create()
    return cv2.aruco.detectMarkers(gray, cv2.aruco.getPredefinedDictionary(ARUCO_DICT),
                                   parameters=p)
"""


def detect_line(image, v_min=200):
    mask = neo_lab.bright_mask(image, v_min)
    h, w = mask.shape
    xl = int(w * xm)
    xr = int(w * (1-xm))
    r = mask[:h//2, xl:xr]

    rows,cols = np.nonzero(r)
    n = cols.size
    band = h//2 * (xr - xl)
    if n < MIN_PIXELS or n > mfill * band:
        return {"ok": False, "n": n}

    cols = cols + xl
    m, b = np.polyfit(rows, cols, 1)
    resid = float(np.std(cols - (m * rows + b)))

    if resid > mresid:
        return {"ok": False, "n": n}

    col_at_drone = m * (h//2 - 1) + b
    lateral = -(col_at_drone  - w/2 ) / (w/2)
    return {
        "ok": True,
        "slope": float(m),
        "angle": float(np.degrees(np.arctan(m))),
        "lateral": float(lateral),
        "resid": resid,
        "n": n,
    }


def find_tags(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = _detect_tags(gray)
    tags = []
    if ids is None:
        return tags
    for quad, tid in zip(corners, ids.flatten()):
        tid = int(tid)
        if GATE_IDS is not None and tid not in GATE_IDS:
            continue
        p = quad.reshape(-1, 2)
        tags.append((tid, (float(p[:, 0].mean()), float(p[:, 1].mean())),
                     float(np.linalg.norm(p[0] - p[2]) / 1.414)))
    return tags


def fit_circle(pts):
    if len(pts) < 3:
        return None
    x, y = pts[:, 0], pts[:, 1]
    A = np.column_stack([2 * x, 2 * y, np.ones(len(x))])
    try:
        sol, *_ = np.linalg.lstsq(A, x * x + y * y, rcond=None)
    except np.linalg.LinAlgError:
        return None
    a, b, c = sol
    r2 = c + a * a + b * b
    if not np.isfinite(r2) or r2 <= 0:
        return None
    return float(a), float(b), float(np.sqrt(r2))


def gate_center(tags):
    global alt_radius, alt_offsets
    n = len(tags)
    if n == 0:
        return None
    pts = np.array([t[1] for t in tags], dtype=float)
    circ = fit_circle(pts) if n >= 3 else None
    if circ is not None:
        alt_radius = circ[2] if alt_radius is None else 0.8 * alt_radius + 0.2 * circ[2]

    if n >= 4:
        center = (float(pts[:, 0].mean()), float(pts[:, 1].mean()))
    elif circ is not None:
        center = (circ[0], circ[1])
    else:
        preds = [(tx + alt_offsets[tid][0] * s, ty + alt_offsets[tid][1] * s)
                 for tid, (tx, ty), s in tags if tid in alt_offsets and s > 1]
        if preds:
            center = (float(np.mean([p[0] for p in preds])),
                      float(np.mean([p[1] for p in preds])))
        else:
            center = (float(pts[:, 0].mean()), float(pts[:, 1].mean()))

    if n >= 3:
        for tid, (tx, ty), s in tags:
            if s <= 1:
                continue
            o = ((center[0] - tx) / s, (center[1] - ty) / s)
            prev = alt_offsets.get(tid)
            alt_offsets[tid] = o if prev is None else (0.8 * prev[0] + 0.2 * o[0],
                                                       0.8 * prev[1] + 0.2 * o[1])
    return center


def altitude_throttle(drone, dt):
    global alt_center, alt_pe, alt_df, alt_lost, alt_thr, alt_dy, alt_ntags
    fwd = drone.camera.get_color_image()
    if fwd is None or fwd.size == 0:
        tags = []
    else:
        tags = find_tags(fwd)
    alt_ntags = len(tags)

    if not tags:
        alt_lost += 1
        if alt_lost > ALT_COAST:
            alt_thr = 0.0
            alt_dy = None
            alt_pe = 0.0
            alt_df = 0.0
        return alt_thr
    alt_lost = 0

    raw = gate_center(tags)
    alt_center = raw if alt_center is None else (0.35 * raw[0] + 0.65 * alt_center[0],
                                                 0.35 * raw[1] + 0.65 * alt_center[1])
    alt_dy = alt_center[1] - ROW_CENTER
    err = -alt_dy / ROW_CENTER
    raw_ed = (err - alt_pe) / dt if dt > 0 else 0.0
    alt_pe = err
    alt_df = 0.8 * alt_df + 0.2 * raw_ed
    alt_thr = uav_utils.clamp(KP_V * err + KD_V * alt_df, -MAX_THROTTLE, MAX_THROTTLE)
    return alt_thr


def reset():
    global _timer, _done, alt_center, alt_radius, alt_offsets, alt_pe, alt_df
    global alt_lost, alt_thr, alt_dy, alt_ntags
    _timer = 0.0
    _done  = False
    alt_center = None
    alt_radius = None
    alt_offsets = {}
    alt_pe = 0.0
    alt_df = 0.0
    alt_lost = 999
    alt_thr = 0.0
    alt_dy = None
    alt_ntags = 0


def update(drone):
    global slopef, _frame, _timer, _done, intg, pe, df, alt_thr
    if _done:
        return True

    _frame += 1
    dt = drone.get_delta_time()
    _timer += dt

    if _frame % ALT_EVERY == 0:
        thr = altitude_throttle(drone, dt * ALT_EVERY)
    else:
        thr = alt_thr

    img = drone.camera.get_downward_image()
    if img is None or img.size == 0:
        print("no img")
        drone.flight.send_pcmd(0, 0, 0, thr)
        return False

    dl = detect_line(img, V_MIN)
    if not dl["ok"]:
        drone.flight.send_pcmd(0, 0, 0, thr)
        if _frame % 8 == 0:
            print(f"no line {dl['n']} px | tags={alt_ntags} thr={thr:+.2f}")
        return False

    slopef = 0.8 * slopef + 0.2 * dl["slope"]

    frac  = min(abs(slopef) / SLOPE_FULL_SLOW, 1.0)
    pitch = FORWARD_PITCH - frac * (FORWARD_PITCH - PITCH_TURN)

    e = dl["lateral"]
    p = KP * e
    intg = uav_utils.clamp(intg + e * dt, -imax, imax)
    i = KI * intg
    deriv = (e - pe) / dt if dt > 0 else 0.0
    pe = e
    df = 0.7 * df + 0.3 * deriv
    d = KD * df
    vr = -1 * (uav_utils.clamp(p + i + d, -MAX_LAT, MAX_LAT))
    yr = uav_utils.clamp(-YAW_GAIN * slopef, -0.4, 0.4)

    drone.flight.send_pcmd(pitch, vr/0.5, yr, thr)
    if _frame % 8 == 0:
        dytxt = f"{alt_dy:+5.0f}px" if alt_dy is not None else "  -- "
        print(f"  angle={dl['angle']:+5.1f} lat={dl['lateral']:+.2f} "
              f"| vr={vr:+.2f} yaw={yr:+.2f} pitch={pitch:+.2f} "
              f"| tags={alt_ntags} dy={dytxt} thr={thr:+.2f}")

    if _timer >= FOLLOW_TIME:
        drone.flight.stop()
        _done = True
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()

    def start():
        reset()
        print("Line follow + gate altitude")

    def _update():
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go(autostart=True)