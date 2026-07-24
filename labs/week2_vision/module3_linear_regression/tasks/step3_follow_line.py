"""

import drone_core
import drone_utils as uav_utils
import cv2
import numpy as np

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
V_MIN         = 200
MIN_PIXELS    = 200
FORWARD_PITCH = 0.15     # constant forward speed
MAX_ROLL      = 0.7    # strafe authority for centering
FOLLOW_TIME   = 60.0     # seconds to follow before landing
IMAGE_CENTER  = 320      # 640-wide image -> center column
YAW_GAIN = 0.2
MAX_YAW = 1

# -- Module-level state -----------------------------------------------------
_timer = 0.0
_done  = False
X_MARGIN   = 0.25
MIN_PIXELS = 200
MAX_FILL   = 0.25
MAX_RESID  = 40.0

def detect_line(image, v_min=200):
    
    mask = neo_lab.bright_mask(image, v_min)          # your HSV-Value mask
    h, w = mask.shape
    r1 = h // 2
    x0, x1 = int(w * X_MARGIN), int(w * (1 - X_MARGIN))

    region = mask[:r1, x0:x1]                          # crop first (fewer pixels)
    ys, xs = np.nonzero(region)                        # vectorized, C-level
    n = xs.size
    band = r1 * (x1 - x0)
    if n < MIN_PIXELS or n > MAX_FILL * band:
        return {"ok": False, "n": n}

    xs = xs + x0                                        # cols back to full-frame coords
    m, b = np.polyfit(ys, xs, 1)                       # col = m*row + b
    resid = float(np.std(xs - (m * ys + b)))
    if resid > MAX_RESID:
        return {"ok": False, "n": n}

    col_at_drone = m * (r1 - 1) + b
    return {
        "ok": True,
        "slope": float(m),
        "angle": float(np.degrees(np.arctan(m))),      # heading error, 0 = straight ahead
        "lateral": float((col_at_drone - w / 2) / (w / 2)),  # -1 left .. +1 right
        "resid": resid,
        "n": n,
    }

def reset():
    global _timer, _done
    _timer = 0.0
    _done  = False

def fit_line(points):
    
    ##################################
    #### START PUT CODE HERE #########
    x = points[:,0]
    y = points[:, 1]
    m, b = np.polyfit(x,y,1)
    ###### END PUT CODE HERE #########
    ##################################
    return m, b
def update(drone):
    global _timer, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: fly forward at FORWARD_PITCH while strafing (roll) to keep the bright
    # edge under the middle of the downward camera.
    #
    # Tools: drone.camera.get_downward_image(); neo_lab.bright_mask(image, V_MIN);
    #        np.argwhere(mask) -> bright pixel (row, col); uav_utils.clamp(...);
    #        drone.flight.send_pcmd(pitch, roll, yaw, throttle).
    #
    # The average column of the bright pixels tells you how far off-center the edge
    # is. Turn that pixel offset into a roll command (clamped to MAX_ROLL): an edge
    # right of center means roll right to chase it. If you see too few bright pixels,
    # hold position rather than steering on noise -- but keep the timer running every
    # frame and finish after FOLLOW_TIME regardless, so losing the edge never hangs.
    """
"""
    _timer += drone.get_delta_time()
    img = drone.camera.get_downward_image()
    d = detect_line(drone.camera.get_downward_image(), V_MIN)
    if not d["ok"]:
        neo_lab.send_velocity(drone, 0.0, v_up, 0.0, 0.0)      # lost line -> hover
    else:
        _slope_f = 0.8*_slope_f + 0.2*d["slope"]                # filter yaw
        v_right  = uav_utils.clamp(LAT_GAIN * d["lateral"], -MAX_LAT, MAX_LAT)
        yaw_rate = uav_utils.clamp(-YAW_GAIN * _slope_f, -MAX_YAW, MAX_YAW)
        neo_lab.send_velocity(drone, v_right, v_up, FORWARD_SPEED, yaw_rate)
    """
"""
    bright = neo_lab.bright_mask(img, V_MIN)
    bright = bright[150:240, :]
    pts = np.argwhere(bright)
    if(len(pts) < MIN_PIXELS):
        return False
    else:
        m,b = fit_line(pts)
        meancol = pts[:,1].mean()
        offset = (meancol - IMAGE_CENTER) / IMAGE_CENTER
        roll = uav_utils.clamp(offset * MAX_ROLL, -MAX_ROLL, MAX_ROLL)
        yaw = uav_utils.clamp(-YAW_GAIN * m, -MAX_YAW, MAX_YAW)
        drone.flight.send_pcmd(FORWARD_PITCH,roll,yaw,0)
        
    if _timer >= FOLLOW_TIME:
        drone.flight.stop()
        _done = True
    ###### END PUT CODE HERE #########
    ##################################
    
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 3: Follow the Edge")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
"""


"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 3: Follow the Edge
Steer the drone to keep the bright edge centered while flying forward.
"""

import drone_core
import drone_utils as uav_utils
import cv2
import numpy as np

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
V_MIN         = 200
MIN_PIXELS    = 200
FORWARD_PITCH = 0.1
MAX_ROLL      = 0.7
FOLLOW_TIME   = 45.0
IMAGE_CENTER  = 320
YAW_GAIN = 1
MAX_YAW = 1


KP   = 0.2
KI   = 0.0
KD   = 0.0
imax = 1.0
MAX_LAT = 0.3

# -- Module-level state -----------------------------------------------------
_timer = 0.0
_done  = False
slopef = 0.0       
_frame = 0           
xm   = 0.15
mfill   = 0.25
mresid  = 100
intg = 0.0
pe   = 0.0
lost = 0
lyr = 0.0
lvr = 0.0

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
    lateral = (col_at_drone  - w/2 ) / (w/2)
    return {
        "ok": True,
        "slope": float(m),
        "angle": float(np.degrees(np.arctan(m))),
        "lateral": float(lateral),
        "resid": resid,
        "n": n,
    }

def reset():
    global _timer, _done, lost
    _timer = 0.0
    _done  = False
    lost = 0

def fit_line(points):
    """Least-squares fit of y = m*x + b. points is the (row, col) array from
    np.argwhere, so column = x and row = y. See the README (Key terms) for the fit."""
    ##################################
    #### START PUT CODE HERE #########
    x = points[:,0]
    y = points[:, 1]
    m, b = np.polyfit(x,y,1)
    ###### END PUT CODE HERE #########
    ##################################
    return m, b

def update(drone):
    global slopef, _frame, _timer, _done, intg, pe, lost, lvr, lyr
    _frame += 1
    dt = drone.get_delta_time()
    _timer += dt
    img = drone.camera.get_downward_image()
    if img is None or img.size == 0:
        print("no img")
        neo_lab.send_velocity(drone, 0,0,0,0)
        return False
    """
    if _frame % 8:
        return False
    """
    dl = detect_line(img, V_MIN)
    if not dl["ok"]:
        lost += 1
        if(lost >= 5):
            drone.flight.send_pcmd(0,0,0,0)
            print(f"no line {dl['n']} px")
            return False
        else:
            drone.flight.send_pcmd(FORWARD_PITCH/0.5, lvr, lyr, 0)
            return False

    slopef = 0.8 * slopef + 0.2 * dl["slope"]
    e = dl["lateral"]
    p = KP * e
    intg = uav_utils.clamp(intg + e * dt, -imax, imax)
    i = KI * intg
    deriv = (e - pe) / dt
    pe = e
    d = KD * deriv
    #vr  = uav_utils.clamp(0.4 * dl["lateral"], -0.3, 0.3)
    vr = (uav_utils.clamp(p + i + d, -MAX_LAT, MAX_LAT))
    yr = uav_utils.clamp(-YAW_GAIN * slopef, -0.4, 0.4)
    lvr, lyr = vr, yr
    
    print(yr)
    #neo_lab.send_velocity(drone, vr, 0, FORWARD_PITCH, yr)
    drone.flight.send_pcmd(FORWARD_PITCH/0.5, vr/0.5, yr, 0)
    if _frame % 8 == 0:
        print(f"  angle={dl['angle']:+5.1f} lat={dl['lateral']:+.2f} | vr={vr:+.2f} yaw={yr:+.2f}")
        
    if _timer >= FOLLOW_TIME:
        drone.flight.stop()
        _done = True
    return _done   
    """
    global _timer, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: fly forward at FORWARD_PITCH while strafing (roll) to keep the bright
    # edge under the middle of the downward camera.
    #
    # Tools: drone.camera.get_downward_image(); neo_lab.bright_mask(image, V_MIN);
    #        np.argwhere(mask) -> bright pixel (row, col); uav_utils.clamp(...);
    #        drone.flight.send_pcmd(pitch, roll, yaw, throttle).
    #
    # The average column of the bright pixels tells you how far off-center the edge
    # is. Turn that pixel offset into a roll command (clamped to MAX_ROLL): an edge
    # right of center means roll right to chase it. If you see too few bright pixels,
    # hold position rather than steering on noise -- but keep the timer running every
    # frame and finish after FOLLOW_TIME regardless, so losing the edge never hangs.
    
    _timer += drone.get_delta_time()
    img = drone.camera.get_downward_image()
    
    d = detect_line(drone.camera.get_downward_image(), V_MIN)
    if not d["ok"]:
        neo_lab.send_velocity(drone, 0.0, v_up, 0.0, 0.0)      # lost line -> hover
    else:
        slopef = 0.8*slopef + 0.2*d["slope"]                # filter yaw
        vr  = uav_utils.clamp(LAT_GAIN * d["lateral"], -MAX_LAT, MAX_LAT)
        yr = uav_utils.clamp(-YAW_GAIN * slopef, -MAX_YAW, MAX_YAW)
        neo_lab.send_velocity(drone, vr, v_up, FORWARD_SPEED, yr)
    
    bright = neo_lab.bright_mask(img, V_MIN)
    bright = bright[150:240, :]
    pts = np.argwhere(bright)
    if(len(pts) < MIN_PIXELS):
        return False
    else:
        m,b = fit_line(pts)
        meancol = pts[:,1].mean()
        offset = (meancol - IMAGE_CENTER) / IMAGE_CENTER
        #roll = uav_utils.clamp(offset * MAX_ROLL, -MAX_ROLL, MAX_ROLL)
        #yaw = uav_utils.clamp(-YAW_GAIN * m, -MAX_YAW, MAX_YAW)
        #drone.flight.send_pcmd(FORWARD_PITCH,roll,yaw,0)
        
    if _timer >= FOLLOW_TIME:
        #drone.flight.stop()
        _done = True
    ###### END PUT CODE HERE #########
    ##################################
    
    return _done

"""
if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 3: Follow the Edge")

    def _update():
        #if not _launcher.done:        # arm + climb to a safe height first
        #    _launcher.update(_drone)
        #    return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()

