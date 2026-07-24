"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 3: Visual Servoing (Vision + PID)
Capstone: use a PID loop on the camera pixel error to keep a glowing
gate centered by yawing. Combines Week 2 vision with Week 3 control.
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
V_MIN = 200
MIN_AREA = 500
COL_CENTER = 320
KP = 0.25
KI = 0.1
KD = 0.03
MAX_YAW = 0.25
SEARCH_YAW = 0.2
CENTER_TOL = 0.01    # normalized error considered centered
HOLD_TIME = 1.0

# -- Module-level state -----------------------------------------------------
_err_int = 0.0
_prev_err = 0.0
_target_col = None
_hold = 0.0
_done = False
edf = 0.0

def pid_control(err, err_int, err_dot, kp, ki, kd):
    """Return the PID controller output from the three gain terms (see README, Key terms)."""
    ##################################
    #### START PUT CODE HERE #########
    output = kp*err + ki*err_int + kd*err_dot
    ###### END PUT CODE HERE #########
    ##################################
    return output

def reset():
    global _err_int, _prev_err, _target_col, _hold, _done
    _err_int = 0.0
    _prev_err = 0.0
    _target_col = None
    _hold = 0.0
    _done = False


def update(drone):
    global _err_int, _prev_err, _target_col, _hold, _done
    if _done:
        return True
    dt = drone.get_delta_time()
    img = drone.camera.get_color_image()
    if _target_col is None:
        best =neo_lab.gate_nearest_center(img, V_MIN, MIN_AREA)
    else:
        best = neo_lab.gate_nearest_to(img,_target_col, V_MIN, MIN_AREA)
        
    if best is None:
        drone.flight.send_pcmd(0,0,SEARCH_YAW,0)
        _target_col = None
        _err_int = 0.0
        _hold = 0.0
        return False
    r,c = uav_utils.get_contour_center(best)
    _target_col = c
    e = (c - COL_CENTER)/COL_CENTER
    _err_int = uav_utils.clamp(_err_int + e * dt,-1, 1)
    raw_ed = (e - _prev_err)/dt
    _prev_err = e
    global edf
    edf = 0.8*edf + 0.2*raw_ed
    ed = edf
    _prev_err = e
    yaw = uav_utils.clamp(pid_control(e, _err_int, ed, KP, KI, KD), -MAX_YAW, MAX_YAW)
    
    drone.flight.send_pcmd(0,0,yaw,0)
    if abs(e) < CENTER_TOL:
        _hold += dt
    else:
        _hold = 0.0
    if _hold > HOLD_TIME:
        drone.flight.stop()
        print("lock on")
        _done = True
    
    
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 3: Visual Servoing (Vision + PID)")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
