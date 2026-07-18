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
FORWARD_PITCH = 0.15     # constant forward speed
MAX_ROLL      = 0.7    # strafe authority for centering
FOLLOW_TIME   = 60.0     # seconds to follow before landing
IMAGE_CENTER  = 320      # 640-wide image -> center column
YAW_GAIN = 0.2
MAX_YAW = 1

# -- Module-level state -----------------------------------------------------
_timer = 0.0
_done  = False

def reset():
    global _timer, _done
    _timer = 0.0
    _done  = False

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
