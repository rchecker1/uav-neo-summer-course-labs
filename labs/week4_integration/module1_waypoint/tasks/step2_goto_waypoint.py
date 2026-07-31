"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 4 · Module 1 — Step 2: Go To a Waypoint
Fly to a target point given as (right, up, forward) meters from the start. This is
your first controller that drives three axes at once: roll for right, pitch for
forward, throttle for up.
"""

import drone_core
import numpy as np
import drone_utils as uav_utils

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
TARGET_RIGHT = 2.0
TARGET_FWD = 4.0
TARGET_HEIGHT = 3.0
KP_POS = 0.15
KD_POS = 0.5            # brake with velocity so you don't overshoot
ALT_KP = 0.12
ROLL_LIMIT = 0.25
PITCH_LIMIT = 0.25
THROTTLE_LIMIT = 0.5
POS_TOL = 0.5          # meters from target counted as arrived
SETTLE_SPEED = 0.25    # must slow below this to finish
HOLD_TIME = 1.5

# -- Module-level state -----------------------------------------------------
_x = 0.0
_z = 0.0
_hold = 0.0
_done = False

def reset():
    global _x, _z, _hold, _done
    _x = 0.0
    _z = 0.0
    _hold = 0.0
    _done = False


def update(drone):
    global _x, _z, _hold, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: fly to (TARGET_RIGHT, TARGET_HEIGHT, TARGET_FWD) and hold there.
    #
    # Tools: drone.physics.get_linear_velocity() -> (vx, vy, vz); drone.get_delta_time();
    #        neo_lab.height(drone); uav_utils.clamp(...); drone.flight.send_pcmd(...).
    #
    # Track right/forward position by integrating vx, vz like Step 1. Drive each
    # horizontal axis with a PD controller (gain KP_POS on position error and KD_POS on
    # velocity, which brakes you): roll for the right error, pitch for the forward error.
    # Hold height with a proportional term (ALT_KP). Clamp each to its limit. Finish when
    # both horizontal errors are under POS_TOL and speed is under SETTLE_SPEED for HOLD_TIME.

    ###### END PUT CODE HERE #########
    ##################################
    dt = drone.get_delta_time()
    vx = drone.physics.get_linear_velocity()[0]
    vy= drone.physics.get_linear_velocity()[1]
    vz = drone.physics.get_linear_velocity()[2]
    _x += dt * vx
    _z += dt * vz
    ze = TARGET_FWD - _z
    xe = TARGET_RIGHT - _x
    roll = uav_utils.clamp(xe * KP_POS - KD_POS * vx, -ROLL_LIMIT, ROLL_LIMIT)
    pitch = uav_utils.clamp(ze * KP_POS - KD_POS * vz, -PITCH_LIMIT, PITCH_LIMIT)
    throttle = uav_utils.clamp(ALT_KP * (TARGET_HEIGHT - neo_lab.height(drone)), -THROTTLE_LIMIT, THROTTLE_LIMIT)
    drone.flight.send_pcmd(pitch,roll,0,throttle)
    speed = np.sqrt(vx * vx + vz * vz)
    if abs(ze) < POS_TOL and abs(xe) < POS_TOL and speed < SETTLE_SPEED:
            _hold += dt
    else:
        _hold = 0.0
    if(_hold >= HOLD_TIME):
        drone.flight.stop()
        print("done")
        _done = True
    
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Go To a Waypoint")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
