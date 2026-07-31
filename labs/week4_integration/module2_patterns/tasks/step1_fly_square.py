"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 4 · Module 2 — Step 1: Fly a Square
A flight path is just a list of waypoints visited in order. Here the list forms a
square. Reuse the waypoint controller from Module 1, one corner at a time, advancing
to the next corner when you reach the current one.
"""

import drone_core
import drone_utils as uav_utils
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
SIDE = 3.0
# Corners as (right, forward) meters from the start, traced as a square.
WAYPOINTS = [(0.0, SIDE), (SIDE, SIDE), (SIDE, 0.0), (0.0, 0.0)]
TARGET_HEIGHT = 3.0
KP_POS = 0.18
KD_POS = 0.5
ALT_KP = 0.12
ROLL_LIMIT = 0.25
PITCH_LIMIT = 0.25
THROTTLE_LIMIT = 0.5
WP_TOL = 0.6           # meters from a corner counted as reached

# -- Module-level state -----------------------------------------------------
_x = 0.0
_z = 0.0
_wp = 0
_done = False
ind = 0

def reset():
    global _x, _z, _wp, _done
    _x = 0.0
    _z = 0.0
    _wp = 0
    _done = False
    ind = 0


def update(drone):
    global _x, _z, _wp, _done
    if _done:
        return True
    if _wp >= len(WAYPOINTS):
        drone.flight.stop()
        print("done")
        _done = True
        return True
    curr = WAYPOINTS[_wp]
    dt = drone.get_delta_time()
    vx, vy, vz = drone.physics.get_linear_velocity()
    _x += dt * vx
    _z += dt * vz
    xe = curr[0] - _x
    ze = curr[1] - _z
    roll     = uav_utils.clamp(KP_POS*xe - KD_POS*vx, -ROLL_LIMIT, ROLL_LIMIT)
    pitch    = uav_utils.clamp(KP_POS*ze - KD_POS*vz, -PITCH_LIMIT, PITCH_LIMIT)
    throttle = uav_utils.clamp(ALT_KP*(TARGET_HEIGHT - neo_lab.height(drone)), -THROTTLE_LIMIT, THROTTLE_LIMIT)
    drone.flight.send_pcmd(pitch, roll, 0, throttle)
    if abs(xe) < WP_TOL and abs(ze) < WP_TOL:
        print(f"corner {_wp} reached")
        _wp += 1
    return False


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 1: Fly a Square")

    def _update():
        if not _launcher.done:
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
