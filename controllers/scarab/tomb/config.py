"""All tunable parameters in one place.

Change values here, not inside the modules, so experiments are easy to
describe in the report (Table: simulation parameters).
"""

import math

# ---------------------------------------------------------------- map frame
# One fixed frame shared by the controller, the ground truth and the metrics.
MAP_RES = 0.05                 # cell size (m)
MAP_SIZE = 16.0                # square map side (m), centred on the world origin
MAP_N = int(round(MAP_SIZE / MAP_RES))
MAP_ORIGIN = (-MAP_SIZE / 2, -MAP_SIZE / 2)   # world (x, y) of cell [0, 0]'s corner

# ------------------------------------------------------- occupancy grid model
L_OCC = 0.85                   # log-odds added to a cell hit by a beam
L_FREE = -0.40                 # log-odds added to a cell a beam passes through
L_MIN, L_MAX = -5.0, 5.0       # clamping keeps the map able to change its mind
P_OCC = 0.65                   # p > P_OCC  -> occupied
P_FREE = 0.35                  # p < P_FREE -> free

# ------------------------------------------------------------------- robot
WHEEL_RADIUS = 0.0975          # Pioneer 3-DX (m)
AXLE_LENGTH = 0.33             # distance between the wheels (m)
MAX_WHEEL_SPEED = 12.0         # rad/s, just under the motor limit
ROBOT_RADIUS = 0.26            # footprint used for collision checks (m)

# ------------------------------------------------------------------- lidar
LIDAR_NAME = "Sick LMS 291"
LIDAR_OFFSET_X = 0.12          # lidar is mounted this far ahead of the axle (m)
LIDAR_FOV = math.pi            # 180 degrees
LIDAR_RAYS = 180
LIDAR_MAX_RANGE = 80.0         # SICK LMS 291 hardware range (m)
LIDAR_RANGE_CAP = 5.0          # use only this much of it for mapping (m)
LIDAR_MIN_RANGE = 0.10         # closer returns are ignored (m)
MAP_UPDATE_EVERY = 2           # update the map every N control steps

# ------------------------------------------------- wander behaviour (v0)
CRUISE_SPEED = 0.35            # m/s
TURN_GAIN = 1.6                # rad/s per rad of heading error
SLOW_DIST = 1.0                # start slowing down below this front clearance (m)
STOP_DIST = 0.45               # rotate in place below this front clearance (m)
FRONT_HALF_ANGLE = math.radians(25)

# ------------------------------------------------------ frontier exploration
STRATEGIES = ("wander", "nearest", "largest", "utility")
FRONTIER_MIN_CELLS = 10        # frontiers narrower than the robot (10 cells = 0.5 m) are ignored
IG_MIN = 0.25                  # frontiers that would reveal less than this (m^2) are not worth a trip
FRONTIER_REACH = 0.45          # a goal may sit this far from its frontier (m)
REPLAN_EVERY_S = 1.5           # re-detect frontiers and re-plan this often (s)
HYSTERESIS = 1.5               # switch goals only if the new one scores this much better...
COMMIT_S = 8.0                 # ...and the current one has been pursued at least this long (s)
GOAL_TRACK_RADIUS = 1.0        # a frontier recedes as it is approached; targets that moved less
                               # than this count as the same goal (m)
UTILITY_LAMBDA = 0.35          # distance discount in U = IG * exp(-lambda * d)  (1/m)
IG_RAYS = 48                   # rays cast from a candidate goal to estimate information gain
GOAL_REACHED_DIST = 0.35       # goal counts as reached within this distance (m)
BLACKLIST_RADIUS = 0.6         # frontier goals near a blacklisted point are skipped (m)
GOAL_TIMEOUT_PAD = 20.0        # extra seconds allowed beyond path_length / speed
BLOCKED_TIMEOUT = 3.0          # seconds blocked by the safety layer before giving up a goal

# -------------------------------------------------------------- planning
INFLATE_RADIUS = ROBOT_RADIUS + 0.05   # C-space obstacle growth (m)
CLEARANCE_SOFT = 0.65          # cells closer than this to an obstacle cost extra (m)
CLEARANCE_WEIGHT = 3.0         # extra step cost right at the inflation boundary

# ---------------------------------------------------------- path following
LOOKAHEAD = 0.5                # pure-pursuit look-ahead distance (m)
TURN_IN_PLACE = math.radians(55)   # rotate on the spot if the path is this far off
TURN_RATE = 1.0                # on-the-spot rotation speed (rad/s)
MAX_TURN_RATE = 1.5            # rad/s

# --------------------------------------------------------------- display
HUD_EVERY_S = 0.5              # refresh the on-robot map display this often (s)
