import math
# Constants
KNOTS_TO_FT_PER_SEC = (6076.0 / 3600.0)
FT_IN_NM = 6076.0

# Simulation Parameters
MIN_DELAY_TIME_S = 300 # in seconds
AIRSPACE = "Simple" # Either "Standard" or "Simple"
UAVS_PER_ROUTE = 100
RUN_TIME_SEC = 1 * 24 * 60 * 60 # run time of simulation, i.e. how long you want the simulation to run for

# Climb/descent rates (ft/s)
CLIMB_RATE_FPS = 900.0 / 60.0
DESCENT_RATE_FPS = 450.0 / 60.0

# Speeds (knots)
CRUISE_SPEED_KT = 115.0

# UAV physical parameters
UAV_RADIUS_FT = 25.0

# Charging
CHARGE_TIME_SEC = 15 * 60 # 900 s
CHARGE_STATIONS = 40

# Separation minimum defaults (ft)
DEFAULT_MIN_LAT_SEP = 600.0
DEFAULT_MIN_VERT_SEP = 100.0

# Evasive turn rates (rad/s)
TURN_RATE_OUTBOUND = math.radians(5.5)
TURN_RATE_RECOVER = math.radians(3.0)

# Evasive minimums
MIN_SPEED_TO_SLOW = 112.0 # knots, UAV must be at least this fast if it intends to slow down
MAX_SPEED_TO_INCREASE = 118.0 # knots, UAV must be at most this fast if it intends to speed up

# Chance of deviation
PROBABILITY_OF_DIVERTION = 0.001 / 60.0 # (0.001 per minute)