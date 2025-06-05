import math

# Climb/descent rates (ft/s)
CLIMB_RATE_FPS = 900.0 / 60.0
DESCENT_RATE_FPS = 450.0 / 60.0

# Speeds (knots)
CRUISE_SPEED_KT = 115.0

# UAV physical parameters
UAV_RADIUS_FT = 25.0

# Charging
CHARGE_TIME_SEC = 15 * 60 # 900 s

# Separation minimum defaults (ft)
DEFAULT_MIN_LAT_SEP = 600.0
DEFAULT_MIN_VERT_SEP = 100.0

# Evasive turn rates (rad/s)
TURN_RATE_OUTBOUND = math.radians(5.5)
TURN_RATE_RECOVER = math.radians(3.0)