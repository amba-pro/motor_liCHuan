"""Runtime configuration for the LC-E EtherCAT tools.

Motion stays disabled until every safety latch below is set by hand after a
physical check. Do not flip these from a script.
"""

IFACE = "enp37s0"

# Physical safety latches. All default to blocked. Set them by hand after the
# check named in the comment. Do not set them from a script.
MOTION_ARMED = False
# External 24 V supply, brake released. Not wired to CN2. Disabling the servo
# does not engage it. Polarity and an independent brake interlock are unchecked.
BRAKE_CIRCUIT_VERIFIED = False
ESTOP_AVAILABLE = False
# The motor is off the SCARA and unloaded. That is not the same as a verified
# fixture, a clear shaft, or known travel limits.
MOTOR_SECURED = False
SHAFT_CLEAR = False
MECHANICALLY_SAFE = False
AXIS_LIMITS_KNOWN = False

# The LC-E manual defines the CiA402 state machine, but section 7.1.1 does not
# define Profile Position controlword bits 4, 5, 6, and 9. Leave this false
# until that behavior is confirmed on this drive.
PP_BITS_CONFIRMED = False

# First motion, when it is eventually allowed, is Cyclic Synchronous Position.
# One measured hold does not accept the 1 ms loop. Leave this false.
CSP_TIMING_ACCEPTED = False

# Conservative command limits used only after the latches above are set.
MAX_MOVE_DEGREES = 1.0
MAX_SPEED_RPM = 5.0
MAX_ACCEL_RPM_PER_S = 20.0
# 0x6077 / torque limits are in 0.1% of rated torque. 200 = 20%.
MAX_TORQUE_PERMILLE = 200

SDO_TIMEOUT_US = 700_000
STATE_TIMEOUT_S = 2.0
MOVE_TIMEOUT_S = 8.0
