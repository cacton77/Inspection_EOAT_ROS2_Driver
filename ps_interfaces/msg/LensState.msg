# Focus-lens state, published by the camera head at LENS_STATE_RATE_HZ (20 Hz)
# independently of any callback, so a host can always tell whether the lens is
# usable. Field order follows README section 5.1, with the diagnostic fields
# appended.

# Never homed and no valid calibration restored from flash. position_norm is
# meaningless; only homing and velocity jogs are accepted.
uint8 STATUS_UNCALIBRATED = 0

# Calibrated and stationary.
uint8 STATUS_IDLE = 1

# Executing a velocity jog or a position seek.
uint8 STATUS_MOVING = 2

# StallGuard fired outside of homing: the lens hit a physical limit the stored
# calibration did not predict, so that calibration has been invalidated. This
# is a latched sub-state of UNCALIBRATED, cleared when a homing goal is
# accepted. It exists so the host can tell "never calibrated" apart from
# "calibration went stale" and apply different policy.
uint8 STATUS_STALLED = 3

# Homing sweep in progress. All lens commands are rejected.
uint8 STATUS_HOMING = 4

# Normalized position in [0.0, 1.0] between the homed min and max limits.
# NaN whenever the calibration is not valid.
float32 position_norm

# Commanded velocity in steps/s, signed (positive = toward the max limit).
float32 velocity

uint8 status

# Raw microstep count since boot. Unlike position_norm this is meaningful
# before homing, so it is the coordinate to use for open-loop bring-up and for
# correlating sg_result against travel.
int32 steps

# TMC2209 StallGuard load reading, refreshed while the motor is turning and
# held at the last value at rest. LOWER means HIGHER mechanical load; a stall
# is declared when it falls below SGTHRS. Meaningless at standstill (reads ~0)
# and whenever driver_ok is false.
uint16 sg_result

# The TMC2209 answered over UART at boot. Without it there is no StallGuard,
# so homing cannot run and a stall can never be detected.
bool driver_ok

# The TMC2209's output stage is powered. The firmware de-energises it after
# LENS_IDLE_DISABLE_MS at a standstill, so an idle lens normally reads false --
# that is the resting state, not a fault. Note that a disabled driver has no
# holding torque: `steps` remains the firmware's best estimate but is only as
# good as the mechanism's friction and the motor's detent torque for as long as
# this is false.
bool driver_enabled

builtin_interfaces/Time stamp
