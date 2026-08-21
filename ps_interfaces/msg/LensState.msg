# Focus-lens state, published by the camera head at LENS_STATE_RATE_HZ (20 Hz)
# independently of any callback, so a host can always tell whether the lens is
# usable. Field order follows README section 5.1.

# Never homed and no valid calibration restored from flash. position_norm is
# meaningless; only homing is accepted.
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

builtin_interfaces/Time stamp
