# Focus-lens command for the macro-ps camera head.
#
# `value` is interpreted according to `mode`. Use the constants below rather
# than the literal numbers: README section 5.1 documents the opposite polarity
# (0=POSITION) to the reference callback in section 5.4, and getting it wrong
# turns a "stop" command into a full-travel seek.
#
# MODE_VELOCITY is 0 deliberately, so a default-constructed message means
# "hold still" rather than "drive to the fully-retracted limit".

# Jog at `value` steps/s, signed; positive drives toward the max limit.
# Subject to the firmware deadman timeout (VEL_WATCHDOG_MS): motion stops
# unless the command is re-issued. Rejected while HOMING or UNCALIBRATED.
uint8 MODE_VELOCITY = 0

# Seek to `value`, a normalized position in [0.0, 1.0], and stop there.
# Requires a valid calibration: rejected while UNCALIBRATED, STALLED or HOMING.
uint8 MODE_POSITION = 1

# Record the lens's current position as the minimum and rezero the step
# counter there, so the minimum is always step 0. Also clears any latched
# stall and invalidates the recorded maximum, since a new origin makes the old
# far end meaningless. `value` is ignored.
#
# Manual alternative to homing, for a mechanism where sensorless homing is not
# available. Accepted while UNCALIBRATED or STALLED; rejected while HOMING.
uint8 MODE_SET_MIN = 2

# Record the lens's current position as the maximum. `value` is ignored.
# Only meaningful after MODE_SET_MIN: the range is invalid, and position seeks
# stay rejected, until the maximum is above the minimum.
uint8 MODE_SET_MAX = 3

uint8 mode
float32 value
