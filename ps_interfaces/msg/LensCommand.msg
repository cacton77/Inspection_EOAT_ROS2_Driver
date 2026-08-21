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

uint8 mode
float32 value
