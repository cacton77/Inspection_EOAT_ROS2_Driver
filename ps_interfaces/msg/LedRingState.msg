# Ring illumination as actually realized on the hardware, published at
# LED_STATE_RATE_HZ (10 Hz). Mirrors LedRingCommand, including its 0x00RRGGBB
# packing and the wire-size reasoning behind it.
#
# This is what the renderer last pushed to the strand, which is not always what
# /led_ring/command asked for: a joystick override pre-empts the commanded frame
# (and deliberately does not publish a command of its own -- README section 4.3),
# homing blanks the ring, and a capture pattern replaces it wholesale. Read this
# as ground truth about the hardware, not as an echo of the command.
#
# One thing it does NOT capture: LED_RING_MAX_BRIGHTNESS is applied inside
# Adafruit_NeoPixel at show() time, downstream of these values. These are the
# pre-scale frame, so they match the command's units rather than the emitted
# light.
uint32[<=72] colors
uint32[<=3] neokey_colors

builtin_interfaces/Time stamp
