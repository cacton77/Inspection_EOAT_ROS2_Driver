# Illumination pattern for the macro-ps LED ring.
#
# The ring is NUM_RING_PIXELS = 72 pixels: 16 inner + 24 mid + 32 outer, indexed
# in that order (see config.h OFFSET_INNER / OFFSET_MID / OFFSET_OUTER, which are
# offset by the 3 NeoKey pixels sharing the strand).
#
# Colours are packed 0x00RRGGBB, one uint32 per pixel -- deliberately NOT
# ColorRGBA. ColorRGBA is 4 x float32 = 16 bytes, so a full ring was 1152 bytes
# against the 512-byte custom-transport MTU
# (UXR_CONFIG_CUSTOM_TRANSPORT_MTU), and best-effort streams do not fragment.
# The two ways out were a reliable endpoint, which does fragment, or packed
# colour. Packed won on two counts: at 292 bytes it fits a single datagram, so
# the QoS stays best-effort exactly as README section 5.3 specifies; and a
# reliable stream's ACK waits would land in core 0's executor -- the same core
# whose stalls the IMU queue depth exists to detect.
#
# The float precision was never real. The renderer quantises to uint8 at the
# point of use, and 0x00RRGGBB is already the representation carried by both
# state.ring_colors[] and Adafruit_NeoPixel::setPixelColor(), so the callback
# reduces to a memcpy.
#
# Treated as a WHOLE FRAME: pixels beyond colors.size are blanked rather than
# left stale, so a zero-length message is a valid "all off" and a short message
# never displays a mix of new and stale pixels.
uint32[<=72] colors

# NeoKey pixels, indexed [PS, Mag+, Mag-] per config.h IDX_NEOKEY_*. Same
# packing. Outside a capture only the PS key follows this -- Mag+/Mag- are driven
# locally from lens position (README section 4.5) and their entries here are
# ignored. During SYS_PS_CAPTURING the action server owns all three.
uint32[<=3] neokey_colors
