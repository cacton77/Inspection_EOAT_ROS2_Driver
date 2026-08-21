# Illumination pattern for the macro-ps LED ring.
#
# The ring is NUM_RING_PIXELS = 72 pixels: 16 inner + 24 mid + 32 outer,
# indexed in that order (see config.h OFFSET_INNER / OFFSET_MID / OFFSET_OUTER,
# which are offset by the 3 NeoKey pixels sharing the strand).
#
# WIRE SIZE WARNING: ColorRGBA is 4 x float32 = 16 bytes, so a full ring is
# 1152 bytes. The micro-ROS custom transport MTU is 512 bytes
# (UXR_CONFIG_CUSTOM_TRANSPORT_MTU) and best-effort streams do not fragment.
# Before the LED path is implemented this must either move to a *reliable*
# publisher/subscription (which does fragment, bounded by
# RMW_UXRCE_STREAM_HISTORY_* x MTU), or switch to packed uint8 RGB, which
# matches what the NeoPixel hardware actually consumes and fits in 216 bytes.
std_msgs/ColorRGBA[<=72] colors
