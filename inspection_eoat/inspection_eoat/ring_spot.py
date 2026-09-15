"""Spatial (Gaussian spot) parameterisation of the macro-ps LED ring.

Compresses the ring's 72-dimensional action space to four numbers -- position,
width and total flux -- so a controller can search illumination rather than
pixels. Intended for shadow / over-exposure compensation, NOT for photometric
stereo: PS wants discrete, well-separated light directions, which is a different
problem that this deliberately does not try to serve.

Everything here runs on the Pi. The firmware is a dumb indexed renderer: it
receives 72 packed colours and displays them. There is no spatial model on the
MCU at all, which is why this module owns the geometry.

Two things make the Cartesian formulation the right one rather than a
preference:

  * A Gaussian separable in (theta, r) is not a spotlight. With sigma fixed in
    radians the physical arc width is r*sigma, so the same parameters paint a
    27 mm spot on the inner ring and a 48 mm spot on the outer one -- width
    would be entangled with radial position, which is exactly what a
    low-dimensional action space must not do.

  * The array is very nearly uniformly sampled in the plane. Arc pitch is
    20.4 mm (inner), 18.8 mm (mid), 18.1 mm (outer), and the rings are 20 mm
    apart radially. One isotropic sigma in millimetres therefore fits the
    hardware in both axes; two sigmas in incompatible units (radians and
    normalised radius) fought it.

Sampling sets a hard floor on sigma, but not the one you would expect. Total
output does NOT ripple as the spot slides: the flux normalisation below forces
the sum to the requested value by construction, so it cancels precisely that
artifact. What degrades instead is the spot's SHAPE. Measured by sweeping a spot
around the outer ring and taking the second moment of the emitted light:

    sigma (mm)  sigma/pitch   centroid err   apparent width swing
       15.0        0.73          0.46 deg          7.4%
       10.2        0.50          0.26 deg          7.5%
        8.0        0.39          0.64 deg           26%
        6.0        0.29          1.79 deg          108%
        4.0        0.20          3.40 deg          200%

The centroid interpolates well -- normalisation is doing real work there -- but
below sigma/pitch ~= 0.5 the apparent width breathes wildly, collapsing onto a
single pixel at some angles and straddling two at others. A controller sweeping
position at such a sigma sees the spot change size as it moves, which is a
worse failure than dimming would be because it is not observable as a
brightness change. SIGMA_MIN_MM is that floor, not a taste setting.
"""

import math

# (name, pixel count, physical radius in mm). Radii are the real ring radii the
# firmware's normalised RADIUS_* constants were derived from (52/92, 72/92, 1).
RINGS = (
    ('inner', 16, 52.0),
    ('mid',   24, 72.0),
    ('outer', 32, 92.0),
)

RING_PIXELS = sum(n for _, n, _ in RINGS)

# Index 0 of every ring points DOWN. In the viewing frame used here -- x right,
# y up, angle measured counter-clockwise from +x -- down is -pi/2.
THETA_ZERO_RAD = -math.pi / 2.0

# Smallest arc pitch bound: see the sampling note above. The inner ring is the
# coarsest at 2*pi*52/16 = 20.4 mm, so it aliases first.
PIXEL_PITCH_MM = 2.0 * math.pi * 52.0 / 16.0
SIGMA_MIN_MM = 0.5 * PIXEL_PITCH_MM          # ~10.2 mm, ~1.5% ripple
SIGMA_MAX_MM = 40.0                          # annulus is only 40 mm wide


def pixel_positions(clockwise=False, theta_zero=THETA_ZERO_RAD):
    """Cartesian position of every ring pixel, in ring-index order.

    Ring-index order is inner, then mid, then outer -- the same concatenation
    LedRingCommand.colors uses, so index i here is index i on the wire.

    `clockwise` is the one piece of geometry that cannot be derived: whether
    pixel index advances clockwise or counter-clockwise from the downward zero
    depends on how the strands were physically routed. Get it wrong and every
    pattern mirrors about the vertical axis. The tuner's canvas draws the live
    readback at these coordinates precisely so a wrong choice is visible at a
    glance rather than subtly wrong forever.
    """
    sign = -1.0 if clockwise else 1.0
    out = []
    for _, count, radius in RINGS:
        for i in range(count):
            theta = theta_zero + sign * (2.0 * math.pi * i / count)
            out.append((radius * math.cos(theta), radius * math.sin(theta)))
    return out


def _tint(color):
    """Normalise an (r,g,b) so the brightest channel is 1.0."""
    peak = max(color) or 255
    return tuple(c / float(peak) for c in color)


def render_spot(positions, x_mm, y_mm, sigma_mm, amplitude, color=(255, 255, 255)):
    """Render a Gaussian spot as packed 0x00RRGGBB values.

    amplitude is TOTAL FLUX, in [0, 1], where 1.0 is every pixel at full scale.
    This is the deliberate choice: with a peak-normalised Gaussian, widening the
    spot silently increases total emitted light (as sigma^2), so width and
    brightness would be coupled and a compensation controller would be chasing
    an exposure change it did not ask for. Normalising the weights to sum to the
    requested flux decouples them -- narrowing sigma concentrates the same light
    rather than dimming.

    The cost is that flux and width are not independently free. Concentrating a
    large flux into few pixels saturates them, after which the requested total
    cannot be met. `headroom` reports the largest amplitude this sigma can carry
    before clipping, and `achieved` reports what was actually emitted; a caller
    that needs the invariant to hold must respect the former.

    Returns (frame, meta).
    """
    n = len(positions)
    sigma_mm = max(float(sigma_mm), 1e-3)
    amplitude = min(max(float(amplitude), 0.0), 1.0)

    two_s2 = 2.0 * sigma_mm * sigma_mm
    raw = [math.exp(-(((x - x_mm) ** 2) + ((y - y_mm) ** 2)) / two_s2)
           for (x, y) in positions]
    total_raw = math.fsum(raw)
    peak_raw = max(raw) if raw else 0.0

    if total_raw <= 0.0 or amplitude <= 0.0:
        return [0] * n, {
            'achieved': 0.0, 'headroom': 1.0, 'clipped': 0,
            'sigma_mm': sigma_mm, 'peak_value': 0,
        }

    # Clipping begins when the peak pixel would exceed full scale.
    headroom = total_raw / (n * peak_raw)

    scale = amplitude * n * 255.0 / total_raw
    vals = [r * scale for r in raw]
    clipped = sum(1 for v in vals if v > 255.0)
    vals = [min(255.0, v) for v in vals]

    cr, cg, cb = _tint(color)
    frame = []
    for v in vals:
        r = int(round(v * cr)); g = int(round(v * cg)); b = int(round(v * cb))
        frame.append(((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF))

    return frame, {
        'achieved': math.fsum(vals) / (n * 255.0),
        'headroom': headroom,
        'clipped': clipped,
        'sigma_mm': sigma_mm,
        'peak_value': int(round(max(vals))),
    }
