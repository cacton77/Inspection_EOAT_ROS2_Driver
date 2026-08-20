#include "led.h"

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <string.h>

#include "config.h"
#include "state.h"

// Chain layout (config.h): [0..2] NeoKeys, [3..74] ring pixels.
static Adafruit_NeoPixel strip(NUM_PIXELS_TOTAL, NEOPIXEL_DATA_PIN,
                               NEO_GRB + NEO_KHZ800);

// -----------------------------------------------------------------------------
// Polar lookup table (README §4.2)
// -----------------------------------------------------------------------------
// Each ring pixel carries a physical angle and a radius normalised to the outer
// ring, so joystick magnitude maps to a physically meaningful radius rather
// than an arbitrary ring index.
struct LedPolar {
  float    angle_rad;     // [0, 2*pi), evenly spaced within each ring
  float    radius_norm;   // physical radius / 92 mm
  uint16_t chain_index;   // index into the strip buffer
};

static LedPolar led_table[NUM_RING_PIXELS];

static void build_led_table() {
  // Physical assumption: pixel 0 of every ring is aligned at 0 radians, and all
  // three rings share that alignment. If a ring is ever remounted at a
  // different physical rotation, add a per-ring angle offset here rather than
  // touching the Gaussian mapping.
  const struct { int n; float r; int offset; } rings[3] = {
    { NUM_RING_INNER, RADIUS_INNER, OFFSET_INNER },
    { NUM_RING_MID,   RADIUS_MID,   OFFSET_MID   },
    { NUM_RING_OUTER, RADIUS_OUTER, OFFSET_OUTER },
  };

  for (int ring = 0; ring < 3; ring++) {
    for (int i = 0; i < rings[ring].n; i++) {
      LedPolar &e  = led_table[rings[ring].offset - OFFSET_INNER + i];
      e.angle_rad   = (2.0f * (float)M_PI * (float)i) / (float)rings[ring].n;
      e.radius_norm = rings[ring].r;
      e.chain_index = (uint16_t)(rings[ring].offset + i);
    }
  }
}

// -----------------------------------------------------------------------------
// Frame buffers
// -----------------------------------------------------------------------------
// We render a whole frame each tick and diff it against what was last pushed
// out. That makes "dirty" exact — show() runs only when a pixel really changed
// — without every render path having to remember to set a flag.
static uint32_t frame[NUM_PIXELS_TOTAL];
static uint32_t shown[NUM_PIXELS_TOTAL];

static inline uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

// -----------------------------------------------------------------------------
// Joystick override — integration point, not yet live
// -----------------------------------------------------------------------------
// joystick.cpp will set these three from the ADC read. Until then
// joystick_active is permanently false and the ring follows /led_ring/command.
static bool  joystick_active    = false;
static float joystick_angle     = 0.0f;   // [-pi, pi]
static float joystick_magnitude = 0.0f;   // [0, 1] -> radius_norm

// 2D Gaussian in (dtheta, dr) space (README §4.2). Angle picks the azimuth,
// magnitude picks which ring(s) light up.
static void render_ring_gaussian(float angle, float magnitude) {
  for (int i = 0; i < NUM_RING_PIXELS; i++) {
    // Angular distance, wrapped to [-pi, pi]
    float dtheta = angle - led_table[i].angle_rad;
    dtheta = atan2f(sinf(dtheta), cosf(dtheta));

    const float dr = magnitude - led_table[i].radius_norm;

    const float weight = expf(-(dtheta * dtheta) / (2.0f * SIGMA_A * SIGMA_A)
                              -(dr     * dr)     / (2.0f * SIGMA_R * SIGMA_R));

    const uint8_t v = (uint8_t)(weight * 255.0f);
    frame[led_table[i].chain_index] = rgb(v, v, v);
  }
}

static void render_ring_from_command(const uint32_t ring_colors[NUM_RING_PIXELS]) {
  for (int i = 0; i < NUM_RING_PIXELS; i++) {
    frame[led_table[i].chain_index] = ring_colors[i];
  }
}

static void render_ring_blank() {
  for (int i = 0; i < NUM_RING_PIXELS; i++) {
    frame[led_table[i].chain_index] = 0;
  }
}

// Opposing brightness ramps for the magnifier keys (README §4.5).
// p = 1.0 -> Mag+ full, Mag- off. p = 0.5 -> both half.
static void render_neokey_lens_position(float p) {
  if (p < 0.0f) p = 0.0f;
  if (p > 1.0f) p = 1.0f;
  const uint8_t v_plus  = (uint8_t)(p * 255.0f);
  const uint8_t v_minus = (uint8_t)((1.0f - p) * 255.0f);
  frame[IDX_NEOKEY_MAG_PLUS]  = rgb(0, v_plus,  0);
  frame[IDX_NEOKEY_MAG_MINUS] = rgb(0, v_minus, 0);
}

// Amber blink per homing phase (README §4.4).
static void render_neokey_homing(HomingPhase phase, bool blink_on) {
  frame[IDX_NEOKEY_PS] = NEOKEY_COLOR_OFF;
  switch (phase) {
    case HOMING_SEEKING_MAX:
      frame[IDX_NEOKEY_MAG_PLUS]  = blink_on ? NEOKEY_COLOR_HOMING : NEOKEY_COLOR_OFF;
      frame[IDX_NEOKEY_MAG_MINUS] = NEOKEY_COLOR_OFF;
      break;
    case HOMING_SEEKING_MIN:
      frame[IDX_NEOKEY_MAG_PLUS]  = NEOKEY_COLOR_OFF;
      frame[IDX_NEOKEY_MAG_MINUS] = blink_on ? NEOKEY_COLOR_HOMING : NEOKEY_COLOR_OFF;
      break;
    case HOMING_ZEROING:
      frame[IDX_NEOKEY_MAG_PLUS]  = NEOKEY_COLOR_HOMING;
      frame[IDX_NEOKEY_MAG_MINUS] = NEOKEY_COLOR_HOMING;
      break;
    case HOMING_NONE:
    default:
      frame[IDX_NEOKEY_MAG_PLUS]  = NEOKEY_COLOR_OFF;
      frame[IDX_NEOKEY_MAG_MINUS] = NEOKEY_COLOR_OFF;
      break;
  }
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
bool led_init() {
  build_led_table();

  strip.begin();
  if (strip.getPixels() == nullptr) {
    return false;   // buffer allocation failed
  }
  strip.setBrightness(255);
  strip.clear();
  strip.show();

  for (int i = 0; i < NUM_PIXELS_TOTAL; i++) {
    frame[i] = 0;
    shown[i] = 0;
  }
  return true;
}

void led_tick() {
  static uint64_t next_us = 0;

  const uint64_t now_us = time_us_64();
  if (now_us < next_us) return;
  next_us = now_us + (1000000ULL / LED_TICK_HZ);

  // Snapshot everything the render needs in one short critical section — the
  // rendering itself (and show()) must not hold the mutex against core 0.
  static uint32_t ring_colors[NUM_RING_PIXELS];
  static uint32_t neokey_colors[NUM_NEOKEYS];

  mutex_enter_blocking(&state_mutex);
  const SystemMode  mode     = state.mode;
  const HomingPhase phase    = state.homing_phase;
  const float       position = state.lens_position_norm;
  memcpy(ring_colors,   state.ring_colors,   sizeof(ring_colors));
  memcpy(neokey_colors, state.neokey_colors, sizeof(neokey_colors));
  mutex_exit(&state_mutex);

  const bool blink_on = ((now_us / LED_BLINK_HALF_PERIOD_US) & 1ULL) != 0ULL;

  switch (mode) {
    case SYS_PS_CAPTURING:
      // The action server owns the ring for the duration of the capture; it
      // writes patterns into state.ring_colors[] and we just realise them.
      render_ring_from_command(ring_colors);
      frame[IDX_NEOKEY_PS] = NEOKEY_COLOR_PS_ACTIVE;   // solid while capturing
      render_neokey_lens_position(position);
      break;

    case SYS_HOMING:
      // Ring stays dark — the lens is sweeping to its hard stops and ring
      // illumination would only be misleading. Swap for a dim white here if
      // the operator needs light on the target during homing.
      render_ring_blank();
      render_neokey_homing(phase, blink_on);
      break;

    case SYS_IDLE:
    case SYS_LENS_MOVING:
    case SYS_UNCALIBRATED:
    default:
      if (joystick_active) {
        // Local joystick control pre-empts the ROS-commanded ring and
        // deliberately does not publish to /led_ring/state.
        render_ring_gaussian(joystick_angle, joystick_magnitude);
      } else {
        render_ring_from_command(ring_colors);
      }
      // Nothing local drives the PS key outside a capture, so it follows
      // whatever /led_ring/command last asked for.
      frame[IDX_NEOKEY_PS] = neokey_colors[IDX_NEOKEY_PS];
      render_neokey_lens_position(position);
      break;
  }

  bool dirty = false;
  for (int i = 0; i < NUM_PIXELS_TOTAL; i++) {
    if (frame[i] != shown[i]) {
      strip.setPixelColor(i, frame[i]);
      shown[i] = frame[i];
      dirty = true;
    }
  }
  if (dirty) {
    strip.show();
  }
}
