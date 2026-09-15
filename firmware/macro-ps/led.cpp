#include "led.h"

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <string.h>

#include "config.h"
#include "state.h"

// Chain layout (config.h): NUM_NEOKEYS keys first, then the ring pixels.
static Adafruit_NeoPixel strip(NUM_PIXELS_TOTAL, NEOPIXEL_DATA_PIN,
                               NEO_GRB + NEO_KHZ800);

// -----------------------------------------------------------------------------
// Ring index -> chain index
// -----------------------------------------------------------------------------
// The three rings are contiguous and sit immediately after the NeoKeys, so this
// is the whole mapping. It used to go through a polar lookup table carrying a
// physical angle and radius per pixel, which existed only to feed an on-board
// 2D Gaussian driven by a local joystick.
//
// Both are gone. Spatial control is a host concern now: a node on the Pi owns
// the geometry and the Gaussian (see inspection_eoat/ring_spot.py) and sends
// indexed frames, and the joystick will land on the Pi rather than the Feather.
// Keeping a second spatial model here would have meant two definitions of where
// pixel 0 points -- and this one was wrong, since it assumed 0 rad was 3
// o'clock where the rig actually indexes from straight down.
static inline uint16_t ring_chain_index(int i) {
  return (uint16_t)(OFFSET_INNER + i);
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

static void render_ring_from_command(const uint32_t ring_colors[NUM_RING_PIXELS]) {
  for (int i = 0; i < NUM_RING_PIXELS; i++) {
    frame[ring_chain_index(i)] = ring_colors[i];
  }
}

static void render_ring_blank() {
  for (int i = 0; i < NUM_RING_PIXELS; i++) {
    frame[ring_chain_index(i)] = 0;
  }
}

// Opposing brightness ramps for the magnifier keys (README §4.5).
// p = 1.0 -> Mag+ full, Mag- off. p = 0.5 -> both half.
static void render_neokey_lens_position(float p) {
#if NUM_NEOKEYS >= 3
  if (p < 0.0f) p = 0.0f;
  if (p > 1.0f) p = 1.0f;
  const uint8_t v_plus  = (uint8_t)(p * 255.0f);
  const uint8_t v_minus = (uint8_t)((1.0f - p) * 255.0f);
  frame[IDX_NEOKEY_MAG_PLUS]  = rgb(0, v_plus,  0);
  frame[IDX_NEOKEY_MAG_MINUS] = rgb(0, v_minus, 0);
#else
  // NeoKeys are not on the strand (config.h NUM_NEOKEYS). frame[] is sized to
  // what is physically there, so these indices do not exist.
  (void)p;
#endif
}

// Amber blink per homing phase (README §4.4).
static void render_neokey_homing(HomingPhase phase, bool blink_on) {
#if NUM_NEOKEYS >= 3
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
#else
  (void)phase; (void)blink_on;
#endif
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
bool led_init() {
  strip.begin();
  if (strip.getPixels() == nullptr) {
    return false;   // buffer allocation failed
  }
  // Power budget, not taste — see LED_RING_MAX_BRIGHTNESS in config.h. An
  // all-white frame across NUM_PIXELS_TOTAL would be ~4.5 A at 5 V unclamped,
  // and a single malformed /led_ring/command can ask for exactly that.
  strip.setBrightness(LED_RING_MAX_BRIGHTNESS);
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
  static uint32_t neokey_colors[LED_NEOKEY_WIRE_MAX];

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
#if NUM_NEOKEYS >= 3
      frame[IDX_NEOKEY_PS] = NEOKEY_COLOR_PS_ACTIVE;   // solid while capturing
#endif
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
      // Nothing local pre-empts the ring any more -- the host owns every
      // pattern, spatial ones included -- so this is unconditional.
      render_ring_from_command(ring_colors);
      // Nothing local drives the PS key outside a capture, so it follows
      // whatever /led_ring/command last asked for.
#if NUM_NEOKEYS >= 3
      frame[IDX_NEOKEY_PS] = neokey_colors[IDX_NEOKEY_PS];
#endif
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

    // Publish-side mirror of what actually went out, for /led_ring/state. Taken
    // after show() and in its own short critical section: the rule at the top of
    // this function is that neither rendering nor show() may hold state_mutex
    // against core 0's executor.
    //
    // Indexed in ring order (inner, mid, outer) rather than chain order, to
    // match LedRingCommand.colors. ring_chain_index() is that mapping.
    mutex_enter_blocking(&state_mutex);
    for (int i = 0; i < NUM_RING_PIXELS; i++) {
      state.shown_ring_colors[i] = frame[ring_chain_index(i)];
    }
    for (int i = 0; i < NUM_NEOKEYS; i++) {
      state.shown_neokey_colors[i] = frame[i];
    }
    mutex_exit(&state_mutex);
  }
}
