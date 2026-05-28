#include "status_led.h"

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include "config.h"
#include "microros.h"

static Adafruit_NeoPixel pixel(1, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);

static uint32_t color_for(AgentState s) {
  switch (s) {
    case AGENT_STATE_WAITING_AGENT: return STATUS_COLOR_WAITING;
    case AGENT_STATE_AVAILABLE:     return STATUS_COLOR_AVAILABLE;
    case AGENT_STATE_CONNECTED:     return STATUS_COLOR_CONNECTED;
    case AGENT_STATE_DISCONNECTED:  return STATUS_COLOR_DISCONNECTED;
  }
  return 0;
}

void status_led_init() {
  if (STATUS_LED_POWER_PIN >= 0) {
    pinMode(STATUS_LED_POWER_PIN, OUTPUT);
    digitalWrite(STATUS_LED_POWER_PIN, HIGH);
  }
  pixel.begin();
  pixel.setBrightness(STATUS_LED_BRIGHTNESS);
  pixel.clear();
  pixel.show();
}

void status_led_tick() {
  static uint64_t next_us  = 0;
  static uint32_t last_rgb = 0xFFFFFFFFu;   // sentinel: forces first write

  const uint64_t now_us = time_us_64();
  if (now_us < next_us) return;
  next_us = now_us + (1000000ULL / STATUS_LED_TICK_HZ);

  const uint32_t rgb = color_for(agent_state);
  if (rgb == last_rgb) return;
  last_rgb = rgb;

  pixel.setPixelColor(0, rgb);
  pixel.show();
}

void status_led_signal_imu_fault() {
  // 2 Hz magenta blink — distinct from any agent_state colour.
  const uint64_t phase = time_us_64() / 250000ULL;
  pixel.setPixelColor(0, (phase & 1) ? 0xFF00FF : 0x000000);
  pixel.show();
}
