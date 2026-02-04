#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <array>
#include <deque>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace esphome {
namespace max30102_custom {

// =============================================================
//  MAX30102 REGISTER DEFINITIONS
// =============================================================
static const uint8_t REG_INTR_STATUS_1  = 0x00;
static const uint8_t REG_INTR_STATUS_2  = 0x01;
static const uint8_t REG_INTR_ENABLE_1  = 0x02;
static const uint8_t REG_INTR_ENABLE_2  = 0x03;
static const uint8_t REG_FIFO_WR_PTR    = 0x04;
static const uint8_t REG_OVF_COUNTER    = 0x05;
static const uint8_t REG_FIFO_RD_PTR    = 0x06;
static const uint8_t REG_FIFO_DATA      = 0x07;
static const uint8_t REG_FIFO_CONFIG    = 0x08;
static const uint8_t REG_MODE_CONFIG    = 0x09;
static const uint8_t REG_SPO2_CONFIG    = 0x0A;
static const uint8_t REG_LED1_PA        = 0x0C;  // RED LED current
static const uint8_t REG_LED2_PA        = 0x0D;  // IR LED current
static const uint8_t REG_PART_ID        = 0xFF;  // expect 0x15

// =============================================================
//  DEVICE STATE MACHINE
// =============================================================
enum class DriverState {
  STANDBY = 0,      // minimal power LED, FIFO not processed
  TOUCHING,         // finger detected, timing starts
  LONG_TOUCH,       // (not used as state; we directly transition to MEASURING)
  MEASURING         // measurement fully active
};

// =============================================================
//  MAIN SENSOR CLASS
// =============================================================
class MAX30102CustomSensor : public sensor::Sensor,
                             public PollingComponent,
                             public i2c::I2CDevice {

 public:
  MAX30102CustomSensor() = default;

  // -------------------------------------------------------------
  // YAML SETTERS — CORE SENSOR SETTINGS
  // -------------------------------------------------------------
  void set_led_red_ma(float v) { led_red_ma_active_ = v; }
  void set_led_ir_ma(float v)  { led_ir_ma_active_  = v; }

  void set_idle_leds(float r, float i)   { led_red_ma_idle_   = r; led_ir_ma_idle_   = i; }
  void set_touch_leds(float r, float i)  { led_red_ma_touch_  = r; led_ir_ma_touch_  = i; }
  void set_active_leds(float r, float i) { led_red_ma_active_ = r; led_ir_ma_active_ = i; }

  void set_sample_rate(uint16_t v)   { sample_rate_hz_  = v; }
  void set_sample_average(uint8_t v) { sample_average_  = v; }
  void set_pulse_width(uint16_t v)   { pulse_width_us_  = v; }
  void set_adc_range(uint16_t v)     { adc_range_na_    = v; }

  void set_baseline_alpha(float v) { baseline_alpha_ = v; }
  void set_rms_beta(float v)       { rms_beta_      = v; }

  void set_min_bpm(int v) { min_bpm_ = v; }
  void set_max_bpm(int v) { max_bpm_ = v; }

  void set_finger_threshold(float v)    { finger_thr_ = v; }
  void set_finger_hysteresis(float v)   { finger_hyst_ = v; }
  void set_touch_long_ms(uint32_t v)    { long_touch_ms_ = v; }

  // ---- Runtime adjustments (API) ----
  // Podesivo preko ESPHome API-ja (slidere/switch)
  void set_idle_led_ir(float v)   { led_ir_ma_idle_     = v; }
  void set_idle_led_red(float v)  { led_red_ma_idle_    = v; }
  void set_touch_led_ir(float v)  { led_ir_ma_touch_    = v; }
  void set_touch_led_red(float v) { led_red_ma_touch_   = v; }
  void set_active_led_ir(float v) { led_ir_ma_active_   = v; }
  void set_active_led_red(float v){ led_red_ma_active_  = v; }

  void set_finger_threshold_runtime(float v) { finger_thr_ = v; }

  // Manual LED override (kill switch)
  void set_led_override(bool v) { led_override_ = v; }
  bool get_led_override() const { return led_override_; }

  // -------------------------------------------------------------
  // YAML SETTERS — SUB‑SENSORS
  // -------------------------------------------------------------
  void set_ir_sensor(sensor::Sensor *s)      { ir_sensor_ = s; }
  void set_red_sensor(sensor::Sensor *s)     { red_sensor_ = s; }
  void set_hr_sensor(sensor::Sensor *s)      { hr_sensor_ = s; }
  void set_spo2_sensor(sensor::Sensor *s)    { spo2_sensor_ = s; }

  void set_finger_sensor(binary_sensor::BinarySensor *s)      { finger_sensor_ = s; }
  void set_short_touch_sensor(binary_sensor::BinarySensor *s) { short_touch_sensor_ = s; }
  void set_long_touch_sensor(binary_sensor::BinarySensor *s)  { long_touch_sensor_ = s; }

  // -------------------------------------------------------------
  // STANDARD ESPHOME COMPONENT OVERRIDES
  // -------------------------------------------------------------
  void setup() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  // =============================================================
  //  I2C OPERATIONS
  // =============================================================
  bool write_reg_(uint8_t reg, uint8_t value);
  bool read_reg_(uint8_t reg, uint8_t *value);
  bool burst_read_(uint8_t reg, uint8_t *data, size_t len);

  // =============================================================
  //  INTERNAL PIPELINE
  // =============================================================
  void configure_sensor_();
  void read_fifo_();
  void process_samples_();
  void compute_hr_spo2_();

  // =============================================================
  //  LED CONTROL LAYER
  // =============================================================
  void apply_led_current_(float red_ma, float ir_ma);

  // =============================================================
  //  TOUCH STATE MACHINE
  // =============================================================
  void update_touch_state_();
  float read_single_ir_sample_();  // low-overhead IR sample for touch detection

  // =============================================================
  //  FILTERING / DSP
  // =============================================================
  float update_median_(std::array<float, 5> &win, uint8_t &idx, uint8_t &cnt, float sample);
  static float median5_compute_(const std::array<float, 5> &win, uint8_t cnt);

  float iir_ac_(float x);
  float iir_hr_(float x);
  float median_spo2_(float x);
  float iir_spo2_(float x);

  // =============================================================
  //  DRIVER STATE
  // =============================================================
  DriverState state_ = DriverState::STANDBY;
  uint32_t touch_start_ms_ = 0;
  bool finger_present_ = false;

  // =============================================================
  //  CONFIGURATION PARAMETERS (with defaults)
  // =============================================================
  uint16_t sample_rate_hz_{100};
  uint8_t  sample_average_{4};
  uint16_t pulse_width_us_{411};
  uint16_t adc_range_na_{16384};

  float baseline_alpha_{0.01f};
  float rms_beta_{0.02f};

  int min_bpm_{35};
  int max_bpm_{220};

  float finger_thr_{15000.0f};
  float finger_hyst_{0.8f};
  uint32_t long_touch_ms_{2000};

  // LED modes
  float led_red_ma_idle_{1.0f};
  float led_ir_ma_idle_{1.0f};

  float led_red_ma_touch_{3.0f};
  float led_ir_ma_touch_{6.0f};

  float led_red_ma_active_{7.6f};
  float led_ir_ma_active_{12.6f};

  // Override OFF LED (kill switch)
  bool led_override_{false};

  // =============================================================
  //  HARD FILTER STATES
  // =============================================================
  float ac_iir_state_{0.0f};
  float hr_iir_state_{0.0f};
  float spo2_iir_state_{0.0f};

  // IIR smoothing coefficients
  float ac_iir_a_{0.90f};
  float hr_iir_a_{0.85f};
  float spo2_iir_a_{0.85f};

  float dc_ir_{0.0f};
  float last_ir_{0.0f};
  uint32_t last_peak_ms_{0};
  bool prev_rising_{false};

  // =============================================================
  //  BUFFERS
  // =============================================================
  static const size_t BUF_CAP = 200;
  std::vector<float> ir_buf_;
  std::vector<float> red_buf_;

  std::deque<float> ac_win_;
  size_t ac_win_cap_{50};

  float ac_min_amp_{300.0f};
  float ac_rms_k_{0.80f};

  // median windows
  std::array<float, 5> raw_ir_win_ {{0}};
  std::array<float, 5> raw_red_win_{{0}};
  uint8_t raw_ir_idx_{0}, raw_ir_cnt_{0};
  uint8_t raw_red_idx_{0}, raw_red_cnt_{0};

  std::array<float, 5> spo2_med_win_{{0}};
  uint8_t spo2_med_idx_{0}, spo2_med_cnt_{0};

  uint8_t sample_rate_code_{0};
  uint8_t pulse_width_code_{0};
  uint8_t adc_range_code_{0};

  // =============================================================
  //  OUTPUT SUB‑SENSORS
  // =============================================================
  sensor::Sensor *ir_sensor_{nullptr};
  sensor::Sensor *red_sensor_{nullptr};
  sensor::Sensor *hr_sensor_{nullptr};
  sensor::Sensor *spo2_sensor_{nullptr};

  binary_sensor::BinarySensor *finger_sensor_{nullptr};
  binary_sensor::BinarySensor *short_touch_sensor_{nullptr};
  binary_sensor::BinarySensor *long_touch_sensor_{nullptr};
};

}  // namespace max30102_custom
}  // namespace esphome
