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

// ---- MAX30102 registri ----
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
static const uint8_t REG_LED1_PA        = 0x0C;  // RED
static const uint8_t REG_LED2_PA        = 0x0D;  // IR
static const uint8_t REG_TEMP_INTR      = 0x1F;
static const uint8_t REG_TEMP_FRAC      = 0x20;
static const uint8_t REG_TEMP_CONFIG    = 0x21;
static const uint8_t REG_PART_ID        = 0xFF;  // očekivano 0x15

// Glavna klasa drivera — usklađena sa sensor.py
class MAX30102CustomSensor : public sensor::Sensor,
                             public PollingComponent,
                             public i2c::I2CDevice {
 public:
  // --- Setteri iz YAML/sensor.py ---
  void set_address(uint8_t v) { this->set_i2c_address(v); }  // I2CDevice helper

  void set_led_red_ma(float v) { led_red_ma_ = v; }
  void set_led_ir_ma(float v)  { led_ir_ma_  = v; }

  void set_sample_rate(uint16_t v)   { sample_rate_hz_  = v; }   // 50/100/200/400
  void set_sample_average(uint8_t v) { sample_average_  = v; }   // 1/2/4/8/16
  void set_pulse_width(uint16_t v)   { pulse_width_us_  = v; }   // 69/118/215/411
  void set_adc_range(uint16_t v)     { adc_range_na_    = v; }   // 2048/4096/8192/16384

  void set_baseline_alpha(float v) { baseline_alpha_ = v; } // DC EWMA α (0..1)
  void set_rms_beta(float v)       { rms_beta_      = v; } // rezervirano

  void set_min_bpm(int v) { min_bpm_ = v; }
  void set_max_bpm(int v) { max_bpm_ = v; }

  void set_hr_median_window(int v)   { hr_median_window_   = v; }    // rezervirano (koristimo IIR)
  void set_spo2_median_window(int v) { spo2_median_window_ = v; }    // koristimo 5 + IIR

  void set_finger_ir_threshold(float v) { finger_thr_  = v; }
  void set_finger_hysteresis(float v)   { finger_hyst_ = v; }
  void set_perf_min(float v)            { perf_min_    = v; }  // rezervirano

  // Pod-senzori (opcionalni izlazi)
  void set_ir_sensor(sensor::Sensor *s)   { ir_sensor_   = s; }
  void set_red_sensor(sensor::Sensor *s)  { red_sensor_  = s; }
  void set_hr_sensor(sensor::Sensor *s)   { hr_sensor_   = s; }
  void set_spo2_sensor(sensor::Sensor *s) { spo2_sensor_ = s; }
  void set_finger_sensor(binary_sensor::BinarySensor *s) { finger_sensor_ = s; }

  // ESPHome lifecycle
  void setup() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  // --- Niskorazinski I2C helperi ---
  bool write_reg_(uint8_t reg, uint8_t val);
  bool read_reg_(uint8_t reg, uint8_t *val);
  bool burst_read_(uint8_t reg, uint8_t *data, size_t len);

  // --- Konfiguracija i obrada ---
  void configure_sensor_();
  void read_fifo_();
  void process_samples_();
  void compute_hr_spo2_();

  // --- Stable-mode: median & IIR helperi ---
  float update_median_(std::array<float, 5> &win, uint8_t &idx, uint8_t &cnt, float sample);
  static float median5_compute_(const std::array<float, 5> &win, uint8_t cnt);
  float iir_ac_(float x);      // IIR na AC (za peak detect)
  float iir_hr_(float x);      // IIR na HR izlaz
  float median_spo2_(float x); // median(5) na SpO2
  float iir_spo2_(float x);    // IIR na SpO2 izlaz

  // --- Parametri stable moda ---
  float ac_iir_a_{0.90f};   // jači LPF na AC
  float hr_iir_a_{0.85f};   // glađi HR prikaz
  float spo2_iir_a_{0.85f}; // glađi SpO₂ prikaz

  // Median prozori (RAW i SpO₂)
  std::array<float, 5> raw_ir_win_ {{0,0,0,0,0}};
  std::array<float, 5> raw_red_win_{{0,0,0,0,0}};
  uint8_t raw_ir_idx_{0},  raw_ir_cnt_{0};
  uint8_t raw_red_idx_{0}, raw_red_cnt_{0};
  std::array<float, 5> spo2_med_win_{{0,0,0,0,0}};
  uint8_t spo2_med_idx_{0}, spo2_med_cnt_{0};

  // IIR stanja
  float ac_iir_state_{0.0f};
  float hr_iir_state_{0.0f};
  float spo2_iir_state_{0.0f};

  // Adaptivni prag (RMS) za peak detekciju
  std::deque<float> ac_win_; // zadnjih ~50 AC vrijednosti
  size_t ac_win_cap_{50};
  float ac_min_amp_{300.0f}; // minimalna amplituda
  float ac_rms_k_{0.80f};    // prag = max(min_amp, k * RMS)

  // --- Postavke iz YAML-a (ljudski, ne kodovi) ---
  float   led_red_ma_{7.6f};
  float   led_ir_ma_{7.6f};
  uint16_t sample_rate_hz_{100};
  uint8_t  sample_average_{4};
  uint16_t pulse_width_us_{411};
  uint16_t adc_range_na_{16384};

  float baseline_alpha_{0.01f};
  float rms_beta_{0.02f};

  int min_bpm_{35};
  int max_bpm_{220};

  int   hr_median_window_{5};
  int   spo2_median_window_{7};
  float finger_thr_{300.0f};
  float finger_hyst_{0.8f};
  float perf_min_{0.002f};

  // --- Interni kodovi za registraciju ---
  uint8_t sample_rate_code_{1}; // 100Hz
  uint8_t pulse_width_code_{3}; // 411us (18-bit)
  uint8_t adc_range_code_{3};   // 16384 nA

  // --- Pod-senzori ---
  sensor::Sensor *ir_sensor_{nullptr};
  sensor::Sensor *red_sensor_{nullptr};
  sensor::Sensor *hr_sensor_{nullptr};
  sensor::Sensor *spo2_sensor_{nullptr};
  binary_sensor::BinarySensor *finger_sensor_{nullptr};

  // --- Buffers i stanja algoritma ---
  static const size_t BUF_CAP = 100; // ~1 s @100 Hz
  std::vector<float> ir_buf_;
  std::vector<float> red_buf_;

  float dc_ir_{0.0f}, dc_red_{0.0f};
  float last_ir_{0.0f};         // za rising/falling logiku
  uint32_t last_peak_ms_{0};
  bool prev_rising_{false};
  bool finger_present_{false};
};

}  // namespace max30102_custom
}  // namespace esphome// placeholder header
