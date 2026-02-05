#include "max30102_custom.h"
#include "esphome/core/log.h"

namespace esphome {
namespace max30102_custom {

static const char *TAG = "max30102_custom";

// ================================================================
//  Utility: convert LED current (mA) → register value (0–255)
// ================================================================
static inline uint8_t led_pa_from_ma(float ma) {
  int v = static_cast<int>(ma / 0.2f + 0.5f);
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  return static_cast<uint8_t>(v);
}

// ================================================================
//  Utility: map user parameters → datasheet codes
// ================================================================
static inline uint8_t map_sample_rate_code(uint16_t hz) {
  if (hz >= 400) return 3;
  if (hz >= 200) return 2;
  if (hz >= 100) return 1;
  return 0;  // 50 Hz
}

static inline uint8_t map_pulse_width_code(uint16_t us) {
  if (us >= 411) return 3;
  if (us >= 215) return 2;
  if (us >= 118) return 1;
  return 0;
}

static inline uint8_t map_adc_range_code(uint16_t na) {
  if (na >= 16384) return 3;
  if (na >= 8192)  return 2;
  if (na >= 4096)  return 1;
  return 0;
}

static inline uint8_t map_sample_avg_code(uint8_t avg) {
  if (avg >= 16) return 4;
  if (avg >= 8)  return 3;
  if (avg >= 4)  return 2;
  if (avg >= 2)  return 1;
  return 0;
}

// ================================================================
//  I2C Helpers
// ================================================================
bool MAX30102CustomSensor::write_reg_(uint8_t reg, uint8_t val) {
  uint8_t data[2] = {reg, val};
  auto err = this->write(data, 2);
  return err == i2c::ERROR_OK;
}

bool MAX30102CustomSensor::read_reg_(uint8_t reg, uint8_t *val) {
  auto err = this->write_read(&reg, 1, val, 1);
  return err == i2c::ERROR_OK;
}

bool MAX30102CustomSensor::burst_read_(uint8_t reg, uint8_t *data, size_t len) {
  auto err = this->write_read(&reg, 1, data, len);
  return err == i2c::ERROR_OK;
}

// ================================================================
//  LED Current Apply
// ================================================================
void MAX30102CustomSensor::apply_led_current_(float red_ma, float ir_ma) {
  write_reg_(REG_LED1_PA, led_pa_from_ma(red_ma));
  write_reg_(REG_LED2_PA, led_pa_from_ma(ir_ma));
}

// ================================================================
//  Read single IR sample for touch detection (NO FIFO RESET)
// ================================================================
float MAX30102CustomSensor::read_single_ir_sample_() {
  uint8_t wr_ptr = 0, rd_ptr = 0;
  if (!read_reg_(REG_FIFO_WR_PTR, &wr_ptr)) return touch_ir_ewma_;
  if (!read_reg_(REG_FIFO_RD_PTR, &rd_ptr)) return touch_ir_ewma_;

  uint8_t n = (wr_ptr - rd_ptr) & 0x1F;
  if (n == 0) {
    // nema novog uzorka → vrati posljednji filtrirani
    return touch_ir_ewma_;
  }

  uint8_t buf[6];
  if (!burst_read_(REG_FIFO_DATA, buf, 6))
    return touch_ir_ewma_;

  // IR komponenta iz 18-bit podataka
  uint32_t ir =
    ((((uint32_t)buf[3] << 16) |
      ((uint32_t)buf[4] << 8)  |
       (uint32_t)buf[5]) & 0x3FFFF);

  // lagani EWMA da “touch” ne flipa
  const float a = 0.2f;
  touch_ir_ewma_ = (1.0f - a) * touch_ir_ewma_ + a * static_cast<float>(ir);
  return touch_ir_ewma_;
}

// ================================================================
//  TOUCH STATE MACHINE (debounce + bez resetiranja FIFO-a)
// ================================================================
void MAX30102CustomSensor::update_touch_state_() {
  // Manual LED OFF override
  if (led_override_) {
    apply_led_current_(0.0f, 0.0f);
    finger_present_ = false;
    if (finger_sensor_) finger_sensor_->publish_state(false);
    state_ = DriverState::STANDBY;
    return;
  }

  uint32_t now = millis();

  // U MEASURING ne smijemo konzumirati FIFO → koristimo EWMA sirovog IR-a
  float ir_level = 0.0f;
  if (state_ == DriverState::MEASURING) {
    ir_level = touch_ir_ewma_; // osvježava se u read_fifo_()
  } else {
    ir_level = read_single_ir_sample_(); // može uzeti 1 sample iz FIFO-a
  }

  bool above = ir_level > finger_thr_;
  bool below = ir_level < finger_thr_ * finger_hyst_;

  switch (state_) {
    case DriverState::STANDBY:
      if (above) {
        if (touch_detect_ms_ == 0) touch_detect_ms_ = now;
        if (now - touch_detect_ms_ >= touch_detect_debounce_ms_) {
          finger_present_ = true;
          if (finger_sensor_) finger_sensor_->publish_state(true);
          state_ = DriverState::TOUCHING;
          touch_start_ms_ = now;
          apply_led_current_(led_red_ma_touch_, led_ir_ma_touch_);
          ESP_LOGI(TAG, "Touch detected → TOUCHING mode");
          touch_detect_ms_ = 0;
        }
      } else {
        touch_detect_ms_ = 0;
      }
      break;

    case DriverState::TOUCHING:
      if (below) {
        if (touch_release_ms_ == 0) touch_release_ms_ = now;
        if (now - touch_release_ms_ >= touch_release_debounce_ms_) {
          // Short-touch pulse event
          if (short_touch_sensor_) {
            short_touch_sensor_->publish_state(true);
            short_touch_sensor_->publish_state(false);
          }
          // Reset LED to idle
          apply_led_current_(led_red_ma_idle_, led_ir_ma_idle_);
          finger_present_ = false;
          if (finger_sensor_) finger_sensor_->publish_state(false);
          state_ = DriverState::STANDBY;
          ESP_LOGI(TAG, "Touch released → short-touch");
          touch_release_ms_ = 0;
        }
      } else {
        touch_release_ms_ = 0;
      }

      if ((now - touch_start_ms_) >= long_touch_ms_) {
        // LONG TOUCH TRIGGER
        if (long_touch_sensor_) {
          long_touch_sensor_->publish_state(true);
          long_touch_sensor_->publish_state(false);
        }

        // Go active
        apply_led_current_(led_red_ma_active_, led_ir_ma_active_);
        state_ = DriverState::MEASURING;

        // Reset FIFO for clean measuring (ovdje je ispravno)
        write_reg_(REG_FIFO_RD_PTR, 0x00);
        write_reg_(REG_FIFO_WR_PTR, 0x00);
        write_reg_(REG_OVF_COUNTER, 0x00);

        ESP_LOGI(TAG, "Long touch! → MEASURING mode");
      }
      break;

    case DriverState::MEASURING:
      if (below) {
        if (touch_release_ms_ == 0) touch_release_ms_ = now;
        if (now - touch_release_ms_ >= touch_release_debounce_ms_) {
          state_ = DriverState::STANDBY;
          apply_led_current_(led_red_ma_idle_, led_ir_ma_idle_);
          finger_present_ = false;
          if (finger_sensor_) finger_sensor_->publish_state(false);
          ESP_LOGI(TAG, "Finger removed → STANDBY");
          touch_release_ms_ = 0;
        }
      } else {
        touch_release_ms_ = 0;
      }
      break;

    default:
      state_ = DriverState::STANDBY;
      break;
  }
}

// ================================================================
//  SETUP
// ================================================================
void MAX30102CustomSensor::setup() {
  ESP_LOGI(TAG, "MAX30102 custom driver setup...");

  uint8_t part_id = 0;
  if (!read_reg_(REG_PART_ID, &part_id)) {
    ESP_LOGE(TAG, "Failed reading PART ID!");
  } else {
    ESP_LOGI(TAG, "Part ID: 0x%02X (expect 0x15)", part_id);
  }

  configure_sensor_();

  // Preallocate buffers
  ir_buf_.reserve(BUF_CAP);
  red_buf_.reserve(BUF_CAP);

  // Start in standby mode
  apply_led_current_(led_red_ma_idle_, led_ir_ma_idle_);
  state_ = DriverState::STANDBY;

  ESP_LOGI(TAG, "Driver initialized → STANDBY");
}

// ================================================================
//  CONFIGURE SENSOR REGISTERS
// ================================================================
void MAX30102CustomSensor::configure_sensor_() {
  // Reset
  write_reg_(REG_MODE_CONFIG, 0x40);
  delay(10);

  // FIFO config
  uint8_t avg_code = map_sample_avg_code(sample_average_);
  uint8_t fifo = (avg_code << 5) | (1 << 4) | 0x0F;  // rollover=1, full=15
  write_reg_(REG_FIFO_CONFIG, fifo);

  write_reg_(REG_FIFO_WR_PTR, 0x00);
  write_reg_(REG_OVF_COUNTER, 0x00);
  write_reg_(REG_FIFO_RD_PTR, 0x00);

  sample_rate_code_ = map_sample_rate_code(sample_rate_hz_);
  pulse_width_code_ = map_pulse_width_code(pulse_width_us_);
  adc_range_code_   = map_adc_range_code(adc_range_na_);

  uint8_t spo2 =
    (adc_range_code_ << 5) |
    (sample_rate_code_ << 2) |
     pulse_width_code_;

  write_reg_(REG_SPO2_CONFIG, spo2);

  // SpO2 mode (RED+IR)
  write_reg_(REG_MODE_CONFIG, 0x03);

  // Interrupts
  write_reg_(REG_INTR_ENABLE_1, 0xC0);
  write_reg_(REG_INTR_ENABLE_2, 0x00);

  ESP_LOGI(TAG,
    "Configured: rate=%u Hz, avg=%u, pw=%u us, range=%u nA",
    sample_rate_hz_, sample_average_, pulse_width_us_, adc_range_na_);
}

// ================================================================
//  FIFO READ ENGINE
// ================================================================
void MAX30102CustomSensor::read_fifo_() {
  uint8_t wr_ptr = 0, rd_ptr = 0;

  if (!read_reg_(REG_FIFO_WR_PTR, &wr_ptr)) return;
  if (!read_reg_(REG_FIFO_RD_PTR, &rd_ptr)) return;

  int16_t n_samples = (wr_ptr - rd_ptr) & 0x1F;
  if (n_samples <= 0)
    return;

  const size_t BYTES_PER_SAMPLE = 6;
  uint8_t buf[BYTES_PER_SAMPLE];

  for (int i = 0; i < n_samples; i++) {
    if (!burst_read_(REG_FIFO_DATA, buf, BYTES_PER_SAMPLE))
      return;

    uint32_t red_raw =
      ((((uint32_t)buf[0] << 16) |
        ((uint32_t)buf[1] << 8)  |
         (uint32_t)buf[2]) & 0x3FFFF);

    uint32_t ir_raw =
      ((((uint32_t)buf[3] << 16) |
        ((uint32_t)buf[4] << 8)  |
         (uint32_t)buf[5]) & 0x3FFFF);

    // update last raw for touch EWMA during measuring (ne jede FIFO posebno)
    last_raw_ir_ = static_cast<float>(ir_raw);
    if (state_ == DriverState::MEASURING) {
      const float a = 0.05f; // sporije, samo za touch heuristiku
      touch_ir_ewma_ = (1.0f - a) * touch_ir_ewma_ + a * last_raw_ir_;
    }

    // Median pipelines
    float red_f = update_median_(raw_red_win_, raw_red_idx_, raw_red_cnt_, (float)red_raw);
    float ir_f  = update_median_(raw_ir_win_,  raw_ir_idx_,  raw_ir_cnt_,  (float)ir_raw);

    // Push to buffers
    if (red_buf_.size() >= BUF_CAP) red_buf_.erase(red_buf_.begin());
    if (ir_buf_.size()  >= BUF_CAP) ir_buf_.erase(ir_buf_.begin());

    red_buf_.push_back(red_f);
    ir_buf_.push_back(ir_f);
  }
}

// ================================================================
//  SAMPLE PROCESSING PIPELINE
// ================================================================
void MAX30102CustomSensor::process_samples_() {
  if (!ir_buf_.empty() && ir_sensor_)
    ir_sensor_->publish_state(ir_buf_.back());

  if (!red_buf_.empty() && red_sensor_)
    red_sensor_->publish_state(red_buf_.back());
}

// ================================================================
//  HEART RATE & AC FILTER PIPELINE (per-sample)
// ================================================================
void MAX30102CustomSensor::compute_hr_spo2_() {
  if (ir_buf_.empty())
    return;

  // Obradi SVE nove uzorke od last_processed_idx_ do zadnjeg
  size_t start_idx = last_processed_idx_;
  if (start_idx > ir_buf_.size()) start_idx = 0;

  for (size_t i = start_idx; i < ir_buf_.size(); i++) {
    float x = ir_buf_[i];  // zadnji filtrirani IR sample (median5)

    // 1) DC removal via slow EWMA baseline
    float a = baseline_alpha_;
    if (a < 0.001f) a = 0.001f;
    if (a > 0.50f)  a = 0.50f;

    dc_ir_ = (1.0f - a) * dc_ir_ + a * x;
    float ac_raw = x - dc_ir_;

    // AC IIR smoothing
    float ac_f = iir_ac_(ac_raw);

    // 2) RMS amplitude for adaptive threshold
    ac_win_.push_back(ac_f);
    if (ac_win_.size() > ac_win_cap_)
      ac_win_.pop_front();

    float rms = 0.0f;
    if (!ac_win_.empty()) {
      for (float v : ac_win_) rms += v * v;
      rms = sqrtf(rms / ac_win_.size());
    }

    float thr = std::max(ac_min_amp_, rms * ac_rms_k_);

    // 3) Peak detection using rising edge + sample-based timing
    bool rising = ac_f > last_ir_;

    // Samples per beat bounds
    float min_ds_f = (60.0f * (float)sample_rate_hz_) / std::max(1, max_bpm_);
    float max_ds_f = (60.0f * (float)sample_rate_hz_) / std::max(1, min_bpm_);
    uint32_t min_ds = (uint32_t) std::floor(min_ds_f);
    uint32_t max_ds = (uint32_t) std::ceil (max_ds_f);
    if (min_ds < 1) min_ds = 1;

    if (!prev_rising_ && rising && std::fabs(ac_f) > thr) {
      if (last_peak_sample_ != 0) {
        uint32_t ds = sample_counter_ - last_peak_sample_;
        if (ds >= min_ds && ds <= max_ds) {
          float bpm_raw = (60.0f * (float)sample_rate_hz_) / (float)ds;
          float bpm_filt = iir_hr_(bpm_raw);

          if (hr_sensor_)
            hr_sensor_->publish_state(bpm_filt);

          // Main sensor reports BPM
          this->publish_state(bpm_filt);
        }
      }
      last_peak_sample_ = sample_counter_;
    }

    prev_rising_ = rising;
    last_ir_ = ac_f;

    // sample clock +
    sample_counter_++;
  }

  last_processed_idx_ = ir_buf_.size();

  // 4) SpO₂ pipeline (ratio of ratios) – kao prije, 50–200 uzoraka
  if (spo2_sensor_ && !red_buf_.empty()) {
    size_t N = std::min((size_t)200, std::min(ir_buf_.size(), red_buf_.size()));
    if (N >= 50) {
      size_t start = ir_buf_.size() - N;

      float ir_mean = 0, red_mean = 0;
      for (size_t i = start; i < ir_buf_.size(); i++) {
        ir_mean  += ir_buf_[i];
        red_mean += red_buf_[i];
      }
      ir_mean  /= N;
      red_mean /= N;

      // AC variances
      float ir_var = 0, red_var = 0;
      for (size_t i = start; i < ir_buf_.size(); i++) {
        float d_ir  = ir_buf_[i]  - ir_mean;
        float d_red = red_buf_[i] - red_mean;
        ir_var  += d_ir  * d_ir;
        red_var += d_red * d_red;
      }

      float ir_rms  = sqrtf(ir_var  / N);
      float red_rms = sqrtf(red_var / N);

      if (ir_rms > 0.1f && ir_mean > 1.0f && red_mean > 1.0f) {
        float R = (red_rms / red_mean) / (ir_rms / ir_mean);
        float spo2 = 110.0f - 25.0f * R;
        if (spo2 < 0)   spo2 = 0;
        if (spo2 > 100) spo2 = 100;

        float spo2_med = median_spo2_(spo2);
        float spo2_filt = iir_spo2_(spo2_med);

        spo2_sensor_->publish_state(spo2_filt);
      }
    }
  }
}

// ================================================================
//  UPDATE LOOP
// ================================================================
void MAX30102CustomSensor::update() {

  // 1) Touch detection & LED mode FSM (NE dira FIFO u MEASURING)
  update_touch_state_();

  // If not measuring, do NOT read FIFO or run HR/SpO2 DSP
  if (state_ != DriverState::MEASURING)
    return;

  // 2) Active measurement pipeline
  read_fifo_();
  process_samples_();
  compute_hr_spo2_();
}

// ================================================================
//  MEDIAN / IIR HELPERS
// ================================================================
float MAX30102CustomSensor::median5_compute_(const std::array<float, 5> &win,
                                             uint8_t cnt) {
  if (cnt == 0)
    return 0.0f;

  uint8_t n = cnt > 5 ? 5 : cnt;
  std::array<float, 5> tmp = win;

  std::sort(tmp.begin(), tmp.begin() + n);

  if (n == 1) return tmp[0];
  if (n == 2) return 0.5f * (tmp[0] + tmp[1]);

  return tmp[(n - 1) / 2];  // for 3,4,5 → middle element
}

float MAX30102CustomSensor::update_median_(std::array<float, 5> &win,
                                           uint8_t &idx,
                                           uint8_t &cnt,
                                           float sample) {
  win[idx] = sample;
  idx = (idx + 1) % 5;

  if (cnt < 5) cnt++;

  return median5_compute_(win, cnt);
}

// ================================================================
//  AC IIR FILTER
// ================================================================
float MAX30102CustomSensor::iir_ac_(float x) {
  ac_iir_state_ = ac_iir_a_ * ac_iir_state_ + (1.0f - ac_iir_a_) * x;
  return ac_iir_state_;
}

// ================================================================
//  HR IIR FILTER
// ================================================================
float MAX30102CustomSensor::iir_hr_(float x) {
  if (hr_iir_state_ == 0.0f)
    hr_iir_state_ = x;
  else
    hr_iir_state_ = hr_iir_a_ * hr_iir_state_ + (1.0f - hr_iir_a_) * x;

  return hr_iir_state_;
}

// ================================================================
//  SpO2 MEDIAN + IIR
// ================================================================
float MAX30102CustomSensor::median_spo2_(float x) {
  spo2_med_win_[spo2_med_idx_] = x;
  spo2_med_idx_ = (spo2_med_idx_ + 1) % 5;
  if (spo2_med_cnt_ < 5) spo2_med_cnt_++;

  return median5_compute_(spo2_med_win_, spo2_med_cnt_);
}

float MAX30102CustomSensor::iir_spo2_(float x) {
  if (spo2_iir_state_ == 0.0f)
    spo2_iir_state_ = x;
  else
    spo2_iir_state_ = spo2_iir_a_ * spo2_iir_state_ + (1.0f - spo2_iir_a_) * x;

  return spo2_iir_state_;
}

// ================================================================
//  END
// ================================================================

}  // namespace max30102_custom
}  // namespace esphome
