#include "max30102_custom.h"
#include "esphome/core/log.h"

namespace esphome {
namespace max30102_custom {

static const char *TAG = "max30102_custom";

// ---------------- Helperi: median & IIR ----------------
float MAX30102CustomSensor::median5_compute_(const std::array<float, 5> &win, uint8_t cnt) {
  uint8_t n = std::min<uint8_t>(cnt, 5);
  if (n == 0) return 0.0f;
  std::array<float, 5> tmp = win;
  std::sort(tmp.begin(), tmp.begin() + n);
  if (n == 1) return tmp[0];
  if (n == 2) return 0.5f * (tmp[0] + tmp[1]);
  return tmp[(n - 1) / 2];  // 3,4,5 -> srednji
}

float MAX30102CustomSensor::update_median_(std::array<float, 5> &win, uint8_t &idx, uint8_t &cnt, float sample) {
  win[idx] = sample;
  idx = static_cast<uint8_t>((idx + 1) % 5);
  if (cnt < 5) cnt++;
  return median5_compute_(win, cnt);
}

float MAX30102CustomSensor::iir_ac_(float x) {
  ac_iir_state_ = ac_iir_a_ * ac_iir_state_ + (1.0f - ac_iir_a_) * x;
  ac_win_.push_back(ac_iir_state_);
  if (ac_win_.size() > ac_win_cap_) ac_win_.pop_front();
  return ac_iir_state_;
}

float MAX30102CustomSensor::iir_hr_(float x) {
  hr_iir_state_ = (hr_iir_state_ == 0.0f) ? x : (hr_iir_a_ * hr_iir_state_ + (1.0f - hr_iir_a_) * x);
  return hr_iir_state_;
}

float MAX30102CustomSensor::median_spo2_(float x) {
  spo2_med_win_[spo2_med_idx_] = x;
  spo2_med_idx_ = static_cast<uint8_t>((spo2_med_idx_ + 1) % 5);
  if (spo2_med_cnt_ < 5) spo2_med_cnt_++;
  return median5_compute_(spo2_med_win_, spo2_med_cnt_);
}

float MAX30102CustomSensor::iir_spo2_(float x) {
  spo2_iir_state_ = (spo2_iir_state_ == 0.0f) ? x : (spo2_iir_a_ * spo2_iir_state_ + (1.0f - spo2_iir_a_) * x);
  return spo2_iir_state_;
}

// ---------------- I2C helperi ----------------
bool MAX30102CustomSensor::write_reg_(uint8_t reg, uint8_t val) { return this->write(reg, &val, 1); }
bool MAX30102CustomSensor::read_reg_(uint8_t reg, uint8_t *val) { return this->read(reg, val, 1); }
bool MAX30102CustomSensor::burst_read_(uint8_t reg, uint8_t *data, size_t len) { return this->read(reg, data, len); }

// ---------------- Konfiguracija MAX30102 ----------------
static inline uint8_t map_sample_rate_code(uint16_t hz) {
  // 50/100/200/400 Hz -> 0/1/2/3 (MAX30102 datasheet)
  if (hz >= 400) return 3;
  if (hz >= 200) return 2;
  if (hz >= 100) return 1;
  return 0; // 50 Hz
}

static inline uint8_t map_pulse_width_code(uint16_t us) {
  // 69/118/215/411 us -> 0/1/2/3
  if (us >= 411) return 3;
  if (us >= 215) return 2;
  if (us >= 118) return 1;
  return 0; // 69 us
}

static inline uint8_t map_adc_range_code(uint16_t na) {
  // 2048/4096/8192/16384 nA -> 0/1/2/3
  if (na >= 16384) return 3;
  if (na >= 8192)  return 2;
  if (na >= 4096)  return 1;
  return 0; // 2048 nA
}

static inline uint8_t map_sample_avg_code(uint8_t avg) {
  // 1/2/4/8/16 -> 0/1/2/3/4
  if (avg >= 16) return 4;
  if (avg >= 8)  return 3;
  if (avg >= 4)  return 2;
  if (avg >= 2)  return 1;
  return 0;
}

static inline uint8_t led_pa_from_ma(float ma) {
  // MAX30102: ~0..51 mA u koraku ~0.2 mA -> 0..255
  int v = static_cast<int>(std::round(ma / 0.2f));
  if (v < 0) v = 0; if (v > 255) v = 255;
  return static_cast<uint8_t>(v);
}

void MAX30102CustomSensor::configure_sensor_() {
  // Reset
  (void) write_reg_(REG_MODE_CONFIG, 0x40);
  delay(10);

  // FIFO config: sample avg + rollover + almost_full
  uint8_t avg_code = map_sample_avg_code(sample_average_);
  uint8_t fifo = static_cast<uint8_t>((avg_code << 5) | (1 << 4) | 0x0F); // rollover=1, almost_full=15
  (void) write_reg_(REG_FIFO_CONFIG, fifo);

  // FIFO pointers clear
  (void) write_reg_(REG_FIFO_WR_PTR, 0x00);
  (void) write_reg_(REG_OVF_COUNTER, 0x00);
  (void) write_reg_(REG_FIFO_RD_PTR, 0x00);

  // Map ljudskih postavki na kodove
  sample_rate_code_ = map_sample_rate_code(sample_rate_hz_);
  pulse_width_code_ = map_pulse_width_code(pulse_width_us_);
  adc_range_code_   = map_adc_range_code(adc_range_na_);

  // SpO2 config
  uint8_t spo2 = static_cast<uint8_t>((adc_range_code_ << 5) | (sample_rate_code_ << 2) | (pulse_width_code_));
  (void) write_reg_(REG_SPO2_CONFIG, spo2);

  // LED amplitude (RED/IR)
  uint8_t red_pa = led_pa_from_ma(led_red_ma_);
  uint8_t ir_pa  = led_pa_from_ma(led_ir_ma_);
  (void) write_reg_(REG_LED1_PA, red_pa);
  (void) write_reg_(REG_LED2_PA, ir_pa);

  // SpO2 mode
  (void) write_reg_(REG_MODE_CONFIG, 0x03);

  // Interrupts (opcionalno: PPG_RDY)
  (void) write_reg_(REG_INTR_ENABLE_1, 0x20);
  (void) write_reg_(REG_INTR_ENABLE_2, 0x00);

  ESP_LOGI(TAG, "Configured (rate=%uHz, avg=%u, pw=%uus, range=%uuna, LED R=%.1fmA IR=%.1fmA)",
           sample_rate_hz_, sample_average_, pulse_width_us_, adc_range_na_, led_red_ma_, led_ir_ma_);
}

// ---------------- ESPHome lifecycle ----------------
void MAX30102CustomSensor::setup() {
  // Ako nema I2C bus registriran u sensor.py, čitanje će failati.
  uint8_t part = 0;
  if (!read_reg_(REG_PART_ID, &part)) {
    ESP_LOGW(TAG, "I2C read failed (PART ID). Provjeri i2c.register_i2c_device u sensor.py");
  } else {
    ESP_LOGI(TAG, "Part ID: 0x%02X (expect 0x15)", part);
  }

  configure_sensor_();
  ir_buf_.reserve(BUF_CAP);
  red_buf_.reserve(BUF_CAP);
}

void MAX30102CustomSensor::read_fifo_() {
  uint8_t wr = 0, rd = 0;
  if (!read_reg_(REG_FIFO_WR_PTR, &wr)) return;
  if (!read_reg_(REG_FIFO_RD_PTR, &rd)) return;
  int16_t num = (wr - rd) & 0x1F; // 5-bit pointeri
  if (num <= 0) return;

  const size_t BYTES_PER_SAMPLE = 6; // RED(3) + IR(3)
  uint8_t buf[BYTES_PER_SAMPLE];

  for (int i = 0; i < num; i++) {
    if (!burst_read_(REG_FIFO_DATA, buf, BYTES_PER_SAMPLE)) break;

    uint32_t red = (((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2]) & 0x3FFFF;
    uint32_t ir  = (((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5]) & 0x3FFFF;

    // RAW median (stable mode) prije spremanja u glavne buffere
    float red_f = update_median_(raw_red_win_, raw_red_idx_, raw_red_cnt_, (float) red);
    float ir_f  = update_median_(raw_ir_win_,  raw_ir_idx_,  raw_ir_cnt_,  (float) ir);

    if (red_buf_.size() >= BUF_CAP) red_buf_.erase(red_buf_.begin());
    if (ir_buf_.size()  >= BUF_CAP)  ir_buf_.erase(ir_buf_.begin());
    red_buf_.push_back(red_f);
    ir_buf_.push_back(ir_f);
  }
}

void MAX30102CustomSensor::process_samples_() {
  if (ir_sensor_ && !ir_buf_.empty())
    ir_sensor_->publish_state(ir_buf_.back());
  if (red_sensor_ && !red_buf_.empty())
    red_sensor_->publish_state(red_buf_.back());

  // Jednostavna detekcija prsta na temelju DC IR (s histerezom)
  if (!ir_buf_.empty()) {
    // EWMA za DC korišten i u compute_hr_spo2_, ovdje gruba detekcija:
    float sample = ir_buf_.back();
    // Za detekciju koristimo usporedbu s pragom (finger_thr_) i histerezom
    bool next = finger_present_;
    if (!finger_present_) {
      if (sample > finger_thr_) next = true;
    } else {
      if (sample < finger_thr_ * finger_hyst_) next = false;
    }
    if (next != finger_present_) {
      finger_present_ = next;
      if (finger_sensor_ != nullptr) finger_sensor_->publish_state(finger_present_);
    }
  }
}

void MAX30102CustomSensor::compute_hr_spo2_() {
  // --- DC uklanjanje (EWMA s bazeline_alpha_) + IIR na AC ---
  if (!ir_buf_.empty()) {
    float x = ir_buf_.back(); // RAW (median) IR
    float a = baseline_alpha_; if (a < 0.001f) a = 0.001f; if (a > 0.5f) a = 0.5f;
    dc_ir_ = (1.0f - a) * dc_ir_ + a * x;  // spor DC
    float ac_ir = x - dc_ir_;
    float ac_f  = iir_ac_(ac_ir);

    // Adaptivni prag iz RMS-a
    float rms = 0.0f;
    if (!ac_win_.empty()) {
      for (float v : ac_win_) rms += v * v;
      rms = sqrtf(rms / (float) ac_win_.size());
    }
    float thr = std::max(ac_min_amp_, ac_rms_k_ * rms);

    // Peak detekcija (rising edge + debounce)
    bool rising = (ac_f > last_ir_);
    uint32_t now = millis();
    if (!prev_rising_ && rising && last_ir_ > 0) {
      if (fabsf(ac_f) > thr) {
        if (last_peak_ms_ != 0) {
          uint32_t dt = now - last_peak_ms_;
          // Granice: min/max BPM iz YAML-a
          float min_ms = 60000.0f / std::max(1, max_bpm_);
          float max_ms = 60000.0f / std::max(1, min_bpm_);
          if (dt >= (uint32_t)min_ms && dt <= (uint32_t)max_ms && hr_sensor_) {
            float bpm_raw = 60000.0f / (float) dt;
            float bpm = iir_hr_(bpm_raw);
            hr_sensor_->publish_state(bpm);
            this->publish_state(bpm); // glavni senzor: HR
          }
        }
        last_peak_ms_ = now;
      }
    }
    prev_rising_ = rising;
    last_ir_ = ac_f;
  }

  // --- SpO2: RMS ratio-of-ratios + median(5) + IIR(0.85) ---
  if (spo2_sensor_ && !ir_buf_.empty() && !red_buf_.empty()) {
    size_t N = std::min<size_t>(200, std::min(ir_buf_.size(), red_buf_.size())); // ~2s @100Hz
    if (N >= 50) {
      float mean_ir = 0, mean_red = 0;
      size_t start = ir_buf_.size() - N;
      for (size_t i = start; i < ir_buf_.size(); ++i) { mean_ir += ir_buf_[i]; mean_red += red_buf_[i]; }
      mean_ir /= N; mean_red /= N;

      float ac2_ir = 0, ac2_red = 0;
      for (size_t i = start; i < ir_buf_.size(); ++i) {
        float dir  = ir_buf_[i]  - mean_ir;
        float dred = red_buf_[i] - mean_red;
        ac2_ir  += dir  * dir;
        ac2_red += dred * dred;
      }
      float rms_ir  = sqrtf(ac2_ir  / N);
      float rms_red = sqrtf(ac2_red / N);

      if (mean_ir > 1.0f && mean_red > 1.0f && rms_ir > 0.1f) {
        float R = (rms_red / mean_red) / (rms_ir / mean_ir);
        // Blaga korekcija (ostavljamo linearnu aproks. za stabilnost)
        float spo2_raw = 110.0f - 25.0f * R;
        if (spo2_raw < 0) spo2_raw = 0; if (spo2_raw > 100) spo2_raw = 100;

        float spo2_m = median_spo2_(spo2_raw);
        float spo2_s = iir_spo2_(spo2_m);
        spo2_sensor_->publish_state(spo2_s);
      }
    }
  }
}

void MAX30102CustomSensor::update() {
  read_fifo_();
  process_samples_();   // objavi IR/RED; ažuriraj finger
  compute_hr_spo2_();   // HR + SpO₂ stabilizirani izlazi
}

}  // namespace max30102_custom
}  // namespace esphome// placeholder cpp
