#include "hitachi_ac344.h"
#include "s06_hitachi_hw.h"

namespace esphome::hitachi_ac344 {

static const char *const TAG = "climate.hitachi_ac344";

static void set_bits(uint8_t *const dst, const uint8_t offset, const uint8_t nbits, const uint8_t data) {
  if (offset >= 8 || !nbits)
    return;
  uint8_t mask = UINT8_MAX >> (8 - ((nbits > 8) ? 8 : nbits));
  *dst &= ~(uint8_t) (mask << offset);
  *dst |= ((data & mask) << offset);
}

static uint8_t *invert_byte_pairs(uint8_t *ptr, const uint16_t length) {
  for (uint16_t i = 1; i < length; i += 2) {
    uint8_t inv = ~*(ptr + i - 1);
    *(ptr + i) = inv;
  }
  return ptr;
}

HitachiClimate::HitachiClimate() {
  this->previous_temp_ = 26;
  this->target_temperature = 26;
  this->mode = climate::CLIMATE_MODE_OFF;
  this->fan_mode = climate::CLIMATE_FAN_AUTO;
  this->swing_mode = climate::CLIMATE_SWING_OFF;
}

void HitachiClimate::setup() {
  s06_hw::ensure_pwm_init();
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
    this->previous_temp_ = (uint8_t) this->target_temperature;
  }
}

climate::ClimateTraits HitachiClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_visual_min_temperature(HITACHI_AC344_TEMP_MIN);
  traits.set_visual_max_temperature(HITACHI_AC344_TEMP_MAX);
  traits.set_visual_temperature_step(1.0f);
  traits.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_DRY,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_FAN_ONLY,
  });
  traits.set_supported_fan_modes({
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_QUIET,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH,
  });
  traits.set_supported_swing_modes({
      climate::CLIMATE_SWING_OFF,
      climate::CLIMATE_SWING_VERTICAL,
      climate::CLIMATE_SWING_HORIZONTAL,
      climate::CLIMATE_SWING_BOTH,
  });
  return traits;
}

void HitachiClimate::control(const climate::ClimateCall &call) {
  bool power_changed = false;
  if (call.get_mode().has_value()) {
    auto new_mode = *call.get_mode();
    if (new_mode != this->mode) {
      if (new_mode == climate::CLIMATE_MODE_OFF || this->mode == climate::CLIMATE_MODE_OFF) {
        this->last_button_ = HITACHI_AC344_BUTTON_POWER;
        power_changed = true;
      } else {
        this->last_button_ = HITACHI_AC344_BUTTON_MODE;
      }
      this->mode = new_mode;
    }
  }
  if (call.get_target_temperature().has_value()) {
    float new_temp = *call.get_target_temperature();
    if (!power_changed) {
      if (new_temp > this->target_temperature) {
        this->last_button_ = HITACHI_AC344_BUTTON_TEMP_UP;
      } else if (new_temp < this->target_temperature) {
        this->last_button_ = HITACHI_AC344_BUTTON_TEMP_DOWN;
      } else {
        if (this->last_button_ != HITACHI_AC344_BUTTON_TEMP_UP && this->last_button_ != HITACHI_AC344_BUTTON_TEMP_DOWN) {
          this->last_button_ = HITACHI_AC344_BUTTON_TEMP_DOWN;
        }
      }
    }
    this->target_temperature = new_temp;
  }
  if (call.get_fan_mode().has_value()) {
    auto new_fan = *call.get_fan_mode();
    if (new_fan != this->fan_mode) {
      if (!power_changed) {
        this->last_button_ = HITACHI_AC344_BUTTON_FAN;
      }
      this->fan_mode = new_fan;
    }
  }
  if (call.get_swing_mode().has_value()) {
    auto new_swing = *call.get_swing_mode();
    if (new_swing != this->swing_mode) {
      if (!power_changed) {
        this->last_button_ = HITACHI_AC344_BUTTON_SWINGV;
      }
      this->swing_mode = new_swing;
    }
  }

  this->publish_state();

  // 250ms 人機工學防抖窗口 (Debounce Window)
  this->debounce_active_ = true;
  this->cancel_timeout("cooldown_retry");
  this->cancel_timeout("debounce_transmit");
  this->set_timeout("debounce_transmit", 250, [this]() {
    this->debounce_active_ = false;
    this->transmit_state();
  });
}

void HitachiClimate::transmit_state() {
  if (this->debounce_active_) return;

  uint32_t now = millis();
  int32_t remaining_ms = (int32_t)(this->cooldown_until_ - now);
  if (remaining_ms > 0) {
    ESP_LOGD(TAG, "In AC refractory window, deferring TX by %d ms", (int)remaining_ms);
    this->cancel_timeout("cooldown_retry");
    this->set_timeout("cooldown_retry", (uint32_t)remaining_ms + 20, [this]() {
      this->transmit_state();
    });
    return;
  }

  this->execute_transmit_();
}

void HitachiClimate::execute_transmit_() {
  // 客廳日立變頻冷氣經真機 1.5 公尺抓包解碼：
  // 1. 物理長度為 43 bytes (344 bits)，單幀發射時長 405ms + 40ms 尾部靜默。
  // 2. 原廠遙控器為嚴格單幀 (repeat: 1)，單次發射立即響應，根除連嗶兩聲。
  // 3. 尾部 Byte 41 動態校驗：Checksum = Checksum_Base - (temp * 2)
  uint8_t temp = (uint8_t) std::min((float)HITACHI_AC344_TEMP_MAX, std::max((float)HITACHI_AC344_TEMP_MIN, this->target_temperature));

  if (this->mode == climate::CLIMATE_MODE_OFF) {
    remote_state_[HITACHI_AC344_POWER_BYTE] = HITACHI_AC344_POWER_OFF;
    remote_state_[HITACHI_AC344_BUTTON_BYTE] = HITACHI_AC344_BUTTON_POWER;
    this->last_button_ = HITACHI_AC344_BUTTON_POWER;
  } else {
    remote_state_[HITACHI_AC344_POWER_BYTE] = HITACHI_AC344_POWER_ON;
    if (this->last_button_ != HITACHI_AC344_BUTTON_POWER && this->last_button_ != HITACHI_AC344_BUTTON_MODE && 
        this->last_button_ != HITACHI_AC344_BUTTON_FAN && this->last_button_ != HITACHI_AC344_BUTTON_SWINGV) {
      if (this->target_temperature > this->previous_temp_) {
        this->last_button_ = HITACHI_AC344_BUTTON_TEMP_UP;
      } else if (this->target_temperature < this->previous_temp_) {
        this->last_button_ = HITACHI_AC344_BUTTON_TEMP_DOWN;
      }
    }
    remote_state_[HITACHI_AC344_BUTTON_BYTE] = this->last_button_;

    uint8_t mode_val = HITACHI_AC344_MODE_COOL;
    switch (this->mode) {
      case climate::CLIMATE_MODE_COOL: mode_val = HITACHI_AC344_MODE_COOL; break;
      case climate::CLIMATE_MODE_DRY:  mode_val = HITACHI_AC344_MODE_DRY; break;
      case climate::CLIMATE_MODE_HEAT: mode_val = HITACHI_AC344_MODE_HEAT; break;
      case climate::CLIMATE_MODE_FAN_ONLY: mode_val = HITACHI_AC344_MODE_FAN; break;
      default: mode_val = HITACHI_AC344_MODE_COOL; break;
    }
    set_bits(&remote_state_[HITACHI_AC344_MODE_BYTE], 0, 4, mode_val);

    auto fan = this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO);
    uint8_t fan_val = HITACHI_AC344_FAN_AUTO;
    switch (fan) {
      case climate::CLIMATE_FAN_QUIET: fan_val = HITACHI_AC344_FAN_QUIET; break;
      case climate::CLIMATE_FAN_LOW:    fan_val = HITACHI_AC344_FAN_LOW; break;
      case climate::CLIMATE_FAN_MEDIUM: fan_val = HITACHI_AC344_FAN_MEDIUM; break;
      case climate::CLIMATE_FAN_HIGH:   fan_val = HITACHI_AC344_FAN_HIGH; break;
      default: fan_val = HITACHI_AC344_FAN_AUTO; break;
    }
    set_bits(&remote_state_[HITACHI_AC344_FAN_BYTE], 4, 4, fan_val);
  }

  // 設置溫度
  set_bits(&remote_state_[HITACHI_AC344_TEMP_BYTE], HITACHI_AC344_TEMP_OFFSET, HITACHI_AC344_TEMP_SIZE, temp);

  // 原廠 43 bytes 真機標準後綴
  remote_state_[29] = 0x06;
  remote_state_[31] = 0x03;
  remote_state_[33] = 0x80;
  remote_state_[35] = 0x00; // SwingH
  remote_state_[37] = 0x60; // SwingV
  remote_state_[39] = 0x00;

  // 動態校驗計算（嚴格區分關機 0x77 與開機/調溫/模式 0xD7/0xDE，杜絕開關機碰撞 Bug）
  uint8_t chk_base = 0xDE;
  if (this->mode == climate::CLIMATE_MODE_OFF) {
    chk_base = 0x77; // 關機專用 (Byte 27 = 0xE1)
  } else if (this->last_button_ == HITACHI_AC344_BUTTON_TEMP_UP) {
    chk_base = 0xD7;
  } else {
    chk_base = 0xDE; // 開機、降溫、風速、模式切換預設基數 (Byte 27 = 0xF1)
  }
  remote_state_[41] = (uint8_t)(chk_base - (temp * 2));

  // 生成 43 bytes 狀態幀（Byte 3 到 Byte 42 進行 20 對按位取反校驗）
  invert_byte_pairs(remote_state_ + 3, 40);

  ESP_LOGI(TAG, "TX Hitachi AC (43 bytes, repeat=%u): Mode=%d, Temp=%.0f, Fan=%d, Button=0x%02X, Power=0x%02X, Chk=0x%02X (~0x%02X)",
           this->repeat_,
           (int)this->mode, this->target_temperature, (int)this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO),
           remote_state_[HITACHI_AC344_BUTTON_BYTE], remote_state_[HITACHI_AC344_POWER_BYTE],
           remote_state_[41], remote_state_[42]);

  this->last_transmission_time_ = millis();
  s06_hw::send_hitachi_344_bytes(remote_state_, HITACHI_AC344_STATE_LENGTH, this->repeat_);
  this->last_transmission_end_ = millis();

  // 發射完畢後設定 300ms 安全隔離期防撞
  this->cooldown_until_ = millis() + 300;
  this->previous_temp_ = (uint8_t) this->target_temperature;
  this->publish_state();
}

bool HitachiClimate::on_receive(remote_base::RemoteReceiveData data) {
  // Echo suppression: 忽略發射結束後 250ms 內的自發自收或機殼反射
  if (millis() - this->last_transmission_end_ < 250) {
    return false;
  }

  if (data.size() < 60) return false;

  size_t idx = 0;
  if (data[0] > 2000 && data[0] < 4500 && data[1] < 0 && std::abs(data[1]) > 1000 && std::abs(data[1]) < 2500) {
    idx = 2;
  } else if (data[0] < 0 && std::abs(data[0]) > 1000 && std::abs(data[0]) < 2500) {
    idx = 1;
  } else {
    return false;
  }

  uint8_t recv_state[HITACHI_AC344_STATE_LENGTH] = {0};
  size_t cur_byte = 0;
  uint8_t cur_bit = 0;

  while (idx + 1 < data.size() && cur_byte < HITACHI_AC344_STATE_LENGTH) {
    int mark = data[idx];
    int space = data[idx + 1];
    if (mark < 100 || mark > 900 || space >= 0) break;
    int space_us = std::abs(space);
    // 0 Space ~500us, 1 Space ~1250us；設定 900us 門檻避免長幀尾部 AGC 飽和拉伸 (711~789us) 誤判為 1
    if (space_us > 900 && space_us <= 2000) {
      recv_state[cur_byte] |= (1 << cur_bit);
    } else if (space_us < 200) {
      break;
    }
    cur_bit++;
    if (cur_bit >= 8) {
      cur_bit = 0;
      cur_byte++;
    }
    idx += 2;
  }

  if (cur_byte < HITACHI_AC344_STATE_LENGTH) return false;

  // 驗證標頭 (0x01, 0x10)
  if (recv_state[0] != 0x01 || recv_state[1] != 0x10) return false;

  // 驗證 20 對取反結構
  for (size_t i = 3; i + 1 < cur_byte; i += 2) {
    if ((recv_state[i] ^ recv_state[i + 1]) != 0xFF) {
      ESP_LOGW(TAG, "Checksum mismatch at pair %zu", i);
      return false;
    }
  }

  ESP_LOGI(TAG, "Decoded valid 43 bytes from Hitachi remote! Button=0x%02X, Temp=0x%02X, Mode=0x%02X, Power=0x%02X, Chk=0x%02X",
           recv_state[HITACHI_AC344_BUTTON_BYTE], recv_state[HITACHI_AC344_TEMP_BYTE],
           recv_state[HITACHI_AC344_MODE_BYTE], recv_state[HITACHI_AC344_POWER_BYTE],
           recv_state[41]);

  // 解析開關與模式
  uint8_t power = recv_state[HITACHI_AC344_POWER_BYTE];
  if (power == HITACHI_AC344_POWER_ON) {
    uint8_t mode = recv_state[HITACHI_AC344_MODE_BYTE] & 0x0F;
    switch (mode) {
      case HITACHI_AC344_MODE_COOL: this->mode = climate::CLIMATE_MODE_COOL; break;
      case HITACHI_AC344_MODE_DRY:  this->mode = climate::CLIMATE_MODE_DRY; break;
      case HITACHI_AC344_MODE_HEAT: this->mode = climate::CLIMATE_MODE_HEAT; break;
      case HITACHI_AC344_MODE_FAN:  this->mode = climate::CLIMATE_MODE_FAN_ONLY; break;
      default: this->mode = climate::CLIMATE_MODE_COOL; break;
    }
  } else {
    this->mode = climate::CLIMATE_MODE_OFF;
  }

  // 解析溫度
  uint8_t temp = GETBITS8(recv_state[HITACHI_AC344_TEMP_BYTE], HITACHI_AC344_TEMP_OFFSET, HITACHI_AC344_TEMP_SIZE);
  if (temp >= HITACHI_AC344_TEMP_MIN && temp <= HITACHI_AC344_TEMP_MAX) {
    this->target_temperature = temp;
    this->previous_temp_ = temp;
  }

  // 解析風速
  uint8_t fan = (recv_state[HITACHI_AC344_FAN_BYTE] >> 4) & 0x0F;
  switch (fan) {
    case HITACHI_AC344_FAN_QUIET:  this->fan_mode = climate::CLIMATE_FAN_QUIET; break;
    case HITACHI_AC344_FAN_LOW:    this->fan_mode = climate::CLIMATE_FAN_LOW; break;
    case HITACHI_AC344_FAN_MEDIUM: this->fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
    case HITACHI_AC344_FAN_HIGH:   this->fan_mode = climate::CLIMATE_FAN_HIGH; break;
    default: this->fan_mode = climate::CLIMATE_FAN_AUTO; break;
  }

  // 解析擺風
  if (cur_byte >= 38) {
    bool swing_v = (recv_state[HITACHI_AC344_SWINGV_BYTE] >> HITACHI_AC344_SWINGV_OFFSET) & 1;
    uint8_t swing_h = GETBITS8(recv_state[HITACHI_AC344_SWINGH_BYTE], HITACHI_AC344_SWINGH_OFFSET, HITACHI_AC344_SWINGH_SIZE);
    if (swing_v && swing_h == HITACHI_AC344_SWINGH_AUTO) {
      this->swing_mode = climate::CLIMATE_SWING_BOTH;
    } else if (swing_v) {
      this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
    } else if (swing_h == HITACHI_AC344_SWINGH_AUTO) {
      this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
    } else {
      this->swing_mode = climate::CLIMATE_SWING_OFF;
    }
  }

  // 保存接收狀態為下次基底
  for (size_t i = 0; i < cur_byte; i++) {
    this->remote_state_[i] = recv_state[i];
  }

  ESP_LOGI(TAG, "Synchronized state from remote: mode=%d, temp=%.0f, fan=%d, swing=%d",
           (int)this->mode, this->target_temperature, (int)this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO), (int)this->swing_mode);

  this->publish_state();
  return true;
}

} // namespace esphome::hitachi_ac344
