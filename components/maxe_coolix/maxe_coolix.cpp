#include "maxe_coolix.h"
#include "s06_hitachi_hw.h"

namespace esphome::maxe_coolix {

static const char *const TAG = "climate.maxe_coolix";

static const uint8_t COOLIX_TEMP_CODES[14] = {
    0x0, // 17
    0x1, // 18
    0x3, // 19
    0x2, // 20
    0x6, // 21
    0x7, // 22
    0x5, // 23
    0x4, // 24
    0xC, // 25
    0xD, // 26
    0x9, // 27
    0x8, // 28
    0xA, // 29
    0xB  // 30
};

static uint8_t temp_to_code(uint8_t temp) {
  if (temp < 17) temp = 17;
  if (temp > 30) temp = 30;
  return COOLIX_TEMP_CODES[temp - 17];
}

static uint8_t code_to_temp(uint8_t code) {
  for (uint8_t i = 0; i < 14; i++) {
    if (COOLIX_TEMP_CODES[i] == code) return 17 + i;
  }
  return 25;
}

MaxeCoolixClimate::MaxeCoolixClimate() {
  this->target_temperature = 25;
  this->mode = climate::CLIMATE_MODE_OFF;
  this->fan_mode = climate::CLIMATE_FAN_AUTO;
}

void MaxeCoolixClimate::setup() {
  s06_hw::ensure_pwm_init();
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
  }
}

climate::ClimateTraits MaxeCoolixClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_visual_min_temperature(MAXE_TEMP_MIN);
  traits.set_visual_max_temperature(MAXE_TEMP_MAX);
  traits.set_visual_temperature_step(1.0f);
  traits.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_AUTO,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_DRY,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_FAN_ONLY,
  });
  traits.set_supported_fan_modes({
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH,
  });
  return traits;
}

void MaxeCoolixClimate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value())
    this->mode = *call.get_mode();
  if (call.get_target_temperature().has_value())
    this->target_temperature = *call.get_target_temperature();
  if (call.get_fan_mode().has_value())
    this->fan_mode = *call.get_fan_mode();

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

void MaxeCoolixClimate::transmit_state() {
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

void MaxeCoolixClimate::execute_transmit_() {
  uint8_t bytes[6] = {0};
  bytes[0] = 0xB2;

  if (this->mode == climate::CLIMATE_MODE_OFF) {
    bytes[2] = 0x7B;
    bytes[4] = 0xE0;
  } else {
    // 1. Fan mode (Byte 2)
    if (this->mode == climate::CLIMATE_MODE_DRY) {
      bytes[2] = 0x1F;
    } else {
      auto fan = this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO);
      switch (fan) {
        case climate::CLIMATE_FAN_LOW:
          bytes[2] = 0x9F;
          break;
        case climate::CLIMATE_FAN_MEDIUM:
          bytes[2] = 0x5F;
          break;
        case climate::CLIMATE_FAN_HIGH:
          bytes[2] = 0x3F;
          break;
        case climate::CLIMATE_FAN_AUTO:
        default:
          bytes[2] = 0xBF;
          break;
      }
    }

    // 2. Mode & Temp (Byte 4)
    uint8_t t_code = temp_to_code((uint8_t) this->target_temperature);
    switch (this->mode) {
      case climate::CLIMATE_MODE_COOL:
        bytes[4] = (t_code << 4) | 0x0;
        break;
      case climate::CLIMATE_MODE_HEAT:
        bytes[4] = (t_code << 4) | 0xC;
        break;
      case climate::CLIMATE_MODE_AUTO:
      case climate::CLIMATE_MODE_HEAT_COOL:
        bytes[4] = (t_code << 4) | 0x8;
        break;
      case climate::CLIMATE_MODE_DRY:
        bytes[4] = (t_code << 4) | 0x4;
        break;
      case climate::CLIMATE_MODE_FAN_ONLY:
        bytes[4] = (0xE << 4) | 0x4;
        break;
      default:
        bytes[4] = (t_code << 4) | 0x0;
        break;
    }
  }

  // 3. Inverted checksum bytes
  bytes[1] = bytes[0] ^ 0xFF;
  bytes[3] = bytes[2] ^ 0xFF;
  bytes[5] = bytes[4] ^ 0xFF;

  ESP_LOGI(TAG, "TX MAXE Coolix command via Hardware PWM: %02X %02X %02X %02X %02X %02X",
           bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5]);
  s06_hw::send_coolix_bytes(bytes, 6, 2);
  this->last_transmission_end_ = millis();
  this->cooldown_until_ = millis() + 300; // 300ms 盲區防撞
  this->publish_state();
}

// Decode 48-bit Coolix payload from a specific offset in receive buffer
static bool try_decode_coolix_at(const remote_base::RemoteReceiveData &data, size_t start_idx, uint8_t *out_bytes) {
  uint8_t recv[6] = {0};
  size_t cur_byte = 0;
  int cur_bit = 7;
  size_t idx = start_idx;

  while (idx + 1 < data.size() && cur_byte < 6) {
    int mark = data[idx];
    int space = data[idx + 1];
    if (mark < 120 || mark > 1200 || space >= 0) break;
    int space_us = std::abs(space);
    if (space_us > 850 && space_us <= 2400) {
      recv[cur_byte] |= (1 << cur_bit); // bit 1 (~1620us)
    } else if (space_us >= 150 && space_us <= 850) {
      // bit 0 (~540us)
    } else {
      break;
    }
    cur_bit--;
    if (cur_bit < 0) {
      cur_bit = 7;
      cur_byte++;
    }
    idx += 2;
  }

  if (cur_byte < 6) return false;

  // Validate Coolix header byte 0xB2 and 3 checksum pairs
  if (recv[0] != 0xB2) return false;
  if ((recv[0] ^ recv[1]) != 0xFF || (recv[2] ^ recv[3]) != 0xFF || (recv[4] ^ recv[5]) != 0xFF) {
    return false;
  }

  memcpy(out_bytes, recv, 6);
  return true;
}

bool MaxeCoolixClimate::on_receive(remote_base::RemoteReceiveData data) {
  // Echo suppression: 忽略發射結束後 250ms 內的自發自收或機殼反射
  if (millis() - this->last_transmission_end_ < 250) {
    return false;
  }

  if (data.size() < 20) return false;

  ESP_LOGD(TAG, "IR RX: %zu pulses, d[0]=%d, d[1]=%d", data.size(), data[0], data.size() > 1 ? data[1] : 0);

  // Scan through buffer to find any Coolix header (Mark ~4400us, Space ~4400us)
  uint8_t recv[6] = {0};
  bool decoded = false;

  for (size_t i = 0; i + 1 < data.size() && i < 120; i++) {
    int m = data[i];
    int s = std::abs(data[i + 1]);
    // Leader header check: Mark 2600~6000us, Space 2600~6000us
    if (m >= 2600 && m <= 6000 && data[i + 1] < 0 && s >= 2600 && s <= 6000) {
      if (try_decode_coolix_at(data, i + 2, recv)) {
        decoded = true;
        break;
      }
    }
  }

  if (!decoded) {
    return false;
  }

  ESP_LOGI(TAG, "Successfully decoded MAXE Coolix frame: %02X %02X %02X %02X %02X %02X",
           recv[0], recv[1], recv[2], recv[3], recv[4], recv[5]);

  if (recv[2] == 0x7B && recv[4] == 0xE0) {
    this->mode = climate::CLIMATE_MODE_OFF;
    ESP_LOGI(TAG, "RC-62 remote: OFF");
  } else {
    uint8_t m_code = recv[4] & 0x0F;
    uint8_t t_code = (recv[4] >> 4) & 0x0F;
    if (m_code == 0x0) {
      this->mode = climate::CLIMATE_MODE_COOL;
      this->target_temperature = code_to_temp(t_code);
    } else if (m_code == 0xC) {
      this->mode = climate::CLIMATE_MODE_HEAT;
      this->target_temperature = code_to_temp(t_code);
    } else if (m_code == 0x8) {
      this->mode = climate::CLIMATE_MODE_AUTO;
      this->target_temperature = code_to_temp(t_code);
    } else if (m_code == 0x4 && t_code == 0xE) {
      this->mode = climate::CLIMATE_MODE_FAN_ONLY;
    } else if (m_code == 0x4) {
      this->mode = climate::CLIMATE_MODE_DRY;
      this->target_temperature = code_to_temp(t_code);
    }

    uint8_t f_code = recv[2];
    if ((f_code & 0xE0) == 0xA0 || f_code == 0xBF) {
      this->fan_mode = climate::CLIMATE_FAN_AUTO;
    } else if ((f_code & 0xE0) == 0x80 || f_code == 0x9F) {
      this->fan_mode = climate::CLIMATE_FAN_LOW;
    } else if ((f_code & 0xE0) == 0x40 || f_code == 0x5F) {
      this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
    } else if ((f_code & 0xE0) == 0x20 || f_code == 0x3F) {
      this->fan_mode = climate::CLIMATE_FAN_HIGH;
    }
    ESP_LOGI(TAG, "RC-62 remote updated: Mode=%d, Temp=%.0f, Fan=%d",
             (int)this->mode, this->target_temperature, (int)this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO));
  }

  this->publish_state();
  return true;
}

} // namespace esphome::maxe_coolix
