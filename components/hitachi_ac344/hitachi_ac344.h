#pragma once

#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/remote_base/remote_base.h"

namespace esphome::hitachi_ac344 {

const uint16_t HITACHI_AC344_HDR_MARK = 3300;
const uint16_t HITACHI_AC344_HDR_SPACE = 1700;
const uint16_t HITACHI_AC344_BIT_MARK = 400;
const uint16_t HITACHI_AC344_ONE_SPACE = 1250;
const uint16_t HITACHI_AC344_ZERO_SPACE = 500;

const uint8_t HITACHI_AC344_BUTTON_BYTE = 11;
const uint8_t HITACHI_AC344_BUTTON_POWER = 0x13;
const uint8_t HITACHI_AC344_BUTTON_MODE = 0x41;
const uint8_t HITACHI_AC344_BUTTON_FAN = 0x42;
const uint8_t HITACHI_AC344_BUTTON_TEMP_DOWN = 0x43;
const uint8_t HITACHI_AC344_BUTTON_TEMP_UP = 0x44;
const uint8_t HITACHI_AC344_BUTTON_SWINGV = 0x81;

const uint8_t HITACHI_AC344_TEMP_BYTE = 13;
const uint8_t HITACHI_AC344_TEMP_OFFSET = 2;
const uint8_t HITACHI_AC344_TEMP_SIZE = 6;
const uint8_t HITACHI_AC344_TEMP_MIN = 16;
const uint8_t HITACHI_AC344_TEMP_MAX = 32;

const uint8_t HITACHI_AC344_MODE_BYTE = 25;
const uint8_t HITACHI_AC344_MODE_FAN = 1;
const uint8_t HITACHI_AC344_MODE_COOL = 3;
const uint8_t HITACHI_AC344_MODE_DRY = 5;
const uint8_t HITACHI_AC344_MODE_HEAT = 6;

const uint8_t HITACHI_AC344_FAN_BYTE = HITACHI_AC344_MODE_BYTE;
const uint8_t HITACHI_AC344_FAN_MIN = 1;
const uint8_t HITACHI_AC344_FAN_QUIET = 1;
const uint8_t HITACHI_AC344_FAN_LOW = 2;
const uint8_t HITACHI_AC344_FAN_MEDIUM = 3;
const uint8_t HITACHI_AC344_FAN_HIGH = 4;
const uint8_t HITACHI_AC344_FAN_AUTO = 5;
const uint8_t HITACHI_AC344_FAN_MAX = 6;

const uint8_t HITACHI_AC344_POWER_BYTE = 27;
const uint8_t HITACHI_AC344_POWER_ON = 0xF1;
const uint8_t HITACHI_AC344_POWER_OFF = 0xE1;

const uint8_t HITACHI_AC344_SWINGH_BYTE = 35;
const uint8_t HITACHI_AC344_SWINGH_OFFSET = 0;
const uint8_t HITACHI_AC344_SWINGH_SIZE = 3;
const uint8_t HITACHI_AC344_SWINGH_AUTO = 0;

const uint8_t HITACHI_AC344_SWINGV_BYTE = 37;
const uint8_t HITACHI_AC344_SWINGV_OFFSET = 5;

const uint16_t HITACHI_AC344_STATE_LENGTH = 43;

#define GETBITS8(data, offset, size) (((data) & (((uint8_t) UINT8_MAX >> (8 - (size))) << (offset))) >> (offset))

class HitachiClimate final : public climate::Climate, public Component, public remote_base::RemoteReceiverListener {
 public:
  HitachiClimate();
  void setup() override;
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;
  bool on_receive(remote_base::RemoteReceiveData data) override;
  void set_repeat(uint8_t repeat) { this->repeat_ = repeat; }

 protected:
  uint8_t repeat_{1};
  uint8_t last_button_{HITACHI_AC344_BUTTON_TEMP_UP};
  uint32_t last_transmission_time_{0};
  uint32_t last_transmission_end_{0};
  uint32_t cooldown_until_{0};
  bool debounce_active_{false};
  void execute_transmit_();

  // Initialized with genuine template captured directly from physical remote (43 bytes HITACHI_AC344)
  uint8_t remote_state_[HITACHI_AC344_STATE_LENGTH]{
      0x01, 0x10, 0x00, 0x40, 0xBF, 0xFF, 0x00, 0xCC,
      0x33, 0xA9, 0x56, 0x44, 0xBB, 0x78, 0x87, 0x00,
      0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
      0xFF, 0x53, 0xAC, 0xF1, 0x0E, 0x06, 0xF9, 0x03,
      0xFC, 0x80, 0x7F, 0x00, 0xFF, 0x60, 0x9F, 0x00,
      0xFF, 0x9B, 0x64
  };
  uint8_t previous_temp_{26};

  void transmit_state();
};

} // namespace esphome::hitachi_ac344
