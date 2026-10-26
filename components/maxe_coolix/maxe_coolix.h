#pragma once

#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/remote_base/remote_base.h"

namespace esphome::maxe_coolix {

const uint8_t MAXE_TEMP_MIN = 17;
const uint8_t MAXE_TEMP_MAX = 30;

class MaxeCoolixClimate final : public climate::Climate, public Component, public remote_base::RemoteReceiverListener {
 public:
  MaxeCoolixClimate();
  void setup() override;
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;
  bool on_receive(remote_base::RemoteReceiveData data) override;

 protected:
  uint32_t last_transmission_end_{0};
  uint32_t cooldown_until_{0};
  bool debounce_active_{false};
  void transmit_state();
  void execute_transmit_();
};

} // namespace esphome::maxe_coolix
