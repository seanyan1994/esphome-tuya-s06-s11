#include "tuya_rf.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "radio.h"
#ifdef USE_LIBRETINY

extern void s11_check_blind_rf_rx(const std::vector<int32_t> &data) __attribute__((weak));
extern bool s11_on_pulse_edge(uint32_t delta, bool level) __attribute__((weak));
extern void s11_process_pending_action() __attribute__((weak));

namespace esphome {
namespace tuya_rf {

static const char *const TAG = "tuya_rf";

static void IRAM_ATTR HOT write_value(RemoteReceiverComponentStore *arg, uint32_t delta, bool level) {
  int32_t multiplier = ((int32_t) level << 1) - 1;
  uint32_t buffer_write = arg->buffer_write;
  arg->buffer[buffer_write++] = (int32_t) delta * multiplier;
  if (buffer_write >= arg->buffer_size) {
    buffer_write = 0;
  }

  if (buffer_write == arg->buffer_read) {
    buffer_write = arg->buffer_start;
    arg->overflow = true;
  }

  if (delta >= arg->idle_us) {
    if (arg->buffer_write == arg->buffer_start) {
      buffer_write = arg->buffer_start;
    } else {
      arg->buffer_start = buffer_write;
    }
  }
  arg->buffer_write = buffer_write;
}

static void IRAM_ATTR HOT commit_value(RemoteReceiverComponentStore *arg, uint32_t micros, bool level) {
  if (level != arg->commit_level) {
    write_value(arg, micros - arg->commit_micros, level);
    arg->commit_micros = micros;
    arg->commit_level = level;
  }
}

void IRAM_ATTR HOT RemoteReceiverComponentStore::gpio_intr(RemoteReceiverComponentStore *arg) {
  arg->interrupt_count++;
  const bool curr_level = !arg->pin.digital_read();
  const uint32_t curr_micros = micros();
  const uint32_t delta = curr_micros - arg->prev_micros;

  // 丟棄小於 filter_us 的高頻雜波毛刺 (Debounce)
  if (delta < arg->filter_us) {
    return;
  }

  if (arg->prev_level != curr_level) {
    bool in_frame = (::s11_on_pulse_edge == nullptr);
    if (::s11_on_pulse_edge != nullptr) {
      in_frame = ::s11_on_pulse_edge(delta, arg->prev_level);
    }
    if (in_frame) {
      commit_value(arg, curr_micros, arg->prev_level);
    }
    arg->prev_micros = curr_micros;
    arg->prev_level = curr_level;
  }
}

void TuyaRfComponent::turn_on_receiver() {
  if (this->receiver_disabled_) {
    this->receiver_disabled_=false;
    this->set_receiver(true);
   } else {
     ESP_LOGD(TAG,"receiver already active");
   }
}

void TuyaRfComponent::turn_off_receiver() {
  if (!this->receiver_disabled_) {
    this->receiver_disabled_=true;
    this->set_receiver(false);
   } else {
     ESP_LOGD(TAG,"receiver already disabled");
   }
}

bool TuyaRfComponent::set_frequency(uint32_t frequency) {
  if (RF_SetFrequency(frequency) != 0) {
    ESP_LOGE(TAG, "Unsupported frequency %u Hz", frequency);
    return false;
  }
  this->frequency_ = frequency;
  ESP_LOGD(TAG, "frequency set to %.3f MHz", frequency / 1e6f);
  // the new registers are loaded by StartTx/StartRx, restart the receiver if it is running
  if (!this->receiver_disabled_ && !this->transmitting_) {
    StartRx();
  }
  return true;
}

void TuyaRfComponent::set_receiver(bool on) {
  if (on) {
    ESP_LOGD(TAG, "Starting RF receiver");
    auto &s = this->store_;
    s.idle_us = this->idle_us_;
    s.filter_us = this->filter_us_;
    s.pin = this->RemoteReceiverBase::pin_->to_isr();
    if (s.buffer == nullptr) {
      ESP_LOGD(TAG, "Allocating RF receiver buffer (%u entries)", s.buffer_size);
      s.buffer = new int32_t[s.buffer_size];
    }
    s.buffer_write = 0;
    s.buffer_read = 0;
    s.buffer_start = 0;
    s.prev_micros = micros();
    s.commit_micros = s.prev_micros;
    s.prev_level = this->RemoteReceiverBase::pin_->digital_read();
    s.commit_level = s.prev_level;
    this->RemoteReceiverBase::pin_->attach_interrupt(RemoteReceiverComponentStore::gpio_intr, &this->store_, gpio::INTERRUPT_ANY_EDGE);
    this->high_freq_.start();
    if (!this->transmitting_) {
      int res = StartRx();
      if (res != 0) {
        ESP_LOGE(TAG, "StartRx failed with code %d", res);
      } else {
        ESP_LOGI(TAG, "StartRx OK: CMT2300A in Direct RX Mode (listening on P22)");
      }
    }
  } else {
    ESP_LOGD(TAG, "Stopping RF receiver");
    if (!this->transmitting_) {
      if (CMT2300A_GoStby()) {
        ESP_LOGD(TAG, "CMT2300A in Standby mode");
      } else {
        ESP_LOGE(TAG, "CMT2300A GoStby error");
      }
    }
    this->RemoteReceiverBase::pin_->detach_interrupt();
    this->high_freq_.stop();
  }
}

void TuyaRfComponent::setup() {
  tuya_sclk = this->sclk_pin_->get_pin();
  tuya_mosi = this->mosi_pin_->get_pin();
  tuya_csb = this->csb_pin_->get_pin();
  tuya_fcsb = this->fcsb_pin_->get_pin();
  ESP_LOGI(TAG, "Assigned SPI pins: SCLK=%u, MOSI(SDIO)=%u, CSB=%u, FCSB=%u",
           tuya_sclk, tuya_mosi, tuya_csb, tuya_fcsb);

  this->RemoteTransmitterBase::pin_->setup();
  this->RemoteTransmitterBase::pin_->digital_write(false);
  this->RemoteReceiverBase::pin_->setup();
  if (RF_SetFrequency(this->frequency_) != 0) {
    ESP_LOGE(TAG, "Unsupported frequency %u Hz", this->frequency_);
    this->mark_failed();
    return;
  }

  uint8_t probe_val = CMT2300A_ReadReg(CMT2300A_CUS_PKT17);
  ESP_LOGI(TAG, "CMT2300A probe initial read reg 0x50 = 0x%02X", probe_val);
  CMT2300A_WriteReg(CMT2300A_CUS_PKT17, 0xAA);
  uint8_t test_val = CMT2300A_ReadReg(CMT2300A_CUS_PKT17);
  ESP_LOGI(TAG, "CMT2300A probe write 0xAA -> read back = 0x%02X", test_val);
  CMT2300A_WriteReg(CMT2300A_CUS_PKT17, probe_val);
  
  if (RF_Init() != 0) {
    ESP_LOGE(TAG, "CMT2300A RF chip not found! Check SPI pins (CSB:%u, SCLK:%u, SDIO:%u, FCSB:%u)",
             tuya_csb, tuya_sclk, tuya_mosi, tuya_fcsb);
  } else {
    ESP_LOGI(TAG, "CMT2300A RF chip detected successfully via 3-Wire SPI!");
  }
  
  auto &s = this->store_;
  s.filter_us = this->filter_us_;
  s.pin = this->RemoteReceiverBase::pin_->to_isr();
  s.buffer_size = this->buffer_size_;

  if (s.buffer_size % 2 != 0) {
    // Make sure divisible by two. This way, we know that every 0bxxx0 index is a space and every 0bxxx1 index is a mark
    s.buffer_size++;
  }
  //the buffer will be allocated the first time the receiver is enabled

  this->set_receiver(!this->receiver_disabled_);
}

void TuyaRfComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Tuya Rf:");
  LOG_PIN("  Sclk Pin: ",this->sclk_pin_);
  LOG_PIN("  Mosi Pin: ",this->mosi_pin_);
  LOG_PIN("  Csb Pin: ",this->csb_pin_);
  LOG_PIN("  Fcsb Pin: ",this->fcsb_pin_);
  LOG_PIN("  Tx Pin: ",this->RemoteTransmitterBase::pin_);
  LOG_PIN("  Rx Pin: ", this->RemoteReceiverBase::pin_);
  ESP_LOGCONFIG(TAG, "  Frequency: %.3f MHz", this->frequency_ / 1e6f);
  if (CMT2300A_IsExist()) {
    ESP_LOGCONFIG(TAG, "  CMT2300A Hardware Status: PRESENT & RESPONDING (SPI OK)");
  } else {
    ESP_LOGE(TAG, "  CMT2300A Hardware Status: NOT RESPONDING (SPI FAIL)");
  }
  //probably the warning isn't useful due to the noisy signal
  if (!this->RemoteReceiverBase::pin_->digital_read()) {
    ESP_LOGW(TAG, "Remote Receiver Signal starts with a HIGH value. Usually this means you have to "
                  "invert the signal using 'inverted: True' in the pin schema!");
    ESP_LOGW(TAG, "It could also be that the signal is noisy.");
  }
  ESP_LOGCONFIG(TAG, "  Buffer Size: %u", this->buffer_size_);
  ESP_LOGCONFIG(TAG, "  Tolerance: %u%s", this->tolerance_,
                (this->tolerance_mode_ == remote_base::TOLERANCE_MODE_TIME) ? " us" : "%");
  ESP_LOGCONFIG(TAG, "  Filter out pulses shorter than: %u us", this->filter_us_);
  ESP_LOGCONFIG(TAG, "  Signal start with a pulse between %u and %u us", this->start_pulse_min_us_, this->start_pulse_max_us_);
  ESP_LOGCONFIG(TAG, "  Signal is done after a pulse of %u us", this->end_pulse_us_);
  if (this->receiver_disabled_) {
    ESP_LOGCONFIG(TAG, "  Receiver disabled");
  } else {
    ESP_LOGCONFIG(TAG, "  Receiver enabled");
  }
}

void TuyaRfComponent::await_target_time_() {
  const uint32_t current_time = micros();
  if (this->target_time_ == 0) {
    this->target_time_ = current_time;
  } else {
    while (this->target_time_ > micros()) {
      // busy loop that ensures micros is constantly called
    }
  }
}

void TuyaRfComponent::mark_(uint32_t usec) {
  this->await_target_time_();
  this->RemoteTransmitterBase::pin_->digital_write(true);
  this->target_time_ += usec;
}

void TuyaRfComponent::space_(uint32_t usec) {
  this->await_target_time_();
  this->RemoteTransmitterBase::pin_->digital_write(false);
  this->target_time_ += usec;
}

void IRAM_ATTR TuyaRfComponent::send_internal(uint32_t send_times, uint32_t send_wait) {
  ESP_LOGD(TAG, "Sending remote code...");
  /*
    for (int32_t item : this->temp_.get_data()) {
      if (item > 0) {
        const auto length = uint32_t(item);
        ESP_LOGD(TAG, "pulse %d",length);
      } else {
        const auto length = uint32_t(-item);
        ESP_LOGD(TAG, "space %d",length);
      }
    }
*/
  
  // 注意：嚴禁在 BK7238 全域關閉中斷超過 2ms，會凍結 SysTick 導致 wiring.c 之 micros() 產生 2.3 倍速時間壓縮！
  // InterruptLock lock;
  
  this->transmitting_=true;
  this->RemoteTransmitterBase::pin_->digital_write(false);

  int res=StartTx();
  switch(res) {
    case 0: {
      uint8_t f5 = CMT2300A_ReadReg(CMT2300A_CUS_RF5);
      uint8_t f6 = CMT2300A_ReadReg(CMT2300A_CUS_RF6);
      uint8_t f7 = CMT2300A_ReadReg(CMT2300A_CUS_RF7);
      uint8_t f8 = CMT2300A_ReadReg(CMT2300A_CUS_RF8);
      ESP_LOGI(TAG, "StartTx OK: TX Freq Regs=0x%02X 0x%02X 0x%02X 0x%02X (433.920MHz Direct OOK)", f5, f6, f7, f8);
      break;
    }
    case 1:
      ESP_LOGE(TAG,"Error Rf_Init");
      this->transmitting_=false;
      return;
    case 2:
      ESP_LOGE(TAG,"Error go tx");
      this->transmitting_=false;
      return;
    default:
      ESP_LOGE(TAG,"Unknown error %d",res);
      this->transmitting_=false;
      return;      
  }
  
  this->RemoteTransmitterBase::pin_->digital_write(false);

  this->target_time_ = 0;
  for (uint32_t i = 0; i < send_times; i++) {
    for (int32_t item : this->RemoteTransmitterBase::temp_.get_data()) {
      if (item > 0) {
        this->mark_(item); // positive -> mark -> carrier ON
      } else {
        this->space_(-item); // negative -> space -> carrier OFF
      }
      App.feed_wdt();
    }
    if (i + 1 < send_times && send_wait > 0)
      this->space_(send_wait);
  }
  this->space_(2000);
  this->await_target_time_();
  this->RemoteTransmitterBase::pin_->digital_write(false);
  
  this->transmitting_=false;
  if (this->receiver_disabled_) {
    if(CMT2300A_GoStby()) {
      //ESP_LOGD(TAG,"go stby ok");
    } else {
      ESP_LOGE(TAG,"go stby error");
    } 
  } else {
    //Go back to rx mode
    StartRx();
  } 
}

/*
The rf input is quite noisy, so some heavy filtering must be done:

  - the signal is inverted (the pin gives 0 for mark, 1 for
    space, with the inversion the space is 0 and pulse is 1).

  - the starting pulse must be more than a specified duration
    (start_pulse_min_us_) but less than start_pulse_max_us_
    The default values work with my remote, I don't know if 
    they are valid for other remotes.

  - the cmt2300a gives a very long pulse (90ms) at the end of a
    reception, the parameter to detect it is end_pulse_us_.
*/
void TuyaRfComponent::loop() {
  if (this->receiver_disabled_) {
    return;
  }
  if (::s11_process_pending_action != nullptr) {
    ::s11_process_pending_action();
  }
  auto &s = this->store_;

  static uint32_t last_status = 0;
  if (millis() - last_status > 5000) {
    last_status = millis();
    uint8_t probe_0 = CMT2300A_ReadReg(CMT2300A_CUS_PKT17);
    CMT2300A_WriteReg(CMT2300A_CUS_PKT17, 0xAA);
    uint8_t probe_aa = CMT2300A_ReadReg(CMT2300A_CUS_PKT17);
    CMT2300A_WriteReg(CMT2300A_CUS_PKT17, probe_0);
    int rssi = CMT2300A_GetRssiDBm();
    ESP_LOGD(TAG, "RF Status: P22 level=%d, interrupts=%u, buffer_write=%u, RSSI=%d dBm | SPI probe reg 0x50 read=0x%02X, write 0xAA read=0x%02X",
             this->RemoteReceiverBase::pin_->digital_read(), s.interrupt_count, s.buffer_write, rssi, probe_0, probe_aa);
  }

  if (s.overflow) {
    ESP_LOGW(TAG, "RF Buffer overflow");
    s.overflow = false;
  }

  uint32_t last_index = s.buffer_start;
  if (last_index == s.buffer_read) {
    InterruptLock lock;
    if (s.buffer_read == s.buffer_start && s.buffer_write != s.buffer_start &&
        micros() - s.prev_micros >= this->idle_us_) {
      commit_value(&s, s.prev_micros, s.prev_level);
      write_value(&s, this->idle_us_, !s.commit_level);
      last_index = s.buffer_start;
    }
  }
  if (last_index == s.buffer_read) {
    return;
  }

  uint32_t temp_read = s.buffer_read;
  uint32_t reserve_size = 0;
  while (temp_read != last_index && (uint32_t) std::abs(s.buffer[temp_read]) < this->idle_us_) {
    reserve_size++;
    temp_read++;
    if (temp_read >= s.buffer_size) {
      temp_read = 0;
    }
  }
  if (reserve_size < 6) {
    // Noise glitch < 6 pulses, skip
    s.buffer_read = last_index;
    return;
  }

  this->RemoteReceiverBase::temp_.clear();
  this->RemoteReceiverBase::temp_.reserve(reserve_size + 1);

  for (uint32_t i = 0; i < reserve_size + 1; i++) {
    this->RemoteReceiverBase::temp_.push_back((int32_t) s.buffer[s.buffer_read++]);
    if (s.buffer_read >= s.buffer_size) {
      s.buffer_read = 0;
    }
  }

  int cur_rssi = CMT2300A_GetRssiDBm();
  ESP_LOGI(TAG, "Received 433MHz RF packet: %u pulses, RSSI: %d dBm", reserve_size + 1, cur_rssi);
  this->call_listeners_dumpers_();
  if (::s11_check_blind_rf_rx != nullptr) {
    ::s11_check_blind_rf_rx(this->RemoteReceiverBase::temp_);
  }
}

}  // namespace tuya_rf
}  // namespace esphome

#endif
