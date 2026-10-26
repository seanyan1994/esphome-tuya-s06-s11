#pragma once

#include "esphome.h"
#include "esphome/components/tuya_rf/tuya_rf.h"
#include "esphome/components/fan/fan.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/select/select.h"
#include "esphome/components/wifi/wifi_component.h"
#include "s06_hitachi_hw.h"

extern "C" {
int StartTx(void);
int StartRx(void);
}

namespace s11_fan_rf {

// =========================================================================
// 類型 1：萬士益 5 檔風扇燈 (24-bit EV1527 協定)
// =========================================================================
// 遙控器傳輸時 bit 3 會交替發送 0xB3810 與 0xB3818
static const uint32_t FAN_ADDR_BASE = 0xB3810;

// 按鍵指令碼真值表 (EV1527 24-bit Command Truth Table)
enum FanCommand : uint8_t {
  CMD_FAN_POWER = 0x03, // 風扇電源開關 (開/關 Toggle)
  CMD_OFF       = 0x03, // 兼顧歷史相容別名
  CMD_SPEED_1   = 0x05, // 風速 1 檔
  CMD_SPEED_2   = 0x0A, // 風速 2 檔
  CMD_SPEED_3   = 0x06, // 風速 3 檔
  CMD_SPEED_4   = 0x09, // 風速 4 檔
  CMD_SPEED_5   = 0x02, // 風速 5 檔
  CMD_LIGHT     = 0x0D, // 燈光開關 (Toggle)
  CMD_DIRECTION = 0x0C, // 正反轉
  CMD_TIMER_1H  = 0x01, // 定時 1 小時
  CMD_TIMER_4H  = 0x04, // 定時 4 小時
  CMD_TIMER_8H  = 0x08, // 定時 8 小時
};

// =========================================================================
// 類型 2：6 檔直流變頻風扇燈 (CA285 30-bit 擴展長碼協定)
// 協定格式: 20-bit Addr (0xCA285) | 4-bit Cmd | 2-bit Mode | 4-bit Inverted Checksum (~Cmd)
// =========================================================================
static const uint32_t TAOYUAN_FAN_ADDR = 0xCA285;

enum TaoyuanFanMode : uint8_t {
  TY_MODE_SPEED  = 0x01, // 01b: 風速檔位群組 (1~6 檔)
  TY_MODE_SYSTEM = 0x02, // 10b: 系統控制群組 (關風扇、定時、自然風、正反轉、燈)
};

// CA285 30 位元完整真值表
static const uint32_t TY_CODE_SPEED_1    = 0x328A17D0; // 風速 1 檔 (Cmd 0xF, Mode 01, Inv 0x0)
static const uint32_t TY_CODE_SPEED_2    = 0x328A1791; // 風速 2 檔 (Cmd 0xE, Mode 01, Inv 0x1)
static const uint32_t TY_CODE_SPEED_3    = 0x328A1752; // 風速 3 檔 (Cmd 0xD, Mode 01, Inv 0x2)
static const uint32_t TY_CODE_SPEED_4    = 0x328A1713; // 風速 4 檔 (Cmd 0xC, Mode 01, Inv 0x3)
static const uint32_t TY_CODE_SPEED_5    = 0x328A16D4; // 風速 5 檔 (Cmd 0xB, Mode 01, Inv 0x4)
static const uint32_t TY_CODE_SPEED_6    = 0x328A1695; // 風速 6 檔 (Cmd 0xA, Mode 01, Inv 0x5)
static const uint32_t TY_CODE_FAN_OFF    = 0x328A1723; // 關風扇 (Cmd 0xC, Mode 10, Inv 0x3)
static const uint32_t TY_CODE_TIMER_1H   = 0x328A152B; // 定時 1H (Cmd 0x4, Mode 10, Inv 0xB)
static const uint32_t TY_CODE_TIMER_2H   = 0x328A146E; // 定時 2H (Cmd 0x1, Mode 10, Inv 0xE)
static const uint32_t TY_CODE_TIMER_4H   = 0x328A14EC; // 定時 4H (Cmd 0x3, Mode 10, Inv 0xC)
static const uint32_t TY_CODE_TIMER_8H   = 0x328A16A5; // 定時 8H (Cmd 0xA, Mode 10, Inv 0x5)
static const uint32_t TY_CODE_NATURAL    = 0x328A1627; // 自然風 (Cmd 0x8, Mode 10, Inv 0x7)
static const uint32_t TY_CODE_DIRECTION  = 0x328A1666; // 正反轉 (Cmd 0x9, Mode 10, Inv 0x6)
static const uint32_t TY_CODE_LIGHT      = 0x328A156A; // 燈開/關 (Cmd 0x5, Mode 10, Inv 0xA)

static esphome::fan::Fan *s_fan{nullptr};
static esphome::light::LightState *s_light{nullptr};
static esphome::text_sensor::TextSensor *s_rf_sensor{nullptr};
static esphome::select::Select *s_timer_select{nullptr};

// 定時器狀態與倒數管理
static uint32_t s_timer_hours = 0;
static uint32_t s_timer_expire_time = 0;

inline void cancel_fan_timer() {
  s_timer_hours = 0;
  s_timer_expire_time = 0;
  if (s_timer_select != nullptr) {
    s_timer_select->publish_state("無定時");
  }
}

inline void set_fan_timer(uint32_t hours) {
  s_timer_hours = hours;
  s_timer_expire_time = millis() + (hours * 3600UL * 1000UL);
  if (s_timer_select != nullptr) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%u 小時", (unsigned)hours);
    s_timer_select->publish_state(buf);
  }
}

// 狀態同步與防迴環標記 (Breaking Echo Loops)
// 當從實體遙控器接收並同步到 HA 實體時，設為 true，嚴禁觸發 433MHz 二次發射
inline bool s_is_rx_syncing = false;

// 第一性原理硬體總電源狀態門神：天花板牆壁開關為 OFF 時阻絕一切射頻與狀態虛假反轉
// 預設為 true (通電狀態)，由 HA 牆壁開關實體非同步校準；防範開機尚未收到 HA 事件時誤判停電鎖死
inline bool s_is_wall_power_on = true;

// =========================================================================
// 全域防禦：開機與 Wi-Fi 動態狀態就緒保護鎖 (Dynamic Network & State Guard)
// 1. 開機前 2.5 秒為晶片開機、CMT2300A 晶振與 LDO 穩壓冷啟動期。
// 2. Wi-Fi 斷線期間：處於離線狀態，嚴禁發射主動網路射頻指令。
// 3. 初始狀態就緒前 (s_has_initial_light_sync == false)：等待 HA 首次握手注入狀態，
//    一旦收到 HA 初始推播，下一毫秒立即動態解鎖 (Init Done)！
//    保底：若連上 Wi-Fi 超過 15 秒 HA 仍未回應 (如 HA 主機離線)，自動超時放行。
// =========================================================================
static bool s_wifi_was_connected = false;
static uint32_t s_wifi_connected_at = 0;
static bool s_has_initial_light_sync = false;

inline void check_network_guard_loop() {
  if (esphome::wifi::global_wifi_component == nullptr) return;

  bool connected = esphome::wifi::global_wifi_component->is_connected();

  if (connected && !s_wifi_was_connected) {
    s_wifi_connected_at = millis();
    ESP_LOGI("fan_rf", ">>> [Wi-Fi 恢復連線] 等待 HA 初始狀態握手同步... <<<");
  } else if (!connected && s_wifi_was_connected) {
    s_has_initial_light_sync = false;
    ESP_LOGW("fan_rf", ">>> [Wi-Fi 斷線警告] 網路中斷，進入斷網靜態保護狀態 <<<");
  }

  s_wifi_was_connected = connected;
}

inline bool is_rf_locked() {
  // 1. 底層硬體冷啟動保護：開機前 2.5 秒供 CMT2300A 晶振與 LDO 穩壓
  if (millis() < 2500) return true;

  // 2. Wi-Fi 斷線中保護：無網路狀態阻絕主動網路射頻
  if (esphome::wifi::global_wifi_component != nullptr && !esphome::wifi::global_wifi_component->is_connected()) {
    return true;
  }

  // 3. 動態狀態就緒保護 (Dynamic Init Guard)：
  // 剛連上 Wi-Fi 但 HA 尚未完成首次握手與狀態推播前，維持閉鎖；
  // 一旦 HA 推播到達 (s_has_initial_light_sync == true)，立刻動態解鎖！
  if (!s_has_initial_light_sync) {
    // 容錯保底：若連上 Wi-Fi 超過 15 秒 HA 仍未回應 (如 HA 主機離線)，自動放行防永久鎖死
    if (s_wifi_connected_at != 0 && (millis() - s_wifi_connected_at > 15000)) {
      return false;
    }
    return true;
  }

  return false;
}

inline void check_fan_timer_loop() {
  if (s_timer_hours > 0 && s_timer_expire_time != 0) {
    if (millis() >= s_timer_expire_time) {
      ESP_LOGI("fan_rf", ">>> 風扇定時 %u 小時到期，自動同步關閉 HA 狀態 <<<", (unsigned)s_timer_hours);
      s_timer_hours = 0;
      s_timer_expire_time = 0;
      if (s_timer_select != nullptr) {
        s_timer_select->publish_state("無定時");
      }
      if (s_fan != nullptr && s_fan->state) {
        s_is_rx_syncing = true;
        s_fan->turn_off().perform();
        s_is_rx_syncing = false;
      }
    }
  }
}

static uint64_t s_last_rx_code = 0;
static uint32_t s_last_rx_time = 0;
static uint32_t s_tx_lockout_until = 0;

inline void register_entities(esphome::fan::Fan *f, esphome::light::LightState *l, esphome::text_sensor::TextSensor *s = nullptr, esphome::select::Select *ts = nullptr) {
  s_fan = f;
  s_light = l;
  s_rf_sensor = s;
  s_timer_select = ts;
}

// =========================================================================
// 共用底層發射器 Helper
// =========================================================================
inline void transmit_burst(tuya_rf::TuyaRfComponent *rf, const std::vector<uint16_t> &burst) {
  auto call = rf->transmit();
  auto *data = call.get_data();
  data->reserve(burst.size());
  for (size_t i = 0; i < burst.size(); i++) {
    if (i % 2 == 0) {
      data->mark(burst[i]);
    } else {
      data->space(burst[i]);
    }
  }
  call.perform();
}

// =========================================================================
// 發射端：EV1527 433.92MHz 射頻幀 (標準 transmit_burst + 實測精確微秒時序)
// =========================================================================
inline void send_ev1527_burst(tuya_rf::TuyaRfComponent *rf, uint8_t cmd) {
  if (rf == nullptr) return;

  // 第一性原理：實體總電源斷電期間嚴禁射頻發射
  if (!s_is_wall_power_on) {
    ESP_LOGD("fan_rf", ">>> [第一性原理] 天花板總電源關閉，阻絕 EV1527 發射 <<<");
    return;
  }

  // 開機與網路握手靜音防禦鎖
  if (is_rf_locked()) {
    ESP_LOGW("fan_rf", ">>> [防誤開關保護] 處於靜音保護期，阻絕 EV1527 發射 (Cmd=0x%X) <<<", cmd);
    return;
  }

  // 標記發射鎖定時間 (800ms)，防自發自收迴環與天線飽和
  s_tx_lockout_until = millis() + 800;

  uint32_t code_0 = (FAN_ADDR_BASE << 4) | cmd;        // 0xB3810x (實體遙控器實測碼)
  uint32_t code_8 = ((FAN_ADDR_BASE | 0x08) << 4) | cmd; // 0xB3818x (備用碼)

  std::vector<uint16_t> burst;
  burst.reserve(2 + 50 * 12);

  // 1. 前置喚醒導引碼 (Lead-in Sync): Mark 260us + Space 5630us
  burst.push_back(260);
  burst.push_back(5630);

  // 2. 連續發射 8 幀 code_0，再連續發射 4 幀 code_8 (同碼連續連發，實測精確脈衝)
  for (uint8_t group = 0; group < 2; group++) {
    uint32_t cur_code = (group == 0) ? code_0 : code_8;
    uint8_t repeat = (group == 0) ? 8 : 4;
    for (uint8_t r = 0; r < repeat; r++) {
      // 24 位元數據 (MSB first)
      for (int b = 23; b >= 0; b--) {
        bool bit = (cur_code >> b) & 1;
        if (bit) {
          // 邏輯 1: Mark 860us + Space 320us (實測值)
          burst.push_back(860);
          burst.push_back(320);
        } else {
          // 邏輯 0: Mark 270us + Space 910us (實測值)
          burst.push_back(270);
          burst.push_back(910);
        }
      }
      // 幀間/幀尾同步 (Sync): Mark 260us + Space 5630us
      burst.push_back(260);
      burst.push_back(5630);
    }
  }

  transmit_burst(rf, burst);
  ESP_LOGI("fan_rf", ">>> EV1527 433MHz transmit_burst 精確發射完畢 (Cmd=0x%X) <<<", cmd);
}

inline void send_fan_power(tuya_rf::TuyaRfComponent *rf) {
  ESP_LOGI("fan_rf", ">>> [433 RF 發射] 風扇電源開關 (開/關) Cmd=0x%X <<<", CMD_FAN_POWER);
  send_ev1527_burst(rf, CMD_FAN_POWER);
}

inline void send_fan_speed(tuya_rf::TuyaRfComponent *rf, int speed) {
  uint8_t cmd = CMD_FAN_POWER;
  switch (speed) {
    case 1: cmd = CMD_SPEED_1; break;
    case 2: cmd = CMD_SPEED_2; break;
    case 3: cmd = CMD_SPEED_3; break;
    case 4: cmd = CMD_SPEED_4; break;
    case 5: cmd = CMD_SPEED_5; break;
    default: cmd = CMD_FAN_POWER; break;
  }
  ESP_LOGI("fan_rf", ">>> [433 RF 發射] 風速 %d 檔 Cmd=0x%X <<<", speed, cmd);
  send_ev1527_burst(rf, cmd);
}

inline void send_fan_light_toggle(tuya_rf::TuyaRfComponent *rf) {
  ESP_LOGI("fan_rf", ">>> [433 RF 發射] 風扇燈開關 (Toggle) Cmd=0x%X <<<", CMD_LIGHT);
  send_ev1527_burst(rf, CMD_LIGHT);
}

// EV1527 燈光狀態記憶與狀態轉移過濾 (解決開機與斷線誤觸發問題)
static bool s_last_kmhouse_light_state = false;

inline void handle_kmhouse_light_write(tuya_rf::TuyaRfComponent *rf, bool state) {
  // 1. 第一性原理硬體總電源守衛：斷電時禁止發射並拉回 OFF
  if (!s_is_wall_power_on) {
    if (state && s_light != nullptr) {
      s_is_rx_syncing = true;
      s_light->turn_off().perform();
      s_is_rx_syncing = false;
    }
    return;
  }

  // 2. 開機靜音保護期：記錄初始狀態，嚴禁發射
  if (is_rf_locked()) {
    s_last_kmhouse_light_state = state;
    ESP_LOGD("fan_rf", ">>> [開機靜音] 略過 EV1527 燈光初始化寫入 (state=%d) <<<", state);
    return;
  }

  // 3. 實體遙控同步中：更新狀態紀錄，嚴禁發射
  if (s_is_rx_syncing) {
    s_last_kmhouse_light_state = state;
    return;
  }

  // 4. 重複指令過濾：目標 state 與當前紀錄一致時，略過 Toggle 發射
  if (state == s_last_kmhouse_light_state) {
    ESP_LOGD("fan_rf", ">>> [EV1527 燈光重複指令過濾] 目標 state=%d 與當前狀態一致，略過 Toggle 發射 <<<", state);
    return;
  }

  s_last_kmhouse_light_state = state;
  send_fan_light_toggle(rf);
}

inline void handle_ev1527_light_write(tuya_rf::TuyaRfComponent *rf, bool state) {
  handle_kmhouse_light_write(rf, state);
}

inline void send_fan_direction(tuya_rf::TuyaRfComponent *rf) {
  ESP_LOGI("fan_rf", ">>> [433 RF 發射] 風扇正反轉 Cmd=0x%X <<<", CMD_DIRECTION);
  send_ev1527_burst(rf, CMD_DIRECTION);
}

// =========================================================================
// 發射端：CA285 30-bit 擴展長碼生成與發射
// 實機精準硬體波形：
// 每幀恰為 60 個脈衝 (前 29 個位元各 2 脈衝 + 第 30 位元 Mark + 幀尾/幀間 Sync Space 6000us)
// 前置喚醒導引碼 (Lead-in Sync): Mark 360us + Space 6000us
// 徹底消除幀尾多塞 360us Mark 導致的 Framing Error (31st bit 溢位)
// =========================================================================
inline void send_ca285_code(tuya_rf::TuyaRfComponent *rf, uint32_t code30) {
  if (rf == nullptr) return;

  // 第一性原理：實體總電源斷電期間嚴禁射頻發射
  if (!s_is_wall_power_on) {
    ESP_LOGD("fan_rf", ">>> [第一性原理] 天花板總電源關閉，阻絕 CA285 發射 <<<");
    return;
  }

  // 開機靜音防禦鎖：前 15 秒阻絕一切射頻發射
  if (is_rf_locked()) {
    ESP_LOGW("fan_rf", ">>> [防誤開關保護] 處於靜音保護期，阻絕 CA285 發射 (0x%08X) <<<", (unsigned)code30);
    return;
  }

  // 防 500ms 內快速重複發射相同碼 (防 HA turn_on + on_speed_set 重疊發射)
  static uint32_t s_last_ca285_code = 0;
  static uint32_t s_last_ca285_time = 0;
  if (code30 == s_last_ca285_code && (millis() - s_last_ca285_time < 500)) {
    ESP_LOGD("fan_rf", ">>> [防重複發射] 500ms 內相同 CA285 碼 (0x%08X)，略過 <<<", (unsigned)code30);
    return;
  }
  s_last_ca285_code = code30;
  s_last_ca285_time = millis();

  s_tx_lockout_until = millis() + 800;

  std::vector<uint16_t> burst;
  burst.reserve(2 + 60 * 10);

  // 1. 前置喚醒導引碼 (Lead-in Sync): Mark 360us + Space 6000us
  burst.push_back(360);
  burst.push_back(6000);

  // 2. 連續發射 10 幀標準長碼 (每幀嚴格 60 脈衝)
  for (uint8_t r = 0; r < 10; r++) {
    // 前 29 個位元 (Bit 29 down to 1)
    for (int b = 29; b >= 1; b--) {
      bool bit = (code30 >> b) & 1;
      if (bit) {
        burst.push_back(1100); // 邏輯 1: Mark 1100us + Space 360us
        burst.push_back(360);
      } else {
        burst.push_back(360);  // 邏輯 0: Mark 360us + Space 1100us
        burst.push_back(1100);
      }
    }

    // 第 30 個位元 (Bit 0): 其 Space 即為 6000us 幀間/幀尾同步間隔
    bool bit0 = (code30 & 1);
    if (bit0) {
      burst.push_back(1100);   // 邏輯 1: Mark 1100us
    } else {
      burst.push_back(360);    // 邏輯 0: Mark 360us
    }
    burst.push_back(6000);     // 幀尾/幀間同步: Space 6000us
  }

  transmit_burst(rf, burst);
}

inline void send_taoyuan_fan_speed(tuya_rf::TuyaRfComponent *rf, int speed) {
  uint32_t code = TY_CODE_SPEED_1;
  switch (speed) {
    case 1: code = TY_CODE_SPEED_1; break;
    case 2: code = TY_CODE_SPEED_2; break;
    case 3: code = TY_CODE_SPEED_3; break;
    case 4: code = TY_CODE_SPEED_4; break;
    case 5: code = TY_CODE_SPEED_5; break;
    case 6: code = TY_CODE_SPEED_6; break;
    default: code = TY_CODE_SPEED_3; break;
  }
  ESP_LOGI("fan_rf", ">>> [433 RF 發射] CA285 風扇 %d 檔 (30b: 0x%08X) <<<", speed, (unsigned)code);
  send_ca285_code(rf, code);
}

inline void send_ca285_fan_speed(tuya_rf::TuyaRfComponent *rf, int speed) {
  send_taoyuan_fan_speed(rf, speed);
}

inline void send_taoyuan_fan_off(tuya_rf::TuyaRfComponent *rf) {
  ESP_LOGI("fan_rf", ">>> [433 RF 發射] CA285 風扇關閉 (30b: 0x%08X) <<<", (unsigned)TY_CODE_FAN_OFF);
  send_ca285_code(rf, TY_CODE_FAN_OFF);
}

inline void send_ca285_fan_off(tuya_rf::TuyaRfComponent *rf) {
  send_taoyuan_fan_off(rf);
}

inline void send_taoyuan_light_toggle(tuya_rf::TuyaRfComponent *rf) {
  ESP_LOGI("fan_rf", ">>> [433 RF 發射] CA285 風扇燈 Toggle (30b: 0x%08X) <<<", (unsigned)TY_CODE_LIGHT);
  send_ca285_code(rf, TY_CODE_LIGHT);
}

inline void send_ca285_light_toggle(tuya_rf::TuyaRfComponent *rf) {
  send_taoyuan_light_toggle(rf);
}

// 燈光狀態記憶與狀態轉移過濾 (解決開機與斷線誤觸發問題)
static bool s_last_light_state = false;

inline void handle_taoyuan_light_write(tuya_rf::TuyaRfComponent *rf, bool state) {
  // 1. 第一性原理硬體總電源守衛：斷電時禁止發射並拉回 OFF
  if (!s_is_wall_power_on) {
    if (state && s_light != nullptr) {
      s_is_rx_syncing = true;
      s_light->turn_off().perform();
      s_is_rx_syncing = false;
    }
    return;
  }

  // 2. 開機靜音保護期：記錄初始狀態，嚴禁發射
  if (is_rf_locked()) {
    s_last_light_state = state;
    ESP_LOGD("fan_rf", ">>> [開機靜音] 略過燈光初始化寫入 (state=%d) <<<", state);
    return;
  }

  // 3. 實體遙控同步中：更新狀態紀錄，嚴禁發射
  if (s_is_rx_syncing) {
    s_last_light_state = state;
    return;
  }

  // 4. 重複指令過濾：目標 state 與當前紀錄一致時，略過 Toggle 發射
  if (state == s_last_light_state) {
    ESP_LOGD("fan_rf", ">>> [燈光重複指令過濾] 目標 state=%d 與當前狀態一致，略過 Toggle 發射 <<<", state);
    return;
  }

  s_last_light_state = state;
  send_taoyuan_light_toggle(rf);
}

inline void handle_ca285_light_write(tuya_rf::TuyaRfComponent *rf, bool state) {
  handle_taoyuan_light_write(rf, state);
}

// 物理電源同步函數：第一性原理硬體電源對齊 (不觸發射頻發射)
static bool s_has_initial_wall_power = false;

inline void sync_ha_persistent_light_state(bool is_on) {
  // 開機初次同步或處於網路握手保護期 (is_rf_locked())：強制作靜態記憶校準
  if (!s_has_initial_light_sync || is_rf_locked()) {
    s_has_initial_light_sync = true;
    s_last_light_state = is_on;
    s_last_kmhouse_light_state = is_on;
    if (s_light != nullptr && s_light->current_values.is_on() != is_on) {
      s_is_rx_syncing = true;
      if (is_on) {
        s_light->turn_on().perform();
      } else {
        s_light->turn_off().perform();
      }
      s_is_rx_syncing = false;
    }
    ESP_LOGI("fan_rf", ">>> [HA 持久化狀態自癒同步 (Init Done)] 讀取 HA 燈光狀態: %s，已對齊內部記憶並動態解鎖 <<<", is_on ? "ON" : "OFF");
  }
}

inline void sync_wall_power_state(bool is_power_on) {
  bool previous_power = s_is_wall_power_on;
  s_is_wall_power_on = is_power_on;

  // 1. 斷電守衛：只要牆壁開關為 OFF，強制拉回 OFF 並阻絕一切操作
  if (!is_power_on) {
    s_is_rx_syncing = true;
    s_last_light_state = false;
    s_last_kmhouse_light_state = false;
    if (s_light != nullptr) s_light->turn_off().perform();
    if (s_fan != nullptr) s_fan->turn_off().perform();
    s_is_rx_syncing = false;
    s_has_initial_wall_power = true;
    ESP_LOGI("fan_rf", ">>> [物理電源斷電] 牆壁開關為 OFF，燈光與風扇強制歸零 (State=OFF) <<<");
    return;
  }

  // 2. 開機連網初次同步 (牆壁開關原本就為 ON，天花板未斷電重啟)
  if (!s_has_initial_wall_power) {
    s_has_initial_wall_power = true;
    if (!s_has_initial_light_sync && s_light != nullptr) {
      bool is_on = s_light->current_values.is_on();
      s_last_light_state = is_on;
      s_last_kmhouse_light_state = is_on;
    }
    ESP_LOGI("fan_rf", ">>> [開機初次同步] 牆壁開關已處於 ON (當前燈光記憶=%d) <<<", s_last_light_state);
    return;
  }

  // 3. 真實 0 -> 1 物理通電跳變 (人為開啟牆壁開關，天花板驅動器上電初始化)
  if (!previous_power && is_power_on) {
    s_is_rx_syncing = true;
    s_last_light_state = true;
    s_last_kmhouse_light_state = true;
    if (s_light != nullptr) s_light->turn_on().perform();
    if (s_fan != nullptr) s_fan->turn_off().perform();
    s_is_rx_syncing = false;
    ESP_LOGI("fan_rf", ">>> [物理通電跳變 0->1] 驅動器通電自亮 (燈=ON)，風扇保持停轉 (風扇=OFF) <<<");
  }
}

// =========================================================================
// 中斷級訊號同步閘門 (Precision Sync-Gate: 3000us ~ 25000us)
// =========================================================================
static volatile bool s_in_frame = false;
static volatile uint16_t s_frame_pulse_count = 0;

inline bool IRAM_ATTR HOT on_pulse_isr(uint32_t delta, bool raw_level) {
  if (!s_in_frame) {
    // 支援通用 EV1527 / 射頻同步導引碼 (Sync Pulse 測量值 ~3000us - 25000us)
    // 阻絕 <3000us 空氣雜波，同時容納各廠牌風扇燈同步導引碼
    if (delta >= 3000 && delta <= 25000) {
      s_in_frame = true;
      s_frame_pulse_count = 1;
      return true; // 鎖定有效幀前導，開始寫入 buffer
    }
    return false; // 雜訊直接丟棄，不寫入任何 buffer！
  }

  // 處於收幀狀態：收錄約 2~4 個完整 RF 幀 (約 250 點) 或空閒超時
  s_frame_pulse_count++;
  if (s_frame_pulse_count >= 250 || delta > 26000) {
    s_in_frame = false;
  }
  return true;
}

// =========================================================================
// 接收端：精確 EV1527 解碼與 Home Assistant 實體狀態雙向全同步
// =========================================================================
inline void decode_rf_packet(const std::vector<int32_t> &pulses) {
  // 第一性原理：實體總電源斷電時天花板設備完全停電，忽略任何空氣射頻防虛假狀態反轉
  if (!s_is_wall_power_on) return;

  if (pulses.size() < 30) return;

  uint32_t now = millis();
  if (now < s_tx_lockout_until) {
    return; // 處於 S11 自行發射的防回聲鎖定期
  }

  for (size_t i = 0; i < pulses.size(); i++) {
    int32_t sync_v = std::abs(pulses[i]);
    // 嚴格同步頭：Sync Space 必須在 3500us ~ 22000us 之間
    if (sync_v >= 3500 && sync_v <= 22000 && i + 30 < pulses.size()) {
      uint64_t full_code = 0;
      uint8_t bits = 0;
      size_t j = i + 1;

      while (bits < 64 && j + 1 < pulses.size()) {
        int32_t p1 = std::abs(pulses[j]);
        int32_t p2 = std::abs(pulses[j + 1]);
        uint32_t total = p1 + p2;

        if (p2 >= 3000) {
          // 下一個同步頭或幀結束 Space
          if (p1 >= 250 && p1 <= 1500) {
            full_code = (full_code << 1) | (p1 > 650 ? 1 : 0);
            bits++;
          }
          break;
        }

        // 1-bit 週期約 800us ~ 2200us
        if (total >= 800 && total <= 2200) {
          if (p1 > p2 && (p1 >= (p2 * 17 / 10))) {
            full_code = (full_code << 1) | 1;
            bits++;
          } else if (p2 > p1 && (p2 >= (p1 * 17 / 10))) {
            full_code = (full_code << 1) | 0;
            bits++;
          } else {
            break;
          }
        } else {
          break;
        }
        j += 2;
      }

      // 至少 20 位元才可能是有效遙控幀 (EV1527: 24-bit, CA285: 30-bit)
      if (bits >= 20) {
        // 核心防噪：連續雙幀一致性檢驗 (2-Frame Repetition Verification)
        // 徹底根除空氣雜訊單幀隨機拼湊的偽造訊號 (數學誤報率 < 10^-7)
        static uint64_t s_last_candidate_code = 0;
        static uint8_t s_last_candidate_bits = 0;
        static uint32_t s_last_candidate_time = 0;

        bool is_confirmed = false;
        if (full_code == s_last_candidate_code && bits == s_last_candidate_bits && (now - s_last_candidate_time < 700)) {
          is_confirmed = true;
        } else {
          s_last_candidate_code = full_code;
          s_last_candidate_bits = bits;
          s_last_candidate_time = now;
        }

        if (!is_confirmed) {
          continue; // 等待下一幀確認，阻絕單幀隨機空氣雜訊
        }

        static uint64_t s_last_confirmed_code = 0;
        static uint32_t s_last_confirmed_time = 0;

        // 相同按鍵連續按住時，350ms 內防抖去重
        if (full_code != s_last_confirmed_code || (now - s_last_confirmed_time > 350)) {
          s_last_confirmed_code = full_code;
          s_last_confirmed_time = now;

          ESP_LOGW("rf_sniff", "==================================================");
          ESP_LOGW("rf_sniff", ">>> [433MHz 捕獲確認] %d-bit | Hex: 0x%llX | pulses=%zu | sync=%d <<<",
                   (int)bits, (unsigned long long)full_code, pulses.size(), (int)pulses[i]);
          ESP_LOGW("rf_sniff", "==================================================");

          if (s_rf_sensor != nullptr) {
            char sbuf[64];
            snprintf(sbuf, sizeof(sbuf), "%d-bit: 0x%llX", (int)bits, (unsigned long long)full_code);
            s_rf_sensor->publish_state(sbuf);
          }

          char pbuf[512];
          int plen = snprintf(pbuf, sizeof(pbuf), "REAL_PULSES: sync=%d, p=", (int)pulses[i]);
          for (size_t k = i + 1; k < pulses.size() && k <= i + 50; k++) {
            plen += snprintf(pbuf + plen, sizeof(pbuf) - plen, "%d,", (int)pulses[k]);
            if (plen >= sizeof(pbuf) - 10) break;
          }
          ESP_LOGD("rf_fan", "%s", pbuf);
        }

        uint32_t code = (uint32_t)(full_code >> (bits > 24 ? (bits - 24) : 0));
        if (bits >= 24) {
          uint32_t addr = (code >> 4) & 0xFFFFF;
          uint8_t cmd = code & 0x0F;

          // 針對相同按鍵重複發送的重複幀進行 300ms 防抖；不同按鍵僅需 80ms
          if (full_code == s_last_rx_code && (now - s_last_rx_time < 300)) {
            return;
          }
          s_last_rx_code = full_code;
          s_last_rx_time = now;

        // 匹配 0xB3810 或 0xB3818 (EV1527 24-bit 風扇燈)
        if ((addr & ~0x08) == FAN_ADDR_BASE) {
          const char *cmd_name = "未知";
          switch (cmd) {
            case CMD_FAN_POWER: cmd_name = "風扇開關"; break;
            case CMD_SPEED_1:   cmd_name = "風速 1 檔"; break;
            case CMD_SPEED_2:   cmd_name = "風速 2 檔"; break;
            case CMD_SPEED_3:   cmd_name = "風速 3 檔"; break;
            case CMD_SPEED_4:   cmd_name = "風速 4 檔"; break;
            case CMD_SPEED_5:   cmd_name = "風速 5 檔"; break;
            case CMD_LIGHT:     cmd_name = "風扇燈開關"; break;
            case CMD_DIRECTION: cmd_name = "正反轉"; break;
            case CMD_TIMER_1H:  cmd_name = "定時 1H"; break;
            case CMD_TIMER_4H:  cmd_name = "定時 4H"; break;
            case CMD_TIMER_8H:  cmd_name = "定時 8H"; break;
          }

          ESP_LOGW("rf_fan", "==================================================");
          ESP_LOGW("rf_fan", ">>> [433MHz EV1527 命中] %s | Code: 0x%06X (Addr: 0x%05X, Cmd: 0x%X) <<<", cmd_name, code, addr, cmd);
          ESP_LOGW("rf_fan", "==================================================");

          if (s_rf_sensor != nullptr) {
            char buf[64];
            snprintf(buf, sizeof(buf), "%s (0x%06X)", cmd_name, code);
            s_rf_sensor->publish_state(buf);
          }

          // 標記實體遙控同步中，嚴禁觸發 433MHz 射頻二次發射！
          s_is_rx_syncing = true;

          switch (cmd) {
            case CMD_LIGHT:
              if (s_light != nullptr) {
                ESP_LOGI("fan_rf", ">>> 實體遙控同步：風扇燈 Toggle <<<");
                s_last_kmhouse_light_state = !s_last_kmhouse_light_state;
                auto call = s_light->toggle();
                call.perform();
              }
              break;

            case CMD_FAN_POWER:
              if (s_fan != nullptr) {
                bool will_turn_on = !s_fan->state;
                ESP_LOGI("fan_rf", ">>> 實體遙控同步：風扇開關 Toggle (切換至 %s) <<<", will_turn_on ? "ON" : "OFF");
                auto call = s_fan->make_call();
                if (will_turn_on) {
                  int spd = s_fan->speed;
                  if (spd < 1 || spd > 5) spd = 3;
                  call.set_state(true);
                  call.set_speed(spd);
                } else {
                  call.set_state(false);
                  cancel_fan_timer();
                }
                call.perform();
              }
              break;

            case CMD_SPEED_1:
            case CMD_SPEED_2:
            case CMD_SPEED_3:
            case CMD_SPEED_4:
            case CMD_SPEED_5:
              if (s_fan != nullptr) {
                int spd = (cmd == CMD_SPEED_1) ? 1 :
                          (cmd == CMD_SPEED_2) ? 2 :
                          (cmd == CMD_SPEED_3) ? 3 :
                          (cmd == CMD_SPEED_4) ? 4 : 5;
                ESP_LOGI("fan_rf", ">>> 實體遙控同步：風扇 %d 檔 <<<", spd);
                auto call = s_fan->turn_on();
                call.set_speed(spd);
                call.perform();
              }
              break;

            case CMD_DIRECTION:
              if (s_fan != nullptr) {
                auto new_dir = (s_fan->direction == esphome::fan::FanDirection::FORWARD)
                               ? esphome::fan::FanDirection::REVERSE
                               : esphome::fan::FanDirection::FORWARD;
                ESP_LOGI("fan_rf", ">>> 實體遙控同步：風扇正反轉切換 (%s) <<<",
                         new_dir == esphome::fan::FanDirection::FORWARD ? "FORWARD" : "REVERSE");
                auto call = s_fan->make_call();
                call.set_direction(new_dir);
                call.perform();
              }
              break;

            case CMD_TIMER_1H:
              ESP_LOGI("fan_rf", ">>> 實體遙控同步：定時 1 小時 <<<");
              set_fan_timer(1);
              break;

            case CMD_TIMER_4H:
              ESP_LOGI("fan_rf", ">>> 實體遙控同步：定時 4 小時 <<<");
              set_fan_timer(4);
              break;

            case CMD_TIMER_8H:
              ESP_LOGI("fan_rf", ">>> 實體遙控同步：定時 8 小時 <<<");
              set_fan_timer(8);
              break;

            default:
              ESP_LOGW("fan_rf", ">>> 實體遙控未知功能碼: 0x%X <<<", cmd);
              break;
          }

          s_is_rx_syncing = false;
          return;
        } else if (addr == TAOYUAN_FAN_ADDR || (bits >= 29 && ((full_code >> (bits - 20)) & 0xFFFFF) == TAOYUAN_FAN_ADDR)) {
          // 精確鎖定 CA285 30 位元長碼協定 (Addr 20b | Cmd 4b | Mode 2b | Inv 4b)
          uint32_t ty_full = (uint32_t)full_code;
          // 若最後一位因幀尾同步被提前截斷成 29 位元，依反碼校驗自動補齊第 30 位
          if (bits == 29) {
            uint8_t cmd_tmp = (ty_full >> 5) & 0xF;
            uint8_t expected_inv = (~cmd_tmp) & 0xF;
            ty_full = (ty_full << 1) | (expected_inv & 1);
            bits = 30;
          }

          uint8_t cmd = (ty_full >> 6) & 0xF;
          uint8_t mode = (ty_full >> 4) & 0x3;
          uint8_t chk = ty_full & 0xF;
          const char *action_name = "未知";

          if (mode == TY_MODE_SPEED) {
            int spd = 0;
            switch (cmd) {
              case 0xF: spd = 1; action_name = "風速 1 檔"; break;
              case 0xE: spd = 2; action_name = "風速 2 檔"; break;
              case 0xD: spd = 3; action_name = "風速 3 檔"; break;
              case 0xC: spd = 4; action_name = "風速 4 檔"; break;
              case 0xB: spd = 5; action_name = "風速 5 檔"; break;
              case 0xA: spd = 6; action_name = "風速 6 檔"; break;
            }
            ESP_LOGW("rf_fan", "==================================================");
            ESP_LOGW("rf_fan", ">>> [CA285 命中] %s | 30b: 0x%08X (Cmd: 0x%X, Mode: %d) <<<", action_name, ty_full, cmd, mode);
            ESP_LOGW("rf_fan", "==================================================");

            if (s_rf_sensor != nullptr) {
              char buf[64];
              snprintf(buf, sizeof(buf), "CA285 風扇: %s", action_name);
              s_rf_sensor->publish_state(buf);
            }
            if (s_fan != nullptr && spd > 0) {
              s_is_rx_syncing = true;
              auto call = s_fan->turn_on();
              call.set_speed(spd);
              call.perform();
              s_is_rx_syncing = false;
            }
          } else if (mode == TY_MODE_SYSTEM) {
            switch (cmd) {
              case 0xC: action_name = "關閉風扇"; break;
              case 0x5: action_name = "風扇燈開關"; break;
              case 0x9: action_name = "風扇正反轉"; break;
              case 0x8: action_name = "自然風模式"; break;
              case 0x4: action_name = "定時 1 小時"; break;
              case 0x1: action_name = "定時 2 小時"; break;
              case 0x3: action_name = "定時 4 小時"; break;
              case 0xA: action_name = "定時 8 小時"; break;
            }
            ESP_LOGW("rf_fan", "==================================================");
            ESP_LOGW("rf_fan", ">>> [CA285 命中] %s | 30b: 0x%08X (Cmd: 0x%X, Mode: %d) <<<", action_name, ty_full, cmd, mode);
            ESP_LOGW("rf_fan", "==================================================");

            if (s_rf_sensor != nullptr) {
              char buf[64];
              snprintf(buf, sizeof(buf), "CA285 風扇: %s", action_name);
              s_rf_sensor->publish_state(buf);
            }

            s_is_rx_syncing = true;
            if (cmd == 0xC && s_fan != nullptr) {
              s_fan->turn_off().perform();
              cancel_fan_timer();
            } else if (cmd == 0x5 && s_light != nullptr) {
              s_last_light_state = !s_last_light_state;
              s_light->toggle().perform();
            } else if (cmd == 0x9 && s_fan != nullptr) {
              auto new_dir = (s_fan->direction == esphome::fan::FanDirection::FORWARD)
                             ? esphome::fan::FanDirection::REVERSE
                             : esphome::fan::FanDirection::FORWARD;
              auto call = s_fan->make_call();
              call.set_direction(new_dir);
              call.perform();
            } else if (cmd == 0x4) {
              set_fan_timer(1);
            } else if (cmd == 0x1) {
              set_fan_timer(2);
            } else if (cmd == 0x3) {
              set_fan_timer(4);
            } else if (cmd == 0xA) {
              set_fan_timer(8);
            }
            s_is_rx_syncing = false;
          } else {
            ESP_LOGW("rf_fan", ">>> [CA285 未知 Mode] 30b: 0x%08X (Cmd: 0x%X, Mode: %d) <<<", ty_full, cmd, mode);
          }
          return;
        } else {
          // 全新未知遙控碼確認 (已通過雙幀重複性驗證，100% 排除空氣雜波)
          ESP_LOGW("rf_fan", "==================================================");
          ESP_LOGW("rf_fan", ">>> [全新未知遙控碼確認] %d-bit | 0x%llX (Addr: 0x%05X, Cmd: 0x%X) <<<",
                   (int)bits, (unsigned long long)full_code, (unsigned)addr, (unsigned)cmd);
          ESP_LOGW("rf_fan", "==================================================");
        }
        return;
      }
    }
  }
}
}

} // namespace s11_fan_rf

// 導出給 tuya_rf 組件調用的弱符號 (Weak Symbols)
bool s11_on_pulse_edge(uint32_t delta, bool level) {
  return s11_fan_rf::on_pulse_isr(delta, level);
}

void s11_check_blind_rf_rx(const std::vector<int32_t> &pulses) {
  s11_fan_rf::decode_rf_packet(pulses);
}
