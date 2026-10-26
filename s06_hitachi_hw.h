#pragma once

#include "esphome.h"
#include <Arduino.h>

extern "C" {
#include <BkDriverPwm.h>
#include <gpio_pub.h>
#include <drv_model_pub.h>
#include <arch.h>
}

// 支援外部模組在紅外發射期間暫停背景 RF 接收（如 CMT2300A）
namespace s06_hw {

using RfPauseHook = void (*)(bool pause);

} // namespace s06_hw

extern "C" {
  __attribute__((weak)) s06_hw::RfPauseHook g_s06_rf_pause_hook = nullptr;
  __attribute__((weak)) bool g_s06_pwm_inited = false;
}

namespace s06_hw {

inline void set_rf_pause_hook(RfPauseHook hook) {
  g_s06_rf_pause_hook = hook;
}
inline void call_rf_pause_hook(bool pause) {
  if (g_s06_rf_pause_hook) g_s06_rf_pause_hook(pause);
}

// BK7238 硬體 Timer0 暫存器（基底 0x00802A00，Timer0 週期、讀取控制與數值）
#define S_TIMER0_PERIOD_REG ((volatile uint32_t *)0x00802A00UL)
#define S_TIMER0_READ_CTL   ((volatile uint32_t *)0x00802A10UL)
#define S_TIMER0_READ_VAL   ((volatile uint32_t *)0x00802A14UL)

// BK7238 GPIO 暫存器（基底 0x00802800）
#define S_GPIO_CFG_ADDR(pin) ((volatile uint32_t *)(0x00802800UL + (pin) * 4))
#define S_GPIO_INTEN_REG     ((volatile uint32_t *)(0x00802800UL + 33 * 4))
#define S_GPIO_INTSTA_REG    ((volatile uint32_t *)(0x00802800UL + 36 * 4))

static inline uint32_t bktimer0_get_period() {
  uint32_t p = *S_TIMER0_PERIOD_REG;
  return (p > 0) ? p : 52000;
}

static inline uint32_t bktimer0_get_ticks() {
  uint32_t timeout = 0;
  *S_TIMER0_READ_CTL = 1;
  while ((*S_TIMER0_READ_CTL & 1) && ++timeout < 120000) {}
  return *S_TIMER0_READ_VAL;
}

// 26MHz 晶振，1µs = 26 ticks。直讀硬體暫存器獲取 Timer0 重載週期（FCLK_DURATION_MS 2ms = 52,000 ticks）
// 基於增量差值累加 (Delta-Tick Accumulation)，徹底消滅 wiring.c 之邊界窄窗漏捕與跨毫秒時間拉伸 Bug
static inline void delay_us_accurate(uint32_t us, uint32_t &last_tick, int32_t &error_ticks) {
  uint32_t timer_period = bktimer0_get_period();

  if (error_ticks > 260) error_ticks = 260;
  if (error_ticks < -260) error_ticks = -260;

  int32_t target_ticks = (int32_t)(us * 26) - error_ticks;
  if (target_ticks < 0) target_ticks = 0;

  int32_t elapsed = 0;
  while (true) {
    uint32_t now = bktimer0_get_ticks();
    uint32_t dt = (now >= last_tick) ? (now - last_tick) : (timer_period - last_tick + now);
    last_tick = now;
    elapsed += (int32_t)dt;
    if (elapsed >= target_ticks) {
      error_ticks = elapsed - target_ticks; // 跨脈衝相位補償，全幀零累積漂移
      break;
    }
  }
}

#ifdef S11_HARDWARE
#define S_PWM_CHAN 3
#define S_PWM_PIN 9
#define S_PWM_GFUNC GFUNC_MODE_PWM3
#define S_PWM_GROUP_CTRL_ADDR ((volatile uint32_t *)0x00802B40UL)
#define S_PWM_ENABLE_BIT (1UL << 11)
#define S_PWM_INT_LEVL_BIT (1UL << 14)
#define S_DEVICE_TAG "s11_hw"
#else
#define S_PWM_CHAN 0
#define S_PWM_PIN 6
#define S_PWM_GFUNC GFUNC_MODE_PWM0
#define S_PWM_GROUP_CTRL_ADDR ((volatile uint32_t *)0x00802B00UL)
#define S_PWM_ENABLE_BIT (1UL << 3)
#define S_PWM_INT_LEVL_BIT (1UL << 6)
#define S_DEVICE_TAG "s06_hw"
#endif

// 硬體極速載波開啟（直寫暫存器 ~8ns，保留 T1~T4 週期暫存器，絕不清除）
static inline void ir_carrier_on() {
  *S_PWM_GROUP_CTRL_ADDR |= S_PWM_ENABLE_BIT;
}

// 硬體極速載波關閉（直寫暫存器 ~8ns，維持 init_levl LOW 0V 接地，絕不清除 T1~T4）
static inline void ir_carrier_off() {
  *S_PWM_GROUP_CTRL_ADDR &= ~S_PWM_ENABLE_BIT;
}

// RAII 防護類別：發射期間屏蔽所有 GPIO 中斷（包含自身 P24 接收頭與 P22 RF 中斷）並暫停 RF
// 發射結束後原子清空累積之自收中斷狀態標記，再安全恢復中斷與 RF，徹底根除 688 次自收中斷風暴與微秒時鐘抖動
class RxInterruptMasker {
 public:
  RxInterruptMasker() {
    call_rf_pause_hook(true);
    saved_gpio_inten_ = *S_GPIO_INTEN_REG;
    *S_GPIO_INTEN_REG = 0;
    ir_carrier_off();
  }
  ~RxInterruptMasker() {
    ir_carrier_off();
    *S_GPIO_INTSTA_REG = 0xFFFFFFFFUL;
    *S_GPIO_INTEN_REG = saved_gpio_inten_;
    call_rf_pause_hook(false);
  }
 private:
  uint32_t saved_gpio_inten_{0};
};

inline void attach_pwm_pin() {
  UINT32 param = S_PWM_GFUNC;
  sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &param);
}

inline void ensure_pwm_init() {
  if (!g_s06_pwm_inited) {
    // 1. 初始化 38kHz PWM (週期 684 ticks ≈ 38.01kHz, 佔空比 342 ticks ≈ 50% 強功率發射)
    bk_pwm_initialize((bk_pwm_t)S_PWM_CHAN, 684, 342);
    // 2. 強制將 init level 設為 LOW，確保載波關閉 (enable=0) 時腳位維持 0V 接地
    UINT32 chan = S_PWM_CHAN;
    sddev_control(PWM_DEV_NAME, CMD_PWM_INIT_LEVL_SET_LOW, &chan);
    *S_PWM_GROUP_CTRL_ADDR &= ~S_PWM_INT_LEVL_BIT;
    // 3. 綁定引腳至 PWM 第二功能
    attach_pwm_pin();
    // 4. 關閉載波輸出，保持 LOW 0V 靜默
    ir_carrier_off();
    g_s06_pwm_inited = true;
    ESP_LOGI(S_DEVICE_TAG, "Hardware PWM%d initialized at 38kHz (init_levl=LOW, T1=342, T4=684)", S_PWM_CHAN);
  }
}

inline void delay_ms_safe(uint32_t ms) {
  uint32_t last_tick = bktimer0_get_ticks();
  int32_t error_ticks = 0;
  for (uint32_t i = 0; i < ms; i++) {
    delay_us_accurate(1000, last_tick, error_ticks);
  }
}

// Send arbitrary 344-bit (or any length) buffer via 38kHz hardware PWM with atomic delta-tick timing
inline void send_hitachi_344_bytes(const uint8_t *data, size_t len, uint8_t repeat = 1) {
  if (!data || len == 0 || repeat == 0) return;

  ensure_pwm_init();
  RxInterruptMasker masker;
  GLOBAL_INT_DECLARATION();

  // 正式發射 43 bytes 數據幀（原子臨界區保護載波翻轉時序）
  GLOBAL_INT_DISABLE();
  for (uint8_t r = 0; r < repeat; r++) {
    uint32_t last_tick = bktimer0_get_ticks();
    int32_t error_ticks = 0;

    // Leader pulse: Mark 3300us, Space 1700us
    ir_carrier_on();
    last_tick = bktimer0_get_ticks();
    delay_us_accurate(3300, last_tick, error_ticks);
    ir_carrier_off();
    delay_us_accurate(1700, last_tick, error_ticks);

    // Data bits (LSB first)
    for (size_t i = 0; i < len; i++) {
      uint8_t b = data[i];
      for (int bit = 0; bit < 8; bit++) {
        bool is_one = (b >> bit) & 1;
        ir_carrier_on();
        delay_us_accurate(400, last_tick, error_ticks);
        ir_carrier_off();
        delay_us_accurate(is_one ? 1250 : 500, last_tick, error_ticks);
      }
    }

    // Stop bit: Mark 400us
    ir_carrier_on();
    delay_us_accurate(400, last_tick, error_ticks);
    ir_carrier_off();

    // 幀間間隔（若有 repeat 需求）
    if (r + 1 < repeat) {
      delay_ms_safe(40);
    }
  }
  // 載波已全數發射完畢並拉低接地，立即恢復中斷，釋放 CPU 給 FreeRTOS / Wi-Fi
  GLOBAL_INT_RESTORE();

  // 尾部 40ms Trailer space (EOF 超時結算) 在開中斷後執行，不鎖死系統中斷
  delay_ms_safe(40);

  ESP_LOGI(S_DEVICE_TAG, "Sent dynamic Hitachi AC frame (%u bytes, repeat=%u)", (unsigned)len, (unsigned)repeat);
}

// Send Coolix (Midea / MAXE RC-62) 48-bit frame via 38kHz hardware PWM with atomic per-frame timing
inline void send_coolix_bytes(const uint8_t *data, size_t len = 6, uint8_t repeat = 2) {
  if (!data || len < 6 || repeat == 0) return;

  ensure_pwm_init();
  RxInterruptMasker masker;
  GLOBAL_INT_DECLARATION();

  for (uint8_t r = 0; r < repeat; r++) {
    // 單幀原子臨界區保護（約 85ms，零 Jitter 且兼顧系統排程）
    GLOBAL_INT_DISABLE();
    uint32_t last_tick = bktimer0_get_ticks();
    int32_t error_ticks = 0;

    // Leader pulse: Mark 4400us, Space 4400us
    ir_carrier_on();
    last_tick = bktimer0_get_ticks();
    delay_us_accurate(4400, last_tick, error_ticks);
    ir_carrier_off();
    delay_us_accurate(4400, last_tick, error_ticks);

    // Data bits (MSB first)
    for (size_t i = 0; i < len; i++) {
      uint8_t b = data[i];
      for (int bit = 7; bit >= 0; bit--) {
        bool is_one = (b >> bit) & 1;
        ir_carrier_on();
        delay_us_accurate(540, last_tick, error_ticks);
        ir_carrier_off();
        delay_us_accurate(is_one ? 1620 : 540, last_tick, error_ticks);
      }
    }

    // Stop bit: Mark 540us
    ir_carrier_on();
    delay_us_accurate(540, last_tick, error_ticks);
    ir_carrier_off();

    // 載波翻轉完成，立即恢復全域中斷，釋放 CPU 給 FreeRTOS / Wi-Fi
    GLOBAL_INT_RESTORE();

    // 幀間 10ms 靜默期在開中斷狀態下延時
    delay_ms_safe(10);
  }

  ESP_LOGI(S_DEVICE_TAG, "Sent Coolix (MAXE RC-62) frame (%u bytes, repeat=%u)", (unsigned)len, repeat);
}

inline void force_p6_off() {
  ir_carrier_off();
}

} // namespace s06_hw
