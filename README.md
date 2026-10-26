# ESPHome Tuya S06 / S11 (BK7238) 紅外線與 433MHz 射頻全雙工控制庫

本專案提供針對 **塗鴉 Tuya S06（純紅外線）** 與 **Tuya S11（紅外線 + 433.92MHz 射頻）**（搭載 **Beken BK7238** 晶片）的底層硬體 PWM 驅動與台灣常見冷氣/風扇全雙工（雙向狀態同步）ESPHome 外部元件：

- **`hitachi_ac344`**：台灣日立冷氣（Hitachi 344-bit / 43-Byte 協定）雙向同步 Climate 元件。
- **`maxe_coolix`**：台灣萬士益冷氣（MAXE RC-62 / Midea Coolix 48-bit 格雷碼協定）雙向同步 Climate 元件，內建前段 120 脈衝滑動視窗前導碼搜尋，徹底克服環境光雜訊漏碼問題。
- **`tuya_rf`**：針對 Tuya S11 內建 **CMOSTEK CMT2300A** 433.92MHz 射頻收發晶片（經硬體電路量測與邏輯分析儀實測驗證 6-Pin SPI/Direct Mode 真值腳位，修正社群舊版腳位錯亂與 Mark/Space 極性顛倒問題）。
- **[`s06_hitachi_hw.h`](s06_hitachi_hw.h)**：BK7238 暫存器級 38kHz 硬體 PWM 載波驅動與 Delta-Tick 相位補償微秒延時。

---

## 一、解決的核心硬體與協定痛點

### 1. BK7238 硬體 PWM 停止後高電平鎖死（「紅外強光手電筒」盲死問題）
在 Beken BK7238 晶片上，若僅呼叫 `pwm_stop()` 停止計時器，當停止瞬間剛好落在高電平週期時，IR 發射腳位（S06 為 `P6`、S11 為 `P9`）會**死鎖在 3.3V 直流輸出**。這會導致機內 6～7 顆大功率紅外發射管持續常亮，淹沒整個房間，使冷氣機與控制器自身的紅外接收頭發生嚴重光飽和（實體遙控器完全失靈）。

本專案於 [`s06_hitachi_hw.h`](s06_hitachi_hw.h) 直接操作 BK7238 PWM 控制暫存器（`0x00802B00` / `0x00802B40`）：
- 初始化時強制設定 `CMD_PWM_INIT_LEVL_SET_LOW` 並清除 `INT_LEVL_BIT`，確保載波關閉時腳位絕對維持 **0V 接地**。
- 透過 `RxInterruptMasker` RAII 保護，在發射期間屏蔽自身 GPIO 中斷並暫停 CMT2300A 射頻接收，發射完畢後原子清空 `S_GPIO_INTSTA_REG`，根除 688 次自收中斷風暴。

### 2. LibreTiny 微秒延時漂移修正（Delta-Tick Accumulation）
直接讀取 BK7238 硬體 Timer0（`0x00802A00`，26MHz 晶振 $1\mu\text{s} = 26\text{ ticks}$）進行跨脈衝相位誤差補償，消除 `delayMicroseconds()` 跨毫秒拉伸造成的長幀（如日立 344-bit）校驗失敗。

### 3. Tuya S11 (CMT2300A) 433.92MHz 射頻腳位定義
經硬體線路與邏輯分析儀實測確認，S11 採用 CMT2300A Direct Mode，正確腳位映射如下：

| 功能 | CMT2300A 腳位 | BK7238 物理 GPIO | 備註 |
| :--- | :--- | :--- | :--- |
| **SPI CSB** | Pin 8 | `P26` | Active-LOW，暫存器片選 |
| **SPI SCLK** | Pin 10 | `P14` | 3-Wire SPI 時脈 |
| **SPI SDIO** | Pin 11 | `P16` (或 `P6`) | 半雙工，讀取時切換 `INPUT_PULLUP` |
| **FIFO FCSB** | Pin 9 | `P17` (或 `P16`) | Direct Mode 下維持 HIGH |
| **RF TX (DIN)** | Pin 5 (GPIO1) | `P23` | 發射微秒脈衝輸入 |
| **RF RX (DOUT)** | Pin 6 (GPIO2) | `P22` | 接收微秒脈衝中斷輸入 (`INPUT_PULLUP`) |

---

## 二、S06 與 S11 硬體腳位對照表

| 功能 | Tuya S06 (`t1-3s`) | Tuya S11 (`generic-bk7238-tuya`) |
| :--- | :--- | :--- |
| **IR TX (紅外發射)** | `P6` (硬體 PWM0) | `P9` (硬體 PWM3，需定義 `-DS11_HARDWARE=1`) |
| **IR RX (紅外接收)** | `P26` (`inverted: true`, `INPUT_PULLUP`) | `P24` (`inverted: true`, `INPUT_PULLUP`) |
| **Status LED (藍燈)** | `P24` (`inverted: true`) | `P21` (`inverted: false`) |
| **實體配對按鈕** | `P28` (`inverted: true`, `INPUT_PULLUP`) | `P20` (`inverted: false`, `INPUT`) |

---

## 三、刷機流程 (`ltchiptool`)

1. **安裝 `ltchiptool`**：
   ```bash
   pip install --upgrade ltchiptool
   ```
2. **USB-TTL 接線（嚴禁接 5V，必須使用 3.3V）**：
   - `3V3` ➔ USB-TTL `3.3V`
   - `GND` ➔ USB-TTL `GND`
   - `TX1 (P11)` ➔ USB-TTL `RXD`
   - `RX1 (P10)` ➔ USB-TTL `TXD`
   - `CEN (RST)` ➔ 出現 `Waiting for bootloader` 時短接 `GND` 約 0.2 秒放開
3. **備份出廠 2MB Flash**：
   ```bash
   ltchiptool flash read -d /dev/cu.usbserial-xxx -f bk72xx -s 0x0 -l 0x200000 factory_backup_2mb.bin
   ```
4. **燒錄 ESPHome 韌體**：
   ```bash
   ltchiptool flash write -d /dev/cu.usbserial-xxx -f bk72xx firmware.uf2
   ```
   首次刷入連上 Wi-Fi 後，後續皆可直接透過 ESPHome 無線 OTA 更新。

---

## 四、快速開始（範例配置）

請參考 [`examples/`](examples/) 目錄下的完整設定檔：
- **[`examples/tuya-s06-maxe-ac.yaml`](examples/tuya-s06-maxe-ac.yaml)**：Tuya S06 + 萬士益（美的 Coolix 48-bit）雙向同步冷氣。
- **[`examples/tuya-s06-hitachi-ac.yaml`](examples/tuya-s06-hitachi-ac.yaml)**：Tuya S06 + 台灣日立（Hitachi 344-bit）雙向同步冷氣。
- **[`examples/tuya-s11-ir-rf.yaml`](examples/tuya-s11-ir-rf.yaml)**：Tuya S11 + 紅外線冷氣 + CMT2300A 433.92MHz 射頻收發。

使用方式：
```bash
git clone https://github.com/<your-username>/esphome-tuya-s06-s11.git
cd esphome-tuya-s06-s11/examples
cp secrets.yaml.example secrets.yaml
# 編輯 secrets.yaml 填入 Wi-Fi SSID 與密碼
esphome run tuya-s06-maxe-ac.yaml
```
