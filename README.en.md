# ESPHome Tuya S06 / S11 (BK7238) Full-Duplex IR & 433MHz RF Components

[![Ko-fi](https://img.shields.io/badge/Ko--fi-Buy%20Me%20a%20Coffee-FF5E5B?style=for-the-badge&logo=ko-fi&logoColor=white)](https://ko-fi.com/sean758486)

🌐 **Language**: **[English](README.en.md)** | **[繁體中文](README.md)**

Production-grade **ESPHome (`libretiny` / `bk72xx`)** external components and register-level hardware PWM drivers for **Tuya S06 (IR-only)** and **Tuya S11 (IR + 433.92MHz RF)** universal remote controllers powered by the **Beken BK7238** SoC:

- **`hitachi_ac344`**: Full-duplex (two-way state synchronization) ESPHome Climate component for **Hitachi 344-bit (43-Byte)** air conditioners.
- **`maxe_coolix`**: Full-duplex ESPHome Climate component for **Midea Coolix 48-bit** air conditioners (used globally by **Midea, Carrier, Comfee, Pioneer, Kaysun, Maxe RC-62**, etc.), featuring a **120-pulse Sliding Window Header Search** to prevent ambient light noise from dropping physical remote presses.
- **`tuya_rf`**: Fixed and hardware-verified **CMOSTEK CMT2300A** 433.92MHz OOK RF transceiver component for **Tuya S11**, resolving wrong SPI pin mappings, floating SDIO reads, and inverted Mark/Space polarity in older community drivers.
- **[`s06_hitachi_hw.h`](s06_hitachi_hw.h)**: Register-level **38kHz Hardware PWM** carrier driver (`PWM0` for S06, `PWM3` for S11) with **Delta-Tick phase-compensated microsecond timing** and **Anti-Jamming 0V Ground Pull-Down protection**.
- **[`s11_fan_rf.h`](s11_fan_rf.h)**: 433.92MHz Ceiling Fan & Light full-duplex state engine supporting both **24-bit EV1527 (5-speed)** and **30-bit CA285 (6-speed DC inverter)** ceiling fans with physical remote interception, echo-loop suppression, and wall-switch power-off guard.

---

## 1. Key Hardware & Protocol Fixes

### 1.1 Anti-Jamming Protection: Fixing BK7238 Hardware PWM Stuck-HIGH Bug ("IR Blindness")
On the Beken BK7238 SoC, simply calling `pwm_stop()` halts the PWM counter without resetting the output flip-flop. If the counter stops during the HIGH duty cycle, the IR TX pin (`P6` on S06, `P9` on S11) remains **latched at 3.3V DC**. This turns the 6–7 high-power IR LEDs into a continuous "infrared flashlight", saturating both the AC receiver and the controller's own IR receiver (`P26`/`P24`) and rendering physical remotes completely unresponsive.

[`s06_hitachi_hw.h`](s06_hitachi_hw.h) directly manipulates the BK7238 PWM group control registers (`0x00802B00` / `0x00802B40`):
- Enforces `CMD_PWM_INIT_LEVL_SET_LOW` and clears `INT_LEVL_BIT` on initialization so the pin is guaranteed to sit at **0V (LOW)** whenever carrier output is disabled (`~8ns` register toggle).
- Uses an RAII **`RxInterruptMasker`** to mask GPIO interrupts and pause the CMT2300A 433MHz RF receiver during IR transmission, then atomically clears `S_GPIO_INTSTA_REG` afterward—eliminating 688+ self-received interrupt storms per frame.

### 1.2 Zero-Drift Microsecond Timing (Delta-Tick Accumulation)
Reads the BK7238 hardware **Timer0 (`0x00802A00`, 26MHz crystal where $1\mu\text{s} = 26\text{ ticks}$)** directly with cross-pulse phase error compensation (`delay_us_accurate`), fixing `delayMicroseconds()` boundary stretching on LibreTiny that otherwise causes long frames (such as 344-bit Hitachi frames) to fail checksum validation.

### 1.3 Tuya S11 (CMT2300A) 433.92MHz RF Pinout Truth Table
Hardware trace and logic analyzer verification confirm that the Tuya S11 operates the onboard **CMOSTEK CMT2300A** transceiver in **Direct Mode** with the following 6-pin mapping:

| Signal | CMT2300A Pin | BK7238 GPIO | Description |
| :--- | :--- | :--- | :--- |
| **SPI CSB** | Pin 8 | `P26` | Active-LOW register chip select |
| **SPI SCLK** | Pin 10 | `P14` | 3-Wire SPI clock |
| **SPI SDIO** | Pin 11 | `P16` (or `P6`) | Half-duplex data; switched to `INPUT_PULLUP` during reads |
| **FIFO FCSB** | Pin 9 | `P17` (or `P16`) | Kept HIGH in Direct Mode |
| **RF TX (DIN)** | Pin 5 (GPIO1) | `P23` | Direct microsecond pulse modulation input |
| **RF RX (DOUT)** | Pin 6 (GPIO2) | `P22` | Demodulated pulse interrupt input (`INPUT_PULLUP`) |

---

## 2. Tuya S06 vs. Tuya S11 Pinout Comparison

| Function | Tuya S06 (`t1-3s`) | Tuya S11 (`generic-bk7238-tuya`) |
| :--- | :--- | :--- |
| **IR TX (Infrared LEDs)** | `P6` (Hardware PWM0) | `P9` (Hardware PWM3, requires `-DS11_HARDWARE=1`) |
| **IR RX (Infrared Receiver)** | `P26` (`inverted: true`, `INPUT_PULLUP`) | `P24` (`inverted: true`, `INPUT_PULLUP`) |
| **Status LED (Blue)** | `P24` (`inverted: true`) | `P21` (`inverted: false`) |
| **Pairing Button** | `P28` (`inverted: true`, `INPUT_PULLUP`) | `P20` (`inverted: false`, `INPUT`) |

---

## 3. Flashing Guide (`ltchiptool`)

1. **Install `ltchiptool`**:
   ```bash
   pip install --upgrade ltchiptool
   ```
2. **USB-to-TTL Wiring (3.3V ONLY — Never connect 5V to 3V3!)**:
   - `3V3` ➔ USB-TTL `3.3V`
   - `GND` ➔ USB-TTL `GND`
   - `TX1 (P11)` ➔ USB-TTL `RXD`
   - `RX1 (P10)` ➔ USB-TTL `TXD`
   - `CEN (RST)` ➔ Briefly short to `GND` for ~0.2s when `Waiting for bootloader` appears
3. **Backup Factory 2MB Flash**:
   ```bash
   ltchiptool flash read -d /dev/cu.usbserial-xxx -f bk72xx -s 0x0 -l 0x200000 factory_backup_2mb.bin
   ```
4. **Flash ESPHome Firmware**:
   ```bash
   ltchiptool flash write -d /dev/cu.usbserial-xxx -f bk72xx firmware.uf2
   ```
   Once connected to Wi-Fi, all subsequent updates can be performed wirelessly via ESPHome OTA.

---

## 4. Quick Start (Example Configurations)

Ready-to-use YAML configurations are available in the [`examples/`](examples/) directory:
- **[`examples/tuya-s06-maxe-ac.yaml`](examples/tuya-s06-maxe-ac.yaml)**: Tuya S06 + Midea Coolix 48-bit (Midea / Carrier / Comfee / Maxe) two-way sync AC.
- **[`examples/tuya-s06-hitachi-ac.yaml`](examples/tuya-s06-hitachi-ac.yaml)**: Tuya S06 + Hitachi 344-bit two-way sync AC.
- **[`examples/tuya-s11-ir-rf.yaml`](examples/tuya-s11-ir-rf.yaml)**: Tuya S11 + IR AC + CMT2300A 433.92MHz RF transceiver.

Usage:
```bash
git clone https://github.com/seanyan1994/esphome-tuya-s06-s11.git
cd esphome-tuya-s06-s11/examples
cp secrets.yaml.example secrets.yaml
# Edit secrets.yaml with your Wi-Fi credentials
esphome run tuya-s06-maxe-ac.yaml
```

---

## ☕ Support & Donate (Buy Me a Coffee)

If this project saved you hours of debugging BK7238 hardware PWM, CMT2300A SPI pins, or AC IR protocols, consider buying me a coffee:

- **International (Credit Card / PayPal via Ko-fi)**:  
  [![Ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/sean758486)
- **Taiwan Local Transfer (TWQR / JKOPay)**:  
  <img src="assets/twqr_donate.png" alt="Buy Sean a Coffee - TWQR" width="280" />
