# Ctrl_CV_2026

棒球機器人 — RP2350 雙核多節點控制韌體（Waveshare RP2350 Plus / Pico2）。

使用 PlatformIO + [Arduino-Pico](https://github.com/earlephilhower/arduino-pico)（RP2350），採 Hub ↔ Move / Shoot / Storage 三個 Slave 節點架構，全車以 COBS + Binary struct + CRC16 通訊。

本 README 記錄**各節點的腳位（GPIO）設定**與**遙控器通道對應參數**。完整開發規範見 [CLAUDE.md](CLAUDE.md)。

---

## 系統拓撲

```
Jetson（USB CDC）⇄ Hub（RP2350）⇄ Move / Shoot / Storage（PIO UART，各一組獨立 SM）
                       ⇄ CRSF 遙控接收機（UART0，GP1 RX）
Ultrasonic（獨立 Pico）— 透過 USB Serial 直接連接 Jetson
```

- **Hub ↔ Slaves**：PIO UART 全雙工（每條 Slave 一組獨立 PIO SM），921600 bps
- **Slave 通訊**：三顆 Slave 共用硬體 UART0（TX=GP0, RX=GP1）收 Hub 控制幀 + 回傳遙測
- **Remote（CRSF）**：Hub Core 1 專職解包，Jetson 斷線（>500ms）時作為控制備援

---

## Hub 腳位設定（RP2350 中央轉發節點）

Hub Core 0 同時負責 USB CDC 收發、PIO UART 對所有 Slave 的 TX/RX；Core 1 專職 CRSF 接收機。

| 功能 | 介面 | 腳位 | 交叉連接 |
|------|------|------|----------|
| Jetson 通訊 | USB CDC (Serial) | — | — |
| CRSF 遙控接收機 | UART0 RX | GP1 | ELRS 接收機 TX → GP1 |
| Move Slave 通訊 | PIO0 SM0 | TX=GP2, RX=GP3 | HUB TX(GP2) -> MOVE RX(GP1); HUB RX(GP3) -> MOVE TX(GP0) |
| Shoot Slave 通訊 | PIO0 SM1 | TX=GP4, RX=GP5 | HUB TX(GP4) -> SHOOT RX(GP1); HUB RX(GP5) -> SHOOT TX(GP0) |
| Storage Slave 通訊 | PIO0 SM2 | TX=GP6, RX=GP7 | HUB TX(GP6) -> STORAGE RX(GP1); HUB RX(GP7) -> STORAGE TX(GP0) |

> **注意**：CRSF @ 420000 baud。GP4/GP5 在 UART1（除錯）與 PIO SM1（Shoot 通訊）間衝突，實作上 PIO SM1 優先，請勿同時啟用 UART1 除錯。

---

## Move Slave 腳位設定（四輪麥克納姆輪）

四輪實測驗證腳位（依 `src/move/main.cpp` `Pin` namespace）：

| 方向 | 馬達 PWM | 馬達 DIR | 編碼器 A | 編碼器 B |
|------|----------|----------|----------|----------|
| **FR**（前右） | GP9 | GP8 | GP14 | GP15 |
| **FL**（前左） | GP7 | GP6 | GP16 | GP17 |
| **BR**（後右） | GP11 | GP10 | GP20 | GP21 |
| **BL**（後左） | GP13 | GP12 | GP18 | GP19 |

### 其他腳位 / 參數

- **通訊 UART0**：TX=GP0, RX=GP1（SlaveComms 內部處理，與 Hub 通訊）
- **狀態 LED**：板載 `LED_BUILTIN`（E-Stop→常亮, 已連線且<100ms 有效命令→閃爍, 斷線→熄滅）
- **PIO 資源**：PIO0 全部 4 SM 做四輪正交編碼器解碼；PIO1 未使用

| 參數 | 值 |
|------|-----|
| 減速比 | 1:27 |
| 實測線數 `ENC_LINE_PPR` | 443 |
| `ENC_COUNTS_PER_REV` | 443 × 4 = 1772 |
| 車身半長/半寬/輪半徑 (m) | 0.25 / 0.25 / 0.05 |
| 馬達極性 `kWheelPolarity[]` | `{1,1,-1,-1}`（輪序 FL/FR/BL/BR，BL/BR 反相） |
| 編碼器讀取極性 `kEncReadPolarity[]` | `{1,-1,+1,+1}`（輪序 FL/FR/BL/BR） |
| Strafe 軸方向 | `kAxisStrafePolarity = -1` |
| Forward 軸方向 | `kAxisForwardPolarity = -1` |
| 三軸 S 曲線加速 / 減速 | 4.0 / 12.0 RPS/s |
| 三軸 Jerk | 60.0 RPS/s² |
| PID 出力上限 `PID_MAX_DUTY` | 100% |
| 積分風扇餘裕 `WINDUP_MARGIN_DUTY` | 6% |
| PWM 頻率 | ~20 kHz（clkdiv=6.25, wrap=1000） |

---

## Shoot Slave 腳位設定（仰角 UART 步進 + 水平 STEP/DIR 步進 + 三顆 BLDC）

依 `src/shoot/main.cpp` `Pin` namespace：

| 功能 | 腳位 |
|------|------|
| 通訊 UART0 TX / RX | GP0 / GP1 |
| 仰角 步進 UART TX（`ELEV_UART_TX` → 驅動器 RX） | GP2 |
| 仰角 步進 UART RX（`ELEV_UART_RX` ← 驅動器 TX） | GP3 |
| UP 上射 BLDC（PWM） | GP4 |
| DOWN 下射 BLDC（PWM） | GP5 |
| ROW 水平 步進 STEP | GP6 |
| ROW 水平 步進 DIR | GP7 |
| ROW 水平 步進 ENA（低電位致能） | GP8 |
| FIXED 固定轉速 BLDC（PWM） | GP9 |

- **仰角 (pitch)**：UART 閉環步進驅動器（ZDT X42S / Emm 固件），PIO 模擬 UART（SerialPIO）115200 baud；3200 脈衝/圈（1.8° × 16 細分）、600 RPM、加速度檔 100；上電自動清零建立零位
- **水平 (row)**：傳統 STEP/DIR 驅動器（A4988 / DRV8825 / TB6600），STEP 由 PIO 產生；1.8°/step 全步進（200 step/rev）、最高速 600 step/s、加速度 4000 step/s²、DIR 建立時間 20µs
- 兩軸輸入限幅 ±97 → ±90°
- **無刷馬達**：50Hz servo-style PPM：**1500μs = 中立/停止，2000μs = 全速**；FIXED 固定轉速脈衝 `PULSE_FIXED` = 1750μs（可用 `Pin::kEnableBldc` 停用）
- upSpeed/downSpeed 為永久性速度設定（EEPROM 持久化，每次開機自動恢復）
- **PIO 資源**：PIO0 — 仰角 UART（SerialPIO）+ 水平 STEP 脈衝；PIO1 — 三顆 BLDC 50Hz 伺服脈衝（需 `pio_set_gpio_base(pio1, 0)`）

---

## Storage Slave 腳位設定（PCA9685 伺服）

依 `src/storage/main.cpp` `Pin` namespace（與 `mechanism_test` 相同）：

| 功能 | 介面 / 腳位 |
|------|-------------|
| 通訊 UART0 TX / RX | GP0 / GP1 |
| PCA9685 I2C0 SDA | GP4 |
| PCA9685 I2C0 SCL | GP5 |

- **PCA9685**：I2C0（SDA=GP4, SCL=GP5），預設位址 `0x40`；通道 0~15（實際啟用哪幾顆由 EEPROM 決定）
- 伺服脈衝範圍：600~2400μs（每通道可個別設定）

---

## Ultrasonic 節點腳位設定（獨立 Pico，8× US-100）

依 `src/ultrasonic/main.cpp`：GPIO Trigger/Echo 模式，PIO 量測 Echo 脈衝寬度，USB Serial 輸出。

| 感測器 | Trigger | Echo |
|--------|---------|------|
| FRONT_LEFT | GP2 | GP3 |
| FRONT_RIGHT | GP4 | GP5 |
| BACK_LEFT | GP6 | GP7 |
| BACK_RIGHT | GP8 | GP9 |
| LEFT_1（前） | GP10 | GP11 |
| LEFT_2（後） | GP12 | GP13 |
| RIGHT_1（前） | GP14 | GP15 |
| RIGHT_2（後） | GP16 | GP17 |

---

## 遙控器通道對應參數（ELRS CRSF）

Hub Core 1 以 420000 baud 接收遙控訊號，Jetson 斷線（>500ms）時作為控制備援。通道值域 **-100 ~ 100**，`rcToInt8()` 套用死區（-10~10 → 0）。

> 統一用語：RC 通道一律寫成 `rc[N]`，`N = 0~9`（0-based，程式碼中就是 `rc_channels[N]`）。
> 文件與程式都不再使用「遙控器 CHx」這種 1-based 說法，避免差一位。

| 通道 | 參數 | 對應動作 | 目標 Slave |
|------|------|----------|-----------|
| `rc[0]` | `vx` | 左右平移（方向 `VX_SIGN=-1`） | Move |
| `rc[1]` | `vy` | 前後移動（`VY_SIGN=+1`） | Move |
| `rc[2]` | `pitch` | 上下仰角 | Shoot |
| `rc[3]` | `row` | 左右水平 | Shoot |
| `rc[4]` | `upSpeed` / `downSpeed` | 投射輪啟停：< 0 → 開（轉速固定 50%），≥ 0 → 0（停轉） | Shoot |
| `rc[5]` | 控制來源 | < -10 → 遙控接管全部 Slave；其餘 → Jetson 持有全部控制權（遙控器只讀本開關） | Hub |
| `rc[6]` | `omega` / 轉速微調 | 上下射轉速：< -10 → +5%、> +10 → -5%（上下兩顆同時）<br>⚠ 同時也是 Move 的 omega | Shoot / Move |
| `rc[7]` | — | 保留 | — |
| `rc[8]` | `trigger` | < 0 → 全部伺服跑一次循環 | Storage |
| `rc[9]` | — | 保留（本版未使用） | — |

> 上下射轉速 = 固定基準 `50%` + `rc[6]` 每次 `±5%` 微調（微調量 ±50，等效 0~100%），
> 方向邊緣觸發；`RcShootCfg::HOLD_REPEAT_MS > 0` 可改成「按住連續微調」。
