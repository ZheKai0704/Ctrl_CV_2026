# 兩顆步進馬達綜合測試（stepper_combined_test）

用 USB Serial 同時檢視／控制兩顆步進馬達，兩個軸都用 PIO：

| 馬達 | 介面 | 腳位 | 控制方式 |
|---|---|---|---|
| A：UART 閉環步進（ZDT X42S / Emm） | UART 115200 | TX=GP2 → 馬達 T/B/L、RX=GP3 ← 馬達 R/A/H | PIO 模擬 UART（`SerialPIO`） |
| B：一般 EN/STEP/DIR 驅動器 | STEP/DIR/EN | STEP=GP6、DIR=GP7、EN=GP8 | PIO0 產生 STEP 脈衝 |

> 兩顆馬達與 RP2350 必須共地。EN 預設低電位致能，可用 `n` 切換。

## 建置 / 燒錄

```bash
pio run -e stepper_combined_test -t upload
pio device monitor -e stepper_combined_test     # 115200
```

## 命令（輸入後按 Enter）

| 命令 | 說明 |
|---|---|
| `h` / `?` | 說明 |
| `i` | 兩軸狀態 |
| `e` / `d` | 兩軸使能 / 去使能 |
| `t` | 自我測試（UART +90°→-90°→0°，水平 +1圈→-1圈→0） |
| `ue` / `ud` | UART 馬達使能 / 去使能 |
| `r` | UART 讀即時位置（0x36） |
| `z` | UART 目前位置清零（0x0A / 0x6D） |
| `p <deg>` | UART 絕對角度（相對零點） |
| `g <deg>` | UART 相對角度（以目前命令位置為基準） |
| `s` | UART 立即停止 |
| `v <rpm>` | UART 速度（預設 600） |
| `k <0~255>` | UART 加速度檔位（預設 100） |
| `F` | 切換位置幀格式（13B 第二代 / 12B 舊 Emm） |
| `a <steps>` | 水平絕對步數（0 = 開機位置） |
| `f <steps>` / `b <steps>` | 水平正轉 / 反轉相對步數 |
| `S` | 水平立即停止（同時取消未走完的目標） |
| `V <rpm>` | 水平轉速（預設 120） |
| `K <sps2>` | 水平加速度 steps/s²（預設 50000，0 = 不加減速） |
| `c <steps/rev>` | 水平每圈脈衝數（預設 3200 = 16 細分；全步進改 `c 200`） |
| `n` | 水平 EN 極性切換 |
| `D` | 水平 DIR 反向切換 |

## 單位換算

- **UART 馬達**：命令 3200 脈衝/圈（1.8° × 16 細分），`p 90` = 800 脈衝。
  回報的 0x36 數值單位是 65536/圈，程式會自動換成角度。
- **STEP/DIR 馬達**：步數 = `角度 / 360 × stepsPerRev`。
  預設 3200 步/圈；若驅動器是全步進（A4988 MS 全低）要 `c 200`，否則角度會差 16 倍。

## 注意

- UART 馬達開機時會送 `0x0A/0x6D` 把「當下位置」清零，所以 **0° = 開機位置**。
- STEP/DIR 是開迴路，`refSteps` 開機歸零，沒有極限開關；E-Stop/`S` 會依已送出的估算步數修正位置。
- STEP 脈衝由 PIO 產生（固定 7 週期/步），CPU 只調 clock divider 做加減速。
- STEP 到位中斷用 **PIO IRQ flag 4（PIO IRQ1）**，避開 `SerialPIO` 使用的 IRQ0，
  兩軸可以同時運作。
- 若 UART 馬達完全沒回覆：確認馬達 `P_Serial = UART`、baud 115200、共地、位址 1，
  並確認 **RP2350 TX(GP2) → 馬達 T/B/L、RP2350 RX(GP3) ← 馬達 R/A/H**。
  接反的話用 `Q` 打開 raw hex，會看到只送不收。
- `Q` 可切換 TX/RX 原始位元組顯示，用來確認 UART 到底有沒有通。
