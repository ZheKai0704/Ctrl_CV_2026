# 機構動作測試（mechanism_test）

依觸發順序逐組：每組先逐顆 30°→120°→30° 驗證行程，再把整組開到 120° 並**保持開啟**；做完的組不會回 HOME，按 `q` 才全部回 30°。

## 行為

```
開機 / 停止：所有「已啟用」通道 → HOME（30°）

s（一次完整循環）、依觸發順序：
  g2(CH4,CH5) → g5(CH10,CH11) → g1(CH2,CH3) → g6(CH12,CH13) → g0(CH0,CH1) → g7(CH14,CH15)
  然後補上順序表外的 g3(CH6,CH7) → g4(CH8,CH9)

  每一組（以 g0 = CH0/CH1 為例）：
    CH0  30° → 120° → 停 1000ms → 30°     ← 逐顆驗證行程
    CH1  30° → 120° → 停 1000ms → 30°
    CH0 → 120°、CH1 → 120° 並保持          ← 整組開啟，之後不再動
    等 500ms 換下一組（前一組維持開啟）

  全部做完：所有組都停在 120°（維持開啟），按 `q` 才全部回 30°

S：同上但一直重複，q 停止
g 0：只跑第 0 組（CH0、CH1）
g  ：依觸發順序跑下一組（每次按自動換組：2 → 5 → 1 → 6 → 0 → 7 → 2）
```

- 每 **2 顆一組**：(0,1) (2,3) (4,5) (6,7) (8,9) (10,11) (12,13) (14,15)
- 組的執行順序與 `src/storage/main.cpp` 的 `SeqCfg::GROUP_ORDER` 一致：`{2, 5, 1, 6, 0, 7}`
- **停用通道會自動跳過**（目前 CH6~9，故此板實際只跑 `g2 g5 g1 g6 g0 g7`，g3/g4 需 `g 3` / `g 4`）
- 動作完成後**不再自動回 HOME**，整組、整輪都維持在 OUT（模擬實際機構「逐組開啟並保持」的狀態）
- 角度使用「邏輯角度」，沿用 `pca9685_angle` 同一份 EEPROM 校正：
  0° 零點、行程、反向、實體工作範圍、啟用狀態都會套用
- 全程非阻塞，Serial 隨時可下命令；LED 執行中快閃、閒置慢閃

## 建置 / 燒錄

```bash
pio run -e mechanism_test -t upload
pio device monitor -e mechanism_test     # 115200
```

## 命令

| 命令 | 說明 |
|---|---|
| `s` | 一次完整循環（依觸發順序，每組驗證完再整組開啟並保持） |
| `S` | 重複循環（再按 `q` 停止） |
| `g [n]` | 第 n 組（0~7）；不給 n = 依觸發順序下一組 |
| `q` | 停止，全部回 HOME |
| `m <home> <out>` | 設定角度（預設 `30 120`） |
| `t <dwell> <settle>` | OUT 停留 / 回位與換組等待 ms（預設 `1000 500`） |
| `p` | 狀態（角度、時間、啟用通道、下一組） |
| `l` | 各通道校正設定 |
| `x` | 緊急關閉全部輸出（伺服放鬆） |
| `r` | 重新開機 |
| `h` | 說明 |

> `settle` 現在是「整組做完後、換下一組前」的等待時間，預設 500ms。
> 若機構到位很快，可 `t 1000 0`；若會卡，調大。

## 自動化驗證

```bash
# 只讀設定，不動機構
~/.platformio/penv/bin/python tools/mechanism_test.py

# 實際跑完整循環並驗證順序 / 角度 / 停留時間
~/.platformio/penv/bin/python tools/mechanism_test.py --run

# 只測第 0 組（CH0/CH1）最安全
~/.platformio/penv/bin/python tools/mechanism_test.py --run --group 0

# 改參數
~/.platformio/penv/bin/python tools/mechanism_test.py --run --home 30 --out 120 --dwell 1000 --settle 500

# 燒錄 + 執行
~/.platformio/penv/bin/python tools/mechanism_test.py --upload --run
```

驗證內容：
- `[SEQ]` 動作順序是否為預期通道，且每組「逐顆驗證 → 整組開啟」兩段都在
- 每次 OUT / HOME / 開啟 角度是否正確
- OUT→HOME 間隔 ≈ `dwell`、HOME→下一顆 ≈ `settle`（容差 400ms）
- 結束後是否回到閒置（不再動作，各組保持開啟）

離開碼：`0` PASS、`1` FAIL、`2` 連線/設定問題。

## 安全提醒

- 第一次請用 `g 0` 只動兩顆，確認方向與行程後再跑 `s`
- 機構會撞到的話，先 `q` 或 `x`（`x` 會關閉輸出，伺服放鬆）
- 角度 30~120 是**邏輯角度**；奇數通道（CH1/3/5/7/9/11/13/15）
  由韌體強制反向（與 EEPROM `invert` 以 OR 合併），所以同一邏輯角度
  在左右成對的機構上會往對稱方向走
