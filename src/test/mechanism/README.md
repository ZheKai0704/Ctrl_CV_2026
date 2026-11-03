# 機構動作測試（mechanism_test）

HOME 30° → 依序逐顆到 OUT 120° 停留 1 秒 → 回 HOME 30°，用來測機構。

## 行為

```
開機 / 停止：所有「已啟用」通道 → HOME（30°）

s（一次完整循環）
  CH0 → 120°，1000ms → 30°，500ms
  CH1 → 120°，1000ms → 30°，500ms
  ...
  CH15 → 120°，1000ms → 30°，500ms
  → 全部回 HOME，結束

S：同上但一直重複，q 停止
g 0：只跑第 0 組（CH0、CH1）
g  ：依序跑下一組（每次按自動換組：0 → 1 → … → 7 → 0）
```

- 每 **2 顆一組**：(0,1) (2,3) (4,5) (6,7) (8,9) (10,11) (12,13) (14,15)
- **停用通道會自動跳過**（目前 CH6~9），所以完整循環實際是
  `CH0 CH1 CH2 CH3 CH4 CH5 CH10 CH11 CH12 CH13 CH14 CH15`
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
| `s` | 一次完整循環（所有啟用通道，依序） |
| `S` | 重複循環（再按 `q` 停止） |
| `g [n]` | 第 n 組（0~7）；不給 n = 依序下一組 |
| `q` | 停止，全部回 HOME |
| `m <home> <out>` | 設定角度（預設 `30 120`） |
| `t <dwell> <settle>` | 停留 / 回位等待 ms（預設 `1000 500`） |
| `p` | 狀態（角度、時間、啟用通道、下一組） |
| `l` | 各通道校正設定 |
| `x` | 緊急關閉全部輸出（伺服放鬆） |
| `r` | 重新開機 |
| `h` | 說明 |

> `settle` 是「回 HOME 後等它真的到位，再換下一顆」的時間，預設 500ms。
> 若機構回位很快，可 `t 1000 0`；若會卡，調大。

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
- `[SEQ]` 動作順序是否為預期通道
- 每次 OUT / HOME 角度是否正確
- OUT→HOME 間隔 ≈ `dwell`、HOME→下一顆 ≈ `settle`（容差 400ms）
- 結束後是否回到閒置（全部 HOME）

離開碼：`0` PASS、`1` FAIL、`2` 連線/設定問題。

## 安全提醒

- 第一次請用 `g 0` 只動兩顆，確認方向與行程後再跑 `s`
- 機構會撞到的話，先 `q` 或 `x`（`x` 會關閉輸出，伺服放鬆）
- 角度 30~120 是**邏輯角度**；CH1/3/5/11/13/15 為反向，
  實體位置和正向通道是對稱的（同一邏輯角度 → 同一方向）
