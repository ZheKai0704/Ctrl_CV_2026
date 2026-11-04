# move_drive_test — Move 閉迴路行進診斷（馬達不動 / 單輪爆衝）

`move_dir_test` 驗的是「開迴路方向、腳位、極性」。
`move_drive_test` 驗的是**正式韌體實際跑的那一套閉迴路**：

- 車身 S 曲線（加/減速 + Jerk）
- 麥克納姆運動學（`kAxis*Polarity`）
- 編碼器 PIO（`clkdiv=500`、RX FIFO join，與 `src/move/main.cpp` 相同）
- 轉速量測一階低通 + 突波丟棄 + resync（`dt > 50ms`）
- 逐輪 PI + 抗風扇 + 前饋（固定 `ff` 或線上 RLS 自適應）
- 輸出斜率限制、`PID_MAX_DUTY` 飽和

因此可以直接重現「馬達不動」與「某一顆爆衝」，並用指令逐項排除。

## 建置 / 上傳

```bash
pio run -e move_drive_test -t upload
pio device monitor -e move_drive_test      # 115200
```

> `quad_phase.pio.h` 沿用 `src/move/`（此 env 已加 `-I src/move`）。

## 先跑 `probe`

```
probe
```

逐輪做兩段量測（預設每段 1.2 秒，車身請架空）：

1. **OL**：開迴路 15%（只看馬達 / 驅動 / 電源 / 編碼器）
2. **CL**：閉迴路 +0.6 RPS（看 PID / 回授方向）

`probe` 現在也會在每輪印出 `fifoFull` / `spike` 增量：

- **`fifoFull` 必須是 +0**。只要不是 0，代表 PIO 編碼器 FIFO 溢位、有相位被丟掉，
  該輪的 RPS / 判定都不可信，先修硬體/PIO 再看結論。
- `spike` 持續增加代表單筆差分超過 `WHEEL_MAX_RPS`，回授有雜訊。

最後印出判定表，先把問題歸類：

| 現象 | 判定 | 下一步 |
|------|------|--------|
| OL、CL 都沒轉 | 馬達電源 / 驅動 / 腳位 / 編碼器 | 先跑 `move_dir_test` 的 `w / dirtest / cal / cycle / sweep` |
| OL 不轉、CL 有轉 | PWM/DIR 腳位或驅動異常 | 對照 `map`，跑 `move_dir_test` 的 `dirtest` |
| OL 會轉、CL 不動 | PID 方向或回授掛 | 看 `status` 的 `out/app/meas`、`spk`、`fifo` |
| CL 反向 | `kEncReadPolarity` 可能反了（正回饋） | `encpol <輪> <±1>` 後重跑 `probe` |
| CL 超速 | 編碼器掉計數 / 極性錯 | 看 `fifo`、`spk`，必要時 `encdiv 100` 再測 |

## 重現症狀

```
cl fwd 0.8        # 正式閉迴路，車身 0.8 RPS
log on            # 20Hz CSV：每輪 target / meas / p / i / ff / out / applied / dcount / dt / fifo_max
```

- 某輪突然 `app=+1000`（100%）、`meas` 遠超 `tgt` → 對照該輪 `encpol`、`spk`、`fifo`。
- 某輪 `app` 有出力但 `meas≈0` → 會自動印 `[STALL]`（1 秒警告、3 秒停）。
- 目標與回授長期反向 → 會自動印 `[POL?]`（0.5 秒判定）。
- 實測遠超目標或零命令自轉 → `[BURST]`（0.6 秒自動停）。
- `[FIFO]` → PIO 編碼器 RX FIFO 滿、掉計數；`encdiv 100` 或提高 drain 頻率再測。
- `[RESYNC]` → 控制週期 > 50ms（掉幀 / `tick > 50`）；該週期會歸零後重新同步。

### 常用對照實驗

```
rls off           # 關掉線上自適應前饋（排除 RLS 學壞造成爆衝）
pid fl 2.40 10.4  # 即時改單輪增益
slew 500          # 更嚴的輸出斜率限制
lpf 20            # 更慢的轉速濾波（看是否為量測雜訊導致）
tick 20           # 模擬控制幀變慢（20ms，仍 < 50ms）
drop 6            # 模擬掉幀 60ms → 下一週期 resync（重現「抖一下 / 不動」）
hold fl 25        # 用「等效 25% duty」當目標，單輪閉迴路比對四輪
allhold 25        # 四輪同一個等效 duty 目標（車子會原地旋轉）
enc               # 手轉輪子看原始計數
```

## 指令表

| 指令 | 說明 |
|------|------|
| `ol <dir> [duty%]` | 開迴路，預設 25%，上限 `cap` |
| `cl <dir> [rps]` | 閉迴路車身速度，預設 0.8 RPS |
| `spin <輪> <rps>` | 閉迴路單輪目標輪轉速 |
| `hold <輪> <duty%>` | 閉迴路單輪，用等效 duty% 指定目標 |
| `allhold <duty%>` | 四輪同一個等效 duty% 目標 |
| `stop` / `s` / 空行 | 停止（PWM=0、積分歸零） |
| `reset` | 停止 + PID / RLS / 計數全部重置 |
| `probe [ms]` | 逐輪 OL vs CL 自動診斷 |
| `tick <ms>` | 控制週期 5~100（預設 10） |
| `drop <n>` | 模擬掉幀 n 個控制週期 |
| `rls on\|off` | 線上自適應前饋開關 |
| `pid <輪> <kP> <kI> [ff]` | 即時改單輪增益 |
| `cap <duty%>` | 測試出力上限（預設 60） |
| `slew <duty%/s>` | 輸出斜率限制（預設 1500） |
| `lpf <ms>` | 轉速量測低通（預設 10ms） |
| `encdiv <v>` | 編碼器 PIO clkdiv（預設 500；越大取樣越慢） |
| `pol` / `encpol <輪> <-1\|1>` | 改馬達 / 編碼器極性 |
| `axis <strafe\|fwd\|rot> <-1\|1>` | 改車體軸向極性 |
| `log on\|off` / `log <hz>` | CSV 記錄（預設 20Hz，最高 100） |
| `enc` | 待機手轉輪子看原始計數 |
| `status` / `map` / `report` / `help` | 狀態 / 腳位 / 參數 / 說明 |

## CSV 欄位

```
# ms,wheel,tgt_mrps,meas_mrps,p_cd,i_cd,ff_cd,out_cd,app_cd,dcount,dt_us,fifo_max
```

- `*_mrps`：×1000 的 RPS（`+800` = +0.8 RPS）
- `*_cd`：×10 的 duty%
- `dcount`：本控制週期的編碼器差分
- `dt_us`：本週期實際 dt
- `fifo_max`：該輪觀測到的 PIO RX FIFO 最大深度（8 = 曾滿）

## 安全

- 預設出力上限 60%；長轉請架空車身。
- 堵轉 1 秒警告、3 秒自動停止；爆衝持續 0.6 秒自動停止。
- `probe` 為阻塞式，按任意鍵即中止；中止後一律 PWM=0。
