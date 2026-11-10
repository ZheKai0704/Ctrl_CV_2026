// ============================================================================
// Move Slave — 四輪麥克納姆輪，四輪速度閉迴路 PID (RP2350)
//
// 依 CLAUDE.md 雙核架構：Hub → COBS 控制幀 → Move UART0 收命令。
//
// ### 控制核心（Core 1, onMoveCommand）###
//   車身速度 [vx,vy,ω] (RPS) → 麥克納姆運動學 → 四輪目標線速度(m/s)
//     → 目標輪轉速 targetRps (RPS = rev/s)
//   每輪: 速度閉迴路 PID（編碼器 RPS 回授 → 前饋 ff + PI + 抗飽和 → duty%）
//   → 硬體 PWM + DIR
//   四輪 PID 由實測（綁 PIO 架構）依序調出，見 kPidCfg[4]。
//
// ### 通訊 ###
//   Hub PIO UART SM0 (GP2 TX, GP3 RX) ↔ Move UART0 (TX=GP0, RX=GP1)
//   921600 baud, COBS 封幀 + CRC16, SlaveComms 框架
//
// ### 編碼器 ###
//   PIO0 4 個 SM，每 SM 一顆輪，硬體採樣相位、push 到 RX FIFO（無 GPIO 中斷）。
//   主迴圈( Core 1 control 週期) drain FIFO + Gray 表累計，並套
//   kEncReadPolarity 統一方向。PIO 取樣沿用實測驗證成功之設定：
//     pio_gpio_init + `in pins,2` left-shift + clkdiv=5000 濾回彈。
// ============================================================================
#include "SlaveComms.h"
#include "quad_phase.pio.h"
#include <AdaptiveFeedforward.h>
#include <Arduino.h>
#include <DebugFmt.h>
#include <MotionProfile.h>
#include <StatusLed.h>
#include <cmath>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>

// ============================================================================
// 腳位定義（2026 新板，move_pinmap / move_dir_test 實測驗證）
// ============================================================================
namespace Pin {
// 硬體 UART0 — 與 Hub 通訊（SlaveComms 內部處理 TX=GP0, RX=GP1）

// PWM + DIR for 4 wheels（新板實測：FL 7/6, FR 9/8, BL 13/12, BR 11/10）
//   注意：新板的 DIR 與 PWM 腳位與舊板互換（舊板 FR 是 PWM=6/DIR=7）。
//   BR 曾經把 PWM/DIR 寫反而造成「duty 越大轉越慢」（DIR 收到 20kHz PWM，
//   馬達看到的是 |50%-duty| 的淨電壓）。已由 w br 5/10/15/25/40 掃描確認。
constexpr uint MOTOR_FR_PWM = 9;
constexpr uint MOTOR_FR_DIR = 8;
constexpr uint MOTOR_FL_PWM = 7;
constexpr uint MOTOR_FL_DIR = 6;
constexpr uint MOTOR_BR_PWM = 11;
constexpr uint MOTOR_BR_DIR = 10;
constexpr uint MOTOR_BL_PWM = 13;
constexpr uint MOTOR_BL_DIR = 12;

// Encoder base pins（base & base+1，A=base）；新板實測：
// FR=14,15 FL=16,17 BR=20,21 BL=18,19
constexpr uint MOTOR_FR_ENC = 14; // FR: A=14, B=15
constexpr uint MOTOR_FL_ENC = 16; // FL: A=16, B=17
constexpr uint MOTOR_BR_ENC = 20; // BR: A=20, B=21
constexpr uint MOTOR_BL_ENC = 18; // BL: A=18, B=19
} // namespace Pin

// ============================================================================
// 參數
// ============================================================================
namespace MoveCfg {
// 硬體 PWM wrap（解析度）— clkdiv=6.25, wrap=1000 → ~20kHz
constexpr uint16_t PWM_WRAP = 1000;
// 麥克納姆：車身半長/半寬/輪半徑 (m)
constexpr float HALF_LENGTH = 0.25f;
constexpr float HALF_WIDTH = 0.25f;
constexpr float WHEEL_RADIUS = 0.05f;

// 編碼器
constexpr uint16_t ENC_LINE_PPR = 443;                    // 實測線數
constexpr uint16_t ENC_COUNTS_PER_REV = ENC_LINE_PPR * 4; // 4X → 1772
// 每輪實際計數/圈（4X）。目前四輪實測同型（probe 相對比例差 ±2%），
// 全部沿用 1772；若日後換到不同線數的編碼器，個別填入實測值即可。
// 量法：move_drive_test 的 `enc` 手轉「剛好一圈」讀 raw Δ，或 `cpr`。
static constexpr uint16_t kEncCountsPerRev[4] = {
    ENC_COUNTS_PER_REV, ENC_COUNTS_PER_REV,
    ENC_COUNTS_PER_REV, ENC_COUNTS_PER_REV}; // [FL, FR, BL, BR]

// 閉迴路 PID 出力上限（duty%）。
//   目前為全開 100%：「命令+ → 馬達最多 100%」。
//   PID_MAX_DUTY 直接作為 ± 飽和上限，並負責當 PID 發散時封頂，避免超物理範圍出力。
constexpr float PID_MAX_DUTY = 100.0f;
// 積分放寬「風扇」餘裕（duty%）：積分上下限 = |ff*target| + 此值。
//   讓積分只補足命令前饋以下的小誤差，避免低速時積分風扇到 ±100
//   造成爆衝、也避免歸零時一次性大幅跳變。若慢速摩擦力需更多出力再加大。
constexpr float WINDUP_MARGIN_DUTY = 6.0f;
// 速度量測一階低通時間常數 (s)
constexpr float VEL_LPF_TAU = 0.010f;

// 停車指令判定閾值 (RPS)：目標輪轉速低於此值視為「停」，
//   強制清空 PID 積分，避免殘留積分造成零指令下馬達緩慢蠕動。
constexpr float StopCmdEpsRps = 0.005f;

// ---- 開機 / 斷訊後的安全護欄 ----
// 控制週期上下限 (s)：跨幀超過 DT_MAX_S 視為「剛開機、E-Stop 後、掉幀」
//   → 該週期不採信編碼器差分，只重新同步狀態（避免幽靈轉速造成暴衝）。
constexpr float DT_MIN_S = 1e-4f;
constexpr float DT_MAX_S = 0.05f;
// 物理合理輪轉速上限 (RPS)：單筆編碼器差分換算超過此值 → 判定為突波丟棄。
//   實體上限約等於 PID 命令上限 MAX_RPS(7)，留一倍餘裕。
constexpr float WHEEL_MAX_RPS = 15.0f;
// 輸出斜率限制 (duty% / s)：安全網。不論回授突波、模型更新或數值異常，
//   單一控制週期內的 duty 變化量都受限，不可能從 0 直接跳到滿載暴衝。
//   1500 %/s → 10ms 週期最多 15%，0→100% 約 67ms。
constexpr float OUTPUT_SLEW_DUTY_PER_S = 1500.0f;

// ---- 車身 S 曲線速度剖面（SCurveRamp）參數 ----
//   單位：車身速度為 RPS，加速度 RPS/s，Jerk RPS/s²。
//   目標行為：
//     起步（從 0）→ 慢段小坡克服靜摩擦 → 中段加速 → 平滑到位
//     減速（往 0）→ 快速但 Jerk 限幅、不瞬停
//   這些值需在實測車上調（依車重/摩擦/PID）；以下為初始合理值。
constexpr float SCURVE_ACCEL = 4.0f;           // 加速上限 (RPS/s) — 平緩加速
constexpr float SCURVE_DECEL = 12.0f;          // 減速上限 (RPS/s) — 快速減速(3×加速)
constexpr float SCURVE_JERK = 60.0f;           // Jerk 上限 (RPS/s²) — S 形圓角
constexpr float SCURVE_START_SLOW_VEL = 0.20f; // 起步慢段速度窗 (RPS)
constexpr float SCURVE_START_ACCEL = 1.5f;     // 起步加速上限 (RPS/s) — 慢提克服靜摩擦
constexpr float SCURVE_SETTLE_TOL = 0.010f;    // 到位判定 (RPS)
} // namespace MoveCfg

// ============================================================================
// 線上自適應 FF（RLS）— 方案 A 參數
//   取代固定 pid.ff：FF = a*targetRps[i]。a 由逐輪遞迴最小平方法
//   (Recursive Least Squares, 遺忘因子 λ) 持續線上更新，模型自動適應
//   「目前電池電壓」，無需電壓感測器。
//
//   模型形式（每輪）：duty ≈ a*rps（rps 為輪轉速，單一參數）
//      a = 增益倒數（duty%/RPS）。
//   （原本的 b 截距在等速時與 a 共線、無法辨識，會亂飄導致前饋失準，已移除。）
//   訓練樣本：(measRps, outDuty)——僅當「追蹤穩定 + 實際移動 + 未飽和」
//   時採樣，讓 RLS 把殘差收斂成真正的「duty→轉速」逆增益 + 摩擦。
//
//   安全網保留：合成後仍經 ±PID_MAX_DUTY clamp、NaN 防護、風扇餘裕，
//   模型輸出必須在既有物理界限內（詳見 onMoveCommand）。
// ============================================================================
namespace RlsCfg {
// 遺忘因子 (0,1)：愈小追得愈快但對雜訊愈敏感；0.995 兼顧追漂移與穩健。
constexpr float LAMBDA = 0.995f;
// 初始協方差（大 → 開機前幾秒快速適應目前電壓/地板）。
constexpr float P_INIT = 1000.0f;
// 協方差上下限：防退化（失去適應力）與防爆炸（數值炸裂）。
constexpr float P_MIN = 1e-3f;
constexpr float P_MAX = 1e4f;
// 僅量測轉速高於此值才採樣：避開零速/低於靜摩擦的雜訊樣本。
constexpr float MIN_SPEED_RPS = 0.02f;
// 追蹤視為穩定：|target - meas| < 此值才採樣，剔除暫態污染。
constexpr float SETTLE_TOL_RPS = 0.05f;
// 增益 / 截距物理界限（duty%，防模型越界）。
constexpr float A_MIN = 0.0f;
constexpr float A_MAX = 80.0f;
constexpr float B_MIN = -15.0f;
constexpr float B_MAX = 15.0f;
} // namespace RlsCfg

// 每輪 RLS 狀態（靜態分配；[FL, FR, BL, BR]）。
//   演算法本體（update / feedforward）在 AdaptiveFeedforward.h（純數學），
//   本檔只保留調參（RlsCfg）與狀態；RlsCfg 於 setup() 組成 rlsCfg 傳入。
static RlsState rls[4];
static RlsConfig rlsCfg; // setup() 由 RlsCfg 填入，之後唯讀

// ============================================================================
// 四輪閉迴路 PID 參數（由實測綁 PIO 架構依序調出；單位 duty% per
// RPS）
//   輪序 = [FL, FR, BL, BR]
// ============================================================================
struct WheelPidCfg {
    float kP, kI, kD, ff;
};
static const WheelPidCfg kPidCfg[4] = {
    {2.40f, 10.4f, 0.0f, 33.0f}, // FL
    {2.11f, 8.80f, 0.0f, 33.0f}, // FR
    {2.09f, 8.70f, 0.0f, 33.0f}, // BL
    {2.40f, 10.4f, 0.0f, 33.0f}, // BR
};

// 每輪極性（新板 move_dir_test 實測），輪序 = [FL, FR, BL, BR]。
//   麥克納姆前進時運動學得到 FL=+, FR=-, BL=-, BR=+（左右鏡像），本表再
//   依實際馬達安裝方向修正，使 fwd 時四輪物理上都往車頭。
static const int8_t kWheelPolarity[4] = {1, 1, -1, -1};
// 編碼器讀取極性（move_dir_test `cal` / move_drive_test `probe` 實測）：
//   讓「韌體命令 + → 回授 +」四輪一致，形成自洽閉環。
//   ⚠ BR 舊值 -1：修正 BR PWM/DIR 腳位後馬達正轉方向反了，
//     命令 + 但回授為 -（正回饋，積分反向風扇）。probe 的 OL 15% 實測
//     四輪應同號 → BR 改為 +1（move_dir_test 的 cal 可重新量測）。
static const int8_t kEncReadPolarity[4] = {1, -1, +1, +1};

// 車身兩條平移軸（vx = 左右 strafe、vy = 前後 forward）各自獨立方向開關。
//   依實測車身方向微調，每條軸獨立 +1/-1，互不干擾 ω（旋轉）。
//   ⚠ 四輪閉迴路為逐輪自洽伺服，且方向與 ω 皆經由同一組輪速組合產生，
//     因此只能「一次翻一條軸」在車上實測確認，勿兩條同時猜。
//   新板實測：fwd/back 正常、left/right 相反 → strafe 軸改 -1（forward 維持 -1）。
static const int8_t kAxisStrafePolarity = -1;
static const int8_t kAxisForwardPolarity = -1;

// ============================================================================
// 車身 S 曲線速度剖面（jerk-limited，非對稱加/減速）
//   每車身軸（vx / vy / omega）一套，單位皆 RPS（加速度 RPS/s）。
//   作用於原始指令 → 平滑後指令，再餵給運動學/PID。
//   目標：
//     起步（從 0）→ 慢段小坡克服靜摩擦 → 中段加速 → 平滑到位
//     減速（往 0）→ 快速但 Jerk 限幅、不瞬停
//   穩態追蹤時 y==target（無誤差、無積分風扇）；僅在目標改變時作用。
//   演算法本體在 MotionProfile.h（純數學，不含車體常數）；MoveCfg::SCURVE_*
//   的調參值於 setup() 以 configure() 傳入。
// ============================================================================
static SCurveRamp rampVx, rampVy, rampOmega; // 車身三軸各一套

// ============================================================================
// 四輪狀態
// ============================================================================
struct WheelState {
    uint pwmPin;
    uint dirPin;
    uint encPin;             // encoder base pin
    float targetDuty = 0.0f; // 目標 duty%（未直接使用，PID 直接算 out）
    float outDuty = 0.0f;    // PID 算出的 duty%（限幅前，供 RLS/遙測除錯）
    float appliedDuty = 0.0f; // 經斜率限制後實際送到 PWM 的 duty%（含方向）
    int32_t encCount = 0;    // 編碼器計數（Host 端累計）
    float measRps = 0.0f;    // 低通後量測轉速（RPS）
    float integral = 0.0f;   // PI 積分項
    float prevMeas = 0.0f;   // 微分用（kD=0 未用）
    bool hasPrev = false;
};
static WheelState wheels[4];
static uint64_t lastTickUs = 0; // Core 1 控制週期時間戳

// ============================================================================
// 四輪編碼器 — PIO 讀取（PIO0 4 SM，無 GPIO 中斷）
//   使用實測驗證成功之架構：
//     - pio_gpio_init 綁腳（RP2350 必需）
//     - `in pins,2` 用 shift_right=false（left shift，base pin 落 bit0）
//     - clkdiv=5000 降採樣 ~25kHz 濾機械回彈
//     - Core 1 控制週期 drain FIFO + Gray 表累計，並套 kEncReadPolarity
//     統一方向
// ============================================================================
static PIO g_pio = pio0;
static uint g_pioOffset = 0;

// host-side 有向累計計數 + 上一相位（已套 kEncReadPolarity 統一符號）
static volatile int32_t enc[4];
static uint8_t encPrev[4];

// 正交解碼表（Gray）。索引 = ab 相位，非相鄰跳變 = 0。
static const int8_t kQuadTable[4][4] = {
    {0, +1, -1, 0},
    {-1, 0, 0, +1},
    {+1, 0, 0, -1},
    {0, -1, +1, 0},
};

// drain FIFO：把 PIO 推進來的相位取樣，逐一累計 to enc[]（套極性）。
//   每輪最多 MAX_DRAIN 筆即停，避免無限追尾卡死。Core 1 控制週期呼叫。
static void drainEncoders() {
    // 上限拉高：PIO 只在相位「變化」時 push，高速時每控制週期可能上百筆。
    // 真正的防護是 setIdleHook(drainEncoders)：讓 Core 1 在 spinloop 中
    // 持續吸血，避免 4~8 深的 RX FIFO 溢出掉計數（掉計數 → RPS 低估
    // → PID 狂輸出 → 車子一衝一衝不順）。
    constexpr int MAX_DRAIN = 512;
    for(int i = 0; i < 4; i++) {
        int n = 0;
        while(!pio_sm_is_rx_fifo_empty(g_pio, i) && n < MAX_DRAIN) {
            const uint32_t cur = pio_sm_get(g_pio, i) & 0x3;
            enc[i] += kEncReadPolarity[i] * kQuadTable[encPrev[i]][cur];
            encPrev[i] = static_cast<uint8_t>(cur);
            n++;
        }
    }
}

static void encoderInit() {
    g_pioOffset = pio_add_program(g_pio, &quad_phase_program);
    // 依 wheels 順序 (FL,FR,BL,BR) 初始化 enc[]：與 wheels[i]/運動學對應
    constexpr uint encPin[4] = {Pin::MOTOR_FL_ENC, Pin::MOTOR_FR_ENC,
                                Pin::MOTOR_BL_ENC, Pin::MOTOR_BR_ENC};
    for(int i = 0; i < 4; i++) {
        const uint aPin = encPin[i];
        const uint bPin = aPin + 1;
        gpio_init(aPin);
        gpio_init(bPin);
        gpio_set_dir(aPin, GPIO_IN);
        gpio_set_dir(bPin, GPIO_IN);
        gpio_pull_up(aPin);
        gpio_pull_up(bPin);
        gpio_set_input_hysteresis_enabled(aPin, true); // 抗 PWM 耦合/擦振
        gpio_set_input_hysteresis_enabled(bPin, true);
        pio_gpio_init(g_pio, aPin);
        pio_gpio_init(g_pio, bPin);

        pio_sm_config c = quad_phase_program_get_default_config(g_pioOffset);
        sm_config_set_in_pins(&c, aPin);              // base pin = A
        sm_config_set_in_shift(&c, false, false, 32); // left shift（關鍵）
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX); // 只用 RX → 8 筆緩衝
        // 取樣率：150MHz / 500 = 300kHz SM clock，loop 約 5 指令
        //   → 約 60kHz 取樣。原本 clkdiv=5000 只有約 6kHz，在 1 RPS 以上
        //   就開始漏正交相位（1772 counts/rev；7 RPS ≈ 12.4k transitions/s），
        //   造成轉速低估 → PID 飽和 → 起步/行進抽搐。
        sm_config_set_clkdiv(&c, 500.0f);

        pio_sm_init(g_pio, i, g_pioOffset, &c);
        pio_sm_clear_fifos(g_pio, i);
        pio_sm_set_enabled(g_pio, i, true);

        enc[i] = 0;
        const uint8_t a = gpio_get(aPin) ? 1u : 0u;
        const uint8_t b = gpio_get(bPin) ? 1u : 0u;
        encPrev[i] = static_cast<uint8_t>(a | (b << 1)); // 與 PIO cur 一致
    }
}

// ============================================================================
// 前向宣告
// ============================================================================
static void onMoveCommand(const ChassisSnapshot &cmd);
static void onMoveEstop();

// ============================================================================
// SlaveApp 回呼 — SlaveComms 在 Core 1 呼叫
// ============================================================================
static SlaveApp moveApp = {
    .targetID = TargetID::MOVE,
    .onCommand = onMoveCommand,
    .onEstop = onMoveEstop,
};

static SlaveComms slave(&moveApp);

// ============================================================================
// 命令除錯快照 — Core 1 寫入，Core 0 印出（遵循專案規範）
// ============================================================================
static volatile float dbg_vx = 0.0f;
static volatile float dbg_vy = 0.0f;
static volatile float dbg_omega = 0.0f;
static volatile uint32_t dbg_cmdCount = 0; // Core 1 實際執行 onCommand 的次數
static volatile float dbg_measRps = 0.0f;  // FL(輪0) 量測轉速
static volatile float dbg_outDuty0 = 0.0f; // FL(輪0) 實際輸出 duty%
static uint8_t tmFrameSeq = 0;             // 遙測幀序號（Core 1 遞增）

// 本 core 的 printf / Serial.printf 不支援 %f（會印成空白），
// 因此序列監控一律用 ×100 整數輸出（vx/vy/omega/RPS = x0.01，
// 實作見 DebugFmt::scaled100）。

// ============================================================================
// 硬體 PWM 輸出 + DIR（對齊 motor.h 行為）
//   - duty>=0 → DIR=LOW(正轉)；duty<0 → DIR=HIGH(反轉)，輸出 |duty|
//   - 輸入為 duty%（-100~100）；極性已在呼叫端乘以 kWheelPolarity[]
// ============================================================================
static void wheelSetPwm(uint pwmPin, uint dirPin, float dutyPercent) {
    // NaN 防護：NaN 經比較/轉型會產生未定義的 duty（可能直接滿載輸出）。
    //   任何非有限值一律視為 0，寧可不出力也不可暴衝。
    if(!std::isfinite(dutyPercent))
        dutyPercent = 0.0f;

    bool fwd = (dutyPercent >= 0.0f);
    gpio_put(dirPin, fwd ? 0 : 1); // LOW=正轉, HIGH=反轉
    float a = fabsf(dutyPercent);
    if(a > 100.0f)
        a = 100.0f;
    uint16_t level = static_cast<uint16_t>(a / 100.0f * MoveCfg::PWM_WRAP);
    pwm_set_gpio_level(pwmPin, level);
}

static void initWheelPwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 6.25f); // 125MHz / (1000+1) / 6.25 ≈ 20kHz
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_FREE_RUNNING);
    pwm_config_set_wrap(&cfg, MoveCfg::PWM_WRAP);
    pwm_init(slice, &cfg, false);
    pwm_set_gpio_level(gpio, 0);
    pwm_set_enabled(slice, true);
}

// ============================================================================
// 馬達腳位表 — 靜態常數（不靠 setup() 執行時才填）
//   關鍵：Core 1 在 Core 0 跑 setup() 的同時就已經在 slave.runCore1()
//   spinloop（arduino-pico 先 launch core1 再呼叫 setup()），因此
//   onMoveEstop() 可能在 wheels[] 尚未填 pin 欄位時就被呼叫；
//   若 pin 欄位還是 0，就會亂寫 GP0／PWM slice 0。
//   以下用 constexpr 表 + setup() 第一行就初始化，消除這個競態窗口。
// ============================================================================
static constexpr uint kPwmPins[4] = {Pin::MOTOR_FL_PWM, Pin::MOTOR_FR_PWM,
                                     Pin::MOTOR_BL_PWM, Pin::MOTOR_BR_PWM};
static constexpr uint kDirPins[4] = {Pin::MOTOR_FL_DIR, Pin::MOTOR_FR_DIR,
                                     Pin::MOTOR_BL_DIR, Pin::MOTOR_BR_DIR};
static constexpr uint kEncPins[4] = {Pin::MOTOR_FL_ENC, Pin::MOTOR_FR_ENC,
                                     Pin::MOTOR_BL_ENC, Pin::MOTOR_BR_ENC};

// ============================================================================
// 馬達硬體安全初始化 — setup() 的「第一個」執行動作
//   目的：把 PWM/DIR 從「未定義（輸入/浮動）」推進「確定零出力」，
//   並把 wheels[] 的 pin 欄位在「任何可能寫 PWM 的程式碼之前」就備妥。
//   必須早於 Serial.begin()/delay()，否則那段時間馬達腳位是未定義的。
// ============================================================================
static void initMotorHardware() {
    for(int i = 0; i < 4; i++) {
        // 1) DIR 先當普通 GPIO 輸出，先給定 LOW（正轉）而非懸空
        gpio_init(kDirPins[i]);
        gpio_set_dir(kDirPins[i], GPIO_OUT);
        gpio_put(kDirPins[i], 0);

        // 2) PWM 腳先當普通輸出拉低 → 就算 PWM 硬體還沒設好也確定是 0V
        gpio_init(kPwmPins[i]);
        gpio_set_dir(kPwmPins[i], GPIO_OUT);
        gpio_put(kPwmPins[i], 0);

        // 3) 控制器狀態清零（含跨核心共用的 pin 欄位）
        wheels[i].pwmPin = kPwmPins[i];
        wheels[i].dirPin = kDirPins[i];
        wheels[i].encPin = kEncPins[i];
        wheels[i].targetDuty = 0.0f;
        wheels[i].outDuty = 0.0f;
        wheels[i].appliedDuty = 0.0f;
        wheels[i].integral = 0.0f;
        wheels[i].measRps = 0.0f;
        wheels[i].prevMeas = 0.0f;
        wheels[i].hasPrev = false;

        // 4) 接上 PWM 硬體（CC=0、duty=0）並再確認一次輸出為零
        initWheelPwm(kPwmPins[i]);
        wheelSetPwm(kPwmPins[i], kDirPins[i], 0.0f);
    }
}

// ============================================================================
// onMoveCommand — Core 1 收到命令時執行一次控制週期
//   四輪皆速度閉迴路 PID：編碼器 RPS 回授 → ff + PI（抗飽和）→ duty%
// ============================================================================
static void onMoveCommand(const ChassisSnapshot &cmd) {
    dbg_cmdCount++;
    dbg_vx = cmd.vx;
    dbg_vy = cmd.vy;
    dbg_omega = cmd.omega;

    // 先消化 PIO 編碼器 FIFO（Core 1 控制週期內）
    drainEncoders();

    // 控制週期 dt（跨幀時間）
    uint64_t nowUs = time_us_64();
    const float rawDt = static_cast<float>(nowUs - lastTickUs) * 1e-6f;
    lastTickUs = nowUs;
    // 首次命令（lastTickUs==0）或距上幀超過 DT_MAX_S（剛開機、E-Stop 後、掉幀）
    //   → 本週期只重新同步狀態，不採信編碼器差分也不出力。
    const bool resync = !(rawDt <= MoveCfg::DT_MAX_S);
    float dt = rawDt;
    if(!(dt > MoveCfg::DT_MIN_S))
        dt = MoveCfg::DT_MIN_S;
    if(dt > MoveCfg::DT_MAX_S)
        dt = MoveCfg::DT_MAX_S;

    // ---- 車身 S 曲線速度剖面 ----
    //   平滑原始指令 → 平滑後指令，再餵給運動學/PID。
    //   起步（從 0）慢提速克服靜摩擦、減速快速但不瞬停。
    const float rVx = rampVx.updatePw(cmd.vx, dt);
    const float rVy = rampVy.updatePw(cmd.vy, dt);
    const float rOmega = rampOmega.updatePw(cmd.omega, dt);

    // 車身速度 (RPS) → 各輪目標線速度 (m/s)
    //   rVy>0（往前）→ 運動學得到 FL=+ FR=- BL=- BR=+（左右鏡像），
    //   再由 kWheelPolarity[] 修正成四輪物理都往車頭（見上方極性說明）。
    float wx = kAxisStrafePolarity * rVx * TWO_PI;  // rad/s（左右軸方向開關）
    float wy = kAxisForwardPolarity * rVy * TWO_PI; // rad/s（前後軸方向開關）
    float K = MoveCfg::HALF_LENGTH + MoveCfg::HALF_WIDTH;
    float lin[4];
    lin[0] = (wx - wy - K * rOmega) * MoveCfg::WHEEL_RADIUS; // FL
    lin[1] = (wx + wy + K * rOmega) * MoveCfg::WHEEL_RADIUS; // FR
    lin[2] = (wx + wy - K * rOmega) * MoveCfg::WHEEL_RADIUS; // BL
    lin[3] = (wx - wy + K * rOmega) * MoveCfg::WHEEL_RADIUS; // BR

    // 目標輪轉速 targetRps[i] = lin / (2π*r)，單位 rev/s (RPS)
    float targetRps[4];
    for(int i = 0; i < 4; i++) {
        targetRps[i] = lin[i] / (TWO_PI * MoveCfg::WHEEL_RADIUS);
    }

    // ---- 四輪閉迴路 PID ----
    const float alpha = dt / (MoveCfg::VEL_LPF_TAU + dt);
    for(int i = 0; i < 4; i++) {
        WheelState &w = wheels[i];
        const int32_t c = enc[i]; // 已套 kEncReadPolarity（統一方向）
        const int32_t d = c - w.encCount;
        w.encCount = c;

        // 開機 / 長時間斷幀後的第一幀：丟棄這段期間累積的編碼器差分。
        //   期間車可能被手推、馬達浮動、PIO 剛起跑帶雜訊；若把這些計數
        //   當成轉速，第一次命令就會被當成「正在高速轉」而猛出力 → 暴衝。
        if(resync) {
            w.measRps = 0.0f;
            w.prevMeas = 0.0f;
            w.integral = 0.0f;
            w.hasPrev = false;
            w.outDuty = 0.0f;
            w.targetDuty = 0.0f;
            w.appliedDuty = 0.0f;
            wheelSetPwm(w.pwmPin, w.dirPin, 0.0f);
            continue;
        }

        // 低通量測轉速（含方向；enc 已統一符號 → 正目標對正量測）。
        //   單筆差分超過物理上限 → 視為編碼器突波，保留上次量測值。
        const float rawRps = static_cast<float>(d) /
                             static_cast<float>(MoveCfg::kEncCountsPerRev[i]) / dt;
        if(fabsf(rawRps) <= MoveCfg::WHEEL_MAX_RPS)
            w.measRps += alpha * (rawRps - w.measRps);

        const float err = targetRps[i] - w.measRps;

        // 「停車指令」：目標轉速≈0 時強制清空積分。
        //   先前移動時積分會蓄積穩態偏置（蓄積越大維持速度的 bias），
        //   若不清空，煞車後輸出 ≈ 殘留積分 → 馬達在零指令下仍緩慢蠕動。
        //   清空後於減速過程僅剩 P 項（-kP*meas）主動煞至 0 再穩住。
        if(fabsf(targetRps[i]) < MoveCfg::StopCmdEpsRps)
            w.integral = 0.0f;

        // ---- PI 控制器（含抗風扇 + 低速精細化）----
        // 前饋：方案 A 以 線上 RLS 自適應模型 (a*targetRps + b) 取代固定 pid.ff。
        //   a = 逆增益（自動適應電池電壓），b = 庫倫摩擦截距（自動適應地板）。
        const WheelPidCfg &pid = kPidCfg[i];
        const float ff =
            AdaptiveFeedforward::feedforward(rls[i], targetRps[i]); // 自適應前饋 (duty%)
        const float p = pid.kP * err;

        // 積分上下限隨指令縮放：只讓積分補足「命令前饋出力以下」的誤差。
        //   低速指令 → 積分維持很小 → 能精細控制、且歸零時不會 80→0 大跳變。
        //   （+WINDUP_MARGIN_DUTY 為摩擦/動態餘裕）
        float iLimit = fabsf(ff) + MoveCfg::WINDUP_MARGIN_DUTY;
        if(iLimit > MoveCfg::PID_MAX_DUTY)
            iLimit = MoveCfg::PID_MAX_DUTY;

        // 抗風扇：積分已達縮放上限且誤差同向（繼續累積只會加劇飽和）→ 暫停
        const bool windup = (w.integral >= iLimit && err > 0.f) ||
                            (w.integral <= -iLimit && err < 0.f);
        if(!windup)
            w.integral += pid.kI * err * dt;
        if(w.integral > iLimit)
            w.integral = iLimit;
        if(w.integral < -iLimit)
            w.integral = -iLimit;

        float dterm = 0.0f;
        if(w.hasPrev) {
            dterm = -pid.kD * (w.measRps - w.prevMeas) / dt;
        }
        w.prevMeas = w.measRps;
        w.hasPrev = true;

        float out = p + w.integral + dterm + ff;
        if(out > MoveCfg::PID_MAX_DUTY)
            out = MoveCfg::PID_MAX_DUTY;
        if(out < -MoveCfg::PID_MAX_DUTY)
            out = -MoveCfg::PID_MAX_DUTY;

        w.outDuty = out;

        // ---- 輸出斜率限制（安全網）----
        //   不論上面的推論是否成立，duty 都不可能在單一週期從 0 跳到滿載。
        //   任何回授突波 / 模型越界 / 數值異常最多造成 maxStep 的變化量。
        const float maxStep = MoveCfg::OUTPUT_SLEW_DUTY_PER_S * dt;
        float applied = out;
        const float step = applied - w.appliedDuty;
        if(step > maxStep)
            applied = w.appliedDuty + maxStep;
        else if(step < -maxStep)
            applied = w.appliedDuty - maxStep;
        if(!std::isfinite(applied))
            applied = 0.0f;
        w.appliedDuty = applied;
        wheelSetPwm(w.pwmPin, w.dirPin, applied * kWheelPolarity[i]);

        // ---- 線上 RLS 訓練（僅在條件成立時採樣）----
        //   門檻：
        //     1. 追蹤穩定   |err| < SETTLE_TOL_RPS
        //     2. 實際移動   |measRps| > MIN_SPEED_RPS（避零速/靜摩擦雜訊）
        //     3. 未飽和     |out| 未貼到 ±PID_MAX_DUTY（飽和樣本非逆增益）
        //   避免在 ramp / 停車 / 飽和期間污染模型。
        const bool settled =
            (fabsf(err) < RlsCfg::SETTLE_TOL_RPS) &&
            (fabsf(w.measRps) > RlsCfg::MIN_SPEED_RPS) &&
            (fabsf(applied) < MoveCfg::PID_MAX_DUTY - 1.0f);
        if(settled) {
            // 以「實際應用 duty」為目標、量測轉速為輸入 → 學得真正
            //   逆增益「duty→轉速」映射（含 PI 已補足的殘餘摩擦）。
            AdaptiveFeedforward::update(rls[i], rlsCfg, w.measRps, applied);
        }
    }
    dbg_measRps = wheels[0].measRps;
    dbg_outDuty0 = wheels[0].outDuty;

    // ---- 遙測（四輪 current/target/輸出；由 Core 0 update() 定時送出）----
    MoveTelemetry tm = {};
    for(int i = 0; i < 4; i++) {
        tm.currentRps[i] = wheels[i].measRps;
        tm.targetRps[i] = targetRps[i];
        tm.pwmOutput[i] = wheels[i].appliedDuty;
    }
    tm.cmdVx = rVx;
    tm.cmdVy = rVy;
    tm.cmdOmega = rOmega;
    tm.mode = static_cast<uint8_t>(RobotMode::AUTO);
    tm.estopFlag = 0;
    // ⚠ 遙測標頭（sync/msgType/frameSeq/targetID）、timestamp 與 CRC 由
    //   SignalUtils::finalizeTelemetry 統一填寫；手寫漏填 sync 或 CRC 會讓
    //   Hub 端驗證失敗而整包丟棄（序列監控只看到 "CRC mismatch"）。
    //   CRC 計算範圍不含 crc16 自身；timestamp 維持 us 計時的低 32 bits。
    SignalUtils::finalizeTelemetry(
        tm, tmFrameSeq++, static_cast<uint32_t>(nowUs & 0xFFFFFFFFULL));
    slave.prepareTelemetry(tm);
}

// ============================================================================
// onMoveEstop — 立即歸零所有 PWM，並清 PID 積分
// ============================================================================
static void onMoveEstop() {
    // S 曲線也一併歸零：若停留在舊速度，E-Stop 解除後的第一個命令
    //   會直接以舊斜率／舊速度餵 PID → 瞬間大出力（暴衝）。
    rampVx.reset();
    rampVy.reset();
    rampOmega.reset();

    for(int i = 0; i < 4; i++) {
        wheels[i].targetDuty = 0.0f;
        wheels[i].outDuty = 0.0f;
        wheels[i].appliedDuty = 0.0f;
        wheels[i].integral = 0.0f;
        wheels[i].measRps = 0.0f;
        wheels[i].prevMeas = 0.0f;
        wheels[i].hasPrev = false;
        wheels[i].encCount = enc[i]; // 解除後不計入停止期間的差分
        wheelSetPwm(wheels[i].pwmPin, wheels[i].dirPin, 0.0f);
    }
}

// ============================================================================
// 狀態 LED（板載 LED，例如 Waveshare RP2350 Plus 板上 LED）
//
// 只有三種狀態，全部由「通訊狀況」決定：
//   ┌────────────────────┬──────────────┬──────────────────────┐
//   │ 狀態               │ 圖樣         │ 亮 / 滅              │
//   ├────────────────────┼──────────────┼──────────────────────┤
//   │ 未連線 / 剛開機    │ 長亮         │ —                    │
//   │ 已連線但 Idle      │ 0.5Hz 慢閃   │ 500ms / 1500ms       │
//   │ 資料通訊中         │ 1Hz 短脈心跳 │ 80ms / 920ms         │
//   └────────────────────┴──────────────┴──────────────────────┘
//
// 判定說明：
//   * 「未連線」= 開機至今從未收到任何有效控制幀（validCmdCount == 0）。
//     此時 SlaveComms 的 100ms 指令逾時保護也已生效（馬達歸零），
//     監控會顯示 E-Stop（cmd timeout），LED 長亮代表「未連線 / 未受控」。
//   * 「資料通訊中」= 最後有效控制幀距今 < ACTIVE_TIMEOUT_MS（Jetson 端
//     韌體以 Hub 收到控制幀的頻率轉發，典型 10–20ms 一幀）
//   * 「已連線但 Idle」= 曾連上 Hub，但目前超過 ACTIVE_TIMEOUT_MS 沒有
//     控制幀。注意此刻馬達也已歸零——這是「沒收到資料」，不是斷線。
//
// 需要更精確的 E-Stop 資訊請看序列監控輸出的 led= / ESTOP 欄位。
// ============================================================================
namespace LedCfg {
// 視為「資料通訊中」的視窗（略大於典型控制週期，避免閃斷）
constexpr uint32_t ACTIVE_TIMEOUT_MS = 150;

// 已連線 + 資料通訊中：1Hz 短脈衝心跳（像路由器資料燈）
constexpr uint32_t ACTIVE_ON_MS = 80;
constexpr uint32_t ACTIVE_OFF_MS = 920;

// 已連線但 Idle：0.5Hz 慢閃，鎮定不擾人
constexpr uint32_t IDLE_ON_MS = 500;
constexpr uint32_t IDLE_OFF_MS = 1500;
} // namespace LedCfg

enum class LedState : uint8_t { BOOT = 0, IDLE = 1, ACTIVE = 2 };

// 亮滅圖樣表：索引 = LedState 的數值（相位推進交給 StatusLed）
//   BOOT   → 長亮（剛開機、從未收到有效幀）
//   IDLE   → 0.5Hz 慢閃，鎮定不擾人
//   ACTIVE → 1Hz 短脈衝心跳（像路由器資料燈）
static const StatusLed::Pattern kLedPatterns[] = {
    {0, 0, true},                                         // BOOT：長亮
    {LedCfg::IDLE_ON_MS, LedCfg::IDLE_OFF_MS, false},     // IDLE
    {LedCfg::ACTIVE_ON_MS, LedCfg::ACTIVE_OFF_MS, false}, // ACTIVE
};
static constexpr uint8_t LED_PATTERN_COUNT =
    static_cast<uint8_t>(sizeof(kLedPatterns) / sizeof(kLedPatterns[0]));
static StatusLed statusLed(LED_BUILTIN, kLedPatterns, LED_PATTERN_COUNT);

static const char *ledStateName(LedState st) {
    switch(st) {
    case LedState::BOOT:
        return "BOOT";
    case LedState::IDLE:
        return "IDLE";
    default:
        return "ACTIVE";
    }
}

// 供 Core 0 除錯輸出用（與 LED 實際狀態保證一致）
static volatile uint8_t dbg_ledState = static_cast<uint8_t>(LedState::BOOT);

static LedState currentLedState() {
    if(slave.validCmdCount() == 0)
        return LedState::BOOT; // 開機至今從未收到有效幀
    if(slave.msSinceLastValidCmd() < LedCfg::ACTIVE_TIMEOUT_MS)
        return LedState::ACTIVE; // 資料通訊中
    return LedState::IDLE; // 已連線，但目前沒有控制流
}

static void updateStatusLed() {
    const uint8_t st = static_cast<uint8_t>(currentLedState());
    dbg_ledState = st; // 除錯輸出與 LED 實際套用的狀態一致

    // 狀態切換 → 重置相位並立刻點亮；長亮狀態持續 HIGH；其餘亮滿
    // onMs 熄、熄滿 offMs 亮（非阻塞，見 StatusLed::update）。
    statusLed.update(st, millis());
}

// ============================================================================
// Core 0
// ============================================================================
void setup() {
    // ⚠ 第一個執行動作：把馬達推向「確定零出力」
    //   arduino-pico 先 launch core1 再呼叫 setup()，Core 1 此刻已經在
    //   slave.runCore1() spinloop 裡；而 Serial.begin()/delay(200) 會拖
    //   上百毫秒。這段期間馬達腳位若還在未定義狀態（或 wheels[] 的 pin
    //   還是 0），就可能出現開機暴衝。因此先鎖定馬達腳位再開始其他的初始化。
    initMotorHardware();

    // 車身 S 曲線調參（MoveCfg::SCURVE_*）→ MotionProfile::SCurveRamp。
    //   必須早於第一個控制幀（slave.begin() 之後 Core 1 才可能執行
    //   onMoveCommand）；未 configure 時輸出會卡住不動。
    const SCurveConfig sCurveCfg = {
        MoveCfg::SCURVE_ACCEL,
        MoveCfg::SCURVE_DECEL,
        MoveCfg::SCURVE_JERK,
        MoveCfg::SCURVE_START_SLOW_VEL,
        MoveCfg::SCURVE_START_ACCEL,
        MoveCfg::SCURVE_SETTLE_TOL,
    };
    rampVx.configure(sCurveCfg);
    rampVy.configure(sCurveCfg);
    rampOmega.configure(sCurveCfg);

    // RLS 調參（RlsCfg）→ AdaptiveFeedforward::RlsConfig
    rlsCfg = RlsConfig{
        RlsCfg::LAMBDA,
        RlsCfg::P_INIT,
        RlsCfg::P_MIN,
        RlsCfg::P_MAX,
        RlsCfg::MIN_SPEED_RPS,
        RlsCfg::SETTLE_TOL_RPS,
        RlsCfg::A_MIN,
        RlsCfg::A_MAX,
        RlsCfg::B_MIN,
        RlsCfg::B_MAX,
    };

    statusLed.begin();

    // 方案 A：RLS 初始增益以現有固定 ff 作為種子（無縫接軌，
    //   開機即擁有與原本固定 FF 相同的行為，再由 RLS 逐步微調）。
    for(int i = 0; i < 4; i++) {
        rls[i].a = kPidCfg[i].ff;
        rls[i].b = 0.0f;
    }

    Serial.begin(115200);
    delay(200);
    Serial.println("[MOVE] FL 閉迴路 PID + 其他開迴路（等待 Hub 命令）");

    slave.begin();

    // Core 1 空閒時持續 drain 編碼器 FIFO（見 drainEncoders 說明）。
    // 只在沒有新命令 / 非 E-Stop 的 spinloop 中執行，不阻塞控制週期。
    slave.setIdleHook(drainEncoders);

    Serial.println("[MOVE] FF = 線上 RLS 自適應（a*rps + b），初始 a=pid.ff，λ=0.995");
}

void loop() {
    slave.update();
    updateStatusLed();

    static uint32_t lastPrint = 0;
    uint32_t nowCfg = millis();
    if(nowCfg - lastPrint >= 100) {
        lastPrint = nowCfg;
        uint32_t since = slave.msSinceLastValidCmd();
        uint32_t cnt = slave.validCmdCount();
        // RX 診斷：rx/crcErr 是「Hub→Move 這條線到底有沒有資料」。
        //   rx 不增加、crcErr 一直增加 → 接線/鮑率/對端 CRC 問題。
        //   rx 有增加但 cnt 不動 → 收到別台 targetID 或 msgType 不是 CONTROL。
        Serial.printf("[MOVE] rx=%u crcErr=%u cmd=%u since=%ums\n",
                      slave.rxFrameCount(), slave.rxCrcErrorCount(), cnt, since);
        Serial.printf("[MOVE] lastRx type=%u id=%u seq=%u onCmd=%u led=%s\n",
                      slave.lastRxMsgType(), slave.lastRxTargetID(),
                      slave.lastRxFrameSeq(), dbg_cmdCount,
                      ledStateName(static_cast<LedState>(dbg_ledState)));
        if(slave.isEstopActive()) {
            // E-Stop 來源：逾時（沒收到資料）或 Hub 明確下令
            const bool timeoutDriven =
                since >= (RobotConfig::COMMAND_TIMEOUT_US / 1000ULL);
            Serial.printf("[MOVE] ** E-STOP active (%s) **\n",
                          timeoutDriven ? "cmd timeout, no data" : "hub estop");
        } else {
            Serial.printf("[MOVE] vx=%+d vy=%+d omega=%+d x0.01\n",
                          DebugFmt::scaled100(dbg_vx), DebugFmt::scaled100(dbg_vy),
                          DebugFmt::scaled100(dbg_omega));
            Serial.printf("[MOVE] FL=%+d/%+d FR=%+d/%+d x0.01,%%\n",
                          DebugFmt::scaled100(wheels[0].measRps),
                          static_cast<int>(wheels[0].outDuty),
                          DebugFmt::scaled100(wheels[1].measRps),
                          static_cast<int>(wheels[1].outDuty));
            Serial.printf("[MOVE] BL=%+d/%+d BR=%+d/%+d x0.01,%%\n",
                          DebugFmt::scaled100(wheels[2].measRps),
                          static_cast<int>(wheels[2].outDuty),
                          DebugFmt::scaled100(wheels[3].measRps),
                          static_cast<int>(wheels[3].outDuty));
            // 方案 A：RLS 自適應 FF 係數（觀察學習狀態）
            //   a = 增益(duty%/RPS)，b = 庫倫摩擦截距(duty%)
            Serial.printf("[MOVE] RLS a=[%d %d %d %d] x0.01\n",
                          DebugFmt::scaled100(rls[0].a), DebugFmt::scaled100(rls[1].a),
                          DebugFmt::scaled100(rls[2].a), DebugFmt::scaled100(rls[3].a));
        }
    }
}

// ============================================================================
// Core 1 — 自旋等待 SlaveComms 派發命令，執行四輪 PID 控制
// ============================================================================
void setup1() {
    // PIO 編碼器初始化（Core 1 啟用；PIO SM 為硬體計數，不綁核心）
    encoderInit();
}

void loop1() { slave.runCore1(); }
