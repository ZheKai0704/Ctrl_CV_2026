// ============================================================================
// move_drive_test — Move 四輪「實際行進 / 閉迴路」診斷測試 (RP2350，獨立程式)
//
// 目的：重現並定位兩種現場症狀
//   (A) 馬達有時不動
//   (B) 某一顆輪子偶發爆衝
//
// 與 move_dir_test 的差別：
//   move_dir_test  → 開迴路（固定 duty）+ 極性 / 腳位校正
//   move_drive_test→ 直接搬入 src/move/main.cpp 的「完整閉迴路」
//                    （S 曲線 + 編碼器 LPF + PI + 抗風扇 + 輸出斜率限制 +
//                      線上 RLS 自適應前饋），並且可以逐項開關 / 調參，
//                    用同一組序列埠指令互相比較，把問題縮小到單一環節。
//
// 內建自動診斷（每個控制週期即時判斷，不需要人眼盯數字）
//   [STALL]  有出力但幾乎無回授   → 馬達電源 / 驅動保護 / 編碼器沒接
//   [BURST]  實測轉速遠超目標     → 回授異常（極性反 → 正回饋）或模型爆走
//   [POL?]   目標與回授長期反向   → kEncReadPolarity 很可能反了（爆衝主因）
//   [SPIKE]  單筆編碼器差分超物理上限 → 韌體會丟棄，但持續發生會讓 PID 誤判
//   [FIFO]   PIO 編碼器 RX FIFO 滿 → 掉計數 → 轉速低估 → PID 飽和
//   [RESYNC] 控制週期 > 50ms（掉幀 / 剛開機）→ 該週期歸零並重新同步
//
// ### 使用流程（建議）
//   pio run -e move_drive_test -t upload
//   pio device monitor -e move_drive_test        # 115200
//
//   1) probe            ← 逐輪「開迴路 15% / 閉迴路 0.6 RPS」自動測試，先看結論
//   2) cl fwd 0.8       ← 正式閉迴路前進（0.8 RPS 車身速度）
//   3) log on           ← 開 CSV 記錄（20Hz），重現時看哪一輪 out/meas 爆走
//   4) rls off          ← 若懷疑自適應前饋造成爆衝，關掉再測一次
//   5) hold fl 25       ← 單輪閉迴路，用「等效 duty%」指定目標，逐輪比對
//   6) tick 20 / drop 6 ← 模擬控制幀變慢 / 掉幀，看重現症狀
//
// ### 指令（115200，Enter = stop）
//   ol   <fwd|back|left|right|cw|ccw> [duty%]   開迴路（預設 25%，上限 cap）
//   cl   <fwd|back|left|right|cw|ccw> [rps]     閉迴路（車身速度，預設 0.8 RPS）
//   spin <fl|fr|bl|br> <rps>                    閉迴路單輪（真實輪轉速）
//   hold <fl|fr|bl|br> <duty%>                  閉迴路單輪，用等效 duty% 指定
//   allhold <duty%>                             四輪同一個等效 duty% 目標
//   stop / s / 空行                              停止並歸零
//   reset                                       停止 + PID/RLS/計數全部重置
//   tick <ms>                                   控制週期（5~100，預設 10）
//   drop <n>                                    模擬掉幀：hold n 個週期不控制
//   rls on|off                                  線上自適應前饋開關
//   pid <fl|fr|bl|br> <kP> <kI> [ff]            即時改單輪增益
//   cap <duty%>                                 測試出力上限（預設 60）
//   slew <duty%/s>                              輸出斜率限制
//   lpf <ms>                                    轉速量測低通時間常數
//   encdiv <v>                                  編碼器 PIO clkdiv（預設 500，越大取樣越慢）
//   pol <輪> <-1|1> / encpol <輪> <-1|1>        改馬達 / 編碼器極性
//   axis <strafe|fwd|rot> <-1|1>                改車體軸向極性
//   probe [ms]                                  逐輪開/閉迴路自動診斷（預設 1200ms/段）
//   log on|off | log <hz>                       CSV 記錄（預設 20Hz）
//   enc                                         待機手轉輪子，看原始計數
//   status / map / report / help                狀態 / 腳位 / 可貼回設定 / 說明
//
// ### 安全 ###
//   - 預設出力上限 60%，長轉請架空車身
//   - 堵轉 1 秒警告、3 秒自動停止；爆衝持續 0.6 秒自動停止
//   - 任何阻塞式動作（probe）按任意鍵即中止
//   - 停止時一律 PWM=0、PID 積分歸零
// ============================================================================
#include "quad_phase.pio.h"
#include <Arduino.h>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>

// ============================================================================
// 腳位（新板實測；輪序一律 [FL, FR, BL, BR]，與 src/move/main.cpp 相同）
// ============================================================================
namespace Pin {
constexpr uint FL_PWM = 7;
constexpr uint FL_DIR = 6;
constexpr uint FL_ENC = 16;

constexpr uint FR_PWM = 9;
constexpr uint FR_DIR = 8;
constexpr uint FR_ENC = 14;

constexpr uint BL_PWM = 13;
constexpr uint BL_DIR = 12;
constexpr uint BL_ENC = 18;

constexpr uint BR_PWM = 11;
constexpr uint BR_DIR = 10;
constexpr uint BR_ENC = 20;
} // namespace Pin

namespace Cfg {
constexpr uint16_t PWM_WRAP = 1000; // firmware 相同：clkdiv 6.25 → ~20kHz
constexpr uint16_t ENC_CPR = 443 * 4;

// 車身幾何 / 編碼器 / 安全參數（全部沿用 src/move/main.cpp）
constexpr float HALF_LENGTH = 0.25f;
constexpr float HALF_WIDTH = 0.25f;
constexpr float WHEEL_RADIUS = 0.05f;
constexpr float DT_MIN_S = 1e-4f;
constexpr float DT_MAX_S = 0.05f;
constexpr float WHEEL_MAX_RPS = 15.0f;
constexpr float STOP_EPS_RPS = 0.005f;

// S 曲線（沿用 main.cpp）
constexpr float SCURVE_ACCEL = 4.0f;
constexpr float SCURVE_DECEL = 12.0f;
constexpr float SCURVE_JERK = 60.0f;
constexpr float SCURVE_START_SLOW_VEL = 0.20f;
constexpr float SCURVE_START_ACCEL = 1.5f;
constexpr float SCURVE_SETTLE_TOL = 0.010f;

// 測試用
constexpr uint32_t STATUS_MS = 250;
constexpr float OL_RAMP_PER_SEC = 150.0f; // 開迴路斜坡（duty%/s）
constexpr uint8_t FIFO_DEPTH = 8;         // RX FIFO join（main.cpp 相同）
constexpr uint32_t AUTO_STOP_STALL_MS = 3000;
constexpr uint32_t AUTO_STOP_BURST_MS = 600;
} // namespace Cfg

// ============================================================================
// 可即時調整的參數（預設 = src/move/main.cpp 現值）
// ============================================================================
struct Gain {
    float kP, kI, kD, ff;
};

// [FL, FR, BL, BR]
static Gain gains[4] = {
    {2.40f, 10.4f, 0.0f, 33.0f},
    {2.11f, 8.80f, 0.0f, 33.0f},
    {2.09f, 8.70f, 0.0f, 33.0f},
    {2.40f, 10.4f, 0.0f, 33.0f},
};
static float tunePidMax = 100.0f;      // PID 內部飽和（duty%）
static float tuneWindupMargin = 6.0f;  // 積分風扇餘裕（duty%）
static float tuneLpfTau = 0.010f;      // 轉速 LPF 時間常數（s）
static float tuneOutputSlew = 1500.0f; // 輸出斜率限制（duty%/s）
static float tuneMaxTestDuty = 60.0f;  // 測試出力上限（duty%）
static bool rlsEnabled = true;
static float tuneRlsLambda = 0.995f;

static int8_t motorPol[4] = {1, 1, -1, -1};  // kWheelPolarity
static int8_t encPol[4] = {1, -1, +1, +1};   // kEncReadPolarity
// 每輪實際計數/圈（4X）。四輪編碼器型號不同時可個別校正，
// 用 `cpr <輪> <counts>` 改；預設全部 = Cfg::ENC_CPR。
static float gCpr[4] = {static_cast<float>(Cfg::ENC_CPR),
                        static_cast<float>(Cfg::ENC_CPR),
                        static_cast<float>(Cfg::ENC_CPR),
                        static_cast<float>(Cfg::ENC_CPR)};
static float axisStrafe = -1.0f;             // kAxisStrafePolarity
static float axisForward = -1.0f;            // kAxisForwardPolarity
static float axisRot = +1.0f;

// ============================================================================
// 常數表 / 名稱
// ============================================================================
static constexpr uint kPwmPin[4] = {Pin::FL_PWM, Pin::FR_PWM, Pin::BL_PWM,
                                    Pin::BR_PWM};
static constexpr uint kDirPin[4] = {Pin::FL_DIR, Pin::FR_DIR, Pin::BL_DIR,
                                    Pin::BR_DIR};
static constexpr uint kEncPin[4] = {Pin::FL_ENC, Pin::FR_ENC, Pin::BL_ENC,
                                    Pin::BR_ENC};
static const char *const kWheelName[4] = {"FL", "FR", "BL", "BR"};

// ============================================================================
// 線上自適應前饋（RLS）— 與 main.cpp 相同
// ============================================================================
namespace RlsCfg {
constexpr float P_INIT = 1000.0f;
constexpr float P_MIN = 1e-3f;
constexpr float P_MAX = 1e4f;
constexpr float MIN_SPEED_RPS = 0.02f;
constexpr float SETTLE_TOL_RPS = 0.05f;
constexpr float A_MIN = 0.0f;
constexpr float A_MAX = 80.0f;
constexpr float B_MIN = -15.0f;
constexpr float B_MAX = 15.0f;
} // namespace RlsCfg

struct RlsState {
    float a = 15.0f;
    float b = 0.0f;
    float P00 = RlsCfg::P_INIT;
    float P01 = 0.0f;
    float P11 = RlsCfg::P_INIT;
};
static RlsState rls[4];

// 整數輸出輔助（此 core 的 Serial.printf 不支援 %f）
static int mrps(float v);
static int dpct(float v);

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void rlsUpdate(RlsState &s, float rps, float duty) {
    if (fabsf(rps) < RlsCfg::MIN_SPEED_RPS) return;
    s.b = 0.0f;
    const float pred = s.a * rps;
    const float err = duty - pred;

    // 純量 RLS：K = P*rps / (λ + rps²*P)
    const float Pphi = s.P00 * rps;
    const float denom = tuneRlsLambda + rps * Pphi;
    if (fabsf(denom) < 1e-6f) return;
    const float K = Pphi / denom;

    s.a += K * err;
    s.P00 = clampf((s.P00 - K * Pphi) / tuneRlsLambda, RlsCfg::P_MIN,
                   RlsCfg::P_MAX);
    s.P01 = 0.0f;
    s.P11 = RlsCfg::P_MIN;
    s.a = clampf(s.a, RlsCfg::A_MIN, RlsCfg::A_MAX);
}

// ============================================================================
// S 曲線（與 main.cpp 相同）
// ============================================================================
struct SCurveRamp {
    float y = 0.f;
    float accel = 0.f;

    float updatePw(float target, float dt) {
        update(target, dt);
        return y;
    }

    void update(float target, float dt) {
        const float err = target - y;
        if (fabsf(err) < Cfg::SCURVE_SETTLE_TOL) {
            y = target;
            accel = 0.f;
            return;
        }
        const bool decel = (y != 0.f) && ((err > 0.f) != (y > 0.f));
        float cap = decel ? Cfg::SCURVE_DECEL : Cfg::SCURVE_ACCEL;
        if (!decel && fabsf(y) < Cfg::SCURVE_START_SLOW_VEL)
            cap = (cap < Cfg::SCURVE_START_ACCEL) ? cap : Cfg::SCURVE_START_ACCEL;

        const float aim = (err > 0.f) ? cap : -cap;
        const float maxDa = Cfg::SCURVE_JERK * dt;
        float da = aim - accel;
        if (da > maxDa) da = maxDa;
        if (da < -maxDa) da = -maxDa;
        accel += da;

        y += accel * dt;
        if ((y - target) * err < 0.f) y = target;
    }
};
static SCurveRamp rampVx, rampVy, rampOmega;

// ============================================================================
// 四輪狀態（欄位刻意與 main.cpp 對齊，方便對照）
// ============================================================================
struct WheelState {
    float targetRps = 0.0f;
    float measRps = 0.0f;
    float rawRps = 0.0f;
    float integral = 0.0f;
    float prevMeas = 0.0f;
    bool hasPrev = false;
    float outDuty = 0.0f;     // PID 限幅後
    float appliedDuty = 0.0f; // 經斜率限制後（韌體座標）
    int32_t encCount = 0;
    int32_t lastDelta = 0; // 本週期編碼器差分（給狀態列印）
    // 診斷
    float termP = 0.0f, termI = 0.0f, termFf = 0.0f, termD = 0.0f;
    uint32_t spikeCount = 0;      // 被丟棄的編碼器突波
    uint32_t fifoMax = 0;         // 觀測到的最大 RX FIFO 深度
    uint32_t fifoFullCount = 0;   // 觀察到 FIFO 滿的次數
    uint32_t polMismatchTicks = 0;
    uint32_t stallSinceMs = 0;
    uint32_t burstSinceMs = 0;
    uint32_t polSinceMs = 0;
    bool stallWarned = false;
    bool burstWarned = false;
    bool polWarned = false;
};
static WheelState wheels[4];

// ============================================================================
// 編碼器（PIO0 4 SM；設定刻意與 main.cpp 相同：clkdiv=500、RX FIFO join）
// ============================================================================
static PIO g_pio = pio0;
static uint g_pioOffset = 0;
static bool g_pioProgramAdded = false;
static float g_encClkDiv = 500.0f;

static const int8_t kQuadTable[4][4] = {
    {0, +1, -1, 0},
    {-1, 0, 0, +1},
    {+1, 0, 0, -1},
    {0, -1, +1, 0},
};
static volatile int32_t enc[4]; // 已套 encPol 的累計
static uint8_t encPrev[4];

static void drainEncoders() {
    constexpr int MAX_DRAIN = 512;
    for (int i = 0; i < 4; i++) {
        const uint lvl = pio_sm_get_rx_fifo_level(g_pio, i);
        if (lvl > wheels[i].fifoMax) wheels[i].fifoMax = lvl;
        if (lvl >= Cfg::FIFO_DEPTH) wheels[i].fifoFullCount++;

        int n = 0;
        while (!pio_sm_is_rx_fifo_empty(g_pio, i) && n < MAX_DRAIN) {
            const uint32_t cur = pio_sm_get(g_pio, i) & 0x3;
            enc[i] += encPol[i] * kQuadTable[encPrev[i]][cur];
            encPrev[i] = static_cast<uint8_t>(cur);
            n++;
        }
    }
}

// 重新設定 PIO 編碼器（encdiv 指令用；也會清零計數）
static void encoderInit(float clkdiv) {
    g_encClkDiv = clkdiv;
    if (!g_pioProgramAdded) {
        g_pioOffset = pio_add_program(g_pio, &quad_phase_program);
        g_pioProgramAdded = true;
    }
    for (int i = 0; i < 4; i++) {
        pio_sm_set_enabled(g_pio, i, false);
        const uint aPin = kEncPin[i];
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
        sm_config_set_in_pins(&c, aPin);
        sm_config_set_in_shift(&c, false, false, 32);
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
        sm_config_set_clkdiv(&c, g_encClkDiv);

        pio_sm_init(g_pio, i, g_pioOffset, &c);
        pio_sm_clear_fifos(g_pio, i);
        pio_sm_set_enabled(g_pio, i, true);

        enc[i] = 0;
        const uint8_t a = gpio_get(aPin) ? 1u : 0u;
        const uint8_t b = gpio_get(bPin) ? 1u : 0u;
        encPrev[i] = static_cast<uint8_t>(a | (b << 1));
        wheels[i].encCount = 0;
        wheels[i].fifoMax = 0;
        wheels[i].fifoFullCount = 0;
    }
}

// ============================================================================
// 硬體輸出（韌體座標 duty：正 = 該輪「正轉慣例」，內部乘 motorPol）
// ============================================================================
static int g_effDir[4] = {0, 0, 0, 0};
static float g_effPct[4] = {0.0f, 0.0f, 0.0f, 0.0f};

static void initWheelHw(int i) {
    gpio_init(kDirPin[i]);
    gpio_set_dir(kDirPin[i], GPIO_OUT);
    gpio_put(kDirPin[i], 0);

    gpio_init(kPwmPin[i]);
    gpio_set_dir(kPwmPin[i], GPIO_OUT);
    gpio_put(kPwmPin[i], 0);

    gpio_set_function(kPwmPin[i], GPIO_FUNC_PWM);
    const uint slice = pwm_gpio_to_slice_num(kPwmPin[i]);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 6.25f);
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_FREE_RUNNING);
    pwm_config_set_wrap(&cfg, Cfg::PWM_WRAP);
    pwm_init(slice, &cfg, false);
    pwm_set_gpio_level(kPwmPin[i], 0);
    pwm_set_enabled(slice, true);
}

static void hwWrite(int i, float firmwareDuty) {
    if (!std::isfinite(firmwareDuty)) firmwareDuty = 0.0f;
    float duty = firmwareDuty * static_cast<float>(motorPol[i]);
    if (duty > tuneMaxTestDuty) duty = tuneMaxTestDuty;
    if (duty < -tuneMaxTestDuty) duty = -tuneMaxTestDuty;

    const bool fwd = (duty >= 0.0f);
    gpio_put(kDirPin[i], fwd ? 0 : 1);
    const float mag = fabsf(duty);
    pwm_set_gpio_level(kPwmPin[i],
                       static_cast<uint16_t>(mag / 100.0f * Cfg::PWM_WRAP));
    g_effDir[i] = fwd ? 0 : 1;
    g_effPct[i] = duty;
}

// ============================================================================
// 模式 / 命令狀態
// ============================================================================
enum class Mode : uint8_t { Idle, OpenLoop, ClosedLoop, EncWatch };
static Mode g_mode = Mode::Idle;
static char g_modeText[48] = "idle";

static uint32_t g_tickMs = 10;
static uint64_t lastTickUs = 0;
static bool g_resyncNext = false;
static uint64_t g_skipUntilUs = 0;
static uint32_t g_resyncCount = 0;

// 開迴路
static float g_olTarget[4] = {0, 0, 0, 0}; // duty%（韌體座標）
static float g_olOut[4] = {0, 0, 0, 0};

// 閉迴路
static float g_clVx = 0, g_clVy = 0, g_clOmega = 0; // 車身速度 RPS
static bool g_clDirect = false;                    // true = 直接指定四輪 RPS
static float g_clDirectTarget[4] = {0, 0, 0, 0};

// 記錄
static bool g_logOn = false;
static uint32_t g_logHz = 20;
static uint32_t lastLogMs = 0;

// ============================================================================
// 前置宣告
// ============================================================================
static void stopAll();
static void flushInput();
static bool anyKeyPressed();
static void printStatus();

// ============================================================================
// 量測更新（兩種模式共用）：編碼器差分 → 低通 RPS
//   與 main.cpp 相同：resync 週期丟棄差分；單筆超物理上限視為突波。
// ============================================================================
static void updateMeasurements(float dt, bool resync) {
    const float alpha = dt / (tuneLpfTau + dt);
    for (int i = 0; i < 4; i++) {
        WheelState &w = wheels[i];
        const int32_t c = enc[i];
        const int32_t d = c - w.encCount;
        w.encCount = c;
        w.lastDelta = d;

        if (resync) {
            w.measRps = 0.0f;
            w.prevMeas = 0.0f;
            w.hasPrev = false;
            w.rawRps = 0.0f;
            continue;
        }

        const float rawRps = static_cast<float>(d) / gCpr[i] / dt;
        w.rawRps = rawRps;
        if (fabsf(rawRps) <= Cfg::WHEEL_MAX_RPS)
            w.measRps += alpha * (rawRps - w.measRps);
        else
            w.spikeCount++;
    }
}

// ============================================================================
// PID / 輸出（與 main.cpp 對齊）
// ============================================================================
static void pidStep(const float targetRps[4], float dt, bool resync) {
    for (int i = 0; i < 4; i++) {
        WheelState &w = wheels[i];
        w.targetRps = targetRps[i];

        if (resync) {
            w.integral = 0.0f;
            w.outDuty = 0.0f;
            w.appliedDuty = 0.0f;
            w.termP = w.termI = w.termD = w.termFf = 0.0f;
            w.stallSinceMs = w.burstSinceMs = w.polSinceMs = 0;
            hwWrite(i, 0.0f);
            g_effPct[i] = 0.0f;
            continue;
        }

        const float err = targetRps[i] - w.measRps;
        if (fabsf(targetRps[i]) < Cfg::STOP_EPS_RPS) w.integral = 0.0f;

        const Gain &g = gains[i];
        const float ff = rlsEnabled ? (rls[i].a * targetRps[i] + rls[i].b)
                                    : (g.ff * targetRps[i]);
        const float p = g.kP * err;

        float iLimit = fabsf(ff) + tuneWindupMargin;
        if (iLimit > tunePidMax) iLimit = tunePidMax;

        const bool windup = (w.integral >= iLimit && err > 0.f) ||
                            (w.integral <= -iLimit && err < 0.f);
        if (!windup) w.integral += g.kI * err * dt;
        w.integral = clampf(w.integral, -iLimit, iLimit);

        float dterm = 0.0f;
        if (w.hasPrev) dterm = -g.kD * (w.measRps - w.prevMeas) / dt;
        w.prevMeas = w.measRps;
        w.hasPrev = true;

        float out = p + w.integral + dterm + ff;
        out = clampf(out, -tunePidMax, tunePidMax);
        w.outDuty = out;
        w.termP = p;
        w.termI = w.integral;
        w.termD = dterm;
        w.termFf = ff;

        const float maxStep = tuneOutputSlew * dt;
        float applied = out;
        const float step = applied - w.appliedDuty;
        if (step > maxStep)
            applied = w.appliedDuty + maxStep;
        else if (step < -maxStep)
            applied = w.appliedDuty - maxStep;
        if (!std::isfinite(applied)) applied = 0.0f;
        w.appliedDuty = applied;
        hwWrite(i, applied);

        // 線上 RLS 訓練（條件與 main.cpp 相同）
        const bool settled =
            (fabsf(err) < RlsCfg::SETTLE_TOL_RPS) &&
            (fabsf(w.measRps) > RlsCfg::MIN_SPEED_RPS) &&
            (fabsf(applied) < tunePidMax - 1.0f);
        if (rlsEnabled && settled) rlsUpdate(rls[i], w.measRps, applied);
    }
}

// ============================================================================
// 即時健康檢查（每控制週期）
// ============================================================================
static void warnf(const char *tag, int i, const char *fmt, ...) {
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    Serial.printf("\n[%s] %s: %s\n", tag, kWheelName[i], buf);
}

static void checkWheelHealth(int i) {
    WheelState &w = wheels[i];
    const uint32_t now = millis();

    // ---- 1) 堵轉：有出力但幾乎無回授 ----
    const bool commanded = fabsf(w.appliedDuty) >= 8.0f;
    const bool stalled = commanded && fabsf(w.measRps) < 0.02f;
    if (stalled) {
        if (w.stallSinceMs == 0) w.stallSinceMs = now ? now : 1;
    } else {
        w.stallSinceMs = 0;
        w.stallWarned = false;
    }
    if (w.stallSinceMs != 0) {
        const uint32_t dur = now - w.stallSinceMs;
        if (dur >= 1000 && !w.stallWarned) {
            w.stallWarned = true;
            warnf("STALL", i,
                  "出力 %+d%% 但幾乎無回授 %+d mRPS → 查馬達電源(VM)/驅動保護/編碼器",
                  static_cast<int>(lroundf(w.appliedDuty)), mrps(w.measRps));
        }
        if (dur >= Cfg::AUTO_STOP_STALL_MS) {
            Serial.printf("\n[STALL] %s 堵轉超過 %us → 自動停止\n", kWheelName[i],
                          static_cast<unsigned>(Cfg::AUTO_STOP_STALL_MS / 1000));
            stopAll();
            return;
        }
    }

    if (g_mode != Mode::ClosedLoop) return;

    // ---- 2) 方向不一致：目標與回授長期反向 → 編碼器極性可能反了 ----
    const bool bothMoving = fabsf(w.targetRps) > 0.05f &&
                            fabsf(w.measRps) > 0.05f;
    const bool opposite = bothMoving &&
                          ((w.targetRps > 0.f) != (w.measRps > 0.f));
    if (opposite) {
        if (w.polSinceMs == 0) w.polSinceMs = now ? now : 1;
    } else {
        w.polSinceMs = 0;
        w.polWarned = false;
    }
    if (w.polSinceMs != 0 && now - w.polSinceMs >= 500 && !w.polWarned) {
        w.polWarned = true;
        w.polMismatchTicks++;
        warnf("POL?", i,
              "目標 %+d 但回授 %+d（mRPS）反向 → 檢查 kEncReadPolarity（正回饋=爆衝）",
              static_cast<int>(lroundf(w.targetRps * 1000.0f)),
              static_cast<int>(lroundf(w.measRps * 1000.0f)));
    }

    // ---- 3) 爆衝：實測遠超目標，或零命令仍自轉 ----
    const float tgtAbs = fabsf(w.targetRps);
    const bool overspeed =
        (tgtAbs > 0.05f && fabsf(w.measRps) > tgtAbs * 1.8f + 0.8f) ||
        (tgtAbs < 0.05f && fabsf(w.measRps) > 0.30f);
    if (overspeed) {
        if (w.burstSinceMs == 0) w.burstSinceMs = now ? now : 1;
    } else {
        w.burstSinceMs = 0;
        w.burstWarned = false;
    }
    if (w.burstSinceMs != 0) {
        const uint32_t dur = now - w.burstSinceMs;
        if (dur >= 150 && !w.burstWarned) {
            w.burstWarned = true;
            warnf("BURST", i,
                  "目標 %+d 但回授 %+d（mRPS）→ 回授/模型異常（applied=%+d%%，ff=%+d%%）",
                  static_cast<int>(lroundf(w.targetRps * 1000.0f)),
                  static_cast<int>(lroundf(w.measRps * 1000.0f)),
                  static_cast<int>(lroundf(w.appliedDuty)),
                  static_cast<int>(lroundf(w.termFf)));
        }
        if (dur >= Cfg::AUTO_STOP_BURST_MS) {
            Serial.printf("\n[BURST] %s 爆衝超過 %ums → 自動停止\n", kWheelName[i],
                          static_cast<unsigned>(Cfg::AUTO_STOP_BURST_MS));
            stopAll();
        }
    }

    // ---- 4) 編碼器 FIFO 滿（掉計數 → 轉速低估 → PID 飽和） ----
    if (w.fifoFullCount == 1) {
        warnf("FIFO", i,
              "PIO RX FIFO 已滿 → 編碼器可能掉計數（測速低估）→ 降低 encdiv 或提高 drain 頻率");
    }
}

// ============================================================================
// 運動學
// ============================================================================
// 閉迴路：車身速度 (RPS) → 四輪目標 RPS（與 main.cpp 相同，不正規化）
static void chassisToWheelRps(float vx, float vy, float omega, float out[4]) {
    const float wx = axisStrafe * vx * TWO_PI;
    const float wy = axisForward * vy * TWO_PI;
    const float K = Cfg::HALF_LENGTH + Cfg::HALF_WIDTH;
    const float R = Cfg::WHEEL_RADIUS;
    float lin[4];
    lin[0] = (wx - wy - K * omega) * R; // FL
    lin[1] = (wx + wy + K * omega) * R; // FR
    lin[2] = (wx + wy - K * omega) * R; // BL
    lin[3] = (wx - wy + K * omega) * R; // BR
    for (int i = 0; i < 4; i++)
        out[i] = lin[i] / (TWO_PI * R);
}

// 開迴路：車身方向 + duty% → 四輪 duty%（正規化，與 move_dir_test 相同）
static void chassisToWheelDuty(float vx, float vy, float omega, float duty,
                               float out[4]) {
    const float wx = axisStrafe * vx * TWO_PI;
    const float wy = axisForward * vy * TWO_PI;
    const float wz = axisRot * omega;
    const float K = Cfg::HALF_LENGTH + Cfg::HALF_WIDTH;
    const float R = Cfg::WHEEL_RADIUS;
    float lin[4];
    lin[0] = (wx - wy - K * wz) * R;
    lin[1] = (wx + wy + K * wz) * R;
    lin[2] = (wx + wy - K * wz) * R;
    lin[3] = (wx - wy + K * wz) * R;

    float maxAbs = 0.0f;
    for (int i = 0; i < 4; i++)
        if (fabsf(lin[i]) > maxAbs) maxAbs = fabsf(lin[i]);
    for (int i = 0; i < 4; i++)
        out[i] = (maxAbs > 1e-6f) ? (lin[i] / maxAbs) * duty : 0.0f;
}

// ============================================================================
// 控制週期
// ============================================================================
static float g_lastTickDt = 0.0f;

static void openLoopTick() {
    const uint64_t now = time_us_64();
    const bool force = g_resyncNext;
    const float rawDt = static_cast<float>(now - lastTickUs) * 1e-6f;
    const bool resync = force || lastTickUs == 0 || !(rawDt <= Cfg::DT_MAX_S);
    lastTickUs = now;
    g_resyncNext = false;
    if (resync) {
        g_resyncCount++;
        Serial.printf("[RESYNC] dt=%+dms → 開迴路重新同步（輸出歸零一震）\n",
                      static_cast<int>(lroundf(rawDt * 1000.0f)));
    }
    float dt = rawDt;
    if (!(dt > Cfg::DT_MIN_S)) dt = Cfg::DT_MIN_S;
    if (dt > Cfg::DT_MAX_S) dt = Cfg::DT_MAX_S;
    g_lastTickDt = dt;

    updateMeasurements(dt, resync);

    const float step = Cfg::OL_RAMP_PER_SEC * dt;
    for (int i = 0; i < 4; i++) {
        const float diff = g_olTarget[i] - g_olOut[i];
        if (fabsf(diff) <= step)
            g_olOut[i] = g_olTarget[i];
        else
            g_olOut[i] += (diff > 0 ? step : -step);

        wheels[i].targetRps = 0.0f;
        wheels[i].appliedDuty = resync ? 0.0f : g_olOut[i];
        hwWrite(i, wheels[i].appliedDuty);
        checkWheelHealth(i);
        if (g_mode != Mode::OpenLoop) return; // 可能已被自動停止
    }
}

static void closedLoopTick() {
    const uint64_t now = time_us_64();
    const bool force = g_resyncNext;
    const float rawDt = static_cast<float>(now - lastTickUs) * 1e-6f;
    const bool resync = force || lastTickUs == 0 || !(rawDt <= Cfg::DT_MAX_S);
    lastTickUs = now;
    g_resyncNext = false;
    if (resync) {
        g_resyncCount++;
        Serial.printf("[RESYNC] dt=%+dms → 閉迴路本週期不出力、重新同步\n",
                      static_cast<int>(lroundf(rawDt * 1000.0f)));
    }
    float dt = rawDt;
    if (!(dt > Cfg::DT_MIN_S)) dt = Cfg::DT_MIN_S;
    if (dt > Cfg::DT_MAX_S) dt = Cfg::DT_MAX_S;
    g_lastTickDt = dt;

    float targets[4];
    if (g_clDirect) {
        for (int i = 0; i < 4; i++) targets[i] = g_clDirectTarget[i];
        // 直接模式下 S 曲線僅作為顯示參考，不參與輸出
        rampVx.y = g_clVx;
        rampVy.y = g_clVy;
        rampOmega.y = g_clOmega;
    } else {
        const float rVx = rampVx.updatePw(g_clVx, dt);
        const float rVy = rampVy.updatePw(g_clVy, dt);
        const float rOmega = rampOmega.updatePw(g_clOmega, dt);
        chassisToWheelRps(rVx, rVy, rOmega, targets);
    }

    updateMeasurements(dt, resync);
    pidStep(targets, dt, resync);
    for (int i = 0; i < 4; i++) {
        checkWheelHealth(i);
        if (g_mode != Mode::ClosedLoop) return; // 自動停止
    }
}

// ============================================================================
// 序列輸入
// ============================================================================
static char g_line[128];
static size_t g_lineLen = 0;
static bool g_swallowLF = false;

static bool readCommand(char *out, size_t outLen) {
    while (Serial.available()) {
        const int c = Serial.read();
        if (c < 0) break;
        if (c == '\n' && g_swallowLF) {
            g_swallowLF = false;
            continue;
        }
        g_swallowLF = false;
        if (c == '\r' || c == '\n') {
            if (c == '\r') g_swallowLF = true;
            if (g_lineLen == 0) {
                strncpy(out, "stop", outLen - 1);
                out[outLen - 1] = '\0';
                return true;
            }
            g_line[g_lineLen] = '\0';
            strncpy(out, g_line, outLen - 1);
            out[outLen - 1] = '\0';
            g_lineLen = 0;
            return true;
        }
        if (c == 8 || c == 127) {
            if (g_lineLen) g_lineLen--;
            continue;
        }
        if (g_lineLen + 1 < sizeof(g_line))
            g_line[g_lineLen++] = static_cast<char>(c);
    }
    return false;
}

static void flushInput() {
    while (Serial.available()) Serial.read();
    g_lineLen = 0;
    g_swallowLF = false;
}

static bool anyKeyPressed() {
    if (Serial.available()) {
        flushInput();
        return true;
    }
    return false;
}

// ============================================================================
// 工具
// ============================================================================
static int wheelIndex(const char *name) {
    for (int i = 0; i < 4; i++)
        if (strcasecmp(name, kWheelName[i]) == 0) return i;
    return -1;
}

static bool parseWheelArg(const char *s, int &wheel) {
    wheel = wheelIndex(s);
    if (wheel < 0) {
        Serial.printf("未知輪別 '%s'（fl/fr/bl/br）\n", s);
        return false;
    }
    return true;
}

static int mrps(float v) { return static_cast<int>(lroundf(v * 1000.0f)); }
static int dpct(float v) { return static_cast<int>(lroundf(v * 10.0f)); }

// ============================================================================
// 停止 / 重置
// ============================================================================
static void resetPidState() {
    for (int i = 0; i < 4; i++) {
        WheelState &w = wheels[i];
        w.targetRps = 0.0f;
        w.measRps = 0.0f;
        w.rawRps = 0.0f;
        w.integral = 0.0f;
        w.prevMeas = 0.0f;
        w.hasPrev = false;
        w.outDuty = 0.0f;
        w.appliedDuty = 0.0f;
        w.encCount = enc[i];
        w.lastDelta = 0;
        w.termP = w.termI = w.termFf = w.termD = 0.0f;
        w.stallSinceMs = w.burstSinceMs = w.polSinceMs = 0;
        w.stallWarned = w.burstWarned = w.polWarned = false;
    }
    rampVx = SCurveRamp();
    rampVy = SCurveRamp();
    rampOmega = SCurveRamp();
}

static void stopAll() {
    for (int i = 0; i < 4; i++) {
        g_olTarget[i] = 0.0f;
        g_olOut[i] = 0.0f;
        g_clDirectTarget[i] = 0.0f;
        wheels[i].appliedDuty = 0.0f;
        hwWrite(i, 0.0f);
        g_effPct[i] = 0.0f;
    }
    g_clVx = g_clVy = g_clOmega = 0.0f;
    g_clDirect = false;
    resetPidState();
    g_mode = Mode::Idle;
    strncpy(g_modeText, "idle", sizeof(g_modeText) - 1);
    lastTickUs = 0;
    g_skipUntilUs = 0;
}

static void reseedRls() {
    for (int i = 0; i < 4; i++) {
        rls[i].a = gains[i].ff;
        rls[i].b = 0.0f;
        rls[i].P00 = RlsCfg::P_INIT;
        rls[i].P01 = 0.0f;
        rls[i].P11 = RlsCfg::P_INIT;
    }
}

static void resetEverything() {
    stopAll();
    reseedRls();
    for (int i = 0; i < 4; i++) {
        wheels[i].spikeCount = 0;
        wheels[i].fifoMax = 0;
        wheels[i].fifoFullCount = 0;
        wheels[i].polMismatchTicks = 0;
    }
    g_resyncCount = 0;
    Serial.println(F("[CMD] reset：PID / RLS / 計數全部歸零"));
}

// ============================================================================
// 啟動模式
// ============================================================================
struct ChassisCmd {
    const char *name;
    float vx, vy, omega;
};
static const ChassisCmd kChassis[] = {
    {"fwd", 0, +1, 0},   {"back", 0, -1, 0},
    {"left", +1, 0, 0},  {"right", -1, 0, 0},
    {"cw", 0, 0, +1},    {"ccw", 0, 0, -1},
};

static const ChassisCmd *findChassis(const char *name) {
    for (const ChassisCmd &cc : kChassis)
        if (!strcasecmp(name, cc.name)) return &cc;
    return nullptr;
}

static void startOpenLoop(const ChassisCmd &cc, float duty) {
    g_clDirect = false;
    resetPidState();
    chassisToWheelDuty(cc.vx, cc.vy, cc.omega, duty, g_olTarget);
    for (int i = 0; i < 4; i++) g_olOut[i] = 0.0f;
    g_mode = Mode::OpenLoop;
    snprintf(g_modeText, sizeof(g_modeText), "OL %s %d%%", cc.name,
             static_cast<int>(lroundf(duty)));
    g_resyncNext = true;
    lastTickUs = 0;
    Serial.printf("\n[CMD] 開迴路 %s duty=%d%%  FL=%+d FR=%+d BL=%+d BR=%+d（%%）\n",
                  cc.name, static_cast<int>(lroundf(duty)),
                  static_cast<int>(lroundf(g_olTarget[0])),
                  static_cast<int>(lroundf(g_olTarget[1])),
                  static_cast<int>(lroundf(g_olTarget[2])),
                  static_cast<int>(lroundf(g_olTarget[3])));
    if (tuneMaxTestDuty > 40.0f)
        Serial.println(F("      ⚠ 出力上限較高，請確認車身有空間 / 已架空。"));
}

// 單輪開迴路（duty% 直接給單輪；正負決定方向，不經過車身運動學）
static void startOpenLoopWheel(int wheel, float duty) {
    g_clDirect = false;
    resetPidState();
    for (int i = 0; i < 4; i++) {
        g_olTarget[i] = 0.0f;
        g_olOut[i] = 0.0f;
    }
    g_olTarget[wheel] = duty;
    g_mode = Mode::OpenLoop;
    snprintf(g_modeText, sizeof(g_modeText), "OL %s %+d%%", kWheelName[wheel],
             static_cast<int>(lroundf(duty)));
    g_resyncNext = true;
    lastTickUs = 0;
    Serial.printf("\n[CMD] 單輪開迴路 %s duty=%+d%%\n", kWheelName[wheel],
                  static_cast<int>(lroundf(duty)));
}

static void startClosedLoop(const ChassisCmd &cc, float rps) {
    g_clVx = cc.vx * rps;
    g_clVy = cc.vy * rps;
    g_clOmega = cc.omega * rps;
    g_clDirect = false;
    resetPidState();
    g_mode = Mode::ClosedLoop;
    snprintf(g_modeText, sizeof(g_modeText), "CL %s %+dmRPS", cc.name,
             mrps(rps));
    g_resyncNext = true;
    lastTickUs = 0;
    float t[4];
    chassisToWheelRps(g_clVx, g_clVy, g_clOmega, t);
    Serial.printf("\n[CMD] 閉迴路 %s 車身 %+dmRPS → 輪目標 mRPS FL=%+d FR=%+d BL=%+d BR=%+d\n",
                  cc.name, mrps(rps), mrps(t[0]), mrps(t[1]), mrps(t[2]),
                  mrps(t[3]));
}

static void startClosedDirect(const char *label, const float targets[4]) {
    for (int i = 0; i < 4; i++) g_clDirectTarget[i] = targets[i];
    g_clDirect = true;
    resetPidState();
    g_mode = Mode::ClosedLoop;
    snprintf(g_modeText, sizeof(g_modeText), "CL %s", label);
    g_resyncNext = true;
    lastTickUs = 0;
    Serial.printf("\n[CMD] 閉迴路 %s 輪目標 mRPS FL=%+d FR=%+d BL=%+d BR=%+d\n", label,
                  mrps(targets[0]), mrps(targets[1]), mrps(targets[2]),
                  mrps(targets[3]));
}

// ============================================================================
// probe — 逐輪自動診斷（開迴路 + 閉迴路）
// ============================================================================
static void probeOpenLoop(int wheel, uint32_t ms, int32_t &delta,
                          bool &aborted) {
    for (int i = 0; i < 4; i++) {
        g_olTarget[i] = 0.0f;
        g_olOut[i] = 0.0f;
        hwWrite(i, 0.0f);
    }
    delay(150);
    drainEncoders();
    const int32_t before = enc[wheel];
    const uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        if (anyKeyPressed()) {
            aborted = true;
            break;
        }
        drainEncoders();
        hwWrite(wheel, 15.0f);
        delayMicroseconds(400);
    }
    hwWrite(wheel, 0.0f);
    drainEncoders();
    delta = enc[wheel] - before;
}

static void probeClosedLoop(int wheel, float target, uint32_t ms,
                            int32_t &delta, bool &aborted) {
    for (int i = 0; i < 4; i++) hwWrite(i, 0.0f);
    delay(150);
    drainEncoders();
    for (int i = 0; i < 4; i++) {
        wheels[i].integral = 0.0f;
        wheels[i].appliedDuty = 0.0f;
        wheels[i].hasPrev = false;
        wheels[i].encCount = enc[i];
    }
    const int32_t before = enc[wheel];
    float targets[4] = {0, 0, 0, 0};
    targets[wheel] = target;
    uint64_t lastUs = 0;
    bool first = true;
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        if (anyKeyPressed()) {
            aborted = true;
            break;
        }
        drainEncoders();
        const uint64_t now = time_us_64();
        float dt = static_cast<float>(now - lastUs) * 1e-6f;
        const bool resync = first || !(dt <= Cfg::DT_MAX_S);
        first = false;
        lastUs = now;
        if (!(dt > Cfg::DT_MIN_S)) dt = Cfg::DT_MIN_S;
        if (dt > Cfg::DT_MAX_S) dt = Cfg::DT_MAX_S;
        updateMeasurements(dt, resync);
        pidStep(targets, dt, resync);
        delayMicroseconds(500);
    }
    for (int i = 0; i < 4; i++) hwWrite(i, 0.0f);
    drainEncoders();
    delta = enc[wheel] - before;
}

static void runProbe(uint32_t ms) {
    Serial.println(F("\n========== PROBE：逐輪 開迴路 vs 閉迴路 =========="));
    Serial.println(F("每輪：OL 15% 轉一段，再 CL 0.6 RPS 轉一段；車身請架空，按任意鍵中止。"));
    stopAll();
    flushInput();
    bool aborted = false;
    const float sec = static_cast<float>(ms) * 1e-3f;

    // ---- 0) 待機雜訊：四輪 PWM=0、完全不動時，編碼器自己跳多少 ----
    //   這裡才是「計數可不可信」的真正判斷點。若靜止就在跳，代表該輪
    //   編碼器有雜訊/回彈/干擾，轉動時會量到假高速（尤其高速時）。
    drainEncoders();
    int32_t idleBefore[4];
    for (int i = 0; i < 4; i++) idleBefore[i] = enc[i];
    const uint32_t nz0 = millis();
    while (millis() - nz0 < 500) {
        drainEncoders();
        delayMicroseconds(200);
    }
    drainEncoders();
    int32_t idleDelta[4];
    for (int i = 0; i < 4; i++) idleDelta[i] = enc[i] - idleBefore[i];
    Serial.printf("[NOISE] 待機 500ms 編碼器跳動：FL=%+d FR=%+d BL=%+d BR=%+d\n",
                  static_cast<int>(idleDelta[0]), static_cast<int>(idleDelta[1]),
                  static_cast<int>(idleDelta[2]), static_cast<int>(idleDelta[3]));
    Serial.println(F("        應接近 0；某輪很大 → 該輪編碼器雜訊/回彈 → 會量到假轉速"));

    // 雜訊階段順便產生的 FIFO 統計不算到正式 probe
    for (int i = 0; i < 4; i++) {
        wheels[i].fifoMax = 0;
        wheels[i].fifoFullCount = 0;
        wheels[i].spikeCount = 0;
    }

    struct Result {
        int32_t olDelta = 0;
        int32_t clDelta = 0;
        float olRps = 0.0f;
        float clRps = 0.0f;
        uint32_t fifoFullDelta = 0;
        uint32_t spikes = 0;
        bool done = false;
    };
    Result r[4];

    for (int w = 0; w < 4 && !aborted; w++) {
        drainEncoders(); // 先吸乾再列印，避免列印期間塞滿 FIFO 被誤判
        Serial.printf("\n[PROBE] %s ...\n", kWheelName[w]);
        const uint32_t fifoBefore = wheels[w].fifoFullCount;
        const uint32_t spikeBefore = wheels[w].spikeCount;
        probeOpenLoop(w, ms, r[w].olDelta, aborted);
        if (aborted) break;
        probeClosedLoop(w, 0.6f, ms, r[w].clDelta, aborted);
        if (aborted) break;

        r[w].olRps = static_cast<float>(r[w].olDelta) / gCpr[w] / sec;
        r[w].clRps = static_cast<float>(r[w].clDelta) / gCpr[w] / sec;
        r[w].done = true;
        r[w].fifoFullDelta = wheels[w].fifoFullCount - fifoBefore;
        r[w].spikes = wheels[w].spikeCount - spikeBefore;

        Serial.printf("  OL 15%%: d=%+5d  %+5dmRPS   CL +0.60: d=%+5d  %+5dmRPS\n",
                      static_cast<int>(r[w].olDelta), mrps(r[w].olRps),
                      static_cast<int>(r[w].clDelta), mrps(r[w].clRps));
        Serial.printf("         fifoFull=+%u  spike=+%u  %s\n",
                      static_cast<unsigned>(r[w].fifoFullDelta),
                      static_cast<unsigned>(r[w].spikes),
                      r[w].fifoFullDelta
                          ? "(FIFO 曾滿 → SM push block 停頓；非掉數，多為列印/drain 間隔)"
                          : "FIFO ok");
    }
    stopAll();
    resetPidState();
    lastTickUs = 0;

    Serial.println(F("\n---------- PROBE 結果 ----------"));
    Serial.println(F("輪  OL(d/RPS)     CL(d/RPS)     判定"));
    for (int w = 0; w < 4; w++) {
        if (!r[w].done) {
            Serial.printf("%s   (未完成)\n", kWheelName[w]);
            continue;
        }
        const bool olMoves = fabsf(r[w].olRps) > 0.05f;
        const bool clMoves = fabsf(r[w].clRps) > 0.05f;
        const bool wrongWay = clMoves && r[w].clRps < -0.05f;
        const bool tooFast = clMoves && fabsf(r[w].clRps) > 1.2f;
        const char *verdict = "OK（開/閉迴路都有轉）";
        if (!olMoves && !clMoves)
            verdict = "★ 完全不動：馬達電源/驅動/腳位/編碼器";
        else if (!olMoves)
            verdict = "★ 開迴路不轉但閉迴路有轉：查 PWM/DIR 腳位或驅動";
        else if (!clMoves)
            verdict = "★ 開迴路會轉、閉迴路不動：PID 方向或回授掛";
        else if (wrongWay)
            verdict = "★ 閉迴路反向：kEncReadPolarity 可能反了（正回饋）";
        else if (tooFast)
            verdict = "★ 閉迴路超速：編碼器掉計數/極性錯 → 爆衝來源";
        if (idleDelta[w] > 50 || idleDelta[w] < -50)
            verdict = "★ 待機就在跳數（編碼器雜訊/回彈）→ 本列量測不可信";
        Serial.printf("%s   %+5d/%+5d   %+5d/%+5d   %s\n", kWheelName[w],
                      static_cast<int>(r[w].olDelta), mrps(r[w].olRps),
                      static_cast<int>(r[w].clDelta), mrps(r[w].clRps),
                      verdict);
    }

    // 每轉計數比例（同等 15% duty 下應相近；某輪特別高 → 該輪 CPR 不同或雜訊）
    if (r[0].done && fabsf(r[0].olRps) > 0.05f) {
        Serial.println(F("每轉計數比例（OL 15% 相對 FL，1.00 ≈ 同型編碼器）："));
        for (int w = 0; w < 4; w++) {
            if (!r[w].done || fabsf(r[w].olRps) <= 0.05f) continue;
            const float ratio = r[w].olRps / r[0].olRps;
            const int rp = static_cast<int>(lroundf(ratio * 100.0f));
            Serial.printf("  %s x%+d%%  cpr=%d %s\n", kWheelName[w], rp - 100,
                          static_cast<int>(lroundf(gCpr[w])),
                          (ratio < 0.7f || ratio > 1.4f)
                              ? "  ← 與 FL 差很多：用 cpr 校正或查編碼器接線"
                              : "");
        }
    }
    printStatus();
    Serial.println(F("--------------------------------"));
    if (aborted) Serial.println(F("[PROBE] 已中止（結果僅部分有效）"));
}

// ============================================================================
// 狀態 / 記錄
// ============================================================================
static void printStatus() {
    Serial.printf("\n[%s] tick=%ums resync=%u\n", g_modeText,
                  static_cast<unsigned>(g_tickMs),
                  static_cast<unsigned>(g_resyncCount));
    for (int i = 0; i < 4; i++) {
        const WheelState &w = wheels[i];
        const char *polFlag =
            (g_mode == Mode::ClosedLoop && fabsf(w.targetRps) > 0.05f &&
             fabsf(w.measRps) > 0.05f &&
             ((w.targetRps > 0.f) != (w.measRps > 0.f)))
                ? " XX"
                : "";
        Serial.printf(
            "  %s tgt=%+6d meas=%+6d out=%+5d app=%+5d d=%+4d spk=%u fifo=%u/%u%s\n",
            kWheelName[i], mrps(w.targetRps), mrps(w.measRps), dpct(w.outDuty),
            dpct(w.appliedDuty), static_cast<int>(w.lastDelta),
            static_cast<unsigned>(w.spikeCount),
            static_cast<unsigned>(w.fifoMax),
            static_cast<unsigned>(w.fifoFullCount), polFlag);
    }
    if (g_mode == Mode::ClosedLoop) {
        Serial.printf("  RLS a(milli)=%d %d %d %d  b(milli)=%d %d %d %d\n",
                      mrps(rls[0].a), mrps(rls[1].a), mrps(rls[2].a),
                      mrps(rls[3].a), mrps(rls[0].b), mrps(rls[1].b),
                      mrps(rls[2].b), mrps(rls[3].b));
    }
}

static void printLogLine(uint32_t now) {
    const uint32_t dtUs = static_cast<uint32_t>(g_lastTickDt * 1e6f);
    for (int i = 0; i < 4; i++) {
        const WheelState &w = wheels[i];
        Serial.printf("%u,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u\n", now, kWheelName[i],
                      mrps(w.targetRps), mrps(w.measRps), dpct(w.termP),
                      dpct(w.integral), dpct(w.termFf), dpct(w.outDuty),
                      dpct(w.appliedDuty), static_cast<int>(w.lastDelta), dtUs,
                      static_cast<unsigned>(w.fifoMax));
    }
}

static void printLogHeader() {
    Serial.println(F("# ms,wheel,tgt_mrps,meas_mrps,p_cd,i_cd,ff_cd,out_cd,app_cd,dcount,dt_us,fifo_max"));
}

// ============================================================================
// 報表
// ============================================================================
static void printMap() {
    Serial.println(F("\n---- 腳位 / 極性（新板） ----"));
    Serial.println(F("輪   PWM   DIR   ENC(A/B)   motorPol  encPol  cpr"));
    for (int i = 0; i < 4; i++)
        Serial.printf("%s   GP%-2u  GP%-2u  GP%-2u/GP%-2u   %+d        %+d     %d\n",
                      kWheelName[i], static_cast<unsigned>(kPwmPin[i]),
                      static_cast<unsigned>(kDirPin[i]),
                      static_cast<unsigned>(kEncPin[i]),
                      static_cast<unsigned>(kEncPin[i] + 1),
                      static_cast<int>(motorPol[i]),
                      static_cast<int>(encPol[i]),
                      static_cast<int>(lroundf(gCpr[i])));
    Serial.printf("軸向：strafe=%+d forward=%+d rot=%+d  encdiv=%d\n",
                  static_cast<int>(lroundf(axisStrafe)),
                  static_cast<int>(lroundf(axisForward)),
                  static_cast<int>(lroundf(axisRot)),
                  static_cast<int>(lroundf(g_encClkDiv)));
    Serial.println(F("------------------------------"));
}

static void printReport() {
    Serial.println(F("\n===== 目前參數（可貼回 src/move/main.cpp 比對） ====="));
    Serial.printf("PID_MAX_DUTY=%d  WINDUP_MARGIN_DUTY=%d(x0.1)  VEL_LPF_TAU=%dus\n",
                  static_cast<int>(lroundf(tunePidMax)),
                  static_cast<int>(lroundf(tuneWindupMargin * 10.0f)),
                  static_cast<int>(lroundf(tuneLpfTau * 1e6f)));
    Serial.printf("OUTPUT_SLEW_DUTY_PER_S=%d  RLS=%s  lambda=%d(x0.001)  encdiv=%d  tick=%ums\n",
                  static_cast<int>(lroundf(tuneOutputSlew)), rlsEnabled ? "on" : "off",
                  static_cast<int>(lroundf(tuneRlsLambda * 1000.0f)),
                  static_cast<int>(lroundf(g_encClkDiv)),
                  static_cast<unsigned>(g_tickMs));
    for (int i = 0; i < 4; i++)
        Serial.printf("  %s kP=%d kI=%d kD=%d ff=%d (x0.01)   RLS a=%d b=%d (x0.01)\n",
                      kWheelName[i], static_cast<int>(lroundf(gains[i].kP * 100.0f)),
                      static_cast<int>(lroundf(gains[i].kI * 100.0f)),
                      static_cast<int>(lroundf(gains[i].kD * 100.0f)),
                      static_cast<int>(lroundf(gains[i].ff * 100.0f)),
                      static_cast<int>(lroundf(rls[i].a * 100.0f)),
                      static_cast<int>(lroundf(rls[i].b * 100.0f)));
    Serial.printf("kWheelPolarity[4]   = {%+d, %+d, %+d, %+d};\n",
                  static_cast<int>(motorPol[0]), static_cast<int>(motorPol[1]),
                  static_cast<int>(motorPol[2]), static_cast<int>(motorPol[3]));
    Serial.printf("kEncReadPolarity[4] = {%+d, %+d, %+d, %+d};\n",
                  static_cast<int>(encPol[0]), static_cast<int>(encPol[1]),
                  static_cast<int>(encPol[2]), static_cast<int>(encPol[3]));
    Serial.println(F("====================================================="));
}

static void printHelp() {
    Serial.println(F(
        "\n---------- move_drive_test ----------\n"
        " ol   <dir> [duty]        開迴路（fwd/back/left/right/cw/ccw，預設 25%）\n"
        " w    <輪> <duty%>        單輪開迴路（掃 duty 看量測是否線性）\n"
        " cl   <dir> [rps]         閉迴路（車身速度，預設 0.8 RPS）\n"
        " spin <輪> <rps>          閉迴路單輪（真實輪 RPS）\n"
        " hold <輪> <duty>         閉迴路單輪，用等效 duty% 指定目標\n"
        " allhold <duty>           四輪同一個等效 duty% 目標\n"
        " stop / s / 空行           停止（PWM=0、積分歸零）\n"
        " reset                    停止 + PID/RLS/計數全部重置\n"
        " probe [ms]               逐輪 OL vs CL 自動診斷（預設 1200）\n"
        " tick <ms>                控制週期（5~100，預設 10）\n"
        " drop <n>                 模擬掉幀：hold n 個控制週期不更新\n"
        " rls on|off               線上自適應前饋開關\n"
        " pid <輪> <kP> <kI> [ff]  即時改單輪增益\n"
        " cap <duty>               測試出力上限（預設 60）\n"
        " slew <duty%/s>           輸出斜率限制\n"
        " lpf <ms>                 轉速量測低通時間常數\n"
        " encdiv <v>               編碼器 PIO clkdiv（預設 500）\n"
        " cpr <輪> <counts>       每輪計數/圈（四輪編碼器不同型號時校正）\n"
        " edges [ms]              量測 8 條編碼器線實際翻轉率（診斷耦合/雜訊）\n"
        " pol/encpol <輪> <-1|1>   改馬達 / 編碼器極性\n"
        " axis <strafe|fwd|rot> <-1|1>\n"
        " log on|off | log <hz>    CSV 記錄（預設 20Hz）\n"
        " enc                      待機手轉輪子看原始計數\n"
        " status / map / report / help\n"
        "------------------------------------"));
}

// ============================================================================
// 指令處理
// ============================================================================
// ============================================================================
// 直接量 8 條編碼器線的翻轉次數（不經過 PIO/Gray 解碼）
//   用途：分辨「輪子真的在轉」vs「PWM 耦合/雜訊」——
//   若馬達沒轉但某條線狂翻，就是電氣干擾或接錯。
// ============================================================================
static void measureEncoderEdges(uint32_t ms) {
    uint8_t prev[8];
    uint32_t cnt[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
        prev[i * 2] = gpio_get(kEncPin[i]) ? 1 : 0;
        prev[i * 2 + 1] = gpio_get(kEncPin[i] + 1) ? 1 : 0;
    }
    const uint32_t t0 = micros();
    const uint32_t durUs = ms * 1000u;
    while (micros() - t0 < durUs) {
        for (int i = 0; i < 4; i++) {
            for (int k = 0; k < 2; k++) {
                const uint8_t v = gpio_get(kEncPin[i] + k) ? 1 : 0;
                const int idx = i * 2 + k;
                if (v != prev[idx]) {
                    cnt[idx]++;
                    prev[idx] = v;
                }
            }
        }
        drainEncoders();
    }
    Serial.printf("[EDGES] %ums 內翻轉次數 / 估算 Hz：\n",
                  static_cast<unsigned>(ms));
    for (int i = 0; i < 4; i++) {
        const uint32_t a = cnt[i * 2], b = cnt[i * 2 + 1];
        const uint32_t ha = a * 1000u / (ms ? ms : 1);
        const uint32_t hb = b * 1000u / (ms ? ms : 1);
        Serial.printf("  %s A(GP%u)=%u(~%uHz)  B(GP%u)=%u(~%uHz)\n",
                      kWheelName[i], static_cast<unsigned>(kEncPin[i]),
                      static_cast<unsigned>(a), static_cast<unsigned>(ha),
                      static_cast<unsigned>(kEncPin[i] + 1),
                      static_cast<unsigned>(b), static_cast<unsigned>(hb));
    }
}

static void handleCommand(char *line) {
    char *argv[4] = {nullptr, nullptr, nullptr, nullptr};
    int argc = 0;
    for (char *p = strtok(line, " \t"); p && argc < 4; p = strtok(nullptr, " \t"))
        argv[argc++] = p;
    if (argc == 0) return;
    const char *cmd = argv[0];

    if (!strcasecmp(cmd, "help") || !strcasecmp(cmd, "?")) {
        printHelp();
        return;
    }
    if (!strcasecmp(cmd, "stop") || !strcasecmp(cmd, "s")) {
        stopAll();
        Serial.println(F("[CMD] stop"));
        return;
    }
    if (!strcasecmp(cmd, "reset")) {
        resetEverything();
        return;
    }
    if (!strcasecmp(cmd, "status")) {
        printStatus();
        return;
    }
    if (!strcasecmp(cmd, "map")) {
        printMap();
        return;
    }
    if (!strcasecmp(cmd, "report")) {
        printReport();
        return;
    }
    if (!strcasecmp(cmd, "log")) {
        if (argc < 2 || !strcasecmp(argv[1], "on")) {
            g_logOn = true;
            printLogHeader();
            lastLogMs = 0;
            Serial.println(F("[CMD] log on"));
        } else if (!strcasecmp(argv[1], "off")) {
            g_logOn = false;
            Serial.println(F("[CMD] log off"));
        } else {
            const int hz = atoi(argv[1]);
            if (hz < 1 || hz > 100) {
                Serial.println(F("用法：log on|off|<1~100 Hz>"));
                return;
            }
            g_logHz = static_cast<uint32_t>(hz);
            g_logOn = true;
            printLogHeader();
            lastLogMs = 0;
            Serial.printf("[CMD] log on @ %uHz\n", static_cast<unsigned>(g_logHz));
        }
        return;
    }
    if (!strcasecmp(cmd, "tick")) {
        if (argc < 2) {
            Serial.printf("目前 tick=%ums\n", static_cast<unsigned>(g_tickMs));
            return;
        }
        int ms = atoi(argv[1]);
        if (ms < 5) ms = 5;
        if (ms > 100) ms = 100;
        g_tickMs = static_cast<uint32_t>(ms);
        lastTickUs = 0;
        Serial.printf("[CMD] tick=%ums（>50ms 每週期都會 resync）\n",
                      static_cast<unsigned>(g_tickMs));
        return;
    }
    if (!strcasecmp(cmd, "drop")) {
        if (argc < 2) {
            Serial.println(F("用法：drop <週期數>"));
            return;
        }
        int n = atoi(argv[1]);
        if (n < 1) n = 1;
        g_skipUntilUs = time_us_64() +
                        static_cast<uint64_t>(n) * g_tickMs * 1000ULL;
        Serial.printf("[CMD] 模擬掉幀 %d 個週期（%dms）\n", n,
                      n * static_cast<int>(g_tickMs));
        return;
    }
    if (!strcasecmp(cmd, "rls")) {
        if (argc < 2) {
            Serial.printf("RLS=%s\n", rlsEnabled ? "on" : "off");
            return;
        }
        if (!strcasecmp(argv[1], "on"))
            rlsEnabled = true;
        else if (!strcasecmp(argv[1], "off"))
            rlsEnabled = false;
        else {
            Serial.println(F("用法：rls on|off"));
            return;
        }
        Serial.printf("[CMD] RLS %s\n", rlsEnabled ? "on" : "off");
        return;
    }
    if (!strcasecmp(cmd, "pid")) {
        int w = -1;
        if (argc < 4 || !parseWheelArg(argv[1], w)) {
            Serial.println(F("用法：pid <輪> <kP> <kI> [ff]"));
            return;
        }
        gains[w].kP = static_cast<float>(atof(argv[2]));
        gains[w].kI = static_cast<float>(atof(argv[3]));
        if (argc >= 5) {
            gains[w].ff = static_cast<float>(atof(argv[4]));
            rls[w].a = gains[w].ff;
        }
        Serial.printf("[CMD] %s kP=%d kI=%d ff=%d (x0.01)\n", kWheelName[w],
                      static_cast<int>(lroundf(gains[w].kP * 100.0f)),
                      static_cast<int>(lroundf(gains[w].kI * 100.0f)),
                      static_cast<int>(lroundf(gains[w].ff * 100.0f)));
        return;
    }
    if (!strcasecmp(cmd, "cap")) {
        if (argc < 2) {
            Serial.printf("目前 cap=%d%%\n", static_cast<int>(lroundf(tuneMaxTestDuty)));
            return;
        }
        float v = static_cast<float>(atof(argv[1]));
        if (v < 5.0f) v = 5.0f;
        if (v > 100.0f) v = 100.0f;
        tuneMaxTestDuty = v;
        Serial.printf("[CMD] cap=%d%%\n", static_cast<int>(lroundf(tuneMaxTestDuty)));
        return;
    }
    if (!strcasecmp(cmd, "slew")) {
        if (argc < 2) {
            Serial.printf("目前 slew=%d %%/s\n", static_cast<int>(lroundf(tuneOutputSlew)));
            return;
        }
        float v = static_cast<float>(atof(argv[1]));
        if (v < 50.0f) v = 50.0f;
        if (v > 10000.0f) v = 10000.0f;
        tuneOutputSlew = v;
        Serial.printf("[CMD] slew=%d %%/s\n", static_cast<int>(lroundf(tuneOutputSlew)));
        return;
    }
    if (!strcasecmp(cmd, "lpf")) {
        if (argc < 2) {
            Serial.printf("目前 lpf=%dms\n",
                          static_cast<int>(lroundf(tuneLpfTau * 1000.0f)));
            return;
        }
        float ms = static_cast<float>(atof(argv[1]));
        if (ms < 1.0f) ms = 1.0f;
        if (ms > 100.0f) ms = 100.0f;
        tuneLpfTau = ms * 1e-3f;
        Serial.printf("[CMD] lpf=%dms\n", static_cast<int>(lroundf(ms)));
        return;
    }
    if (!strcasecmp(cmd, "encdiv")) {
        if (argc < 2) {
            Serial.printf("目前 encdiv=%d\n", static_cast<int>(lroundf(g_encClkDiv)));
            return;
        }
        float v = static_cast<float>(atof(argv[1]));
        if (v < 1.0f) v = 1.0f;
        if (v > 20000.0f) v = 20000.0f;
        stopAll();
        encoderInit(v);
        resetPidState();
        Serial.printf("[CMD] encdiv=%d（編碼器計數已清零）\n",
                      static_cast<int>(lroundf(g_encClkDiv)));
        return;
    }
    if (!strcasecmp(cmd, "cpr")) {
        int w = -1;
        if (argc < 3 || !parseWheelArg(argv[1], w)) {
            Serial.println(F("用法：cpr <輪> <counts/圈>（4X 計數；預設 1772）"));
            return;
        }
        float v = static_cast<float>(atof(argv[2]));
        if (v < 100.0f) v = 100.0f;
        if (v > 100000.0f) v = 100000.0f;
        gCpr[w] = v;
        Serial.printf("[CMD] %s cpr=%d counts/rev\n", kWheelName[w],
                      static_cast<int>(lroundf(gCpr[w])));
        return;
    }
    if (!strcasecmp(cmd, "pol") || !strcasecmp(cmd, "encpol")) {
        int w = -1;
        if (argc < 3 || !parseWheelArg(argv[1], w)) {
            Serial.println(F("用法：pol/encpol <輪> <-1|1>"));
            return;
        }
        const int v = atoi(argv[2]);
        if (v != 1 && v != -1) {
            Serial.println(F("極性只能是 -1 或 1"));
            return;
        }
        if (!strcasecmp(cmd, "pol"))
            motorPol[w] = static_cast<int8_t>(v);
        else
            encPol[w] = static_cast<int8_t>(v);
        Serial.printf("[CMD] %s %s = %+d", strcasecmp(cmd, "pol") ? "encPol" : "motorPol",
                      kWheelName[w], v);
        if (g_mode == Mode::ClosedLoop || g_mode == Mode::OpenLoop)
            Serial.println(F("  ⚠ 行進中改極性會瞬間反向，建議先 stop 再改"));
        else
            Serial.println();
        return;
    }
    if (!strcasecmp(cmd, "axis")) {
        if (argc < 3) {
            Serial.printf("目前 axis：strafe=%+d fwd=%+d rot=%+d\n",
                          static_cast<int>(lroundf(axisStrafe)),
                          static_cast<int>(lroundf(axisForward)),
                          static_cast<int>(lroundf(axisRot)));
            Serial.println(F("用法：axis <strafe|fwd|rot> <-1|1>"));
            return;
        }
        const int v = atoi(argv[2]);
        if (v != 1 && v != -1) {
            Serial.println(F("極性只能是 -1 或 1"));
            return;
        }
        if (!strcasecmp(argv[1], "strafe") || !strcasecmp(argv[1], "left"))
            axisStrafe = static_cast<float>(v);
        else if (!strcasecmp(argv[1], "fwd") || !strcasecmp(argv[1], "forward"))
            axisForward = static_cast<float>(v);
        else if (!strcasecmp(argv[1], "rot") || !strcasecmp(argv[1], "cw"))
            axisRot = static_cast<float>(v);
        else {
            Serial.println(F("用法：axis <strafe|fwd|rot> <-1|1>"));
            return;
        }
        Serial.printf("[CMD] axis %s = %+d\n", argv[1], v);
        return;
    }
    if (!strcasecmp(cmd, "probe")) {
        uint32_t ms = 1200;
        if (argc >= 2) {
            const int v = atoi(argv[1]);
            if (v >= 300) ms = static_cast<uint32_t>(v);
        }
        runProbe(ms);
        return;
    }
    if (!strcasecmp(cmd, "enc")) {
        stopAll();
        g_mode = Mode::EncWatch;
        strncpy(g_modeText, "enc-watch（手轉輪子）", sizeof(g_modeText) - 1);
        Serial.println(F("[CMD] enc：手轉輪子看原始計數（Enter/stop 結束）"));
        return;
    }
    if (!strcasecmp(cmd, "edges")) {
        uint32_t ms = 200;
        if (argc >= 2) {
            const int v = atoi(argv[1]);
            if (v >= 20 && v <= 2000) ms = static_cast<uint32_t>(v);
        }
        measureEncoderEdges(ms);
        return;
    }
    if (!strcasecmp(cmd, "w")) {
        int w = -1;
        if (argc < 3 || !parseWheelArg(argv[1], w)) {
            Serial.println(F("用法：w <輪> <duty%>（單輪開迴路，可負）"));
            return;
        }
        float duty = static_cast<float>(atof(argv[2]));
        if (duty > tuneMaxTestDuty) duty = tuneMaxTestDuty;
        if (duty < -tuneMaxTestDuty) duty = -tuneMaxTestDuty;
        startOpenLoopWheel(w, duty);
        return;
    }
    if (!strcasecmp(cmd, "ol")) {
        const ChassisCmd *cc = (argc >= 2) ? findChassis(argv[1]) : nullptr;
        if (!cc) {
            Serial.println(F("用法：ol <fwd|back|left|right|cw|ccw> [duty%]"));
            return;
        }
        float duty = 25.0f;
        if (argc >= 3) duty = static_cast<float>(atof(argv[2]));
        if (duty < 0) duty = -duty;
        if (duty < 5.0f) duty = 5.0f;
        if (duty > tuneMaxTestDuty) duty = tuneMaxTestDuty;
        startOpenLoop(*cc, duty);
        return;
    }
    if (!strcasecmp(cmd, "cl")) {
        const ChassisCmd *cc = (argc >= 2) ? findChassis(argv[1]) : nullptr;
        if (!cc) {
            Serial.println(F("用法：cl <fwd|back|left|right|cw|ccw> [rps]"));
            return;
        }
        float rps = 0.8f;
        if (argc >= 3) rps = static_cast<float>(atof(argv[2]));
        if (rps < 0) rps = -rps;
        if (rps > 5.0f) rps = 5.0f;
        startClosedLoop(*cc, rps);
        return;
    }
    if (!strcasecmp(cmd, "spin")) {
        int w = -1;
        if (argc < 3 || !parseWheelArg(argv[1], w)) {
            Serial.println(F("用法：spin <輪> <rps>"));
            return;
        }
        float rps = static_cast<float>(atof(argv[2]));
        if (rps > 5.0f) rps = 5.0f;
        if (rps < -5.0f) rps = -5.0f;
        float t[4] = {0, 0, 0, 0};
        t[w] = rps;
        char label[32];
        snprintf(label, sizeof(label), "spin %s %+.2f RPS", kWheelName[w],
                 static_cast<double>(rps));
        startClosedDirect(label, t);
        return;
    }
    if (!strcasecmp(cmd, "hold") || !strcasecmp(cmd, "allhold")) {
        float percent = 25.0f;
        float t[4] = {0, 0, 0, 0};
        char label[40];
        if (!strcasecmp(cmd, "hold")) {
            int w = -1;
            if (argc < 3 || !parseWheelArg(argv[1], w)) {
                Serial.println(F("用法：hold <輪> <duty%>"));
                return;
            }
            percent = static_cast<float>(atof(argv[2]));
            if (percent < -tuneMaxTestDuty) percent = -tuneMaxTestDuty;
            if (percent > tuneMaxTestDuty) percent = tuneMaxTestDuty;
            const float a = (gains[w].ff > 1e-3f) ? gains[w].ff : 15.0f;
            t[w] = percent / a;
            snprintf(label, sizeof(label), "hold %s %+d%%", kWheelName[w],
                     static_cast<int>(lroundf(percent)));
        } else {
            if (argc >= 2) percent = static_cast<float>(atof(argv[1]));
            if (percent < -tuneMaxTestDuty) percent = -tuneMaxTestDuty;
            if (percent > tuneMaxTestDuty) percent = tuneMaxTestDuty;
            for (int i = 0; i < 4; i++) {
                const float a = (gains[i].ff > 1e-3f) ? gains[i].ff : 15.0f;
                t[i] = percent / a;
            }
            snprintf(label, sizeof(label), "allhold %+d%%",
                     static_cast<int>(lroundf(percent)));
        }
        startClosedDirect(label, t);
        return;
    }

    Serial.printf("未知指令 '%s'，輸入 help 看說明。\n", cmd);
}

// ============================================================================
// setup / loop
// ============================================================================
void setup() {
    // 第一件事：把馬達腳位鎖在「確定零出力」
    for (int i = 0; i < 4; i++) initWheelHw(i);
    reseedRls();

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(115200);
    delay(300);
    encoderInit(g_encClkDiv);
    stopAll();

    Serial.println(F("\n=================================================="));
    Serial.println(F(" Move 閉迴路行進診斷 (move_drive_test)"));
    Serial.println(F(" 直接搬入正式韌體的 S 曲線 + PI + RLS + 斜率限制"));
    Serial.println(F(" 建議：probe → cl fwd 0.8 → log on → 重現症狀"));
    Serial.println(F(" 輸入 help 看完整指令。"));
    Serial.println(F("=================================================="));
    printMap();
}

void loop() {
    drainEncoders();

    // 序列指令
    char line[128];
    if (readCommand(line, sizeof(line))) handleCommand(line);

    const uint64_t nowUs = time_us_64();

    // 掉幀模擬：期間保持輸出、不更新（下一個 tick 會看到大 dt → resync）
    if (g_skipUntilUs != 0) {
        if (nowUs < g_skipUntilUs) {
            // 仍要印狀態，讓使用者看到「卡住」期間的實際行為
        } else {
            g_skipUntilUs = 0;
            Serial.println(F("[DROP] 恢復控制（下一個週期會 resync）"));
        }
    }

    bool ticked = false;
    if (g_skipUntilUs == 0) {
        if (g_mode == Mode::OpenLoop || g_mode == Mode::ClosedLoop) {
            const uint64_t tickUs = static_cast<uint64_t>(g_tickMs) * 1000ULL;
            if (lastTickUs == 0 || nowUs - lastTickUs >= tickUs) {
                if (g_mode == Mode::OpenLoop)
                    openLoopTick();
                else
                    closedLoopTick();
                ticked = true;
            }
        }
    }

    // 狀態列印（待機且非 EncWatch 時不洗畫面，避免洗掉指令回覆）
    static uint32_t lastStatusMs = 0;
    const uint32_t nowMs = millis();
    if (g_mode != Mode::Idle && nowMs - lastStatusMs >= Cfg::STATUS_MS &&
        !g_logOn) {
        lastStatusMs = nowMs;
        if (g_mode == Mode::EncWatch) {
            static int32_t prev[4] = {0, 0, 0, 0};
            Serial.printf("[enc] raw FL=%+d FR=%+d BL=%+d BR=%+d  Δ FL=%+d FR=%+d BL=%+d BR=%+d\n",
                          static_cast<int>(enc[0]), static_cast<int>(enc[1]),
                          static_cast<int>(enc[2]), static_cast<int>(enc[3]),
                          static_cast<int>(enc[0] - prev[0]),
                          static_cast<int>(enc[1] - prev[1]),
                          static_cast<int>(enc[2] - prev[2]),
                          static_cast<int>(enc[3] - prev[3]));
            for (int i = 0; i < 4; i++) prev[i] = enc[i];
        } else {
            printStatus();
        }
    }

    // CSV 記錄
    if (g_logOn && ticked &&
        nowMs - lastLogMs >= 1000U / (g_logHz ? g_logHz : 20)) {
        lastLogMs = nowMs;
        printLogLine(nowMs);
    }
}
