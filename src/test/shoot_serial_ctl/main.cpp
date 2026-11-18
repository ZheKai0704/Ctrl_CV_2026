// ============================================================================
// shoot_serial_ctl — Shoot 節點手動控制測試（序列監控版）
//
// 1. 仰角步進馬達：STEP / DIR / EN 脈衝控制（PIO0 SM0，同 stepdir_gp14_test）
//      STEP=GP14（PIO 脈衝）  DIR=GP15  EN=GP13（低電位有效）
//      控制方式 = 絕對位置（脈衝數）：行程 0 ~ 400000
//      （0 位置 = 下極限；'p 400000' = 上極限）
//      內部記錄目前位置，'p N' 自動算出差值與方向，到位後更新位置。
//
// 2. 上下無刷馬達（ESC）：50Hz servo 脈衝由 PIO1 產生
//      上輪=GP16、下輪=GP17；1000µs 停轉 / 2000µs 全速（複製自 src/shoot）
//
// 按鍵（USB Serial, 115200）：
//   'p' + 數值  仰角絕對位置（0~400000 脈衝）
//   'u' + 數值  上輪無刷轉速（0~100%）
//   'w' + 數值  下輪無刷轉速（0~100%）
//   '0'         仰角立即停止
//   's'         全停（BLDC 歸 0 + 仰角停止）
//   'e' 使能    'd' 去使能   'i' 狀態   '?' 說明
//   'v' 轉速(RPM)  'k' 加速度(steps/s²)  'c' 每圈脈衝數
//   'n' EN 極性    'r' DIR 反向
//
// ⚠ 轉速下限（PIO 硬體限制）：divider 最大 65536 ⇒ 最低 ≈ clk_sys/(7×65536)
//   ≈ 327 steps/s（150MHz）；3200 步/圈時約 6 RPM，做不出更慢的連續脈衝。
// ⚠ ESC arming：上電後 3 秒內不接受非 0 轉速。
//
// 接線：
//   RP2350 GP14 → 步進驅動器 PUL/STEP（PIO0 SM0）
//   RP2350 GP15 → 步進驅動器 DIR
//   RP2350 GP13 → 步進驅動器 ENA（低電位有效；'n' 可切極性）
//   驅動器 COM → GND（共陰）或 +5V（共陽，'n' 切）
//   RP2350 GP16 → 上輪 ESC 訊號線
//   RP2350 GP17 → 下輪 ESC 訊號線
//   共地
// ============================================================================
#include "servo_pulse.pio.h"
#include "step_pulse.pio.h"
#include <Arduino.h>
#include <MathUtils.h>
#include <hardware/clocks.h>
#include <hardware/pio.h>
#include <pico/stdlib.h>
#include <cmath>

// ============================================================================
// 腳位
// ============================================================================
namespace Pin {
constexpr uint STEP = 14;  // → 步進驅動器 PUL（PIO0 SM0）
constexpr uint DIR = 15;   // → 步進驅動器 DIR
constexpr uint EN = 13;    // → 步進驅動器 ENA
constexpr uint UP_BLDC_PWM = 16;
constexpr uint DOWN_BLDC_PWM = 17;
constexpr bool kEnableBldc = true;
} // namespace Pin

// ============================================================================
// 仰角步進：PIO 脈衝 + 絕對位置控制（複製自 stepdir_gp14_test，加位置記錄）
// ============================================================================
namespace PioStepper {
// 行程限制（脈衝）
constexpr int32_t POS_MIN = 0;
constexpr int32_t POS_MAX = 400000;

// PIO 組態
static PIO gPio = pio0;
static uint gSm = 0;
static uint gOffset = 0;
constexpr uint gIrqFlag = 0;
constexpr float kCyclesPerStep = 7.0f;
constexpr float kDivMax = 65536.0f;
static float gClkSysHz = 150000000.0f;

// 可調參數
static bool gEnActiveLow = true;            // EN 低電位致能（TB6600/DM542 多數）
static bool gDirInvert = false;             // DIR 方向反向
static float gStepsPerRev = 200.0f * 16.0f; // 1.8° × 16 細分 = 3200（對上驅動器 DIP）
static float gSpeedRpm = 15.0f;             // 目標轉速
static float gAccelSps2 = 20000.0f;         // 加速度 steps/s²（0 = 等速）
static float gMinSps = 150000000.0f / (kCyclesPerStep * kDivMax);
constexpr float kMaxSps = 150000.0f;

// 運動狀態（靜態配置，專案禁止 new/malloc）
struct StepperState {
    bool running = false;
    bool enabled = false;
    int32_t remaining = 0;  // 剩餘步數
    int8_t dir = 1;         // +1 位置增加, -1 位置減少
    float speedSps = 0.0f;
    float targetSps = 0.0f;
    float estSteps = 0.0f;  // 本次命令已發步數（積分）
    uint32_t lastRampUs = 0;
};
static StepperState gSt;

// 位置記錄（0~400000）：命令完成時更新；運動中用 estSteps 估算
static int32_t gPos = POS_MIN; // 上電假設在 0 位置（馬達上電即視為行程原點）

// ----------------------------------------------------------------------------
// PIO 底層（同 stepdir_gp14_test）
// ----------------------------------------------------------------------------
static void pioSetup() {
    gpio_init(Pin::STEP);
    gpio_set_dir(Pin::STEP, GPIO_OUT);
    gpio_put(Pin::STEP, 0);

    gOffset = pio_add_program(gPio, &step_pulse_program);
    pio_sm_config c = step_pulse_program_get_default_config(gOffset);
    sm_config_set_set_pins(&c, Pin::STEP, 1);
    pio_gpio_init(gPio, Pin::STEP);
    pio_sm_init(gPio, gSm, gOffset, &c);
    pio_sm_set_consecutive_pindirs(gPio, gSm, Pin::STEP, 1, true);
    pio_sm_set_pins(gPio, gSm, 0);
    pio_sm_set_enabled(gPio, gSm, true);
}

static float divForSps(float sps) {
    float div = (sps > 0.0f) ? gClkSysHz / (kCyclesPerStep * sps) : kDivMax;
    if(div < 1.0f) {
        div = 1.0f;
    }
    if(div > kDivMax) {
        div = kDivMax;
    }
    return div;
}

static float actualSps(float spsRequest) {
    return gClkSysHz / (kCyclesPerStep * divForSps(spsRequest));
}

static void applySpeed(float sps) {
    pio_sm_set_clkdiv(gPio, gSm, divForSps(sps));
}

static void haltPulses() {
    pio_sm_set_enabled(gPio, gSm, false);
    pio_sm_clear_fifos(gPio, gSm);
    pio_sm_set_pins(gPio, gSm, 0);
    pio_interrupt_clear(gPio, gIrqFlag);
}

static void startPulses(uint32_t count) {
    pio_sm_set_enabled(gPio, gSm, false);
    pio_sm_clear_fifos(gPio, gSm);
    pio_sm_restart(gPio, gSm);
    pio_sm_exec(gPio, gSm, pio_encode_jmp(gOffset));
    pio_interrupt_clear(gPio, gIrqFlag);
    applySpeed(gSt.speedSps);
    pio_sm_put(gPio, gSm, count);
    pio_sm_set_enabled(gPio, gSm, true);
}

// ----------------------------------------------------------------------------
// GPIO（DIR / EN）
// ----------------------------------------------------------------------------
static void applyEnablePin() {
    const bool level = (gSt.enabled != gEnActiveLow);
    gpio_put(Pin::EN, level ? 1 : 0);
}

static void applyDir(int8_t dir) {
    bool level = (dir > 0);
    if(gDirInvert) {
        level = !level;
    }
    gpio_put(Pin::DIR, level ? 1 : 0);
    delayMicroseconds(10);
}

void setEnabled(bool en) {
    gSt.enabled = en;
    applyEnablePin();
    Serial.printf(">> %s（EN 準位=%d, 低電位有效=%d）\n", en ? "使能" : "去使能",
                  gpio_get(Pin::EN), gEnActiveLow ? 1 : 0);
}

// ----------------------------------------------------------------------------
// 換算
// ----------------------------------------------------------------------------
static float rpmToSps(float rpm) {
    return rpm * gStepsPerRev / 60.0f;
}
static float spsToRpm(float sps) {
    return sps * 60.0f / gStepsPerRev;
}
static float clampSps(float sps) {
    if(sps < gMinSps) {
        return gMinSps;
    }
    if(sps > kMaxSps) {
        return kMaxSps;
    }
    return sps;
}

// ----------------------------------------------------------------------------
// 運動控制（相對步數）
// ----------------------------------------------------------------------------
static void stopMotion(const char *why) {
    bool wasRunning = gSt.running;
    if(wasRunning) {
        // 停止時把估算步數記入位置（實際停在估算處，誤差 = 未發完的最後幾步）
        gPos += gSt.dir * static_cast<int32_t>(gSt.estSteps + 0.5f);
    }
    if(gPos < POS_MIN) {
        gPos = POS_MIN;
    }
    if(gPos > POS_MAX) {
        gPos = POS_MAX;
    }
    haltPulses();
    gSt.running = false;
    gSt.speedSps = 0.0f;
    if(wasRunning) {
        Serial.printf(">> 停止（%s），已發約 %ld 步，位置約 %ld\n", why,
                      static_cast<long>(gSt.estSteps), static_cast<long>(gPos));
    }
}

// 發出相對步數（dir=+1/-1）。到位/停止後 gPos 自動更新。
static bool startMove(int32_t steps, int8_t dir) {
    if(!gSt.enabled) {
        Serial.println(">> ⚠ 尚未使能，先按 'e'");
        return false;
    }
    if(steps <= 0) {
        return false;
    }
    haltPulses();

    gSt.dir = (dir >= 0) ? 1 : -1;
    applyDir(gSt.dir);

    float wantSps = rpmToSps(gSpeedRpm);
    if(wantSps < gMinSps * 0.999f) {
        Serial.printf(">> ⚠ 要求 %.1f steps/s 低於硬體下限 %.1f steps/s，以 %.1f steps/s 執行\n",
                      static_cast<double>(wantSps), static_cast<double>(gMinSps),
                      static_cast<double>(gMinSps));
    }
    gSt.targetSps = clampSps(wantSps);
    // 短行程：三角曲線 v_peak = √(a·S)
    if(gAccelSps2 > 0.0f && steps > 0) {
        float peak = sqrtf(gAccelSps2 * static_cast<float>(steps));
        if(peak < gSt.targetSps) {
            gSt.targetSps = (peak < gMinSps) ? gMinSps : peak;
        }
    }
    gSt.speedSps = (gAccelSps2 > 0.0f) ? gMinSps : gSt.targetSps;
    gSt.remaining = steps;
    gSt.estSteps = 0.0f;
    gSt.lastRampUs = micros();
    gSt.running = true;

    startPulses(static_cast<uint32_t>(steps));
    Serial.printf(">> %s %ld 步 → 位置 %ld（目標 %.0f steps/s）\n",
                  gSt.dir > 0 ? "正轉" : "反轉", static_cast<long>(steps),
                  static_cast<long>(gPos + gSt.dir * steps),
                  static_cast<double>(gSt.targetSps));
    return true;
}

/// 相對移動（診斷用，同 stepdir_gp14_test 的 'f'/'b'）：不更新 gPos 基準！
/// 用途：確認 PIO 脈衝/接線有沒有作用；用完後請以實際位置重新 'p' 校準，
///      或按 'z' 把目前位置設為 0。
void moveRelative(int32_t steps, bool positive) {
    if(gSt.running) {
        stopMotion("改目標");
    }
    startMove(steps < 0 ? -steps : steps, positive ? 1 : -1);
}

/// 把目前位置重新定義為 0（重新校準原點）
void zeroPosition() {
    if(gSt.running) {
        stopMotion("歸零");
    }
    gPos = POS_MIN;
    Serial.println(">> 位置已歸零（目前位置 = 0）");
}

/// 阻塞等到到位（自我測試用）
void service();
static void waitDone() {
    uint32_t t0 = millis();
    while(gSt.running) {
        service();
        if(millis() - t0 > 120000) {
            stopMotion("逾時保護");
            break;
        }
    }
}

/// 一鍵自我測試（同 stepdir_gp14_test 的 't'，不含低速 1 圈段）
void selfTest() {
    Serial.println("\n========== 自我測試開始 ==========");
    Serial.printf("設定：%.0f 步/圈, %.1f RPM, 加速度 %.0f steps/s²\n",
                  static_cast<double>(gStepsPerRev), static_cast<double>(gSpeedRpm),
                  static_cast<double>(gAccelSps2));
    if(!gSt.enabled) {
        setEnabled(true);
        delay(200);
    }
    int32_t rev = static_cast<int32_t>(gStepsPerRev);
    Serial.println("[1] 正轉 1 圈");
    startMove(rev, +1);
    waitDone();
    delay(500);
    Serial.println("[2] 反轉 1 圈（應回到原位）");
    startMove(rev, -1);
    waitDone();
    Serial.println("========== 結束（有轉=硬體OK，問題在位置邏輯；沒轉=接線/EN極性）==========");
}

/// 絕對位置命令：pos 0~400000（超出限幅），自動算差值與方向
void setTargetPos(int32_t pos) {
    if(pos < POS_MIN) {
        Serial.printf("  ⚠ 限幅到 %ld\n", static_cast<long>(POS_MIN));
        pos = POS_MIN;
    }
    if(pos > POS_MAX) {
        Serial.printf("  ⚠ 限幅到 %ld\n", static_cast<long>(POS_MAX));
        pos = POS_MAX;
    }
    if(gSt.running) {
        stopMotion("改目標");
    }
    int32_t delta = pos - gPos;
    if(delta == 0) {
        Serial.printf(">> 已在位置 %ld\n", static_cast<long>(gPos));
        return;
    }
    // startMove 成功後：gPos 維持原值，service() 到位/停止時再累加步數
    startMove((delta < 0) ? -delta : delta, (delta > 0) ? 1 : -1);
}

// 每 loop() 呼叫：調速 + 到位偵測（脈衝由 PIO 產生）
void service() {
    if(!gSt.running) {
        return;
    }

    if(pio_interrupt_get(gPio, gIrqFlag)) {
        pio_interrupt_clear(gPio, gIrqFlag);
        gSt.running = false;
        gPos += gSt.dir * gSt.remaining; // 全部發完
        if(gPos < POS_MIN) {
            gPos = POS_MIN;
        }
        if(gPos > POS_MAX) {
            gPos = POS_MAX;
        }
        Serial.printf(">> 到位：位置 %ld（結束轉速 %.1f RPM）\n", static_cast<long>(gPos),
                      static_cast<double>(spsToRpm(gSt.speedSps)));
        return;
    }

    uint32_t now = micros();
    uint32_t dtUs = now - gSt.lastRampUs;
    if(dtUs < 200) {
        return;
    }
    gSt.lastRampUs = now;
    gSt.estSteps += gSt.speedSps * static_cast<float>(dtUs) * 1e-6f;

    // --- 梯形加減速 ---
    float targetSps = gSt.targetSps;
    if(gAccelSps2 > 0.0f && gSt.speedSps > gMinSps) {
        float left = static_cast<float>(gSt.remaining) - gSt.estSteps;
        float decelDist = (gSt.speedSps * gSt.speedSps) / (2.0f * gAccelSps2);
        if(left <= decelDist + 1.0f) {
            targetSps = gMinSps;
        }
    }
    if(gAccelSps2 > 0.0f) {
        float dv = gAccelSps2 * static_cast<float>(dtUs) * 1e-6f;
        if(gSt.speedSps < targetSps) {
            gSt.speedSps += dv;
            if(gSt.speedSps > targetSps) {
                gSt.speedSps = targetSps;
            }
        } else if(gSt.speedSps > targetSps) {
            gSt.speedSps -= dv;
            if(gSt.speedSps < targetSps) {
                gSt.speedSps = targetSps;
            }
        }
    } else {
        gSt.speedSps = targetSps;
    }
    applySpeed(clampSps(gSt.speedSps));
}

// ----------------------------------------------------------------------------
// 狀態
// ----------------------------------------------------------------------------
int32_t position() { return gPos; }
bool isRunning() { return gSt.running; }
bool isEnabled() { return gSt.enabled; }
float speedRpm() { return spsToRpm(gSt.speedSps); }
float targetRpm() { return spsToRpm(gSt.targetSps); }
bool dirPositive() { return gSt.dir > 0; }
bool &enPolarity() { return gEnActiveLow; }
bool &dirInvert() { return gDirInvert; }
float &stepsPerRev() { return gStepsPerRev; }
float &speedRpmRef() { return gSpeedRpm; }
float &accelRef() { return gAccelSps2; }
float minSps() { return gMinSps; }
void applyDirNow() { applyDir(gSt.dir); }
void applyEnableNow() { applyEnablePin(); }
float spsToRpmPublic(float sps) { return spsToRpm(sps); }
void setSpeedRpm(float rpm) {
    float sps = clampSps(rpmToSps(rpm));
    gSpeedRpm = spsToRpm(sps);
    float rampS = (gAccelSps2 > 0.0f) ? (sps - gMinSps) / gAccelSps2 : 0.0f;
    Serial.printf(">> 轉速 = %.1f RPM（%.0f steps/s，加速約 %.2f 秒）\n",
                  static_cast<double>(gSpeedRpm), static_cast<double>(sps),
                  static_cast<double>(rampS));
}

void init() {
    gClkSysHz = static_cast<float>(clock_get_hz(clk_sys));
    gMinSps = gClkSysHz / (kCyclesPerStep * kDivMax);

    gpio_init(Pin::DIR);
    gpio_set_dir(Pin::DIR, GPIO_OUT);
    gpio_put(Pin::DIR, 0);

    gpio_init(Pin::EN);
    gpio_set_dir(Pin::EN, GPIO_OUT);
    gSt.enabled = false;
    applyEnablePin(); // 上電先保持去使能

    pioSetup();
}
} // namespace PioStepper

// ============================================================================
// BLDC 馬達控制 (ESC) — 50Hz servo 脈衝由 PIO1 產生 — 複製自 src/shoot/main.cpp
// ============================================================================
namespace BldcControl {
constexpr uint32_t FRAME_US = 20000;
constexpr uint16_t PULSE_MID = 1000;
constexpr uint16_t PULSE_MAX = 2000;

constexpr int32_t HIGH_OVERHEAD_US = 3;
constexpr int32_t FRAME_OVERHEAD_US = 6;

enum Channel : uint8_t { CH_UP = 0, CH_DOWN = 1, CH_COUNT = 2 };

static PIO sPio = pio1;
static uint sSm[CH_COUNT] = {0, 0};
static uint sOffset = 0;
static bool sReady = false;

static volatile uint32_t sPulseUs[CH_COUNT] = {PULSE_MID, PULSE_MID};

static constexpr uint32_t ARM_DELAY_MS = 3000;
static uint32_t sBootMs = 0;

static void pushFrame(uint8_t ch) {
    int32_t pulseUs = static_cast<int32_t>(sPulseUs[ch]);
    int32_t high = pulseUs - HIGH_OVERHEAD_US;
    if(high < 1) {
        high = 1;
    }
    int32_t low = static_cast<int32_t>(FRAME_US) - high - FRAME_OVERHEAD_US;
    if(low < 1) {
        low = 1;
    }
    pio_sm_put(sPio, sSm[ch], static_cast<uint32_t>(high));
    pio_sm_put(sPio, sSm[ch], static_cast<uint32_t>(low));
}

void update() {
    if(!sReady) {
        return;
    }
    for(uint8_t i = 0; i < CH_COUNT; i++) {
        if(pio_sm_get_tx_fifo_level(sPio, sSm[i]) <= 2) {
            pushFrame(i);
        }
    }
}

void init() {
    sBootMs = millis();
    if(!Pin::kEnableBldc) {
        return;
    }
    const uint pins[CH_COUNT] = {Pin::UP_BLDC_PWM, Pin::DOWN_BLDC_PWM};

    sOffset = pio_add_program(sPio, &servo_pulse_program);
    pio_sm_config base = servo_pulse_program_get_default_config(sOffset);
    sm_config_set_clkdiv(&base, static_cast<float>(clock_get_hz(clk_sys)) / 1000000.0f);

    for(uint8_t i = 0; i < CH_COUNT; i++) {
        int sm = pio_claim_unused_sm(sPio, false);
        if(sm < 0) {
            sReady = false;
            return;
        }
        sSm[i] = static_cast<uint>(sm);

        pio_gpio_init(sPio, pins[i]);
        pio_sm_set_consecutive_pindirs(sPio, sSm[i], pins[i], 1, true);

        pio_sm_config cfg = base;
        sm_config_set_set_pins(&cfg, pins[i], 1);
        pio_sm_init(sPio, sSm[i], sOffset, &cfg);
        pio_sm_set_enabled(sPio, sSm[i], true);
    }
    sReady = true;

    // 先把 FIFO 塞滿，避免上電時 ESC 收到半截訊號
    for(uint8_t i = 0; i < CH_COUNT; i++) {
        pushFrame(i);
        pushFrame(i);
    }
}

static uint16_t speedToPulse(float speed) {
    speed = MathUtils::sanitizePercent(speed);
    if(speed == 0.0f)
        return PULSE_MID;
    float absSpd = fabsf(speed);
    if(absSpd > 100.0f)
        absSpd = 100.0f;
    uint16_t pulse = static_cast<uint16_t>(
        PULSE_MID + (absSpd / 100.0f) * (PULSE_MAX - PULSE_MID));
    return (pulse > PULSE_MAX) ? PULSE_MAX : pulse;
}

void setUpSpeed(float speed) {
    if(!Pin::kEnableBldc)
        return;
    if(millis() - sBootMs < ARM_DELAY_MS && speed != 0.0f)
        return;
    sPulseUs[CH_UP] = speedToPulse(speed);
}
void setDownSpeed(float speed) {
    if(!Pin::kEnableBldc)
        return;
    if(millis() - sBootMs < ARM_DELAY_MS && speed != 0.0f)
        return;
    sPulseUs[CH_DOWN] = speedToPulse(speed);
}
void estop() {
    if(!Pin::kEnableBldc)
        return;
    sPulseUs[CH_UP] = PULSE_MID;
    sPulseUs[CH_DOWN] = PULSE_MID;
}
} // namespace BldcControl

// ============================================================================
// 序列輸入（參考 stepdir_gp14_test，可負數、非阻塞等待）
// ============================================================================
static bool readIntInput(int32_t &val, const char *prompt, uint32_t waitMs = 15000) {
    Serial.print(prompt);
    long v = 0;
    bool neg = false;
    bool got = false;
    uint32_t t0 = millis();
    while(millis() - t0 < waitMs) {
        PioStepper::service();  // 等待輸入時維持 PIO 加減速 / 到位偵測
        BldcControl::update();  // 維持 ESC 脈衝
        if(millis() - t0 > 2000 && !got && !Serial.available()) {
            break;
        }
        if(Serial.available()) {
            char ch = static_cast<char>(Serial.read());
            if(ch == '\n' || ch == '\r') {
                if(got) {
                    val = static_cast<int32_t>(neg ? -v : v);
                    Serial.println();
                    return true;
                }
            } else if(ch == '-' && !got) {
                neg = true;
            } else if(ch >= '0' && ch <= '9') {
                got = true;
                v = v * 10 + (ch - '0');
                Serial.write(ch);
            }
        }
    }
    Serial.println("  (逾時/取消)");
    return false;
}

// ============================================================================
// 狀態 / 說明（不用 %f：newlib-nano 沒開 _printf_float，角度用 DebugFmt 手法）
// ============================================================================
static void printFixed1(float v) {
    if(v < 0) {
        Serial.print('-');
        v = -v;
    }
    Serial.print(static_cast<long>(v));
    Serial.print('.');
    Serial.print(static_cast<int>(v * 10.0f) % 10);
}

static void printStatus() {
    Serial.print(">> 仰角: 位置 ");
    Serial.print(PioStepper::position());
    Serial.print("/400000");
    Serial.printf(" 使能=%d 運轉=%d 方向=%s ", PioStepper::isEnabled() ? 1 : 0,
                  PioStepper::isRunning() ? 1 : 0, PioStepper::dirPositive() ? "+" : "-");
    Serial.print("rpm ");
    printFixed1(PioStepper::speedRpm());
    Serial.print("/");
    printFixed1(PioStepper::targetRpm());
    Serial.printf(" | 上/下輪 PWM: %uµs / %uµs\n",
                  static_cast<unsigned>(BldcControl::sPulseUs[0]),
                  static_cast<unsigned>(BldcControl::sPulseUs[1]));
}

static void printHelp() {
    Serial.println();
    Serial.println("=== Shoot 手動控制測試（仰角 STEP/DIR 脈衝 + 上下無刷）===");
    Serial.printf("  腳位：STEP=GP%u(PIO0 SM0)  DIR=GP%u  EN=GP%u | 無刷 上=GP%u 下=GP%u\n",
                  Pin::STEP, Pin::DIR, Pin::EN, Pin::UP_BLDC_PWM, Pin::DOWN_BLDC_PWM);
    Serial.printf("  目前：%.0f 步/圈, %.1f RPM, 加速度 %.0f steps/s²\n",
                  static_cast<double>(PioStepper::stepsPerRev()),
                  static_cast<double>(PioStepper::speedRpmRef()),
                  static_cast<double>(PioStepper::accelRef()));
    Serial.println("  'p' + 數值  仰角絕對位置 (0~400000, 0=下極限)");
    Serial.println("  'u' + 數值  上輪無刷轉速 0~100%");
    Serial.println("  'w' + 數值  下輪無刷轉速 0~100%");
    Serial.println("  '0' 仰角停止    's' 全停    'e'/'d' 使能/去使能    'i' 狀態");
    Serial.println("  'v' 轉速(RPM)   'k' 加速度(steps/s²)   'c' 每圈脈衝數");
    Serial.println("  'n' EN 極性     'r' DIR 反向    '?' 說明");
    Serial.println("  --- 診斷 ---");
    Serial.println("  'f' + 數值  相對正轉 N 步（不動位置基準）");
    Serial.println("  'b' + 數值  相對反轉 N 步（不動位置基準）");
    Serial.println("  'z' 目前位置設為 0（重新校準原點）");
    Serial.println("  't' 自我測試（正反轉 1 圈）");
    Serial.println("  ⚠ 上電 3 秒內 ESC 不接受非 0 轉速（arming）");
    Serial.println("  ⚠ 位置以「上電位置 = 0」為基準；馬達實際位置請自行對齊");
    Serial.println();
}

// ============================================================================
// setup / loop（單核心：所有控制都在 Core 0）
// ============================================================================
void setup() {
    // 第一個動作：先把 ESC 鎖在停轉、步進保持去使能，再開 Serial
    BldcControl::init();
    PioStepper::init();

    Serial.begin(115200);
    delay(300);
    Serial.println("\n[shoot_serial_ctl] boot: 仰角=STEP/DIR脈衝(GP14/15/13) 無刷=上GP16/下GP17");
    Serial.println("[SHOOT] 仰角去使能，按 'e' 使能（安全起見不自動使能）");

    printHelp();
}

void loop() {
    PioStepper::service(); // PIO 加減速 + 到位偵測
    BldcControl::update(); // 持續餵 50Hz 伺服脈衝

    if(!Serial.available()) {
        return;
    }
    char c = static_cast<char>(Serial.read());
    static int32_t sArg = 0;

    switch(c) {
    case '\r':
    case '\n':
    case ' ':
        break;
    case 'p': {
        if(!readIntInput(sArg, "仰角絕對位置 (0~400000)：")) {
            break;
        }
        PioStepper::setTargetPos(sArg);
        break;
    }
    case 'u': {
        if(!readIntInput(sArg, "上輪轉速 % (0~100)：")) {
            break;
        }
        if(sArg < 0)
            sArg = 0;
        if(sArg > 100)
            sArg = 100;
        BldcControl::setUpSpeed(static_cast<float>(sArg));
        Serial.printf(">> 上輪轉速 = %ld%%\n", static_cast<long>(sArg));
        break;
    }
    case 'w': {
        if(!readIntInput(sArg, "下輪轉速 % (0~100)：")) {
            break;
        }
        if(sArg < 0)
            sArg = 0;
        if(sArg > 100)
            sArg = 100;
        BldcControl::setDownSpeed(static_cast<float>(sArg));
        Serial.printf(">> 下輪轉速 = %ld%%\n", static_cast<long>(sArg));
        break;
    }
    case '0':
        PioStepper::stopMotion("使用者停止");
        break;
    case 'f':
    case 'b': {
        if(!readIntInput(sArg, "步數：")) {
            break;
        }
        if(sArg <= 0) {
            break;
        }
        PioStepper::moveRelative(sArg, c == 'f');
        break;
    }
    case 'z':
        PioStepper::zeroPosition();
        break;
    case 't':
        PioStepper::selfTest();
        break;
    case 's':
        BldcControl::estop();
        PioStepper::stopMotion("全停");
        Serial.println(">> 全停（BLDC 歸 0、仰角停止）");
        break;
    case 'e':
        PioStepper::setEnabled(true);
        break;
    case 'd':
        PioStepper::stopMotion("去使能");
        PioStepper::setEnabled(false);
        break;
    case 'i':
        printStatus();
        break;
    case 'v': {
        Serial.printf("  目前每圈 %.0f 步，可設定 %.1f ~ %.1f RPM\n",
                      static_cast<double>(PioStepper::stepsPerRev()),
                      static_cast<double>(PioStepper::spsToRpmPublic(PioStepper::minSps())),
                      static_cast<double>(PioStepper::spsToRpmPublic(150000.0f)));
        if(!readIntInput(sArg, "轉速 RPM：")) {
            break;
        }
        if(sArg < 1) {
            sArg = 1;
        }
        PioStepper::setSpeedRpm(static_cast<float>(sArg));
        break;
    }
    case 'k': {
        if(!readIntInput(sArg, "加速度 steps/s² (0=等速, 建議 20000-500000)：")) {
            break;
        }
        if(sArg < 0) {
            sArg = 0;
        }
        PioStepper::accelRef() = static_cast<float>(sArg);
        Serial.printf(">> 加速度 = %ld steps/s²\n", static_cast<long>(sArg));
        break;
    }
    case 'c': {
        if(!readIntInput(sArg, "每圈脈衝數 (200×細分，例如 16細分=3200)：")) {
            break;
        }
        if(sArg < 200) {
            Serial.println("  ⚠ 至少 200");
            break;
        }
        PioStepper::stepsPerRev() = static_cast<float>(sArg);
        Serial.printf(">> 每圈脈衝數 = %ld\n", static_cast<long>(sArg));
        break;
    }
    case 'n': {
        PioStepper::enPolarity() = !PioStepper::enPolarity();
        PioStepper::applyEnableNow();
        Serial.printf(">> EN 極性 = %s\n", PioStepper::enPolarity() ? "低電位致能" : "高電位致能");
        break;
    }
    case 'r': {
        PioStepper::dirInvert() = !PioStepper::dirInvert();
        PioStepper::applyDirNow();
        Serial.printf(">> DIR 反向 = %d\n", PioStepper::dirInvert() ? 1 : 0);
        break;
    }
    case '?':
    default:
        printHelp();
        break;
    }
}
