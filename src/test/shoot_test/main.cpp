// ============================================================================
// shoot_test — Shoot Slave 上下射無刷馬達 RC 控制測試 (RP2350)
//
// 用途：
//   驗證「RC CH2 → Hub → PIO UART GP4/GP5 → Shoot UART0 → 上下兩顆 BLDC」
//   這條備援控制鏈路。本程式只驅動 UP/DOWN 兩顆無刷馬達，
//   完全不碰俯仰/橫向步進馬達。
//
// 控制映射（收到的是 Hub 轉發的 CH2 原始值 -100~100）：
//     -97 以下 →   0%  (停止)
//        0      →  50%
//     +97 以上 → 100%  (全速)
//   上下兩顆馬達「同時」套用同一個出力百分比。
//
// 通訊（與正式 Shoot Slave 完全相同，使用 SlaveComms 框架）：
//    Hub PIO SM1 (GP4 TX, GP5 RX) ↔ Shoot UART0 (TX=GP0, RX=GP1)
//    921600 baud, COBS 封幀 + CRC16
//    Hub 端由 tryRcFallback() 的 rcToShootPacket() 發送：
//    shoot.upSpeed = shoot.downSpeed = CH2
//
// 安全：
//    - SlaveComms 逾時保護：>100ms 無有效控制幀 → E-Stop → 兩顆回 0%
//    - 上電先輸出「0%」脈寬並等待電調解鎖
//    - 步進驅動器 STEPPER_ENABLE (GP9) 拉高停用
//
// ⚠️ 電調脈寬慣例（選錯馬達不會照預期動作，請先確認硬體）：
//    A) 單向航模油門電調（Skywalker，見 src/test/esc_test）：
//       PULSE_ZERO = 1000 (熄火/最低點), PULSE_FULL = 2000 (全速)
//       上電必須先看到低電位(1000µs)才會解鎖 arming
//    B) Servo-style 雙向電調（CLAUDE.md 對 Shoot 的記載）：
//       PULSE_ZERO = 1500 (中立/停止), PULSE_FULL = 2000 (全速)
//    預設採 B：若你的電調其實是 A 型，最壞情況只是「沒有解鎖 → 馬達不轉」，
//    不會突然猛轉；確認後把 PULSE_ZERO 改成 1000 即可。
//
// ⚠️ 時脈：RP2350 (Pico2) 雙核跑 150MHz，故 clkdiv = 150 → 切片時脈 1MHz
//    (每 tick = 1µs)、wrap = 19999 → 20ms = 50Hz。
//    正式 src/shoot/main.cpp 目前用 clkdiv=125（那是 RP2040/125MHz 的值），
//    在 RP2350 上會變成 1.2MHz、脈寬與週期都偏差 20%，本測試已修正。
// ============================================================================
#include "SlaveComms.h"
#include <Arduino.h>
#include <cmath>
#include <hardware/pwm.h>
#include <pico/stdlib.h>

// ============================================================================
// 腳位定義
// ============================================================================
namespace Pin {
constexpr uint UP_BLDC_PWM = 2;    // GP2 上射 BLDC
constexpr uint DOWN_BLDC_PWM = 3;  // GP3 下射 BLDC
constexpr uint STEPPER_ENABLE = 9; // 低電位致能 → 本測試拉高停用
} // namespace Pin

// ============================================================================
// 電調 (ESC) 參數
// ============================================================================
namespace Esc {
constexpr uint32_t PWM_CLKDIV = 150; // 150MHz / 150 = 1MHz → 1 tick = 1µs
constexpr uint16_t PWM_WRAP = 19999; // 20000 ticks = 20ms → 50Hz

constexpr uint16_t PULSE_ZERO = 1500; // 0%（見檔頭慣例說明）
constexpr uint16_t PULSE_FULL = 2000; // 100%

// CH2 映射端點：-97 → 0%，+97 → 100%
constexpr float CH2_MIN = -97.0f;
constexpr float CH2_MAX = 97.0f;

constexpr uint32_t ARM_DELAY_MS = 3000; // 上電後等待電調辨識解鎖
} // namespace Esc

// ============================================================================
// 跨核心除錯狀態 — Core 1 寫入，Core 0 讀取後經 USB Serial 印出
// （Serial 綁定 Core 0，禁止在 Core 1 呼叫）
// ============================================================================
struct DebugState {
    volatile int32_t ch2Raw;   // 收到的 CH2 原始值
    volatile int32_t pct;      // 映射後出力 %
    volatile int32_t pulseUs;  // 實際輸出脈寬
    volatile uint32_t rxCount; // 累計收到的有效控制幀數
    volatile uint8_t estop;    // 1 = E-Stop 中
};
static DebugState g_dbg = {};

// ============================================================================
// 硬體 PWM 設定
// ============================================================================
static void setupPwm(uint gpio, uint16_t pulseUs) {
    const uint slice = pwm_gpio_to_slice_num(gpio);
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    pwm_set_clkdiv(slice, Esc::PWM_CLKDIV);
    pwm_set_wrap(slice, Esc::PWM_WRAP);
    pwm_set_gpio_level(gpio, pulseUs);
    pwm_set_enabled(slice, true);
}

/// 兩顆無刷同時輸出相同脈寬
static void applyPulse(uint16_t us) {
    pwm_set_gpio_level(Pin::UP_BLDC_PWM, us);
    pwm_set_gpio_level(Pin::DOWN_BLDC_PWM, us);
}

// ============================================================================
// 映射：CH2 (-100~100) → 出力 % (0~100)
//   線性斜率 = 100 / (97 - (-97)) = 0.5155 %/unit，超出範圍夾在 0/100
// ============================================================================
static int pctFromCh2(float ch2) {
    constexpr float kSpan = Esc::CH2_MAX - Esc::CH2_MIN; // 194
    float pct = (ch2 - Esc::CH2_MIN) * (100.0f / kSpan);
    if(pct < 0.0f)
        pct = 0.0f;
    if(pct > 100.0f)
        pct = 100.0f;
    return static_cast<int>(lroundf(pct));
}

static uint16_t pctToPulse(int pct) {
    const int32_t span = static_cast<int32_t>(Esc::PULSE_FULL) -
                         static_cast<int32_t>(Esc::PULSE_ZERO);
    return static_cast<uint16_t>(static_cast<int32_t>(Esc::PULSE_ZERO) +
                                 (pct * span) / 100);
}

/// 套用 CH2 → 兩顆無刷，並記錄除錯狀態（Core 1 context）
static void applyCh2(float ch2) {
    const int pct = pctFromCh2(ch2);
    const uint16_t us = pctToPulse(pct);
    applyPulse(us);

    g_dbg.ch2Raw = static_cast<int32_t>(lroundf(ch2));
    g_dbg.pct = pct;
    g_dbg.pulseUs = us;
}

// ============================================================================
// SlaveApp 回呼（Core 1 context，不可阻塞）
// ============================================================================
static void onShootCommand(const ChassisSnapshot &cmd) {
    // Hub 把 CH2 原始值同時寫入 upSpeed / downSpeed
    applyCh2(cmd.shootUpSpeed);
    g_dbg.rxCount++;
    g_dbg.estop = 0;
}

static void onShootEstop() {
    applyPulse(Esc::PULSE_ZERO); // 兩顆歸零
    g_dbg.pct = 0;
    g_dbg.pulseUs = Esc::PULSE_ZERO;
    g_dbg.estop = 1;
}

static SlaveApp shootApp = {
    .targetID = TargetID::SHOOT,
    .onCommand = onShootCommand,
    .onEstop = onShootEstop,
};

static SlaveComms slave(&shootApp);

// ============================================================================
// Core 0
// ============================================================================
void setup() {
    Serial.begin(115200); // USB CDC 除錯輸出
    delay(200);

    Serial.println();
    Serial.println("========================================");
    Serial.println("[shoot_test] Shoot 上下射無刷 — RC CH2 測試");
    Serial.println("========================================");
    Serial.printf("  CH2 -97→0%% / 0→50%% / +97→100%%（夾在 0~100%%）\n");
    Serial.printf("  PWM: clkdiv=%u wrap=%u → 50Hz | 0%%=%uµs 100%%=%uµs\n",
                  static_cast<unsigned>(Esc::PWM_CLKDIV),
                  static_cast<unsigned>(Esc::PWM_WRAP),
                  static_cast<unsigned>(Esc::PULSE_ZERO),
                  static_cast<unsigned>(Esc::PULSE_FULL));
    Serial.println("  UP=GP2  DOWN=GP3  步進驅動(GP9)已停用");
    Serial.println("  通訊: UART0 GP0(TX)/GP1(RX) @921600");
    Serial.println("        ← Hub PIO SM1 GP4(TX)/GP5(RX)");
    Serial.println("  逾時 100ms 無控制幀 → 自動 E-Stop 回 0%");
    Serial.println("----------------------------------------");

    slave.begin();
}

void loop() {
    slave.update(); // Core 0：收 Hub COBS 幀 + 逾時保護 + 遙測

    // ---- 每 200ms 印一次狀態（Serial 僅限 Core 0）----
    static uint64_t lastPrintUs = 0;
    uint64_t now = time_us_64();
    if(now - lastPrintUs > 200000ULL) {
        lastPrintUs = now;

        // 先快照 volatile 欄位，避免同一行讀到不一致的組合
        const int32_t ch2 = g_dbg.ch2Raw;
        const int32_t pct = g_dbg.pct;
        const int32_t us = g_dbg.pulseUs;
        const uint32_t rx = g_dbg.rxCount;
        const uint8_t estop = g_dbg.estop;

        Serial.printf("[SHOOT_TEST] ch2=%4d → %3d%% (%4dµs) | rx=%u | "
                      "%s | silent=%ums\n",
                      static_cast<int>(ch2), static_cast<int>(pct),
                      static_cast<int>(us), static_cast<unsigned>(rx),
                      estop ? "E-STOP" : "RUN   ",
                      static_cast<unsigned>(slave.msSinceLastValidCmd()));
    }

    delayMicroseconds(20);
}

// ============================================================================
// Core 1
// ============================================================================
void setup1() {
    // 步進馬達不使用 → STEPPER_ENABLE 拉高（停用，避免驅動器通電鎖軸）
    gpio_init(Pin::STEPPER_ENABLE);
    gpio_set_dir(Pin::STEPPER_ENABLE, GPIO_OUT);
    gpio_put(Pin::STEPPER_ENABLE, 1);

    // 先輸出 0% 脈寬讓電調完成上電解鎖，再開始接受命令
    setupPwm(Pin::UP_BLDC_PWM, Esc::PULSE_ZERO);
    setupPwm(Pin::DOWN_BLDC_PWM, Esc::PULSE_ZERO);
    applyPulse(Esc::PULSE_ZERO);

    g_dbg.pct = 0;
    g_dbg.pulseUs = Esc::PULSE_ZERO;

    delay(Esc::ARM_DELAY_MS);
}

void loop1() { slave.runCore1(); }
