// ============================================================================
// move_dir_test — Move 新板「移動方向 / 極性」測試 (RP2350，獨立程式)
//
// 目的：新板腳位改為
//   FR: DIR=8  PWM=9  ENC=14,15
//   FL: DIR=6  PWM=7  ENC=16,17
//   BL: DIR=12 PWM=13 ENC=18,19
//   BR: PWM=11 DIR=10 ENC=20,21
// 之後，用序列埠即時指令驗證：
//   1. 每顆輪子單獨轉動的方向
//   2. 四輪同動（前後 / 左右平移 / 旋轉）時方向是否一致
//   3. DIR 腳是否真的對應到自己的輪子
//   4. 編碼器極性（kEncReadPolarity）自動量測 → 印出可貼回 main.cpp 的陣列
//
// 與正式韌體一致性：
//   - 輪序一律 [FL, FR, BL, BR]（與 src/move/main.cpp 相同）
//   - 麥克納姆運動學公式、軸向極性沿用 main.cpp
//   - 編碼器用同一套 PIO（quad_phase.pio.h，PIO0 4 SM，clkdiv=5000）
//   - PWM wrap=1000 / clkdiv=6.25 → 20kHz，DIR：LOW=正轉、HIGH=反轉
//
// 使用流程（建議照順序）
//   pio run -e move_dir_test -t upload
//   pio device monitor -e move_dir_test            # 115200
//
//   1) fwd 25      觀察四輪是否都往「車頭方向」轉
//      有哪一顆反了就 pol <輪> <-1|1> 翻過來，再 fwd 25 確認
//   2) cal         自動量測編碼器極性（每輪 +25% 各 0.8 秒）
//   3) left / right / cw / ccw  驗證平移與旋轉方向
//   4) report      印出建議貼回 src/move/main.cpp 的極性陣列
//
// 指令（115200，Enter=停止）：
//   fwd|back|left|right|cw|ccw [duty%]   四輪同動（預設 25%，上限 55）
//   w <fl|fr|bl|br> <duty%>              單輪（正 = 韌體正轉慣例，可負）
//   stop                                 停止（空行 Enter 亦可）
//   cal                                  自動量測 kEncReadPolarity
//   dirtest <fl|fr|bl|br>                DIR 低/高各轉一次，驗證 DIR 腳歸屬
//   pol <fl|fr|bl|br> <-1|1>             改馬達極性（kWheelPolarity）
//   encpol <fl|fr|bl|br> <-1|1>          改編碼器極性（kEncReadPolarity）
//   enc                                  待機：手轉輪子，印原始計數（驗編碼器）
//   report / map / help
//
// ### 安全 ###
//   - 四輪同動 / 單輪出力上限 55%，長轉請架空車身
//   - 任何長動作（cal / dirtest）中途按任意鍵即中止
//   - 停止時一律 PWM=0
// ============================================================================
#include "quad_phase.pio.h"
#include <Arduino.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>

// ============================================================================
// 腳位（新板實測；輪序 = [FL, FR, BL, BR]）
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
constexpr uint16_t PWM_WRAP = 1000; // clkdiv 6.25 → ~20kHz（與正式韌體相同）
constexpr uint16_t ENC_CPR = 443 * 4; // 4X 計數 / 圈
constexpr float PI_F = 3.14159265f;
// 麥克納姆車身幾何（沿用 main.cpp）
constexpr float HALF_LENGTH = 0.25f;
constexpr float HALF_WIDTH = 0.25f;
constexpr float WHEEL_RADIUS = 0.05f;
// 軸向極性（沿用 main.cpp 的 kAxisStrafePolarity / kAxisForwardPolarity）
//   新板實測：fwd/back 正常、left/right 相反 → strafe 軸改 -1。
constexpr float AXIS_STRAFE_POL = -1.0f;
constexpr float AXIS_FORWARD_POL = -1.0f;
constexpr float AXIS_ROT_POL = +1.0f; // cw/ccw（omega 軸）
// 指令斜坡（%/s），避免瞬間大電流
constexpr float RAMP_PER_SEC = 150.0f;
// 測試出力上限（%）
constexpr float MAX_TEST_DUTY = 55.0f;
constexpr uint32_t STATUS_MS = 300;
} // namespace Cfg

// ============================================================================
// 常量表
// ============================================================================
static constexpr uint kPwmPin[4] = {Pin::FL_PWM, Pin::FR_PWM, Pin::BL_PWM,
                                    Pin::BR_PWM};
static constexpr uint kDirPin[4] = {Pin::FL_DIR, Pin::FR_DIR, Pin::BL_DIR,
                                    Pin::BR_DIR};
static constexpr uint kEncPin[4] = {Pin::FL_ENC, Pin::FR_ENC, Pin::BL_ENC,
                                    Pin::BR_ENC};
static const char *const kWheelName[4] = {"FL", "FR", "BL", "BR"};

// 車體三軸極性（可用 `axis` 指令即時改；對應 main.cpp 的 kAxisStrafePolarity 等）
static float axisStrafe = Cfg::AXIS_STRAFE_POL;
static float axisForward = Cfg::AXIS_FORWARD_POL;
static float axisRot = Cfg::AXIS_ROT_POL;

// ============================================================================
// 極性狀態
//   motorPol：韌體輸出 → 硬體方向（對應 kWheelPolarity）
//   encPol  ：編碼器原始計數 → 韌體方向（對應 kEncReadPolarity）
//   預設先沿用舊板實測值 {+1,-1,+1,-1}，可用指令即時修改
// ============================================================================
static int8_t motorPol[4] = {1, 1, -1, -1};
static int8_t encPol[4] = {1, -1, +1, -1};

// ============================================================================
// 輸出狀態
// ============================================================================
enum class Mode : uint8_t { Idle, Chassis, Wheel, EncWatch };
static Mode g_mode = Mode::Idle;
static char g_modeText[32] = "idle";
// true = 忽略 motorPol，直接把 curCmd 當硬體方向送出（`same` 指令用）
static bool g_forceHw = false;
static float targetCmd[4] = {0, 0, 0, 0}; // 韌體座標命令（%）
static float curCmd[4] = {0, 0, 0, 0};    // 斜坡後命令（%）
static int g_effDir[4] = {0, 0, 0, 0};    // 實際 DIR 電位 0=LOW(正) 1=HIGH(逆)
static float g_effPct[4] = {0, 0, 0, 0};  // 實際 PWM 大小（%）

// ============================================================================
// 編碼器（PIO0 4 SM，與 main.cpp 相同架構）
// ============================================================================
static PIO g_pio = pio0;
static uint g_pioOffset = 0;
static const int8_t kQuadTable[4][4] = {
    {0, +1, -1, 0},
    {-1, 0, 0, +1},
    {+1, 0, 0, -1},
    {0, -1, +1, 0},
};
static volatile int32_t encRaw[4];
static uint8_t encPrev[4];

static void drainEncoders() {
    constexpr int MAX_DRAIN = 128;
    for (int i = 0; i < 4; i++) {
        int n = 0;
        while (!pio_sm_is_rx_fifo_empty(g_pio, i) && n < MAX_DRAIN) {
            const uint32_t cur = pio_sm_get(g_pio, i) & 0x3;
            encRaw[i] += kQuadTable[encPrev[i]][cur];
            encPrev[i] = static_cast<uint8_t>(cur);
            n++;
        }
    }
}

static void encoderInit() {
    g_pioOffset = pio_add_program(g_pio, &quad_phase_program);
    for (int i = 0; i < 4; i++) {
        const uint aPin = kEncPin[i];
        const uint bPin = aPin + 1;
        gpio_init(aPin);
        gpio_init(bPin);
        gpio_set_dir(aPin, GPIO_IN);
        gpio_set_dir(bPin, GPIO_IN);
        gpio_pull_up(aPin);
        gpio_pull_up(bPin);
        pio_gpio_init(g_pio, aPin);
        pio_gpio_init(g_pio, bPin);

        pio_sm_config c = quad_phase_program_get_default_config(g_pioOffset);
        sm_config_set_in_pins(&c, aPin);              // base pin = A
        sm_config_set_in_shift(&c, false, false, 32); // left shift
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);
        sm_config_set_clkdiv(&c, 5000.0f); // ~25kHz 取樣，濾回彈

        pio_sm_init(g_pio, i, g_pioOffset, &c);
        pio_sm_clear_fifos(g_pio, i);
        pio_sm_set_enabled(g_pio, i, true);

        encRaw[i] = 0;
        const uint8_t a = gpio_get(aPin) ? 1u : 0u;
        const uint8_t b = gpio_get(bPin) ? 1u : 0u;
        encPrev[i] = static_cast<uint8_t>(a | (b << 1));
    }
}

// ============================================================================
// 硬體輸出
// ============================================================================
static void initWheelHw(int i) {
    gpio_init(kDirPin[i]);
    gpio_set_dir(kDirPin[i], GPIO_OUT);
    gpio_put(kDirPin[i], 0);

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

// effPercent：硬體座標（正 = DIR LOW）。forceDir >= 0 時強制 DIR 電位。
static void hwWrite(int i, float effPercent, int forceDir = -1) {
    const float mag = fminf(fabsf(effPercent), 100.0f);
    int dirLevel = (effPercent >= 0.0f) ? 0 : 1;
    if (forceDir >= 0) dirLevel = forceDir ? 1 : 0;
    gpio_put(kDirPin[i], dirLevel ? HIGH : LOW);
    pwm_set_gpio_level(kPwmPin[i],
                       static_cast<uint16_t>(mag / 100.0f * (Cfg::PWM_WRAP + 1)));
    g_effDir[i] = dirLevel;
    g_effPct[i] = dirLevel ? -mag : mag;
}

static void stopAll() {
    g_forceHw = false;
    for (int i = 0; i < 4; i++) {
        targetCmd[i] = 0.0f;
        curCmd[i] = 0.0f;
        hwWrite(i, 0.0f);
    }
    g_mode = Mode::Idle;
    strncpy(g_modeText, "stop", sizeof(g_modeText) - 1);
}

// ============================================================================
// 麥克納姆運動學（沿用 main.cpp，輸出歸一化成 duty%）
// ============================================================================
static void chassisTargets(float vx, float vy, float omega, float duty,
                           float out[4]) {
    const float wx = axisStrafe * vx * 2.0f * Cfg::PI_F;
    const float wy = axisForward * vy * 2.0f * Cfg::PI_F;
    const float wz = axisRot * omega;
    const float K = Cfg::HALF_LENGTH + Cfg::HALF_WIDTH;
    const float R = Cfg::WHEEL_RADIUS;
    float lin[4];
    lin[0] = (wx - wy - K * wz) * R; // FL
    lin[1] = (wx + wy + K * wz) * R; // FR
    lin[2] = (wx + wy - K * wz) * R; // BL
    lin[3] = (wx - wy + K * wz) * R; // BR

    float maxAbs = 0.0f;
    for (int i = 0; i < 4; i++) {
        out[i] = lin[i] / (2.0f * Cfg::PI_F * R); // RPS
        if (fabsf(out[i]) > maxAbs) maxAbs = fabsf(out[i]);
    }
    for (int i = 0; i < 4; i++)
        out[i] = (maxAbs > 1e-6f) ? (out[i] / maxAbs) * duty : 0.0f;
}

// ============================================================================
// 序列埠輸入（行導向，支援 CRLF / 退格，空行 = stop）
// ============================================================================
static char g_line[96];
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
        if (c == 8 || c == 127) { // backspace
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
// 輪名 / 數值工具
// ============================================================================
static int wheelIndex(const char *name) {
    for (int i = 0; i < 4; i++)
        if (strcasecmp(name, kWheelName[i]) == 0) return i;
    return -1;
}

static float clampDuty(float d) {
    if (d > Cfg::MAX_TEST_DUTY) d = Cfg::MAX_TEST_DUTY;
    if (d < -Cfg::MAX_TEST_DUTY) d = -Cfg::MAX_TEST_DUTY;
    return d;
}

static const char *dirChar(int dirLevel) { return dirLevel ? "H" : "L"; }

// ============================================================================
// 阻塞式量測（cal / dirtest 用）：只驅動一輪，回傳編碼器原始增量
// ============================================================================
static int32_t measureSpin(int wheel, float cmdPercent, uint32_t ms,
                           int forceDir, bool &aborted) {
    for (int i = 0; i < 4; i++) {
        targetCmd[i] = 0.0f;
        curCmd[i] = 0.0f;
        hwWrite(i, 0.0f);
    }
    delay(120);
    drainEncoders();
    const int32_t before = encRaw[wheel];
    const float eff = cmdPercent * static_cast<float>(motorPol[wheel]);
    const uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        if (anyKeyPressed()) {
            aborted = true;
            break;
        }
        drainEncoders();
        hwWrite(wheel, eff, forceDir);
        delayMicroseconds(400);
    }
    hwWrite(wheel, 0.0f);
    drainEncoders();
    return encRaw[wheel] - before;
}

// ============================================================================
// cal — 自動量測 kEncReadPolarity
//   對每輪下「韌體 +25%」命令（DIR 依 motorPol），編碼器原始增量為正 → +1，
//   為負 → -1。與 main.cpp 的定義一致：正命令 → 正回授。
// ============================================================================
static void runCal() {
    Serial.println(F("\n[CAL] 每輪依序 +25%（韌體正轉）0.8s，量測編碼器原始增量 …"));
    Serial.println(F("      車子請架空；按任意鍵中止。"));
    stopAll();
    flushInput();
    bool aborted = false;
    for (int i = 0; i < 4 && !aborted; i++) {
        const int32_t d = measureSpin(i, 25.0f, 800, -1, aborted);
        int8_t p = (d > 0) ? 1 : ((d < 0) ? -1 : 0);
        if (p == 0) {
            Serial.printf("  %s  raw=%+d  → 沒有計數！檢查編碼器 A/B（GP%u/GP%u）\n",
                          kWheelName[i], static_cast<int>(d),
                          static_cast<unsigned>(kEncPin[i]),
                          static_cast<unsigned>(kEncPin[i] + 1));
            continue;
        }
        const int8_t old = encPol[i];
        encPol[i] = p;
        Serial.printf("  %s  raw=%+d  → kEncReadPolarity[%s]=%+d%s\n",
                      kWheelName[i], static_cast<int>(d), kWheelName[i],
                      static_cast<int>(p),
                      (old != p) ? "  (原本相反，已更新)" : "");
    }
    stopAll();
    if (aborted) Serial.println(F("[CAL] 已中止，結果僅部分更新。"));
    Serial.println(F("[CAL] 完成。可用 report 看陣列，或 fwd 25 / left 25 驗證。"));
}

// ============================================================================
// dirtest — 固定 PWM、DIR 低/高各一次，檢查計數是否反向
// ============================================================================
static void runDirTest(int wheel) {
    Serial.printf("\n[DIRTEST] %s：PWM=GP%u DIR=GP%u ENC=GP%u/GP%u\n",
                  kWheelName[wheel], static_cast<unsigned>(kPwmPin[wheel]),
                  static_cast<unsigned>(kDirPin[wheel]),
                  static_cast<unsigned>(kEncPin[wheel]),
                  static_cast<unsigned>(kEncPin[wheel] + 1));
    stopAll();
    flushInput();
    bool aborted = false;
    const int32_t dLow = measureSpin(wheel, 25.0f, 700, 0, aborted);
    if (!aborted) {
        const int32_t dHigh = measureSpin(wheel, 25.0f, 700, 1, aborted);
        Serial.printf("  DIR=LOW  raw=%+d\n", static_cast<int>(dLow));
        Serial.printf("  DIR=HIGH raw=%+d\n", static_cast<int>(dHigh));
        if (dLow != 0 && dHigh != 0 && ((dLow > 0) != (dHigh > 0)))
            Serial.println(F("  → 方向確實反轉：DIR 腳歸屬正確 ✓"));
        else if (dLow == 0 && dHigh == 0)
            Serial.println(F("  → 兩次都沒有計數：DIR 腳或 PWM 腳接錯 / 編碼器沒接"));
        else
            Serial.println(F("  → 方向沒反轉：這支 DIR 可能不是此輪的 DIR（或腳位對應有誤）"));
    }
    stopAll();
    if (aborted) Serial.println(F("[DIRTEST] 已中止。"));
}

// ============================================================================
// cycle — 四輪 fwd 週期性 on/off，用來分辨「程式問題」與「電源/驅動板保護」
// ============================================================================
// 阻塞式四輪輸出（firmware 命令座標）；回傳 false = 使用者中止
static bool driveBlocking(const float cmd[4], uint32_t ms) {
    flushInput();
    const uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        if (anyKeyPressed()) return false;
        drainEncoders();
        for (int i = 0; i < 4; i++) {
            targetCmd[i] = 0.0f;
            curCmd[i] = 0.0f;
            hwWrite(i, cmd[i] * static_cast<float>(motorPol[i]));
        }
        delayMicroseconds(400);
    }
    return true;
}

static void runCycle(float duty) {
    constexpr uint32_t ON_MS = 2000;
    constexpr uint32_t OFF_MS = 600;
    constexpr uint32_t CYCLES = 5;
    Serial.printf("\n[CYCLE] fwd %d%%：on %ums / off %ums ×%u\n",
                  static_cast<int>(lroundf(duty)),
                  static_cast<unsigned>(ON_MS), static_cast<unsigned>(OFF_MS),
                  static_cast<unsigned>(CYCLES));
    Serial.println(F("        若 on 段突然歸零、off 後又恢復 → 電源(BMS)/驅動板過流保護，不是程式。"));
    Serial.println(F("        按任意鍵中止。"));
    stopAll();
    flushInput();

    float onCmd[4];
    const float offCmd[4] = {0, 0, 0, 0};
    chassisTargets(0, 1, 0, duty, onCmd);

    for (uint32_t c = 1; c <= CYCLES; c++) {
        int32_t before[4];
        for (int i = 0; i < 4; i++) before[i] = encRaw[i];
        if (!driveBlocking(onCmd, ON_MS)) {
            stopAll();
            Serial.println(F("[CYCLE] 中止"));
            return;
        }
        int32_t dOn[4];
        for (int i = 0; i < 4; i++) dOn[i] = encRaw[i] - before[i];

        if (!driveBlocking(offCmd, OFF_MS)) {
            stopAll();
            Serial.println(F("[CYCLE] 中止"));
            return;
        }
        int32_t dOff[4];
        for (int i = 0; i < 4; i++) dOff[i] = encRaw[i] - before[i] - dOn[i];

        Serial.printf("[CYCLE] #%u on : FL=%+d FR=%+d BL=%+d BR=%+d\n",
                      static_cast<unsigned>(c), static_cast<int>(dOn[0]),
                      static_cast<int>(dOn[1]), static_cast<int>(dOn[2]),
                      static_cast<int>(dOn[3]));
        Serial.printf("            off: FL=%+d FR=%+d BL=%+d BR=%+d\n",
                      static_cast<int>(dOff[0]), static_cast<int>(dOff[1]),
                      static_cast<int>(dOff[2]), static_cast<int>(dOff[3]));
    }
    stopAll();
    Serial.println(F("[CYCLE] 完成。on 段計數都正常 = 驅動與供電在「重上電」後會恢復。"));
}

// ============================================================================
// sweep — 四輪 fwd 逐段加出力，找出「從哪個 duty 開始掉轉 / 斷電」
//   用來看供電/保護的門檻：正常時轉速應該大致跟 duty 成正比；
//   若某段之後計數明顯不增或直接變 0 → 電流限制（BMS / PTC / PSU / 驅動 OCP）。
// ============================================================================
static void runSweep(uint32_t onMs) {
    static const float kDuty[] = {10, 15, 20, 25, 30, 35, 40, 45};
    Serial.printf("\n[SWEEP] 四輪 fwd，每段 %ums（按任意鍵中止）\n",
                  static_cast<unsigned>(onMs));
    stopAll();
    flushInput();

    float onCmd[4];
    for (size_t s = 0; s < sizeof(kDuty) / sizeof(kDuty[0]); s++) {
        chassisTargets(0, 1, 0, kDuty[s], onCmd);
        int32_t before[4];
        for (int i = 0; i < 4; i++) before[i] = encRaw[i];
        if (!driveBlocking(onCmd, onMs)) {
            stopAll();
            Serial.println(F("[SWEEP] 中止"));
            return;
        }
        int32_t d[4];
        for (int i = 0; i < 4; i++) d[i] = encRaw[i] - before[i];

        // 每輪平均 RPS ×100（整數輸出）
        const float sec = static_cast<float>(onMs) * 1e-3f;
        int32_t rps100[4];
        for (int i = 0; i < 4; i++)
            rps100[i] = static_cast<int32_t>(lroundf(
                static_cast<float>(encPol[i] * d[i]) /
                static_cast<float>(Cfg::ENC_CPR) / sec * 100.0f));

        Serial.printf("[SWEEP] %2d%%  d: FL=%+5d FR=%+5d BL=%+5d BR=%+5d  rps/100: %+d %+d %+d %+d\n",
                      static_cast<int>(kDuty[s]), static_cast<int>(d[0]),
                      static_cast<int>(d[1]), static_cast<int>(d[2]),
                      static_cast<int>(d[3]), static_cast<int>(rps100[0]),
                      static_cast<int>(rps100[1]), static_cast<int>(rps100[2]),
                      static_cast<int>(rps100[3]));
        if (d[0] == 0 && d[1] == 0 && d[2] == 0 && d[3] == 0)
            Serial.println(F("        ↑ 這一段完全沒轉 → 前一段出了力就進了保護，或一段就超電流"));
    }
    stopAll();
    Serial.println(F("[SWEEP] 完成。"));
}

// ============================================================================
// wizard — 逐輪問「這顆往車頭還是車尾」，自動算出 kWheelPolarity 並重跑 cal
//   原理：fwd 的韌體命令固定是 FL=+ FR=- BL=- BR=+（左右鏡像）。
//   設 p = 「DIR=L 時該輪實際往車頭(+) / 車尾(-)」，則
//   kWheelPolarity[i] = p[i] * fwdSign[i]，fwd 時就會四輪都往車頭。
// ============================================================================
static void printReport();

static char askWheelDir(int wheel) {
    Serial.printf("\n[WIZARD] %s：以 DIR=L / 20%% 持續轉動（最多 8 秒）\n",
                  kWheelName[wheel]);
    Serial.println(F("         輪胎上線往「車頭」按 f；往「車尾」按 r；s 跳過"));
    flushInput();
    char c = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < 8000) {
        if (Serial.available()) {
            const int ch = Serial.read();
            if (ch == '\r' || ch == '\n') continue;
            c = static_cast<char>(ch);
            if (c >= 'A' && c <= 'Z')
                c = static_cast<char>(c - 'A' + 'a');
            break;
        }
        drainEncoders();
        for (int i = 0; i < 4; i++) hwWrite(i, (i == wheel) ? 20.0f : 0.0f);
        delayMicroseconds(400);
    }
    for (int i = 0; i < 4; i++) hwWrite(i, 0.0f);
    delay(80);
    flushInput();
    return (c == 'f' || c == 'r' || c == 's') ? c : 0;
}

static void runWizard() {
    Serial.println(F("\n========== 方向校正精靈 =========="));
    Serial.println(F("請把車身架空（四輪離地），才能看清楚每顆輪子的轉向。"));
    Serial.println(F("精靈會逐輪以「DIR=L」慢轉，你只要回答該輪往車頭(f)還是車尾(r)。"));
    Serial.println(F("fwd 的韌體方向是 FL=+ FR=- BL=- BR=+（左右鏡像），依你的回答算出極性。"));
    stopAll();
    flushInput();

    const float fwdSign[4] = {+1.0f, -1.0f, -1.0f, +1.0f}; // 與 fwd 命令相同
    int8_t p[4] = {0, 0, 0, 0};
    int answered = 0;
    for (int i = 0; i < 4; i++) {
        const char c = askWheelDir(i);
        if (c == 'f') {
            p[i] = +1;
            answered++;
        } else if (c == 'r') {
            p[i] = -1;
            answered++;
        } else {
            Serial.printf("  %s 跳過（保留原極性 %+d）\n", kWheelName[i],
                          static_cast<int>(motorPol[i]));
        }
    }
    stopAll();
    if (answered == 0) {
        Serial.println(F("[WIZARD] 沒有回答任何一輪，未變更。"));
        return;
    }

    for (int i = 0; i < 4; i++) {
        if (p[i] == 0) continue;
        motorPol[i] = static_cast<int8_t>(p[i] * (fwdSign[i] > 0.0f ? 1 : -1));
    }

    Serial.println(F("\n[WIZARD] 新的馬達極性："));
    for (int i = 0; i < 4; i++)
        Serial.printf("  %s  DIR=L 實際往 %s  → kWheelPolarity=%+d\n",
                      kWheelName[i],
                      (p[i] > 0) ? "車頭" : ((p[i] < 0) ? "車尾" : "(未量)"),
                      static_cast<int>(motorPol[i]));

    Serial.println(F("[WIZARD] 馬達極性改了，編碼器極性必須重量 → 現在自動執行 cal。"));
    runCal();
    printReport();
    Serial.println(F("[WIZARD] 請再跑 fwd 15 確認四輪同向、車子往車頭走。"));
    Serial.println(F("[WIZARD] 若四輪都同向了還是不直：檢查麥克納姆輪安裝方向"));
    Serial.println(F("         （對角兩顆同向、左右兩顆互為鏡像，不能四顆裝一樣）。"));
}

// ============================================================================
// 報表
// ============================================================================
static void printMap() {
    Serial.println(F("\n---- 目前使用的腳位（新板） ----"));
    Serial.println(F("輪   PWM   DIR   ENC(A/B)   motorPol  encPol"));
    for (int i = 0; i < 4; i++) {
        Serial.printf("%s   GP%-2u  GP%-2u  GP%-2u/GP%-2u   %+d        %+d\n",
                      kWheelName[i], static_cast<unsigned>(kPwmPin[i]),
                      static_cast<unsigned>(kDirPin[i]),
                      static_cast<unsigned>(kEncPin[i]),
                      static_cast<unsigned>(kEncPin[i] + 1),
                      static_cast<int>(motorPol[i]),
                      static_cast<int>(encPol[i]));
    }
    Serial.printf("軸向極性：strafe=%+d  forward=%+d  rot=%+d\n",
                  static_cast<int>(lroundf(axisStrafe)),
                  static_cast<int>(lroundf(axisForward)),
                  static_cast<int>(lroundf(axisRot)));
    Serial.println(F("--------------------------------"));
}

static void printReport() {
    Serial.println(F("\n===== 可貼回 src/move/main.cpp 的設定 ====="));
    Serial.printf("constexpr uint MOTOR_FL_PWM = %u;  constexpr uint MOTOR_FL_DIR = %u;\n",
                  static_cast<unsigned>(Pin::FL_PWM),
                  static_cast<unsigned>(Pin::FL_DIR));
    Serial.printf("constexpr uint MOTOR_FR_PWM = %u;  constexpr uint MOTOR_FR_DIR = %u;\n",
                  static_cast<unsigned>(Pin::FR_PWM),
                  static_cast<unsigned>(Pin::FR_DIR));
    Serial.printf("constexpr uint MOTOR_BR_PWM = %u;  constexpr uint MOTOR_BR_DIR = %u;\n",
                  static_cast<unsigned>(Pin::BR_PWM),
                  static_cast<unsigned>(Pin::BR_DIR));
    Serial.printf("constexpr uint MOTOR_BL_PWM = %u;  constexpr uint MOTOR_BL_DIR = %u;\n",
                  static_cast<unsigned>(Pin::BL_PWM),
                  static_cast<unsigned>(Pin::BL_DIR));
    Serial.printf("constexpr uint MOTOR_FR_ENC = %u;  // A=%u B=%u\n",
                  static_cast<unsigned>(Pin::FR_ENC),
                  static_cast<unsigned>(Pin::FR_ENC),
                  static_cast<unsigned>(Pin::FR_ENC + 1));
    Serial.printf("constexpr uint MOTOR_FL_ENC = %u;  // A=%u B=%u\n",
                  static_cast<unsigned>(Pin::FL_ENC),
                  static_cast<unsigned>(Pin::FL_ENC),
                  static_cast<unsigned>(Pin::FL_ENC + 1));
    Serial.printf("constexpr uint MOTOR_BR_ENC = %u;  // A=%u B=%u\n",
                  static_cast<unsigned>(Pin::BR_ENC),
                  static_cast<unsigned>(Pin::BR_ENC),
                  static_cast<unsigned>(Pin::BR_ENC + 1));
    Serial.printf("constexpr uint MOTOR_BL_ENC = %u;  // A=%u B=%u\n",
                  static_cast<unsigned>(Pin::BL_ENC),
                  static_cast<unsigned>(Pin::BL_ENC),
                  static_cast<unsigned>(Pin::BL_ENC + 1));
    Serial.println();
    Serial.printf("static const int8_t kWheelPolarity[4]   = {%+d, %+d, %+d, %+d};  // [FL, FR, BL, BR]\n",
                  static_cast<int>(motorPol[0]), static_cast<int>(motorPol[1]),
                  static_cast<int>(motorPol[2]), static_cast<int>(motorPol[3]));
    Serial.printf("static const int8_t kEncReadPolarity[4] = {%+d, %+d, %+d, %+d};  // [FL, FR, BL, BR]\n",
                  static_cast<int>(encPol[0]), static_cast<int>(encPol[1]),
                  static_cast<int>(encPol[2]), static_cast<int>(encPol[3]));
    Serial.println();
    Serial.printf("constexpr float kAxisStrafePolarity = %+d.0f;   // left/right\n",
                  static_cast<int>(lroundf(axisStrafe)));
    Serial.printf("constexpr float kAxisForwardPolarity = %+d.0f;  // fwd/back\n",
                  static_cast<int>(lroundf(axisForward)));
    Serial.printf("// cw/ccw 相反時：rOmega 取負（目前 rot=%+d）\n",
                  static_cast<int>(lroundf(axisRot)));
    Serial.println(F("=========================================="));
}

static void printHelp() {
    Serial.println(F(
        "\n---------- move_dir_test ----------\n"
        " fwd|back|left|right|cw|ccw [duty]  四輪同動（預設 25%，上限 55）\n"
        " w <fl|fr|bl|br> <duty>             單輪（正=韌體正轉，可負）\n"
        " stop / 空行 Enter                   停止\n"
        " cal                                自動量測 kEncReadPolarity\n"
        " cycle [duty]                       四輪 on/off 循環（判別電源/驅動板保護）\n"
        " sweep [ms]                         四輪 10→45% 分段加出力（找斷電門檻）\n"
        " same [duty]                        四輪直接 DIR=L 轉（看原始硬體方向）\n"
        " wizard / dirfix                    逐輪回答轉向，自動算極性 + cal\n"
        " axis <strafe|fwd|rot> <-1|1>       改車體軸向極性（left/right 相反就翻 strafe）\n"
        " dirtest <輪>                       驗證 DIR 腳歸屬（低/高各轉一次）\n"
        " pol <輪> <-1|1>                    改 kWheelPolarity（fwd 時哪顆反了就翻）\n"
        " encpol <輪> <-1|1>                 改 kEncReadPolarity\n"
        " enc                                待機手轉輪子，印原始計數\n"
        " report                             印可貼回 main.cpp 的設定\n"
        " map                                印目前腳位 / 極性\n"
        " help                               本說明\n"
        "-----------------------------------"));
}

// ============================================================================
// 命令
// ============================================================================
static bool parseWheelArg(const char *s, int &wheel) {
    wheel = wheelIndex(s);
    if (wheel < 0) {
        Serial.printf("未知輪別 '%s'（fl/fr/bl/br）\n", s);
        return false;
    }
    return true;
}

static void resetStatusWindow();

static void startChassis(const char *label, float vx, float vy, float omega,
                         float duty) {
    g_forceHw = false;
    float out[4];
    chassisTargets(vx, vy, omega, duty, out);
    for (int i = 0; i < 4; i++) targetCmd[i] = out[i];
    g_mode = Mode::Chassis;
    const int dutyI = static_cast<int>(lroundf(duty));
    snprintf(g_modeText, sizeof(g_modeText), "%s %d%%", label, dutyI);
    resetStatusWindow();
    Serial.printf("\n[CMD] %s  duty=%d%%  (firmware cmd FL=%+d FR=%+d BL=%+d BR=%+d)\n",
                  label, dutyI,
                  static_cast<int>(lroundf(targetCmd[0])),
                  static_cast<int>(lroundf(targetCmd[1])),
                  static_cast<int>(lroundf(targetCmd[2])),
                  static_cast<int>(lroundf(targetCmd[3])));
    if (duty > 30.0f)
        Serial.println(F("      ⚠ 出力較大，請確認車身有空間 / 已架空。"));
    flushInput();
}

static void handleCommand(char *line) {
    // 切 token
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
    if (!strcasecmp(cmd, "map")) {
        printMap();
        return;
    }
    if (!strcasecmp(cmd, "report")) {
        printReport();
        return;
    }
    if (!strcasecmp(cmd, "cal")) {
        runCal();
        return;
    }
    if (!strcasecmp(cmd, "enc")) {
        stopAll();
        g_mode = Mode::EncWatch;
        strncpy(g_modeText, "enc-watch", sizeof(g_modeText) - 1);
        Serial.println(F("[CMD] enc：手轉輪子看原始計數（Enter/stop 結束）"));
        return;
    }
    if (!strcasecmp(cmd, "cycle")) {
        float duty = 25.0f;
        if (argc >= 2) duty = static_cast<float>(atof(argv[1]));
        if (duty < 0) duty = -duty;
        duty = clampDuty(duty);
        if (duty < 5.0f) duty = 5.0f;
        runCycle(duty);
        return;
    }
    if (!strcasecmp(cmd, "sweep")) {
        uint32_t onMs = 3500;
        if (argc >= 2) onMs = static_cast<uint32_t>(atoi(argv[1]));
        if (onMs < 500) onMs = 500;
        runSweep(onMs);
        return;
    }
    if (!strcasecmp(cmd, "same")) {
        float duty = 20.0f;
        if (argc >= 2) duty = static_cast<float>(atof(argv[1]));
        if (duty < 0) duty = -duty;
        duty = clampDuty(duty);
        if (duty < 5.0f) duty = 5.0f;
        for (int i = 0; i < 4; i++) targetCmd[i] = duty;
        g_forceHw = true;
        g_mode = Mode::Chassis;
        snprintf(g_modeText, sizeof(g_modeText), "same DIR=L %d%%",
                 static_cast<int>(lroundf(duty)));
        resetStatusWindow();
        Serial.println(F("[CMD] 四輪都直接以 DIR=L 轉（不看 motorPol）。"));
        Serial.println(F("      馬達鏡像安裝時，左右兩側『往同一側轉』是正常的；"));
        Serial.println(F("      要的是各輪『輪子上線往車頭』＝車子前進，用 wizard 逐輪校正。"));
        return;
    }
    if (!strcasecmp(cmd, "wizard") || !strcasecmp(cmd, "dirfix")) {
        runWizard();
        return;
    }
    if (!strcasecmp(cmd, "encpol") || !strcasecmp(cmd, "pol")) {
        int w = -1;
        if (argc < 3 || !parseWheelArg(argv[1], w)) {
            Serial.println(F("用法：pol <輪> <-1|1> / encpol <輪> <-1|1>"));
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
        Serial.printf("[CMD] %s %s = %+d\n",
                      strcasecmp(cmd, "pol") ? "encPol" : "motorPol",
                      kWheelName[w], v);
        return;
    }
    if (!strcasecmp(cmd, "axis")) {
        if (argc < 3) {
            Serial.printf("目前 axis：strafe=%+d  forward=%+d  rot=%+d\n",
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
        Serial.printf("[CMD] axis %s = %+d（重下 left/right/fwd/cw 驗證）\n",
                      argv[1], v);
        return;
    }
    if (!strcasecmp(cmd, "dirtest")) {
        int w = -1;
        if (argc < 2 || !parseWheelArg(argv[1], w)) {
            Serial.println(F("用法：dirtest <fl|fr|bl|br>"));
            return;
        }
        runDirTest(w);
        return;
    }
    if (!strcasecmp(cmd, "w")) {
        int w = -1;
        if (argc < 3 || !parseWheelArg(argv[1], w)) {
            Serial.println(F("用法：w <fl|fr|bl|br> <duty%>"));
            return;
        }
        const float duty = clampDuty(static_cast<float>(atof(argv[2])));
        const int dutyI = static_cast<int>(lroundf(duty));
        g_forceHw = false;
        for (int i = 0; i < 4; i++) targetCmd[i] = 0.0f;
        targetCmd[w] = duty;
        g_mode = Mode::Wheel;
        snprintf(g_modeText, sizeof(g_modeText), "wheel %s %+d%%",
                 kWheelName[w], dutyI);
        resetStatusWindow();
        Serial.printf("\n[CMD] 單輪 %s cmd=%+d%%（DIR 依 motorPol=%+d）\n",
                      kWheelName[w], dutyI, static_cast<int>(motorPol[w]));
        flushInput();
        return;
    }

    struct ChassisCmd {
        const char *name;
        float vx, vy, omega;
    };
    static const ChassisCmd kChassis[] = {
        {"fwd", 0, +1, 0},   {"back", 0, -1, 0},
        {"left", +1, 0, 0},  {"right", -1, 0, 0},
        {"cw", 0, 0, +1},    {"ccw", 0, 0, -1},
    };
    for (const ChassisCmd &cc : kChassis) {
        if (!strcasecmp(cmd, cc.name)) {
            float duty = 25.0f;
            if (argc >= 2) duty = static_cast<float>(atof(argv[1]));
            if (duty < 0) duty = -duty;
            duty = clampDuty(duty);
            if (duty < 5.0f) duty = 5.0f;
            startChassis(cc.name, cc.vx, cc.vy, cc.omega, duty);
            return;
        }
    }

    Serial.printf("未知指令 '%s'，輸入 help 看說明。\n", cmd);
}

// ============================================================================
// 狀態列印
// ============================================================================
static int32_t lastSnap[4] = {0, 0, 0, 0};
static uint32_t lastStatusMs = 0;
static uint32_t lastSnapMs = 0;

// 卡死（有出力但幾乎無回授）偵測 — 以 300ms 狀態視窗累計。
//   用「轉速門檻」而不是「是否剛好 0 個 edge」，否則編碼器零星雜訊
//   （例如接腳浮動每 1 秒冒 1~2 個脈衝）會把倒數一直歸零，永遠偵測不到。
static uint8_t stallWindows[4] = {0, 0, 0, 0};
static bool stallWarned[4] = {false, false, false, false};

// 新命令開始時重置量測窗，避免第一次印出用到很久以前的快照
static void resetStatusWindow() {
    lastSnapMs = millis();
    for (int i = 0; i < 4; i++) {
        lastSnap[i] = encRaw[i];
        stallWindows[i] = 0;
        stallWarned[i] = false;
    }
}

// 有出力但幾乎無回授 → 多半是馬達電源 / 驅動板保護 / 編碼器供電掉了。
//   1 秒（約 3 個視窗）印警告、3 秒（10 個視窗）自動停（避免堵轉燒馬達）。
//   回傳 true = 需要緊急停止。
static bool stallUpdate(int i, float rps, float eff) {
    const bool commanded = fabsf(eff) >= 8.0f;
    const bool stalled = commanded && fabsf(rps) < 0.02f;
    if (stalled) {
        if (stallWindows[i] < 255) stallWindows[i]++;
    } else {
        stallWindows[i] = 0;
        stallWarned[i] = false;
    }
    if (stallWindows[i] >= 3 && !stallWarned[i]) {
        stallWarned[i] = true;
        Serial.printf("\n[STALL] %s 出力 %+d%% 但幾乎無回授 → 查馬達電源(VM)/驅動板保護/編碼器\n",
                      kWheelName[i], static_cast<int>(lroundf(eff)));
    }
    return stallWindows[i] >= 10;
}

static void printStatus() {
    const uint32_t now = millis();
    const float dt = (now - lastSnapMs) > 0
                         ? static_cast<float>(now - lastSnapMs) * 1e-3f
                         : 1e-3f;
    lastSnapMs = now;

    Serial.printf("[%s]\n", g_modeText);
    bool cut = false;
    for (int i = 0; i < 4; i++) {
        const int32_t d = encRaw[i] - lastSnap[i];
        lastSnap[i] = encRaw[i];
        const float rps =
            static_cast<float>(encPol[i] * d) / static_cast<float>(Cfg::ENC_CPR) / dt;
        const float eff = g_effPct[i];
        // 一致性：要跟「韌體命令」（curCmd，cal 也是以此定義 encPol）比，
        //   不是跟硬體 DIR 方向（eff）比。motorPol=-1 時兩者符號本來就相反。
        const float cmd = g_forceHw ? g_effPct[i] : curCmd[i];
        const char *flag = "  ";
        if (fabsf(cmd) > 5.0f && fabsf(rps) > 0.05f)
            flag = ((cmd > 0) == (rps > 0)) ? "ok" : "XX";
        Serial.printf("  %s cmd=%+4d%% DIR=%s d=%+5d  rps=%+d/100  %s\n",
                      kWheelName[i], static_cast<int>(lroundf(cmd)),
                      dirChar(g_effDir[i]), static_cast<int>(d),
                      static_cast<int>(lroundf(rps * 100.0f)), flag);
        if (stallUpdate(i, rps, eff)) cut = true;
    }
    if (cut) {
        Serial.println(F("[STALL] 出力中幾乎無回授超過 3 秒 → 自動停止（保護馬達）"));
        stopAll();
    }
}

static void printEncWatch() {
    static int32_t prev[4] = {0, 0, 0, 0};
    Serial.printf("[enc] raw == FL=%+d FR=%+d BL=%+d BR=%+d  (Δ FL=%+d FR=%+d BL=%+d BR=%+d)\n",
                  static_cast<int>(encRaw[0]), static_cast<int>(encRaw[1]),
                  static_cast<int>(encRaw[2]), static_cast<int>(encRaw[3]),
                  static_cast<int>(encRaw[0] - prev[0]),
                  static_cast<int>(encRaw[1] - prev[1]),
                  static_cast<int>(encRaw[2] - prev[2]),
                  static_cast<int>(encRaw[3] - prev[3]));
    for (int i = 0; i < 4; i++) prev[i] = encRaw[i];
}

// ============================================================================
// setup / loop
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(300);

    for (int i = 0; i < 4; i++) initWheelHw(i);
    encoderInit();
    stopAll();

    Serial.println(F("\n=================================================="));
    Serial.println(F(" Move 新板移動方向測試 (move_dir_test)"));
    Serial.println(F(" 腳位："));
    Serial.println(F("   FR DIR=8  PWM=9  ENC=14/15"));
    Serial.println(F("   FL DIR=6  PWM=7  ENC=16/17"));
    Serial.println(F("   BL DIR=12 PWM=13 ENC=18/19"));
    Serial.println(F("   BR PWM=11 DIR=10 ENC=20/21"));
    Serial.println(F(" 建議流程：fwd 25 → pol 修正 → cal → left/right/cw/ccw → report"));
    Serial.println(F(" 輸入 help 看指令。"));
    Serial.println(F("=================================================="));
    printMap();
    g_mode = Mode::Idle;
    strncpy(g_modeText, "idle", sizeof(g_modeText) - 1);
}

void loop() {
    drainEncoders();

    char line[96];
    if (readCommand(line, sizeof(line))) handleCommand(line);

    // 斜坡 + 輸出
    static uint32_t lastTick = 0;
    const uint32_t now = millis();
    const float dt = (now - lastTick) > 0 ? static_cast<float>(now - lastTick) * 1e-3f
                                          : 1e-3f;
    lastTick = now;

    for (int i = 0; i < 4; i++) {
        const float step = Cfg::RAMP_PER_SEC * dt;
        const float diff = targetCmd[i] - curCmd[i];
        if (fabsf(diff) <= step)
            curCmd[i] = targetCmd[i];
        else
            curCmd[i] += (diff > 0 ? step : -step);
        hwWrite(i, g_forceHw ? curCmd[i]
                             : curCmd[i] * static_cast<float>(motorPol[i]));
    }

    // 狀態輸出（待機不洗畫面，避免洗掉指令回覆）
    if (now - lastStatusMs >= Cfg::STATUS_MS && g_mode != Mode::Idle) {
        lastStatusMs = now;
        if (g_mode == Mode::EncWatch)
            printEncWatch();
        else
            printStatus();
    }
}
