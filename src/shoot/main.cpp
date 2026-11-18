// ============================================================================
// Shoot — Slave 節點 (RP2350)
//
// 雙軸步進架構（兩種驅動方式並存）：
//   仰角 (pitch)：UART 閉環步進驅動器（張大頭 X42S / Emm 固件）
//                 ── PIO 模擬 UART（SerialPIO），TX=GP14, RX=GP15, 115200
//   水平 (row)  ：已移至 Storage 節點（src/storage/main.cpp），本節點不再處理
//
// 使用 SlaveComms 通用框架 + idleHook 機制：
//   Core 0 (loop)：   SlaveComms::update() — 收 Hub 控制幀（硬體 UART0）
//   Core 1 (loop1)：  SlaveComms::runCore1() — 命令驅動 + 兩軸步進推進
//
// 通訊：
//   Hub PIO SM1 (GP4 TX, GP5 RX) ↔ Shoot UART0 (TX=GP0, RX=GP1)
//   921600 baud, COBS 封幀 + CRC16
//
// 控制資料 (ShootPayload)：
//   upSpeed    (int8) — 上輪無刷馬達轉速，永久性速度設定 (0~100)
//   downSpeed  (int8) — 下輪無刷馬達轉速，永久性速度設定 (0~100)
//   pitch      (int8) — 仰角位置 (-97~97) → UART 步進馬達
//                        -97 = 最低角, 0 = 0°, +97 = 最高角
//   row        (int8) — 水平位置 (-97~97)；本節點忽略，由 Storage 處理
//                        -97 = 逆時針 90°, 0 = 0°, +97 = 順時針 90°
//
// ⚠ 轉速值由 Hub 決定「絕對值」，本節點只負責套用：
//     * 遙控備援模式：Hub 固定基準 50%，再用 rc[6]（遙控器第 7 支撥桿）
//       每次 ±5% 微調，上下兩顆永遠同一個值
//     * Jetson 自駕模式：由 Jetson 直接指定 0~100
//     * 0% = 1000µs 停轉；50% ≈ 1500µs；100% = 2000µs 全速
//     * 電調為單向，負值一律視為等量正轉（speedToPulse() 取 fabsf）
//
// 兩軸輸入一律限幅在 ±97（超過的部分忽略），用於保留機構行程餘裕。
//
// 硬體配置：
//   上輪無刷馬達      : 50Hz servo 脈衝，PIO1 SM（GP4）
//   下輪無刷馬達      : 50Hz servo 脈衝，PIO1 SM（GP5）
//   固定轉速無刷馬達  : 50Hz servo 脈衝，PIO1 SM（GP9，固定 1750µs）
//
// upSpeed/downSpeed 為永久性速度設定 — 一經設定即寫入 EEPROM 模擬區，
// 每次開機自動恢復上次設定的速度值，直到收到新的轉速指令為止。
// ============================================================================
#include "SlaveComms.h"
#include "servo_pulse.pio.h"
#include "step_pulse.pio.h"
#include <Arduino.h>
#include <DebugFmt.h>
#include <EEPROM.h>
#include <EepromStore.h>
#include <MathUtils.h>
#include <PioStepRate.h>
#include <StatusLed.h>
#include <cmath>
#include <cstring> // memmove()
#include <hardware/pio.h>
#include <pico/stdlib.h>

// ============================================================================
// 腳位定義（依實際硬體接線修改）
// ============================================================================
namespace Pin {
// ——— 仰角步進（STEP/DIR/EN 脈衝, PIO0 SM0） ———
constexpr uint STEP = 14; // → 驅動器 PUL / STEP
constexpr uint DIR = 15;  // → 驅動器 DIR
constexpr uint EN = 13;   // → 驅動器 ENA（低電位有效）


// ——— 無刷馬達 (ESC PWM) ———
// ⚠ GP0/GP1 給 Hub UART、GP14/GP15 給仰角 UART，
//    無刷馬達使用以下腳位。若新硬體沒有無刷馬達，把 kEnableBldc 設為 false。
// （2026 改版：上輪 GP16、下輪 GP17，固定轉速無刷已移除）
constexpr uint UP_BLDC_PWM = 16;
constexpr uint DOWN_BLDC_PWM = 17;
constexpr bool kEnableBldc = true;
} // namespace Pin

// ============================================================================
// 共同的輸入映射
// ============================================================================
namespace AxisConfig {
// 兩軸輸入範圍：-97 ~ +97（超過限幅，保留機構餘裕）
constexpr float INPUT_LIMIT = 97.0f;
// 輸入 -100~100 → ±90°
constexpr float INPUT_TO_DEG = 90.0f / 100.0f; // 0.9 deg/input
} // namespace AxisConfig

// ============================================================================
// 仰角步進：STEP / DIR / EN 脈衝控制（PIO0 SM0，同 stepdir_gp14_test）
//
//   STEP=GP14（PIO 脈衝）  DIR=GP15  EN=GP13（低電位有效）
//
// 控制方式 = 相對上電位置的絕對脈衝數（遙控器會自動回中 → 上電回中 = 0 中間）：
//   pitch 輸入 -97 ~ -30 → -75000（最前方向）
//   pitch 輸入 -29 ~ +29 →      0（中間 = 上電位置）
//   pitch 輸入 +30 ~ +97 → +75000（最後方向）
// 行程限幅：-195000（最前）~ +205000（最後），確保不會多轉。
//
// ⚠ 本模組在 Core 1 執行（SlaveComms runCore1 / idleHook），
//   禁止呼叫 Serial —— 所有診斷數據經 volatile 變數交 Core 0 印出。
// ⚠ PIO 轉速下限：divider 上限 65536 ⇒ 最低 ≈ 327 steps/s（150MHz）。
// ============================================================================
namespace PioStepper {
namespace Pos {
// 遙控器自動回中 → 上電位置（回中）定義為 0（中間）
// 行程：最前（下）-195000 ~ 最後（上）+205000，確保兩側都不會多轉
constexpr int32_t MIN = -195000; // 最前（pitch -97 ~ -30 對應）
constexpr int32_t MAX = 205000;  // 最後（pitch +30 ~ +97 對應）
constexpr int32_t LOW = -75000;  // pitch -97 ~ -30
constexpr int32_t MID = 0;       // pitch -29 ~ +29（= 上電回中位置）
constexpr int32_t HIGH = 75000;  // pitch +30 ~ +97
constexpr float SPLIT_LOW = -29.5f; // 低/中段邊界
constexpr float SPLIT_HIGH = 29.5f; // 中/高段邊界
} // namespace Pos

// PIO 組態
static PIO gPio = pio0;
static uint gSm = 0;
static uint gOffset = 0;
constexpr uint gIrqFlag = 0;
constexpr float kCyclesPerStep = 7.0f;
constexpr float kDivMax = 65536.0f;
static float gClkSysHz = 150000000.0f;

// 可調參數
static bool gEnActiveLow = true;            // EN 低電位致能
static bool gDirInvert = true;              // DIR 方向反向（實機實測：需反向）
static float gStepsPerRev = 200.0f * 16.0f; // 1.8° × 16 細分 = 3200
static float gSpeedSps = 25000.0f;          // 目標速率 steps/s（≈469 RPM @3200 細分）
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

static int32_t gPos = 0;          // 目前位置：上電位置 = 0（中間）
static int32_t gTarget = -1;      // 最近一次命令的目標（運動中去重用）

// —— 診斷（Core 1 寫、Core 0 讀）——
static volatile int32_t dbg_targetPos = 0;  // 最近一次命令的目標位置
static volatile uint32_t dbg_cmdCount = 0;  // 位置命令次數
static volatile uint32_t dbg_doneCount = 0; // 到位次數
static volatile uint32_t dbg_speedSps = 0;  // 目前命令速率 steps/s（ramp 中會變動）
static volatile uint32_t dbg_targetSps = 0; // 目標速率 steps/s
static volatile uint32_t dbg_clkdiv = 0;    // 目前 PIO divider ×256
static volatile uint32_t dbg_clkSysMhz = 0; // clk_sys（MHz），驗證脈衝換算基準

// ----------------------------------------------------------------------------
// PIO 底層
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
    pio_sm_exec(gPio, gSm, pio_encode_jmp(gOffset)); // PC 指回程式開頭
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
    delayMicroseconds(10); // DIR 建立時間
}

// ----------------------------------------------------------------------------
// 位置換算：pitch 輸入 → 三段固定位置
// ----------------------------------------------------------------------------
static int32_t pitchToPos(float input) {
    if(input > AxisConfig::INPUT_LIMIT)
        input = AxisConfig::INPUT_LIMIT;
    if(input < -AxisConfig::INPUT_LIMIT)
        input = -AxisConfig::INPUT_LIMIT;
    if(input < Pos::SPLIT_LOW)
        return Pos::LOW; // -97 ~ -30
    if(input > Pos::SPLIT_HIGH)
        return Pos::HIGH; // +30 ~ +97
    return Pos::MID;      // -29 ~ +29
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
// 運動控制
// ----------------------------------------------------------------------------
static void stopMotion() {
    bool wasRunning = gSt.running;
    if(wasRunning) {
        gPos += gSt.dir * static_cast<int32_t>(gSt.estSteps + 0.5f);
    }
    if(gPos < Pos::MIN) {
        gPos = Pos::MIN;
    }
    if(gPos > Pos::MAX) {
        gPos = Pos::MAX;
    }
    haltPulses();
    gSt.running = false;
    gSt.speedSps = 0.0f;
    (void)wasRunning;
}

static void startMove(int32_t steps, int8_t dir) {
    if(steps <= 0) {
        return;
    }
    if(!gSt.enabled) {
        gSt.enabled = true; // 未使能時自動重新使能（幂等）
        applyEnablePin();
    }
    haltPulses();

    gSt.dir = (dir >= 0) ? 1 : -1;
    applyDir(gSt.dir);

    float wantSps = clampSps(gSpeedSps);
    // 短行程：三角曲線 v_peak = √(a·S)
    if(gAccelSps2 > 0.0f) {
        float peak = sqrtf(gAccelSps2 * static_cast<float>(steps));
        if(peak < wantSps) {
            wantSps = (peak < gMinSps) ? gMinSps : peak;
        }
    }
    gSt.targetSps = wantSps;
    gSt.speedSps = (gAccelSps2 > 0.0f) ? gMinSps : gSt.targetSps;
    gSt.remaining = steps;
    gSt.estSteps = 0.0f;
    gSt.lastRampUs = micros();
    gSt.running = true;

    startPulses(static_cast<uint32_t>(steps));
}

/// 絕對位置命令（Core 1 context，不阻塞、不印字）
void setTargetPos(int32_t pos) {
    if(pos < Pos::MIN) {
        pos = Pos::MIN;
    }
    if(pos > Pos::MAX) {
        pos = Pos::MAX;
    }
    dbg_cmdCount++;
    dbg_targetPos = pos;
    // Hub 每個控制週期都會送同一目標 → 運動中且目標相同時直接忽略，
    // 否則每次都 stopMotion + startMove，速度被歸零永遠加速不上去
    if(gSt.running && pos == gTarget) {
        return;
    }
    if(gSt.running) {
        stopMotion(); // 改目標：先停（stopMotion 會用 estSteps 更新 gPos）
    }
    gTarget = pos;
    int32_t delta = pos - gPos;
    if(delta == 0) {
        return; // 已在目標
    }
    startMove((delta < 0) ? -delta : delta, (delta > 0) ? 1 : -1);
}

/// pitch 輸入（-97~97）→ 三段位置
void setTargetPitch(float input) {
    setTargetPos(pitchToPos(input));
}

// ----------------------------------------------------------------------------
// 每 idleHook 迭代呼叫：調速 + 到位偵測（脈衝由 PIO 產生，CPU 不忙等）
// ----------------------------------------------------------------------------
void service() {
    if(!gSt.running) {
        return;
    }

    if(pio_interrupt_get(gPio, gIrqFlag)) {
        pio_interrupt_clear(gPio, gIrqFlag);
        gSt.running = false;
        gPos += gSt.dir * gSt.remaining;
        if(gPos < Pos::MIN) {
            gPos = Pos::MIN;
        }
        if(gPos > Pos::MAX) {
            gPos = Pos::MAX;
        }
        gSt.speedSps = 0.0f;
        dbg_doneCount++;
        return;
    }

    uint32_t now = micros();
    uint32_t dtUs = now - gSt.lastRampUs;
    if(dtUs < 200) { // 5kHz 更新加減速即可
        return;
    }
    gSt.lastRampUs = now;
    gSt.estSteps += gSt.speedSps * static_cast<float>(dtUs) * 1e-6f;

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
    dbg_speedSps = static_cast<uint32_t>(gSt.speedSps);
    dbg_targetSps = static_cast<uint32_t>(gSt.targetSps);
    dbg_clkdiv = static_cast<uint32_t>(divForSps(clampSps(gSt.speedSps)) * 256.0f);
    dbg_clkSysMhz = static_cast<uint32_t>(gClkSysHz / 1000000.0f);
}

// ----------------------------------------------------------------------------
// E-Stop：立即停止（保持使能鎖住位置）
// ⚠ 必須幂等：SlaveComms 在 E-Stop 期間每個 spinloop 迭代都會呼叫 onEstop()
// ----------------------------------------------------------------------------
static bool stopSent = false;
void estop() {
    if(stopSent) {
        return;
    }
    stopMotion();
    stopSent = true;
}
bool isStopped() { return stopSent; }

void enable() {
    gSt.enabled = true;
    applyEnablePin();
    stopSent = false;
}

int32_t position() { return gPos; }
/// 運動中的即時位置估算（gPos + 已發步數），到位/靜止時等同 position()
int32_t posLive() {
    if(!gSt.running)
        return gPos;
    float done = gSt.estSteps;
    if(done > static_cast<float>(gSt.remaining))
        done = static_cast<float>(gSt.remaining);
    int32_t p = gPos + gSt.dir * static_cast<int32_t>(done + 0.5f);
    if(p < Pos::MIN)
        p = Pos::MIN;
    if(p > Pos::MAX)
        p = Pos::MAX;
    return p;
}
int32_t targetPos() { return dbg_targetPos; }
uint32_t cmdCount() { return dbg_cmdCount; }
uint32_t doneCount() { return dbg_doneCount; }
uint32_t speedSps() { return dbg_speedSps; }
uint32_t targetSps() { return dbg_targetSps; }
uint32_t clkdivX256() { return dbg_clkdiv; }
uint32_t clkSysMhz() { return dbg_clkSysMhz; }
bool isRunning() { return gSt.running; }
bool isEnabled() { return gSt.enabled; }

/// 上電初始化（Core 0 setup() 呼叫，Core 1 之後只做 service()）
void init() {
    gClkSysHz = static_cast<float>(clock_get_hz(clk_sys));
    gMinSps = gClkSysHz / (kCyclesPerStep * kDivMax);

    gpio_init(Pin::DIR);
    gpio_set_dir(Pin::DIR, GPIO_OUT);
    gpio_put(Pin::DIR, 0);

    gpio_init(Pin::EN);
    gpio_set_dir(Pin::EN, GPIO_OUT);
    gSt.enabled = false;
    applyEnablePin(); // 先去使能，init() 完成後再 enable()

    pioSetup();
}
} // namespace PioStepper

// ============================================================================
// BLDC 速度持久化儲存（EEPROM 模擬）
// ============================================================================
namespace SpeedStorage {
constexpr int EEPROM_SIZE = 16;
constexpr int ADDR_UP_SPEED = 0;
constexpr int ADDR_DOWN_SPEED = 4;

static float persistentUpSpeed = 0.0f;
static float persistentDownSpeed = 0.0f;

void init() {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(ADDR_UP_SPEED, persistentUpSpeed);
    EEPROM.get(ADDR_DOWN_SPEED, persistentDownSpeed);
    // 若 flash 尚未初始化（全 0xFF），float 會是 NaN → 歸零
    // ⚠ 本節點維持「兩個裸 float 欄位」的既有 flash 版面（不做 magic/checksum），
    //   只借用 EepromStore::sanitizeFloat 擋抹除態，版面位元等價。
    persistentUpSpeed = EepromStore::sanitizeFloat(persistentUpSpeed);
    persistentDownSpeed = EepromStore::sanitizeFloat(persistentDownSpeed);
}

// 只在值真的改變時才寫 flash。
// 原本每收到一幀控制命令（10-20ms）就 EEPROM.commit() 兩次，後果：
//   1. flash 抹寫次數爆掉（很快壞塊）
//   2. commit 期間 XIP 被卡住 ~數十 ms → 會拖到 BLDC 伺服脈衝 FIFO 的續命
void saveUpSpeed(float speed) {
    if(fabsf(speed - persistentUpSpeed) < 0.01f)
        return;
    persistentUpSpeed = speed;
    EEPROM.put(ADDR_UP_SPEED, speed);
    EEPROM.commit();
}

void saveDownSpeed(float speed) {
    if(fabsf(speed - persistentDownSpeed) < 0.01f)
        return;
    persistentDownSpeed = speed;
    EEPROM.put(ADDR_DOWN_SPEED, speed);
    EEPROM.commit();
}

float getUpSpeed() { return persistentUpSpeed; }
float getDownSpeed() { return persistentDownSpeed; }
} // namespace SpeedStorage

// ============================================================================
// BLDC 馬達控制 (ESC) — 50Hz servo 脈衝由 PIO 產生
//
// 為什麼用 PIO1：PIO0 的 SM 已被「仰角 UART 2」佔用。
// RP2350 的 PIO1 預設 GPIO base = 16，GP16/GP17 直接可用，不需改 base。
//
// 一個通道一個 SM，同一份 servo_pulse.pio（共用 offset，只佔 8 條指令）：
//   clkdiv = clk_sys / 1MHz → 1 週期 = 1µs
//   CPU 每幀 push 2 個值（高電位週期、低電位週期），TX FIFO 深度 4 = 2 幀緩衝
//
// 餵資料放在 Core 0 的 loop()：E-Stop 期間 Core 1 的 idleHook 不再執行，
// 但 ESC 必須繼續收到 1000µs 停轉訊號才不會亂轉，所以由 Core 0 持續補 FIFO。
// 跨核心傳遞用 32-bit volatile（單一字組寫入即 atomic，不需鎖）。
// ============================================================================

namespace BldcControl {
constexpr uint32_t FRAME_US = 20000;   // 50Hz
// 規格與 Storage 的 ESC 相同（單向油門）：1000µs = 0%（停轉）、2000µs = 100%
constexpr uint16_t PULSE_MID = 1000;   // 停轉（0%）
constexpr uint16_t PULSE_MAX = 2000;   // 全速（100%）


// PIO 程式的固定額外週期（見 servo_pulse.pio 週期註解）
constexpr int32_t HIGH_OVERHEAD_US = 3;  // set pins,1 + pull + mov
constexpr int32_t FRAME_OVERHEAD_US = 6; // 高+低兩段的 set/pull/mov

enum Channel : uint8_t { CH_UP = 0, CH_DOWN = 1, CH_COUNT = 2 };

static PIO sPio = pio1;
static uint sSm[CH_COUNT] = {0, 0};
static uint sOffset = 0;
static bool sReady = false;

// Core 1 寫、Core 0 讀（32-bit 單字組 → atomic）
static volatile uint32_t sPulseUs[CH_COUNT] = {PULSE_MID, PULSE_MID};

// ESC arming：上電後先持續送停轉 1000µs 這麼久，之後才接受非 0 轉速。
// 若上電直接收到油門（含 EEPROM 恢復的舊速度），ESC 會判定訊號無效拒轉。
static constexpr uint32_t ARM_DELAY_MS = 3000;
static uint32_t sBootMs = 0;

/// 依快取脈寬推入一幀（高電位週期 + 低電位週期）
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

/// Core 0（loop）：維持每個通道的 FIFO 不空
void update() {
    if(!sReady) {
        return;
    }
    for(uint8_t i = 0; i < CH_COUNT; i++) {
        // FIFO 深度 4：保持 ≥2 幀（≈40ms）緩衝；不足就補一幀
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
            sReady = false; // PIO1 SM 不足（理論上不會：共 4 個，只用 2）
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

// speed 0~100 → 1000~2000µs（0% = 1000µs 停轉，規格同 Storage ESC）
// 輸入先過 MathUtils::sanitizePercent：NaN → 0、負值取大小（電調單向）、上限 100
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
    // arming 期間壓回中立，開機就直接套 EEPROM 舊速度會被 ESC 拒收
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
// SlaveApp 回呼
// ============================================================================
static uint8_t tmFrameSeq = 0;       // 遙測幀序號
static MoveTelemetry pendingTm = {}; // Core 1 寫入 → Core 0 發送

// 除錯用計數（Core 1 寫、Core 0 讀）
static volatile uint32_t dbg_cmdCount = 0;
static volatile uint32_t dbg_estopCount = 0;

// 最近一次收到的命令（Core 1 寫、Core 0 讀；供序列監控比對解析結果）
static volatile int8_t dbg_lastPitch = 0;
static volatile int8_t dbg_lastUp = 0;
static volatile int8_t dbg_lastDown = 0;

// E-Stop 期間仍要告訴 Hub「我被停了」（遙測只在收到命令時才組，所以另外補送）
constexpr uint32_t ESTOP_TM_INTERVAL_MS = 200;

/// 組遙測帧（Core 1 context）
static void buildTelemetry(uint8_t estopFlag) {
    pendingTm = {};
    // Shoot 遙測：復用 float 陣列
    // currentRps[0] = 上輪速度, [1] = 下輪速度
    // currentRps[2] = 仰角實測角度(°), [3] = 保留（水平已移至 Storage）
    pendingTm.currentRps[0] = SpeedStorage::getUpSpeed();
    pendingTm.currentRps[1] = SpeedStorage::getDownSpeed();
    pendingTm.currentRps[2] = static_cast<float>(PioStepper::posLive());
    pendingTm.currentRps[3] = 0.0f;
    pendingTm.targetRps[0] = SpeedStorage::getUpSpeed();
    pendingTm.targetRps[1] = SpeedStorage::getDownSpeed();
    pendingTm.targetRps[2] = static_cast<float>(PioStepper::targetPos());
    pendingTm.targetRps[3] = 0.0f;
    pendingTm.mode = RobotMode::AUTO;
    pendingTm.estopFlag = estopFlag;
    // ⚠ 遙測標頭（sync/msgType/frameSeq/targetID）、timestamp 與 CRC 由
    //   SignalUtils::finalizeTelemetry 統一填寫；手寫漏填 sync 或 CRC 會讓
    //   Hub 端驗證失敗而整包丟棄（序列監控只看到 "CRC mismatch"）。
    //   CRC 計算範圍不含 crc16 自身；timestamp 維持 us 計時的低 32 bits。
    SignalUtils::finalizeTelemetry(
        pendingTm, tmFrameSeq++,
        static_cast<uint32_t>(time_us_64() & 0xFFFFFFFFULL));
}

static void onShootCommand(const ChassisSnapshot &cmd) {
    dbg_cmdCount++;
    dbg_lastPitch = static_cast<int8_t>(lroundf(cmd.pitch));

    // 轉速：單向電調 → 正規化成 0~100（NaN/負值/超範圍都在這裡收乾）
    const float upSpeed = MathUtils::sanitizePercent(cmd.shootUpSpeed);
    const float downSpeed = MathUtils::sanitizePercent(cmd.shootDownSpeed);
    dbg_lastUp = static_cast<int8_t>(lroundf(upSpeed));
    dbg_lastDown = static_cast<int8_t>(lroundf(downSpeed));

    // 永久性速度：每次命令都储存並套用（包括速度=0 的清零指令）
    // saveUpSpeed 內部會比較差異，值沒變就不會 EEPROM.commit()
    SpeedStorage::saveUpSpeed(upSpeed);
    SpeedStorage::saveDownSpeed(downSpeed);
    BldcControl::setUpSpeed(upSpeed);
    BldcControl::setDownSpeed(downSpeed);

    // 兩軸步進目標
    PioStepper::setTargetPitch(cmd.pitch); // 仰角：STEP/DIR 脈衝三段位置

    // 遙測（收到有效帧 = E-Stop 已解除）
    buildTelemetry(0);
}

// 前向宣告：E-Stop 遙測（定義在 slave 之後）
static void pushEstopTelemetry();

static void onShootEstop() {
    dbg_estopCount++;

    // BLDC：只改快取值（Core 0 會在 loop() 繼續餵 1000µs 停轉訊號）
    BldcControl::estop();

    // 步進：只由 Core 1 操作 PIO / UART（避免與 idleHook 同時改 PIO 暫存器）
    if(get_core_num() != 1) {
        // Core 0 觸發（指令逾時）時什麼都不做：
        // runCore1() 的 E-Stop 分支每個迭代都會在 Core 1 再呼叫一次 onEstop()
        return;
    }
    PioStepper::estop();

    // 定期補送 E-Stop 遙測；用 200ms 限制避免 tight loop 灌满 UART
    pushEstopTelemetry();
}

// 前向宣告：shootCommandWrapper 定義在 slave 之後
static void shootCommandWrapper(const ChassisSnapshot &cmd);

static SlaveApp shootApp = {
    .targetID = TargetID::SHOOT,
    .onCommand = shootCommandWrapper, // wrapper: onShootCommand + 遙測發送
    .onEstop = onShootEstop,
};

// Hub 通訊：硬體 UART0（TX=GP0, RX=GP1）— 預設建構子會自行 begin()
static SlaveComms slave(&shootApp);

static void shootCommandWrapper(const ChassisSnapshot &cmd) {
    onShootCommand(cmd);
    slave.prepareTelemetry(pendingTm);
}

// E-Stop 期間的遙測（Core 1）：讓 Hub 看到 estopFlag=1
static void pushEstopTelemetry() {
    static uint32_t lastMs = 0;
    const uint32_t now = millis();
    if(now - lastMs < ESTOP_TM_INTERVAL_MS) {
        return;
    }
    lastMs = now;
    buildTelemetry(1);
    slave.prepareTelemetry(pendingTm);
}

// ============================================================================
// Core 1 idle hook — 每 spinloop 迭代空閒時推進兩軸
// ============================================================================
static void shootIdleHook() {
    PioStepper::service(); // 仰角：PIO 加減速 + 到位偵測
}

// ============================================================================
// 狀態 LED（板載 LED_BUILTIN，例如 Waveshare RP2350 Plus 板上 LED）
//
// 依通訊 / E-Stop 狀況顯示（優先權：E-STOP > BOOT > ACTIVE > IDLE）：
//   ┌────────────────────┬──────────────┬──────────────────────┐
//   │ 狀態               │ 圖樣         │ 亮 / 滅              │
//   ├────────────────────┼──────────────┼──────────────────────┤
//   │ 未連線 / 剛開機    │ 長亮         │ —                    │
//   │ E-Stop 生效        │ 5Hz 快閃     │ 100ms / 100ms        │
//   │ 資料通訊中         │ 1Hz 短脈心跳 │ 80ms / 920ms         │
//   │ 已連線但 Idle      │ 0.5Hz 慢閃   │ 500ms / 1500ms       │
//   └────────────────────┴──────────────┴──────────────────────┘
//
// 「未連線」優先於 E-Stop：開機至今從未收到有效帧時本來就會因 100ms 逾時
// 進入 E-Stop，此時長亮比快閃更有意義（代表根本沒受控過）。
// E-Stop 來源：Hub 廣播 ESTOP，或本機 100ms 指令逾時（SlaveComms 自動觸發）。
// 監控細節（位置/速度/計數）直接看序列監控的 [SHOOT] 輸出。
// ============================================================================
namespace LedCfg {
constexpr uint32_t ACTIVE_TIMEOUT_MS = 150; // 視為「資料通訊中」的視窗
constexpr uint32_t ACTIVE_ON_MS = 80;
constexpr uint32_t ACTIVE_OFF_MS = 920;
constexpr uint32_t IDLE_ON_MS = 500;
constexpr uint32_t IDLE_OFF_MS = 1500;
constexpr uint32_t ESTOP_ON_MS = 100;
constexpr uint32_t ESTOP_OFF_MS = 100;
} // namespace LedCfg

enum class LedState : uint8_t { BOOT = 0, IDLE = 1, ACTIVE = 2, ESTOP = 3 };

// 亮滅圖樣表：索引 = LedState 的數值（相位推進交給 StatusLed）
//   BOOT   → 長亮（剛開機、從未收到有效帧）
//   IDLE   → 0.5Hz 慢閃，鎮定不擾人
//   ACTIVE → 1Hz 短脈衝心跳（像路由器資料燈）
//   ESTOP  → 5Hz 快閃
static const StatusLed::Pattern kLedPatterns[] = {
    {0, 0, true},                                         // BOOT：長亮
    {LedCfg::IDLE_ON_MS, LedCfg::IDLE_OFF_MS, false},     // IDLE
    {LedCfg::ACTIVE_ON_MS, LedCfg::ACTIVE_OFF_MS, false}, // ACTIVE
    {LedCfg::ESTOP_ON_MS, LedCfg::ESTOP_OFF_MS, false},   // ESTOP
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
    case LedState::ACTIVE:
        return "ACTIVE";
    default:
        return "ESTOP";
    }
}

// 供 Core 0 除錯輸出用（與 LED 實際狀態保證一致）
static volatile uint8_t dbg_ledState = static_cast<uint8_t>(LedState::BOOT);

static LedState currentLedState() {
    if(slave.validCmdCount() == 0)
        return LedState::BOOT; // 開機至今從未收到有效帧
    if(slave.isEstopActive())
        return LedState::ESTOP; // 連線過，但現在被 E-Stop 切斷出力
    if(slave.msSinceLastValidCmd() < LedCfg::ACTIVE_TIMEOUT_MS)
        return LedState::ACTIVE; // 資料通訊中
    return LedState::IDLE;       // 曾連線，但目前沒控制流
}

static void updateStatusLed() {
    const uint8_t st = static_cast<uint8_t>(currentLedState());
    dbg_ledState = st; // 除錯輸出與 LED 實際套用的狀態一致

    // 狀態切換 → 重置相位並立刻點亮；長亮狀態持續 HIGH；其餘亮滿 onMs 熄、
    // 熄滿 offMs 亮（非阻塞，見 StatusLed::update）。
    statusLed.update(st, millis());
}

// ============================================================================
// Core 0
// ============================================================================
void setup() {
    // ⚠ 第一個執行動作：把馬達推向「確定零出力」
    //   arduino-pico 先 launch core1 再呼叫 setup()，Core 1 此刻已在
    //   slave.runCore1() spinloop 裡；Serial.begin()/delay() 會拖上百 ms。
    //   先鎖定硬體再把腳位交給 PIO，避免開機期間馬達亂動。
    BldcControl::init();
    PioStepper::init();   // PIO0 SM0 脈衝 + DIR/EN GPIO
    PioStepper::enable(); // 上電即使能（位置基準 = 上電時的機構位置 = 0）

    statusLed.begin();
    digitalWrite(LED_BUILTIN, HIGH); // 開機先亮（BOOT 狀態）

    Serial.begin(115200);
    delay(200);
    Serial.println("[SHOOT] boot: 仰角=UART步進(GP14/15)");

    slave.begin(); // 硬體 UART0 (GP0/GP1) → Hub
    slave.setIdleHook(shootIdleHook);

    Serial.println("[SHOOT] 等待 Hub 命令（100ms 無帧 → E-Stop）");
}

void loop() {
    slave.update();
    BldcControl::update(); // Core 0 持續餵 50Hz 伺服脈衝（E-Stop 期間也保持 1000µs 停轉）
    updateStatusLed();

    // 200ms 監控輸出（E-Stop 時改印警示）
    static uint32_t lastPrint = 0;
    const uint32_t nowMs = millis();
    if(nowMs - lastPrint >= 200) {
        lastPrint = nowMs;
        const uint32_t since = slave.msSinceLastValidCmd();
        if(slave.isEstopActive()) {
            const bool timeoutDriven = since >= (RobotConfig::COMMAND_TIMEOUT_US / 1000ULL);
            Serial.printf("[SHOOT] ** E-STOP ** (%s) sinceCmd=%ums led=%s estopCb=%u 仰角停=%d\n",
                          timeoutDriven ? "cmd timeout" : "hub estop", since,
                          ledStateName(static_cast<LedState>(dbg_ledState)),
                          static_cast<unsigned>(dbg_estopCount),
                          PioStepper::isStopped() ? 1 : 0);
        } else {
            const int8_t upSpeed = static_cast<int8_t>(lroundf(SpeedStorage::getUpSpeed()));
            const int8_t downSpeed =
                static_cast<int8_t>(lroundf(SpeedStorage::getDownSpeed()));
            Serial.printf("[SHOOT] sinceCmd=%ums cnt=%u led=%s | cmd pitch=%d row=%d | ",
                          since, static_cast<unsigned>(dbg_cmdCount),
                          ledStateName(static_cast<LedState>(dbg_ledState)),
                          static_cast<int>(dbg_lastPitch));
            // 仰角改為絕對步數位置，不再有角度回報；印位置與目標
            Serial.printf("仰角 pos=%ld (目標 %ld) 運轉=%d | up=%d down=%d | rx=%u crc=%u | elev cmd=%u done=%u",
                          static_cast<long>(PioStepper::posLive()),
                          static_cast<long>(PioStepper::targetPos()),
                          PioStepper::isRunning() ? 1 : 0,
                          static_cast<int>(upSpeed), static_cast<int>(downSpeed),
                          static_cast<unsigned>(slave.rxFrameCount()),
                          static_cast<unsigned>(slave.rxCrcErrorCount()),
                          static_cast<unsigned>(PioStepper::cmdCount()),
                          static_cast<unsigned>(PioStepper::doneCount()));
            Serial.printf(" | sps=%u/%u clkdiv=%.2f clk=%uMHz",
                          static_cast<unsigned>(PioStepper::speedSps()),
                          static_cast<unsigned>(PioStepper::targetSps()),
                          static_cast<double>(PioStepper::clkdivX256()) / 256.0,
                          static_cast<unsigned>(PioStepper::clkSysMhz()));
        }
    }

    // 仰角步進診斷 — 每秒一次（運轉中/尚未到位才提醒）
    {
        static uint32_t lastElevDbgMs = 0;
        if(nowMs - lastElevDbgMs >= 1000) {
            lastElevDbgMs = nowMs;
            if(PioStepper::isRunning()) {
                Serial.printf("[SHOOT] 仰角運轉中 pos=%ld 目標=%ld\n",
                              static_cast<long>(PioStepper::posLive()),
                              static_cast<long>(PioStepper::targetPos()));
            }
        }
    }
}

// ============================================================================
// Core 1
// ============================================================================
void setup1() {
    SpeedStorage::init();

    // 套用保留的速度值（固定轉速無刷已在 BldcControl::init() 中設定為 1750µs）
    BldcControl::setUpSpeed(SpeedStorage::getUpSpeed());
    BldcControl::setDownSpeed(SpeedStorage::getDownSpeed());
}

void loop1() { slave.runCore1(); }
