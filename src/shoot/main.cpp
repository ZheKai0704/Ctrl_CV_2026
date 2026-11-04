// ============================================================================
// Shoot — Slave 節點 (RP2350)
//
// 雙軸步進架構（兩種驅動方式並存）：
//   仰角 (pitch)：UART 閉環步進驅動器（張大頭 X42S / Emm 固件）
//                 ── PIO 模擬 UART（SerialPIO），TX=GP2, RX=GP3, 115200
//   水平 (row)  ：傳統 STEP/DIR/EN 驅動器（A4988 / DRV8825 / TB6600）
//                 ── STEP=GP6 由 PIO 產生脈衝，DIR=GP7，EN=GP8（低電位致能）
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
//   upSpeed    (int8) — 上輪無刷馬達轉速，永久性速度設定 (-100~100)
//   downSpeed  (int8) — 下輪無刷馬達轉速，永久性速度設定 (-100~100)
//   pitch      (int8) — 仰角位置 (-97~97) → UART 步進馬達
//                        -97 = 最低角, 0 = 0°, +97 = 最高角
//   row        (int8) — 水平位置 (-97~97) → STEP/DIR 步進馬達
//                        -97 = 逆時針 90°, 0 = 0°, +97 = 順時針 90°
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
#include <EEPROM.h>
#include <cmath>
#include <hardware/pio.h>
#include <pico/stdlib.h>

// ============================================================================
// 腳位定義（依實際硬體接線修改）
// ============================================================================
namespace Pin {
// ——— 仰角 UART 步進（PIO 模擬 UART） ———
constexpr uint ELEV_UART_TX = 2; // → 馬達 R/A/H 或 T/B/L
constexpr uint ELEV_UART_RX = 3; // ← 馬達 T/B/L 或 R/A/H

// ——— 水平 STEP/DIR 步進（STEP 由 PIO 產生） ———
constexpr uint ROW_STEP = 6; // → 驅動器 PUL
constexpr uint ROW_DIR = 7;  // → 驅動器 DIR
constexpr uint ROW_EN = 8;   // → 驅動器 ENA（低電位致能）

// ——— 無刷馬達 (ESC PWM) ———
// ⚠ GP0/GP1 給 Hub UART、GP2/GP3 給仰角 UART、GP6/7/8 給水平步進，
//    無刷馬達使用以下腳位。若新硬體沒有無刷馬達，把 kEnableBldc 設為 false。
constexpr uint UP_BLDC_PWM = 4;
constexpr uint DOWN_BLDC_PWM = 5;
constexpr uint FIXED_BLDC_PWM = 9;
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
// 仰角步進：UART 閉環驅動器（Emm 固件）
//
// 仰角馬達 UART：PIO 模擬（不吃硬體 UART，GP0/GP1 留給 Hub 通訊）
// ============================================================================
static SerialPIO elevSerial(Pin::ELEV_UART_TX, Pin::ELEV_UART_RX);

// 協定（ZDT_X42S 第二代 / Emm）：
//   幀 = [位址][功能碼][參數…][尾碼 0x6B]，位址預設 1
//   位置單位 = 脈衝，1.8°步進 ×16 細分 = 3200 脈衝/圈
//
// 流程：
//   init()  → 使能 + 「目前位置清零」(0A/6D)，之後一切以絕對位置命令
//   setTarget() → 只在目標改變時送 FD 絕對位置命令（避免塞滿 UART）
//   poll()  → 每 200ms 非阻塞讀一次即時位置（0x36），供遙測/除錯
// ============================================================================
namespace ElevStepper {
namespace Emm {
constexpr uint8_t ADDR = 0x01;      // 馬達位址
constexpr uint8_t TAIL = 0x6B;      // 幀尾碼
constexpr uint8_t FD_EN = 0xF3;     // 使能 / 去使能
constexpr uint8_t FD_POS = 0xFD;    // 位置模式控制
constexpr uint8_t FD_STOP = 0xFE;   // 立即停止
constexpr uint8_t FD_READPOS = 0x36; // 讀即時位置
constexpr uint8_t FD_CLEARPOS = 0x0A; // 目前位置角度清零
constexpr uint8_t AX_EN = 0xAB;
constexpr uint8_t AX_STOP = 0x98;
constexpr uint8_t AX_CLEARPOS = 0x6D;
constexpr uint8_t POS_MODE_ABS = 0x01; // 絕對位置（相對零位）
} // namespace Emm

// 3200 脈衝/圈（16 細分）
constexpr float PULSES_PER_REV = 3200.0f;
constexpr float PULSES_PER_DEG = PULSES_PER_REV / 360.0f;
constexpr uint16_t SPEED_RPM = 600;  // 驅動器內部轉速（≤3000 上限）
constexpr uint8_t ACC_STEP = 100;    // 加速度檔位 0-255
// FD 位置幀格式：true = 13B（第二代，含運動模式位）；false = 12B（V4.2 舊 Emm）
constexpr bool kModeByte = true;
constexpr uint32_t POLL_INTERVAL_US = 200000; // 位置讀取間隔

static uint8_t txBuf[16];
static uint8_t rxBuf[16];
static uint8_t rxLen = 0;
static int32_t targetPulses = 0;
static int32_t lastSentPulses = 0;
static bool sentOnce = false;
static bool stopSent = false; // E-Stop 停止命令已送出（避免 tight loop 灌爆 UART）
static float reportedDeg = 0.0f;
static uint32_t lastPollUs = 0;
static bool enabled = false;

// —— 診斷（Core 1 寫、Core 0 讀）——
static volatile uint32_t rxByteCount = 0; // 從驅動器收到的總位元組
static volatile uint32_t replyCount = 0;  // 成功解析的 0x36 回覆次數
static volatile int32_t lastReplyRaw = 0; // 最近一次回覆的原始值
static volatile uint8_t lastReplySign = 0;
static volatile uint8_t lastReplyHex[8] = {0}; // 最近一次完整回覆（8 bytes）
static volatile bool lastReplyHexValid = false;

static void send(const uint8_t *buf, uint8_t len) { elevSerial.write(buf, len); }

static void sendEnable(bool en) {
    txBuf[0] = Emm::ADDR;
    txBuf[1] = Emm::FD_EN;
    txBuf[2] = Emm::AX_EN;
    txBuf[3] = en ? 0x01 : 0x00;
    txBuf[4] = 0x00; // 同步標誌
    txBuf[5] = Emm::TAIL;
    send(txBuf, 6);
    enabled = en;
}

static void sendClearPos() {
    txBuf[0] = Emm::ADDR;
    txBuf[1] = Emm::FD_CLEARPOS;
    txBuf[2] = Emm::AX_CLEARPOS;
    txBuf[3] = Emm::TAIL;
    send(txBuf, 4);
}

static void sendStop() {
    txBuf[0] = Emm::ADDR;
    txBuf[1] = Emm::FD_STOP;
    txBuf[2] = Emm::AX_STOP;
    txBuf[3] = 0x00;
    txBuf[4] = Emm::TAIL;
    send(txBuf, 5);
}

static void sendReadPos() {
    txBuf[0] = Emm::ADDR;
    txBuf[1] = Emm::FD_READPOS;
    txBuf[2] = Emm::TAIL;
    send(txBuf, 3);
}

/// 絕對位置命令：[Addr][FD][dir][vel 2B][acc][pulses 4B][mode][sync][6B]
static void sendPosition(int32_t pulses) {
    uint32_t val = static_cast<uint32_t>(pulses);
    uint8_t i = 0;
    txBuf[i++] = Emm::ADDR;
    txBuf[i++] = Emm::FD_POS;
    txBuf[i++] = (pulses < 0) ? 0x01 : 0x00; // 方向位（部分固件忽略）
    txBuf[i++] = static_cast<uint8_t>(SPEED_RPM >> 8);
    txBuf[i++] = static_cast<uint8_t>(SPEED_RPM & 0xFF);
    txBuf[i++] = ACC_STEP;
    txBuf[i++] = static_cast<uint8_t>(val >> 24);
    txBuf[i++] = static_cast<uint8_t>(val >> 16);
    txBuf[i++] = static_cast<uint8_t>(val >> 8);
    txBuf[i++] = static_cast<uint8_t>(val);
    if(kModeByte) {
        txBuf[i++] = Emm::POS_MODE_ABS;
    }
    txBuf[i++] = 0x00; // 同步標誌
    txBuf[i++] = Emm::TAIL;
    send(txBuf, i);
}

/// 輸入 -97~97 → 脈衝（超出範圍限幅）
static int32_t inputToPulses(float input) {
    if(input > AxisConfig::INPUT_LIMIT)
        input = AxisConfig::INPUT_LIMIT;
    if(input < -AxisConfig::INPUT_LIMIT)
        input = -AxisConfig::INPUT_LIMIT;
    float deg = input * AxisConfig::INPUT_TO_DEG;
    return static_cast<int32_t>(roundf(deg * PULSES_PER_DEG));
}

/// 上電初始化：使能 + 目前位置清零（建立絕對零位）
void init() {
    sendEnable(true);
    sendClearPos();
    targetPulses = 0;
    lastSentPulses = 0;
    sentOnce = true;
    stopSent = false;
    lastPollUs = micros();
}

/// 設定目標角度（input -97~97）。Core 1 context，不阻塞。
void setTarget(float input) {
    int32_t pulses = inputToPulses(input);
    if(sentOnce && pulses == lastSentPulses) {
        return; // 目標沒變，不重送（省 UART 頻寬）
    }

    // 閉環驅動器若中途被擋住（未使能/堵轉），重新送一次使能
    if(!enabled) {
        sendEnable(true);
    }
    sendPosition(pulses);
    lastSentPulses = pulses;
    targetPulses = pulses;
    stopSent = false; // 有新目標 → 下次 E-Stop 要再送停止
}

/// 非阻塞輪詢即時位置（idleHook 呼叫）
void poll() {
    // 1) 把 UART 收到的位元組收集起來（不阻塞）
    while(elevSerial.available() > 0) {
        if(rxLen < sizeof(rxBuf)) {
            rxBuf[rxLen++] = static_cast<uint8_t>(elevSerial.read());
            rxByteCount++;
        } else {
            // 溢位：丟掉最舊的一半，重新同步
            memmove(rxBuf, rxBuf + sizeof(rxBuf) / 2, sizeof(rxBuf) / 2);
            rxLen = sizeof(rxBuf) / 2;
            elevSerial.read();
            rxByteCount++;
        }
    }

    // 2) 解析 [Addr][0x36][sign][pulses 4B][6B] = 8 bytes
    for(uint8_t i = 0; i + 8 <= rxLen; i++) {
        if(rxBuf[i] == Emm::ADDR && rxBuf[i + 1] == Emm::FD_READPOS &&
           rxBuf[i + 7] == Emm::TAIL) {
            uint32_t raw = (static_cast<uint32_t>(rxBuf[i + 3]) << 24) |
                           (static_cast<uint32_t>(rxBuf[i + 4]) << 16) |
                           (static_cast<uint32_t>(rxBuf[i + 5]) << 8) |
                           static_cast<uint32_t>(rxBuf[i + 6]);
            float deg = static_cast<float>(raw) * 360.0f / 65536.0f;
            reportedDeg = (rxBuf[i + 2] != 0) ? -deg : deg;
            // 診斷快照
            replyCount++;
            lastReplyRaw = static_cast<int32_t>(raw);
            lastReplySign = rxBuf[i + 2];
            for(uint8_t k = 0; k < 8; k++) {
                lastReplyHex[k] = rxBuf[i + k];
            }
            lastReplyHexValid = true;
            // 吃掉這 8 bytes
            memmove(rxBuf + i, rxBuf + i + 8, rxLen - (i + 8));
            rxLen = static_cast<uint8_t>(rxLen - 8);
            break;
        }
    }
    // 太舊的殘留資料直接清掉，避免卡住
    if(rxLen >= sizeof(rxBuf)) {
        rxLen = 0;
    }

    // 3) 定時送讀取命令
    uint32_t now = micros();
    if(now - lastPollUs >= POLL_INTERVAL_US) {
        lastPollUs = now;
        sendReadPos();
    }
}

/// E-Stop：立即停止（保持使能以便鎖住位置）
///
/// ⚠ 必須幂等：SlaveComms 在 E-Stop 期間會每個 spinloop 迭代都呼叫 onEstop()，
///   若每次都送停止帧，SerialPIO 的 TX FIFO（4 深）寫滿後 write() 就會阻塞，
///   Core 1 會被卡在 E-Stop 迴圈裡出不來。
void estop() {
    if(stopSent) {
        return;
    }
    sendStop();
    stopSent = true;
    // 讓下一次命令一定重送絕對位置（否則「目標沒變」會被去重而卡住不動）
    sentOnce = false;
}

/// 是否已送出 E-Stop 停止命令（診斷用）
bool isStopped() { return stopSent; }

/// 最近一次送出的目標脈衝數（遙測用）
int32_t commandedPulses() { return lastSentPulses; }

float reportedAngleDeg() { return reportedDeg; }

// —— 診斷用 accessor ——
uint32_t rxBytes() { return rxByteCount; }
uint32_t replies() { return replyCount; }
int32_t lastRaw() { return lastReplyRaw; }
uint8_t lastSign() { return lastReplySign; }
bool hasValidReplyFrame() { return lastReplyHexValid; }
/// 把最近一次 0x36 回覆的 8 bytes 印成 hex（無回覆時不印）
void printLastReplyHex() {
    if(!lastReplyHexValid) {
        return;
    }
    for(uint8_t i = 0; i < 8; i++) {
        Serial.printf("%02X ", static_cast<unsigned>(lastReplyHex[i]));
    }
}

/// 命令脈衝 → 角度（°），供「命令 vs 實測」對照
float commandedAngleDeg() {
    return static_cast<float>(lastSentPulses) * 360.0f / PULSES_PER_REV;
}

uint8_t deviceAddr() { return Emm::ADDR; }
} // namespace ElevStepper

// ============================================================================
// 水平步進：STEP 脈衝由 PIO 產生
//
// 架構：
//   PIO0 一個 SM：step_pulse.pio，固定 7 週期/步
//     f_step = clk_sys / (7 × clkdiv) → 以 clkdiv 調速與加減速
//   DIR / EN 由 CPU GPIO 控制（DIR 切換後等 20µs）
//
// 流程：
//   1. onCommand → setTarget()（只寫目標步數，不阻塞）
//   2. Core 1 idleHook → update()：
//        未動作且目標 ≠ 位置 → 設 DIR、下令 PIO 發 |diff| 個脈衝
//        動作中 → 依加速度調整 clkdiv；PIO irq 旗標 → 到位
//   3. E-Stop → abort()：停 SM、STEP 拉低、以估算步數修正位置
// ============================================================================
namespace RowStepper {
constexpr float STEP_PER_REV = 200.0f;                    // 1.8°/step（全步進）
constexpr float PULSE_PER_DEGREE = STEP_PER_REV / 360.0f; // ~0.556 step/deg
constexpr float SPEED_SPS = 600.0f;    // 到位過程最高脈衝速率
constexpr float ACCEL_SPS2 = 4000.0f;  // 加速度 steps/s²
constexpr float MIN_SPS = 200.0f;      // 起步/收尾速率
constexpr uint32_t DIR_SETTLE_US = 20; // DIR 建立時間
constexpr uint32_t RAMP_INTERVAL_US = 300;

static int32_t refSteps = 0;        // 目前位置估算（steps，0 = 上電位置）
static int32_t targetSteps = 0;     // 命令目標（steps）
static int32_t planTargetSteps = 0; // 本次動作的目標（動作中不變）
static float planSteps = 0.0f;      // 本次動作總步數（正值）
static float estSteps = 0.0f;       // 本次動作已發出步數估算
static float curSps = 0.0f;
static float peakSps = 0.0f;
static int8_t curDir = 1;
static bool moving = false;
static bool pioReady = false;
static uint32_t lastRampUs = 0;

static PIO sPio = pio0;
static uint sSm = 0;
static uint sOffset = 0;
constexpr uint kIrqFlag = 0;
constexpr float kCyclesPerStep = 7.0f; // set[3] 4 + set[1] 2 + jmp 1
static float sClkSysHz = 150000000.0f;

static float divForSps(float sps) {
    float div = sClkSysHz / (kCyclesPerStep * sps);
    if(div < 1.0f) {
        div = 1.0f;
    }
    if(div > 65536.0f) {
        div = 65536.0f;
    }
    return div;
}

/// 只把本 SM 的 STEP 腳位拉低。
/// ⚠ 不可用 pio_sm_set_pins()：SDK 會用「victim SM」把整個 PIO 區塊 32 支腳全部寫一次，
///   會把同一 PIO0 上仰角 UART（SerialPIO）的 TX(GP2) 一起拉低（UART 變 break 準位）。
static inline void stepPinLow() {
    if(!pioReady) {
        return;
    }
    pio_sm_exec(sPio, sSm, pio_encode_set(pio_pins, 0));
}

static int32_t inputToSteps(float input) {
    if(input > AxisConfig::INPUT_LIMIT)
        input = AxisConfig::INPUT_LIMIT;
    if(input < -AxisConfig::INPUT_LIMIT)
        input = -AxisConfig::INPUT_LIMIT;
    float deg = input * AxisConfig::INPUT_TO_DEG;
    return static_cast<int32_t>(roundf(deg * PULSE_PER_DEGREE));
}

void init() {
    sClkSysHz = static_cast<float>(clock_get_hz(clk_sys));

    gpio_init(Pin::ROW_DIR);
    gpio_set_dir(Pin::ROW_DIR, GPIO_OUT);
    gpio_put(Pin::ROW_DIR, 0);

    gpio_init(Pin::ROW_EN);
    gpio_set_dir(Pin::ROW_EN, GPIO_OUT);
    gpio_put(Pin::ROW_EN, 0); // 致能（低電位）

    int smIdx = pio_claim_unused_sm(sPio, false);
    if(smIdx < 0) {
        pioReady = false;
        return;
    }
    sSm = static_cast<uint>(smIdx);

    sOffset = pio_add_program(sPio, &step_pulse_program);
    pio_sm_config cfg = step_pulse_program_get_default_config(sOffset);
    sm_config_set_set_pins(&cfg, Pin::ROW_STEP, 1);
    pio_gpio_init(sPio, Pin::ROW_STEP);
    pio_sm_set_consecutive_pindirs(sPio, sSm, Pin::ROW_STEP, 1, true);

    pio_sm_init(sPio, sSm, sOffset, &cfg);
    pio_sm_set_enabled(sPio, sSm, true);
    pioReady = true;
    stepPinLow();
}

static void haltPulses() {
    if(!pioReady) {
        return;
    }
    pio_sm_set_enabled(sPio, sSm, false);
    pio_sm_clear_fifos(sPio, sSm);
    stepPinLow();
    pio_interrupt_clear(sPio, kIrqFlag);
}

static void startSegment(int32_t steps) {
    curDir = (steps > 0) ? 1 : -1;
    gpio_put(Pin::ROW_DIR, (curDir > 0) ? 1 : 0);
    delayMicroseconds(DIR_SETTLE_US);

    planTargetSteps = targetSteps;
    planSteps = static_cast<float>(steps > 0 ? steps : -steps);
    estSteps = 0.0f;

    peakSps = SPEED_SPS;
    float triPeak = sqrtf(2.0f * ACCEL_SPS2 * planSteps);
    if(triPeak < peakSps) {
        peakSps = (triPeak < MIN_SPS) ? MIN_SPS : triPeak;
    }
    curSps = MIN_SPS;
    lastRampUs = micros();
    moving = true;

    if(pioReady) {
        pio_sm_set_enabled(sPio, sSm, false);
        pio_sm_clear_fifos(sPio, sSm);
        pio_sm_restart(sPio, sSm);
        pio_sm_exec(sPio, sSm, pio_encode_jmp(sOffset));
        pio_interrupt_clear(sPio, kIrqFlag);
        pio_sm_set_clkdiv(sPio, sSm, divForSps(curSps));
        pio_sm_put(sPio, sSm, static_cast<uint32_t>(steps > 0 ? steps : -steps));
        pio_sm_set_enabled(sPio, sSm, true);
    }
}

/// 設定目標（input -97~97）。Core 1 context。
void setTarget(float input) { targetSteps = inputToSteps(input); }

/// idleHook：推進水平軸
void update() {
    if(!moving) {
        int32_t diff = targetSteps - refSteps;
        if(diff == 0) {
            return; // 已到位，零開銷
        }
        startSegment(diff);
        return;
    }

    if(pioReady && pio_interrupt_get(sPio, kIrqFlag)) {
        pio_interrupt_clear(sPio, kIrqFlag);
        moving = false;
        refSteps = planTargetSteps;
        curSps = 0.0f;
        return;
    }

    uint32_t now = micros();
    uint32_t dtUs = now - lastRampUs;
    if(dtUs < RAMP_INTERVAL_US) {
        return;
    }
    lastRampUs = now;
    estSteps += curSps * static_cast<float>(dtUs) * 1e-6f;

    float destSps = peakSps;
    float left = planSteps - estSteps;
    float decelDist = (curSps * curSps) / (2.0f * ACCEL_SPS2);
    if(left <= decelDist + 1.0f) {
        destSps = MIN_SPS;
    }
    float dv = ACCEL_SPS2 * static_cast<float>(dtUs) * 1e-6f;
    if(curSps < destSps) {
        curSps += dv;
        if(curSps > destSps) {
            curSps = destSps;
        }
    } else if(curSps > destSps) {
        curSps -= dv;
        if(curSps < destSps) {
            curSps = destSps;
        }
    }
    if(pioReady) {
        pio_sm_set_clkdiv(sPio, sSm, divForSps(curSps));
    }
}

/// E-Stop：立即停止脈衝；保留命令目標，E-Stop 解除後自動補回原位
void estop() {
    haltPulses();
    if(moving) {
        refSteps += (curDir > 0 ? 1 : -1) * static_cast<int32_t>(estSteps + 0.5f);
    }
    moving = false;
    curSps = 0.0f;
    estSteps = 0.0f;
}

int32_t positionSteps() { return moving ? refSteps : targetSteps; }
} // namespace RowStepper

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
    if(std::isnan(persistentUpSpeed))
        persistentUpSpeed = 0.0f;
    if(std::isnan(persistentDownSpeed))
        persistentDownSpeed = 0.0f;
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
// 為什麼用 PIO1：PIO0 的 4 個 SM 已被「水平軸脈衝 1 + 仰角 UART 2」佔用。
// RP2350 的 PIO1 預設 GPIO base = 16，要驅動 GP4/5/9 必須先 pio_set_gpio_base(pio1, 0)
// （只能在該 PIO 尚未載入任何程式時呼叫）。
//
// 一個通道一個 SM，同一份 servo_pulse.pio（共用 offset，只佔 8 條指令）：
//   clkdiv = clk_sys / 1MHz → 1 週期 = 1µs
//   CPU 每幀 push 2 個值（高電位週期、低電位週期），TX FIFO 深度 4 = 2 幀緩衝
//
// 餵資料放在 Core 0 的 loop()：E-Stop 期間 Core 1 的 idleHook 不再執行，
// 但 ESC 必須繼續收到 1500µs 中立訊號才不會亂轉，所以由 Core 0 持續補 FIFO。
// 跨核心傳遞用 32-bit volatile（單一字組寫入即 atomic，不需鎖）。
// ============================================================================
namespace BldcControl {
constexpr uint32_t FRAME_US = 20000;   // 50Hz
constexpr uint16_t PULSE_MID = 1500;   // 中立點/停止
constexpr uint16_t PULSE_MAX = 2000;   // 全速
constexpr uint16_t PULSE_FIXED = 1750; // 固定轉速無刷

// PIO 程式的固定額外週期（見 servo_pulse.pio 週期註解）
constexpr int32_t HIGH_OVERHEAD_US = 3;  // set pins,1 + pull + mov
constexpr int32_t FRAME_OVERHEAD_US = 6; // 高+低兩段的 set/pull/mov

enum Channel : uint8_t { CH_UP = 0, CH_DOWN = 1, CH_FIXED = 2, CH_COUNT = 3 };

static PIO sPio = pio1;
static uint sSm[CH_COUNT] = {0, 0, 0};
static uint sOffset = 0;
static bool sReady = false;

// Core 1 寫、Core 0 讀（32-bit 單字組 → atomic）
static volatile uint32_t sPulseUs[CH_COUNT] = {PULSE_MID, PULSE_MID, PULSE_FIXED};

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
    if(!Pin::kEnableBldc) {
        return;
    }
    const uint pins[CH_COUNT] = {Pin::UP_BLDC_PWM, Pin::DOWN_BLDC_PWM, Pin::FIXED_BLDC_PWM};

    // 必須在任何程式載入 PIO1 之前設定 GPIO base
    pio_set_gpio_base(sPio, 0);
    sOffset = pio_add_program(sPio, &servo_pulse_program);
    pio_sm_config base = servo_pulse_program_get_default_config(sOffset);
    sm_config_set_clkdiv(&base, static_cast<float>(clock_get_hz(clk_sys)) / 1000000.0f);

    for(uint8_t i = 0; i < CH_COUNT; i++) {
        int sm = pio_claim_unused_sm(sPio, false);
        if(sm < 0) {
            sReady = false; // PIO1 SM 不足（理論上不會：共 4 個，只用 3）
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
    sPulseUs[CH_UP] = speedToPulse(speed);
}
void setDownSpeed(float speed) {
    if(!Pin::kEnableBldc)
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
static volatile int8_t dbg_lastRow = 0;
static volatile int8_t dbg_lastUp = 0;
static volatile int8_t dbg_lastDown = 0;

// 把 float 以「整數 + 一位小數」印出來
// （newlib-nano 的 printf 沒開浮點（_printf_float），%f/%.1f 會直接印不出東西）
static void printFixed1(float value) {
    int32_t v10 = static_cast<int32_t>(lroundf(value * 10.0f));
    const char *sign = (v10 < 0) ? "-" : "";
    if(v10 < 0) {
        v10 = -v10;
    }
    Serial.printf("%s%ld.%ld", sign, static_cast<long>(v10 / 10),
                  static_cast<long>(v10 % 10));
}

// E-Stop 期間仍要告訴 Hub「我被停了」（遙測只在收到命令時才組，所以另外補送）
constexpr uint32_t ESTOP_TM_INTERVAL_MS = 200;

/// 組遙測帧（Core 1 context）
static void buildTelemetry(uint8_t estopFlag) {
    pendingTm = {};
    pendingTm.sync = 0xA5;
    pendingTm.msgType = MsgType::TELEMETRY;
    pendingTm.frameSeq = tmFrameSeq++;
    pendingTm.targetID = TargetID::HUB;
    // Shoot 遙測：復用 float 陣列
    // currentRps[0] = 上輪速度, [1] = 下輪速度
    // currentRps[2] = 仰角實測角度(°), [3] = 水平目前步數
    pendingTm.currentRps[0] = SpeedStorage::getUpSpeed();
    pendingTm.currentRps[1] = SpeedStorage::getDownSpeed();
    pendingTm.currentRps[2] = ElevStepper::reportedAngleDeg();
    pendingTm.currentRps[3] = static_cast<float>(RowStepper::positionSteps());
    pendingTm.targetRps[0] = SpeedStorage::getUpSpeed();
    pendingTm.targetRps[1] = SpeedStorage::getDownSpeed();
    pendingTm.targetRps[2] = ElevStepper::commandedPulses();
    pendingTm.targetRps[3] = static_cast<float>(RowStepper::positionSteps());
    pendingTm.mode = RobotMode::AUTO;
    pendingTm.estopFlag = estopFlag;
    pendingTm.timestamp = time_us_64();
    pendingTm.crc16 = 0;
    pendingTm.crc16 =
        SignalUtils::calculateCrc(reinterpret_cast<const uint8_t *>(&pendingTm),
                                  sizeof(MoveTelemetry) - 2);
}

static void onShootCommand(const ChassisSnapshot &cmd) {
    dbg_cmdCount++;
    dbg_lastPitch = static_cast<int8_t>(lroundf(cmd.pitch));
    dbg_lastRow = static_cast<int8_t>(lroundf(cmd.row));
    dbg_lastUp = static_cast<int8_t>(lroundf(cmd.shootUpSpeed));
    dbg_lastDown = static_cast<int8_t>(lroundf(cmd.shootDownSpeed));

    // 永久性速度：每次命令都储存並套用（包括速度=0 的清零指令）
    SpeedStorage::saveUpSpeed(cmd.shootUpSpeed);
    SpeedStorage::saveDownSpeed(cmd.shootDownSpeed);
    BldcControl::setUpSpeed(cmd.shootUpSpeed);
    BldcControl::setDownSpeed(cmd.shootDownSpeed);

    // 兩軸步進目標
    ElevStepper::setTarget(cmd.pitch); // 仰角：UART 閉環步進
    RowStepper::setTarget(cmd.row);    // 水平：STEP/DIR 脈衝

    // 遙測（收到有效帧 = E-Stop 已解除）
    buildTelemetry(0);
}

// 前向宣告：E-Stop 遙測（定義在 slave 之後）
static void pushEstopTelemetry();

static void onShootEstop() {
    dbg_estopCount++;

    // BLDC：只改快取值（Core 0 會在 loop() 繼續餵 1500µs 中立訊號）
    BldcControl::estop();

    // 步進：只由 Core 1 操作 PIO / UART（避免與 idleHook 同時改 PIO 暫存器）
    if(get_core_num() != 1) {
        // Core 0 觸發（指令逾時）時什麼都不做：
        // runCore1() 的 E-Stop 分支每個迭代都會在 Core 1 再呼叫一次 onEstop()
        return;
    }
    ElevStepper::estop();
    RowStepper::estop();

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
    RowStepper::update();  // 水平：PIO 脈衝 + 加減速
    ElevStepper::poll();   // 仰角：非阻塞收位置回覆 + 定時讀取
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
    static LedState lastState = LedState::BOOT;
    static bool ledOn = false;
    static uint32_t phaseStartMs = 0;

    const LedState st = currentLedState();
    const uint32_t now = millis();

    dbg_ledState = static_cast<uint8_t>(st);

    // 狀態切換 → 重置相位並立刻點亮（新狀態立即可見）
    if(st != lastState) {
        lastState = st;
        phaseStartMs = now;
        ledOn = true;
        digitalWrite(LED_BUILTIN, HIGH);
    }

    // 未連線 / 剛開機 → 長亮
    if(st == LedState::BOOT) {
        digitalWrite(LED_BUILTIN, HIGH);
        return;
    }

    uint32_t onMs = LedCfg::ACTIVE_ON_MS;
    uint32_t offMs = LedCfg::ACTIVE_OFF_MS;
    if(st == LedState::IDLE) {
        onMs = LedCfg::IDLE_ON_MS;
        offMs = LedCfg::IDLE_OFF_MS;
    } else if(st == LedState::ESTOP) {
        onMs = LedCfg::ESTOP_ON_MS;
        offMs = LedCfg::ESTOP_OFF_MS;
    }

    // 非阻塞相位推進：亮滿 onMs → 焭，焭滿 offMs → 亮
    const uint32_t phaseMs = ledOn ? onMs : offMs;
    if(now - phaseStartMs >= phaseMs) {
        phaseStartMs = now;
        ledOn = !ledOn;
        digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
    }
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
    // 再取用水平軸的 PIO SM，最後啟動仰角 UART（同為 PIO0，先取先贏）
    RowStepper::init();
    elevSerial.begin(115200); // PIO 模擬 UART → 仰角步進驅動器

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // 開機先亮（BOOT 狀態）

    Serial.begin(115200);
    delay(200);
    Serial.println("[SHOOT] boot: 仰角=UART步進(GP2/3) 水平=STEP/DIR/EN(GP6/7/8)");

    slave.begin(); // 硬體 UART0 (GP0/GP1) → Hub
    slave.setIdleHook(shootIdleHook);

    Serial.println("[SHOOT] 等待 Hub 命令（100ms 無帧 → E-Stop）");
}

void loop() {
    slave.update();
    BldcControl::update(); // Core 0 持續餵 50Hz 伺服脈衝（E-Stop 期間也保持 1500µs）
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
                          ElevStepper::isStopped() ? 1 : 0);
        } else {
            const int8_t upSpeed = static_cast<int8_t>(lroundf(SpeedStorage::getUpSpeed()));
            const int8_t downSpeed =
                static_cast<int8_t>(lroundf(SpeedStorage::getDownSpeed()));
            Serial.printf("[SHOOT] sinceCmd=%ums cnt=%u led=%s | cmd pitch=%d row=%d | ",
                          since, static_cast<unsigned>(dbg_cmdCount),
                          ledStateName(static_cast<LedState>(dbg_ledState)),
                          static_cast<int>(dbg_lastPitch),
                          static_cast<int>(dbg_lastRow));
            Serial.print("仰角 ");
            printFixed1(ElevStepper::reportedAngleDeg());
            Serial.print("° (cmd ");
            printFixed1(ElevStepper::commandedAngleDeg());
            Serial.printf("°) 水平 %ld 步 | up=%d down=%d | rx=%u crc=%u | elev rep=%u frx=%u\n",
                          static_cast<long>(RowStepper::positionSteps()),
                          static_cast<int>(upSpeed), static_cast<int>(downSpeed),
                          static_cast<unsigned>(slave.rxFrameCount()),
                          static_cast<unsigned>(slave.rxCrcErrorCount()),
                          static_cast<unsigned>(ElevStepper::replies()),
                          static_cast<unsigned>(ElevStepper::rxBytes()));
        }
    }

    // 仰角驅動器（0x36 回覆）診斷 — 每秒一次
    {
        static uint32_t lastElevDbgMs = 0;
        if(nowMs - lastElevDbgMs >= 1000) {
            lastElevDbgMs = nowMs;
            if(ElevStepper::replies() > 0) {
                Serial.print("[SHOOT] elev reply: ");
                ElevStepper::printLastReplyHex();
                Serial.printf("| raw=%ld sign=%u\n",
                              static_cast<long>(ElevStepper::lastRaw()),
                              static_cast<unsigned>(ElevStepper::lastSign()));
            } else if(slave.validCmdCount() > 0 && nowMs > 5000) {
                Serial.printf(
                    "[SHOOT] !! 仰角驅動器 0x36 完全無回覆（frx=%u）→ 檢查："
                    "馬達 TX→GP3、GP2→馬達 R/A、共地、P_Serial=UART、"
                    "位址=%u、baud=115200\n",
                    static_cast<unsigned>(ElevStepper::rxBytes()),
                    static_cast<unsigned>(ElevStepper::deviceAddr()));
            }
        }
    }
}

// ============================================================================
// Core 1
// ============================================================================
void setup1() {
    ElevStepper::init(); // 使能 + 位置清零（建立絕對零位）
    SpeedStorage::init();

    // 套用保留的速度值（固定轉速無刷已在 BldcControl::init() 中設定為 1750µs）
    BldcControl::setUpSpeed(SpeedStorage::getUpSpeed());
    BldcControl::setDownSpeed(SpeedStorage::getDownSpeed());
}

void loop1() { slave.runCore1(); }
