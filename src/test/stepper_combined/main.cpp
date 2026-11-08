// ============================================================================
// stepper_combined_test — 兩顆步進馬達綜合測試（RP2350）
//
// 馬達 A：UART 閉環步進（ZDT X42S / Emm 固件）
//   TX = GP2 → 馬達 R/A/H
//   RX = GP3 ← 馬達 T/B/L
//   115200 baud，PIO 模擬 UART（SerialPIO），不用硬體 UART
//
// 馬達 B：一般 EN / STEP / DIR 驅動器（A4988 / DRV8825 / TB6600 / DM542）
//   STEP = GP6（PIO0 產生脈衝，固定 7 週期/步）
//   DIR  = GP7
//   EN   = GP8（預設低電位致能）
//   兩顆的 GND 要與 RP2350 共地
//
// 兩顆馬達的 GND 要與 RP2350 共地；EN 極性、DIR 方向、每圈脈衝數都可調。
//
// Serial 命令（115200，行結尾 Enter）：
//   h / ?               說明
//   i                   狀態（兩軸）
//   e / d               兩軸使能 / 去使能
//   t                   自我測試（UART +90° → -90° → 0°，水平 +1圈 → -1圈 → 0）
//
//   UART 馬達（A）：
//     ue / ud           UART 馬達使能 / 去使能
//     r                 讀即時位置（0x36）
//     z                 把目前位置清零（0x0A / 0x6D）
//     p <deg>           絕對角度（相對開機零點）
//     g <deg>           相對移動（以目前命令位置為基準）
//     s                 UART 馬達立即停止
//     v <rpm>           UART 馬達速度（預設 600）
//     k <0~255>         UART 馬達加速度檔位（預設 100）
//     F                 切換位置幀格式（13B 第二代 / 12B 舊 Emm）
//
//   EN/STEP/DIR 馬達（B）：
//     a <steps>         絕對步數（0 = 開機位置）
//     f <steps>         正轉相對步數
//     b <steps>         反轉相對步數
//     S                 水平立即停止
//     V <rpm>           水平轉速（預設 120）
//     K <steps/s²>      水平加速度（預設 50000，0 = 不加減速）
//     c <steps/rev>     水平每圈脈衝數（預設 3200 = 16 細分）
//     n                 水平 EN 極性切換（預設低電位致能）
//     D                 水平 DIR 方向反向切換
// ============================================================================
#include "step_pulse.pio.h"
#include <Arduino.h>
#include <SerialPIO.h>
#include <cmath>
#include <initializer_list>
#include <hardware/clocks.h>
#include <hardware/pio.h>
#include <pico/stdlib.h>
#include <stdlib.h>
#include <string.h>

// ============================================================================
// 腳位
// ============================================================================
namespace Pin {
constexpr uint UART_TX = 0; // → 馬達 R/A/H
constexpr uint UART_RX = 1; // ← 馬達 T/B/L
constexpr uint STEP = 2;    // → 驅動器 PUL / STEP（PIO0）
constexpr uint DIR = 3;     // → 驅動器 DIR
constexpr uint EN = 4;      // → 驅動器 ENA / EN
} // namespace Pin

// ============================================================================
// 馬達 A：UART 閉環步進（Emm / ZDT X42S）
// ============================================================================
namespace UartMotor {
namespace Emm {
constexpr uint8_t ADDR = 0x01;
constexpr uint8_t TAIL = 0x6B;
constexpr uint8_t FD_EN = 0xF3;
constexpr uint8_t FD_POS = 0xFD;
constexpr uint8_t FD_STOP = 0xFE;
constexpr uint8_t FD_READPOS = 0x36;
constexpr uint8_t FD_CLEARPOS = 0x0A;
constexpr uint8_t AX_EN = 0xAB;
constexpr uint8_t AX_STOP = 0x98;
constexpr uint8_t AX_CLEARPOS = 0x6D;
constexpr uint8_t POS_MODE_ABS = 0x01;
} // namespace Emm

// 命令單位：3200 脈衝/圈（1.8° × 16 細分）
constexpr float PULSES_PER_REV = 3200.0f;
// 0x36 回覆單位：65536/圈
constexpr float REPLY_UNITS_PER_REV = 65536.0f;

static SerialPIO serial(Pin::UART_TX, Pin::UART_RX, 128);

static uint8_t txBuf[24];
static uint8_t rxBuf[48];
static uint8_t rxLen = 0;

static uint16_t speedRpm = 600;
static uint8_t accStep = 100;
static bool modeByte = true; // true = 13B（第二代）, false = 12B（舊 Emm）

static bool enabled = false;
static bool stopSent = false;
static int32_t lastSentPulses = 0;
static int32_t reportedRaw = 0;
static float reportedDeg = 0.0f;
static uint32_t replyCount = 0;
static uint32_t rxBytes = 0;
static uint32_t lastPollUs = 0;
static constexpr uint32_t POLL_INTERVAL_US = 100000; // 100ms 讀一次
static bool dumpRaw = false; // 'Q' 切換：把 TX/RX 位元組印成 hex

static void send(const uint8_t *buf, uint8_t len) {
    if(dumpRaw) {
        Serial.print("[UART TX] ");
        for(uint8_t i = 0; i < len; i++)
            Serial.printf("%02X ", buf[i]);
        Serial.println();
    }
    serial.write(buf, len);
}

static void sendEnable(bool en) {
    txBuf[0] = Emm::ADDR;
    txBuf[1] = Emm::FD_EN;
    txBuf[2] = Emm::AX_EN;
    txBuf[3] = en ? 0x01 : 0x00;
    txBuf[4] = 0x00;
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
    lastSentPulses = 0;
    reportedRaw = 0;
    reportedDeg = 0.0f;
}

static void sendStop() {
    txBuf[0] = Emm::ADDR;
    txBuf[1] = Emm::FD_STOP;
    txBuf[2] = Emm::AX_STOP;
    txBuf[3] = 0x00;
    txBuf[4] = Emm::TAIL;
    send(txBuf, 5);
    stopSent = true;
}

static void sendReadPos() {
    txBuf[0] = Emm::ADDR;
    txBuf[1] = Emm::FD_READPOS;
    txBuf[2] = Emm::TAIL;
    send(txBuf, 3);
}

/// 絕對位置命令：[Addr][FD][dir][vel 2B][acc][pulses 4B][mode][sync][6B]
static void sendPosition(int32_t pulses) {
    const uint32_t val = static_cast<uint32_t>(pulses);
    uint8_t i = 0;
    txBuf[i++] = Emm::ADDR;
    txBuf[i++] = Emm::FD_POS;
    txBuf[i++] = (pulses < 0) ? 0x01 : 0x00;
    txBuf[i++] = static_cast<uint8_t>(speedRpm >> 8);
    txBuf[i++] = static_cast<uint8_t>(speedRpm & 0xFF);
    txBuf[i++] = accStep;
    txBuf[i++] = static_cast<uint8_t>(val >> 24);
    txBuf[i++] = static_cast<uint8_t>(val >> 16);
    txBuf[i++] = static_cast<uint8_t>(val >> 8);
    txBuf[i++] = static_cast<uint8_t>(val);
    if(modeByte)
        txBuf[i++] = Emm::POS_MODE_ABS;
    txBuf[i++] = 0x00;
    txBuf[i++] = Emm::TAIL;
    send(txBuf, i);
    lastSentPulses = pulses;
    stopSent = false;
}

static int32_t degToPulses(float deg) {
    return static_cast<int32_t>(roundf(deg * PULSES_PER_REV / 360.0f));
}

static float pulsesToDeg(int32_t pulses) {
    return static_cast<float>(pulses) * 360.0f / PULSES_PER_REV;
}

static void moveAbsoluteDeg(float deg) {
    if(!enabled)
        sendEnable(true);
    sendPosition(degToPulses(deg));
}

static void moveRelativeDeg(float deltaDeg) {
    moveAbsoluteDeg(pulsesToDeg(lastSentPulses) + deltaDeg);
}

static void init() {
    serial.begin(115200);
    delay(50);
    sendEnable(true);
    delay(20);
    sendClearPos(); // 開機位置 = 0
    lastSentPulses = 0;
    stopSent = false;
    lastPollUs = micros();
}

/// 非阻塞收 0x36 回覆：raw 為 65536/圈
static void poll() {
    while(serial.available() > 0) {
        const uint8_t b = static_cast<uint8_t>(serial.read());
        if(dumpRaw)
            Serial.printf("[UART RX] %02X\n", b);
        if(rxLen < sizeof(rxBuf)) {
            rxBuf[rxLen++] = b;
            rxBytes++;
        } else {
            rxBytes++;
            rxLen = 0;
        }
    }

    for(uint8_t i = 0; i + 8 <= rxLen; i++) {
        if(rxBuf[i] == Emm::ADDR && rxBuf[i + 1] == Emm::FD_READPOS &&
           rxBuf[i + 7] == Emm::TAIL) {
            const uint32_t raw = (static_cast<uint32_t>(rxBuf[i + 3]) << 24) |
                                 (static_cast<uint32_t>(rxBuf[i + 4]) << 16) |
                                 (static_cast<uint32_t>(rxBuf[i + 5]) << 8) |
                                 static_cast<uint32_t>(rxBuf[i + 6]);
            reportedRaw = static_cast<int32_t>(raw);
            const float deg = static_cast<float>(raw) * 360.0f /
                              REPLY_UNITS_PER_REV;
            reportedDeg = (rxBuf[i + 2] != 0) ? -deg : deg;
            replyCount++;
            memmove(rxBuf + i, rxBuf + i + 8, rxLen - (i + 8));
            rxLen = static_cast<uint8_t>(rxLen - 8);
            break;
        }
    }
    if(rxLen >= sizeof(rxBuf))
        rxLen = 0;

    const uint32_t now = micros();
    if(now - lastPollUs >= POLL_INTERVAL_US) {
        lastPollUs = now;
        sendReadPos();
    }
}

static void printStatus() {
    Serial.printf(
        "[UART] en=%d fmt=%s rpm=%u acc=%u | cmd=%ld p (%+.2f°) | "
        "reported=%+.2f° (raw=%ld) replies=%lu rxB=%lu\n",
        enabled ? 1 : 0, modeByte ? "13B" : "12B", static_cast<unsigned>(speedRpm),
        static_cast<unsigned>(accStep), static_cast<long>(lastSentPulses),
        static_cast<double>(pulsesToDeg(lastSentPulses)),
        static_cast<double>(reportedDeg), static_cast<long>(reportedRaw),
        static_cast<unsigned long>(replyCount), static_cast<unsigned long>(rxBytes));
}
} // namespace UartMotor

// ============================================================================
// 馬達 B：EN / STEP / DIR（PIO0 產生 STEP 脈衝）
// ============================================================================
namespace StepDirMotor {
constexpr uint kIrqFlag = 4; // PIO IRQ flag 4 → PIO IRQ1，避開 SerialPIO 的 IRQ0
constexpr float kCyclesPerStep = 7.0f; // set[3] + set[1] + jmp
constexpr float MIN_SPS = 330.0f;      // divider 上限對應的最低速率
constexpr float MAX_SPS = 150000.0f;

static PIO sPio = pio0;
static uint sSm = 0;
static uint sOffset = 0;
static bool pioReady = false;
static float sClkSysHz = 150000000.0f;

static bool enActiveLow = true;
static bool dirInvert = false;
static float stepsPerRev = 3200.0f; // 1.8° × 16 細分
static float speedRpm = 120.0f;
static float accelSps2 = 50000.0f;

static bool enabled = false;
static bool moving = false;
static int32_t refSteps = 0;
static int32_t targetSteps = 0;
static int32_t planTargetSteps = 0;
static float planSteps = 0.0f;
static float estSteps = 0.0f;
static float curSps = 0.0f;
static float peakSps = 0.0f;
static int8_t curDir = 1;
static uint32_t lastRampUs = 0;

static float divForSps(float sps) {
    float div = sClkSysHz / (kCyclesPerStep * sps);
    if(div < 1.0f)
        div = 1.0f;
    if(div > 65536.0f)
        div = 65536.0f;
    return div;
}

static float clampSps(float sps) {
    if(sps < MIN_SPS)
        return MIN_SPS;
    if(sps > MAX_SPS)
        return MAX_SPS;
    return sps;
}

static float rpmToSps(float rpm) { return rpm * stepsPerRev / 60.0f; }

static inline void stepPinLow() {
    if(pioReady)
        pio_sm_exec(sPio, sSm, pio_encode_set(pio_pins, 0));
}

static void applyEnablePin() {
    const bool level = enabled ? !enActiveLow : enActiveLow;
    gpio_put(Pin::EN, level ? 1 : 0);
}

static void applyDir(int8_t dir) {
    bool level = (dir > 0);
    if(dirInvert)
        level = !level;
    gpio_put(Pin::DIR, level ? 1 : 0);
    delayMicroseconds(20);
}

static void haltPulses() {
    if(!pioReady)
        return;
    pio_sm_set_enabled(sPio, sSm, false);
    pio_sm_clear_fifos(sPio, sSm);
    stepPinLow();
    pio_interrupt_clear(sPio, kIrqFlag);
}

static void init() {
    sClkSysHz = static_cast<float>(clock_get_hz(clk_sys));

    gpio_init(Pin::DIR);
    gpio_set_dir(Pin::DIR, GPIO_OUT);
    gpio_put(Pin::DIR, 0);
    gpio_init(Pin::EN);
    gpio_set_dir(Pin::EN, GPIO_OUT);
    gpio_put(Pin::EN, 1); // 先不使能
    enabled = false;

    const int sm = pio_claim_unused_sm(sPio, false);
    if(sm < 0) {
        pioReady = false;
        Serial.println("[STEP] PIO SM 取得失敗");
        return;
    }
    sSm = static_cast<uint>(sm);
    sOffset = pio_add_program(sPio, &step_pulse_program);
    pio_sm_config cfg = step_pulse_program_get_default_config(sOffset);
    sm_config_set_set_pins(&cfg, Pin::STEP, 1);
    pio_gpio_init(sPio, Pin::STEP);
    pio_sm_set_consecutive_pindirs(sPio, sSm, Pin::STEP, 1, true);
    pio_sm_init(sPio, sSm, sOffset, &cfg);
    pio_sm_set_enabled(sPio, sSm, true);
    pioReady = true;
    stepPinLow();
    pio_interrupt_clear(sPio, kIrqFlag);
    Serial.printf("[STEP] PIO0 SM%u 就緒，STEP=GP%u DIR=GP%u EN=GP%u\n",
                  static_cast<unsigned>(sSm), Pin::STEP, Pin::DIR, Pin::EN);
}

static void setEnabled(bool en) {
    enabled = en;
    applyEnablePin();
    if(!en)
        haltPulses();
    Serial.printf("[STEP] %s（EN 準位=%d，低電位致能=%d）\n",
                  en ? "使能" : "去使能", gpio_get(Pin::EN),
                  enActiveLow ? 1 : 0);
}

static void startSegment(int32_t steps) {
    curDir = (steps > 0) ? 1 : -1;
    applyDir(curDir);

    planTargetSteps = targetSteps;
    planSteps = static_cast<float>(steps > 0 ? steps : -steps);
    estSteps = 0.0f;

    peakSps = clampSps(rpmToSps(speedRpm));
    if(accelSps2 > 0.0f) {
        const float triPeak = sqrtf(2.0f * accelSps2 * planSteps);
        if(triPeak < peakSps)
            peakSps = (triPeak < MIN_SPS) ? MIN_SPS : triPeak;
    }
    curSps = (accelSps2 > 0.0f) ? MIN_SPS : peakSps;
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

static void stop(const char *why) {
    const bool was = moving;
    haltPulses();
    if(moving)
        refSteps += (curDir > 0 ? 1 : -1) * static_cast<int32_t>(estSteps + 0.5f);
    moving = false;
    estSteps = 0.0f;
    curSps = 0.0f;
    targetSteps = refSteps; // 取消尚未走完的目標，避免 update() 立刻又啟動
    if(was)
        Serial.printf("[STEP] 停止（%s），目前約 %ld 步\n", why,
                      static_cast<long>(refSteps));
}

static void update() {
    if(!moving) {
        const int32_t diff = targetSteps - refSteps;
        if(diff != 0 && enabled && pioReady)
            startSegment(diff);
        return;
    }

    if(pioReady && pio_interrupt_get(sPio, kIrqFlag)) {
        pio_interrupt_clear(sPio, kIrqFlag);
        moving = false;
        refSteps = planTargetSteps;
        curSps = 0.0f;
        Serial.printf("[STEP] 到位：%ld 步（%+.1f°）\n",
                      static_cast<long>(refSteps),
                      static_cast<double>(static_cast<float>(refSteps) * 360.0f /
                                          stepsPerRev));
        return;
    }

    const uint32_t now = micros();
    const uint32_t dtUs = now - lastRampUs;
    if(dtUs < 300)
        return;
    lastRampUs = now;
    estSteps += curSps * static_cast<float>(dtUs) * 1e-6f;

    float dest = peakSps;
    if(accelSps2 > 0.0f) {
        const float left = planSteps - estSteps;
        const float decelDist = (curSps * curSps) / (2.0f * accelSps2);
        if(left <= decelDist + 1.0f)
            dest = MIN_SPS;
        const float dv = accelSps2 * static_cast<float>(dtUs) * 1e-6f;
        if(curSps < dest)
            curSps = (curSps + dv > dest) ? dest : curSps + dv;
        else if(curSps > dest)
            curSps = (curSps - dv < dest) ? dest : curSps - dv;
    } else {
        curSps = dest;
    }
    if(pioReady)
        pio_sm_set_clkdiv(sPio, sSm, divForSps(clampSps(curSps)));
}

static void printStatus() {
    Serial.printf("[STEP] en=%d polar=%s dirInv=%s spr=%.0f rpm=%.0f acc=%.0f | "
                  "ref=%ld p (%+.1f°) target=%ld moving=%d\n",
                  enabled ? 1 : 0, enActiveLow ? "low" : "high",
                  dirInvert ? "yes" : "no", static_cast<double>(stepsPerRev),
                  static_cast<double>(speedRpm),
                  static_cast<double>(accelSps2), static_cast<long>(refSteps),
                  static_cast<double>(static_cast<float>(refSteps) * 360.0f /
                                      stepsPerRev),
                  static_cast<long>(targetSteps), moving ? 1 : 0);
}
} // namespace StepDirMotor

// ============================================================================
// 命令列
// ============================================================================
static bool parseFloatArg(const char *s, float *out) {
    if(s == nullptr || *s == '\0')
        return false;
    char *end = nullptr;
    const float v = strtof(s, &end);
    if(end == s)
        return false;
    *out = v;
    return true;
}

static bool parseLongArg(const char *s, long *out) {
    if(s == nullptr || *s == '\0')
        return false;
    char *end = nullptr;
    const long v = strtol(s, &end, 0);
    if(end == s)
        return false;
    *out = v;
    return true;
}

static void printHelp() {
    Serial.print(
        "\n=== stepper_combined_test ===\n"
        "  h / ?             本說明\n"
        "  i                 狀態（兩軸）\n"
        "  e / d             兩軸使能 / 去使能\n"
        "  t                 自我測試\n"
        "\n"
        "  UART 閉環馬達（GP2/GP3，X42S / Emm）：\n"
        "    ue / ud         使能 / 去使能\n"
        "    r               讀即時位置（0x36）\n"
        "    z               目前位置清零（開機零點）\n"
        "    p <deg>         絕對角度（相對零點）\n"
        "    g <deg>         相對移動\n"
        "    s               立即停止\n"
        "    v <rpm>         速度（預設 600）\n"
        "    k <0~255>       加速度檔位（預設 100）\n"
        "    F               切換位置幀格式（13B / 12B）\n"
        "    Q               切換 TX/RX raw hex 顯示（除錯接線用）\n"
        "\n"
        "  EN/STEP/DIR 馬達（GP6/7/8）：\n"
        "    a <steps>       絕對步數（0 = 開機位置）\n"
        "    f <steps>       正轉相對步數\n"
        "    b <steps>       反轉相對步數\n"
        "    S               立即停止\n"
        "    V <rpm>         轉速（預設 120）\n"
        "    K <sps2>        加速度 steps/s²（預設 50000，0=不加減速）\n"
        "    c <steps/rev>   每圈脈衝數（預設 3200 = 16 細分）\n"
        "    n               切換 EN 極性（預設低電位致能）\n"
        "    D               切換 DIR 反向\n\n");
}

static void printAllStatus() {
    UartMotor::printStatus();
    StepDirMotor::printStatus();
}

static void selfTest() {
    Serial.println("\n========== 自我測試 ==========");

    Serial.println(">> UART 馬達：使能 + 清零");
    UartMotor::sendEnable(true);
    delay(50);
    UartMotor::sendClearPos();
    delay(200);

    for(const float deg : {90.0f, -90.0f, 0.0f}) {
        Serial.printf(">> UART → %+.0f°\n", static_cast<double>(deg));
        UartMotor::moveAbsoluteDeg(deg);
        const uint32_t t0 = millis();
        while(millis() - t0 < 5000) {
            UartMotor::poll();
            if(fabsf(UartMotor::reportedDeg - deg) < 1.0f)
                break;
            delay(10);
        }
        UartMotor::poll();
        Serial.printf("   實測 %.2f°（raw %ld）\n",
                      static_cast<double>(UartMotor::reportedDeg),
                      static_cast<long>(UartMotor::reportedRaw));
    }

    Serial.println(">> STEP/DIR 馬達：使能");
    StepDirMotor::setEnabled(true);

    const int32_t rev = static_cast<int32_t>(StepDirMotor::stepsPerRev);
    for(const int32_t steps : {rev, -rev, static_cast<int32_t>(0)}) {
        Serial.printf(">> STEP 目標 %ld 步\n", static_cast<long>(steps));
        StepDirMotor::targetSteps = steps;
        const uint32_t t0 = millis();
        while(millis() - t0 < 15000) {
            StepDirMotor::update();
            if(!StepDirMotor::moving && StepDirMotor::refSteps == steps)
                break;
            delay(1);
        }
        StepDirMotor::update();
        Serial.printf("   實測 %ld 步\n", static_cast<long>(StepDirMotor::refSteps));
    }

    Serial.println("========== 自我測試結束 ==========\n");
}

static void handleCommand(char *line) {
    char *argv[4] = {nullptr, nullptr, nullptr, nullptr};
    uint8_t argc = 0;
    char *save = nullptr;
    for(char *tok = strtok_r(line, " \t\r\n", &save); tok != nullptr && argc < 4;
        tok = strtok_r(nullptr, " \t\r\n", &save))
        argv[argc++] = tok;
    if(argc == 0)
        return;

    const char *cmd = argv[0];
    float f = 0.0f;
    long n = 0;

    if(strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0) {
        printHelp();
    } else if(strcmp(cmd, "i") == 0 || strcmp(cmd, "I") == 0) {
        printAllStatus();
    } else if(strcmp(cmd, "e") == 0) {
        UartMotor::sendEnable(true);
        StepDirMotor::setEnabled(true);
    } else if(strcmp(cmd, "d") == 0) {
        UartMotor::sendEnable(false);
        StepDirMotor::setEnabled(false);
    } else if(strcmp(cmd, "t") == 0) {
        selfTest();
    } else if(strcmp(cmd, "ue") == 0) {
        UartMotor::sendEnable(true);
        delay(20);
    } else if(strcmp(cmd, "ud") == 0) {
        UartMotor::sendEnable(false);
        delay(20);
    } else if(strcmp(cmd, "r") == 0) {
        UartMotor::sendReadPos();
        delay(50);
        UartMotor::poll();
        UartMotor::printStatus();
    } else if(strcmp(cmd, "z") == 0) {
        UartMotor::sendClearPos();
        Serial.println("[UART] 目前位置已清零");
    } else if(strcmp(cmd, "p") == 0) {
        if(argc < 2 || !parseFloatArg(argv[1], &f)) {
            Serial.println("用法：p <deg>（例：p 90）");
            return;
        }
        UartMotor::moveAbsoluteDeg(f);
        Serial.printf("[UART] 絕對 %.2f° → %ld pulses\n", static_cast<double>(f),
                      static_cast<long>(UartMotor::degToPulses(f)));
    } else if(strcmp(cmd, "g") == 0) {
        if(argc < 2 || !parseFloatArg(argv[1], &f)) {
            Serial.println("用法：g <deg>（例：g -15）");
            return;
        }
        UartMotor::moveRelativeDeg(f);
        Serial.printf("[UART] 相對 %+.2f° → 目標 %ld pulses\n",
                      static_cast<double>(f),
                      static_cast<long>(UartMotor::lastSentPulses));
    } else if(strcmp(cmd, "s") == 0) {
        UartMotor::sendStop();
        Serial.println("[UART] 已送停止命令");
    } else if(strcmp(cmd, "v") == 0) {
        if(argc < 2 || !parseLongArg(argv[1], &n) || n <= 0 || n > 3000) {
            Serial.println("用法：v <rpm 1~3000>");
            return;
        }
        UartMotor::speedRpm = static_cast<uint16_t>(n);
        UartMotor::printStatus();
    } else if(strcmp(cmd, "k") == 0) {
        if(argc < 2 || !parseLongArg(argv[1], &n) || n < 0 || n > 255) {
            Serial.println("用法：k <0~255>");
            return;
        }
        UartMotor::accStep = static_cast<uint8_t>(n);
        UartMotor::printStatus();
    } else if(strcmp(cmd, "F") == 0) {
        UartMotor::modeByte = !UartMotor::modeByte;
        Serial.printf("[UART] 位置幀改為 %s\n",
                      UartMotor::modeByte ? "13B（第二代）" : "12B（舊 Emm）");
    } else if(strcmp(cmd, "a") == 0) {
        if(argc < 2 || !parseLongArg(argv[1], &n)) {
            Serial.println("用法：a <steps>");
            return;
        }
        if(!StepDirMotor::enabled) {
            Serial.println("[STEP] 未使能 → 自動使能");
            StepDirMotor::setEnabled(true);
        }
        StepDirMotor::targetSteps = static_cast<int32_t>(n);
        Serial.printf("[STEP] 絕對目標 %ld 步\n", static_cast<long>(n));
    } else if(strcmp(cmd, "f") == 0 || strcmp(cmd, "b") == 0) {
        if(argc < 2 || !parseLongArg(argv[1], &n) || n < 0) {
            Serial.println("用法：f <steps> / b <steps>");
            return;
        }
        if(!StepDirMotor::enabled) {
            Serial.println("[STEP] 未使能 → 自動使能");
            StepDirMotor::setEnabled(true);
        }
        const int32_t delta =
            (cmd[0] == 'f') ? static_cast<int32_t>(n) : -static_cast<int32_t>(n);
        StepDirMotor::targetSteps += delta;
        Serial.printf("[STEP] %s %ld 步 → 目標 %ld 步\n",
                      cmd[0] == 'f' ? "正轉" : "反轉", static_cast<long>(n),
                      static_cast<long>(StepDirMotor::targetSteps));
    } else if(strcmp(cmd, "Q") == 0) {
        UartMotor::dumpRaw = !UartMotor::dumpRaw;
        Serial.printf("[UART] raw 位元組輸出 %s\n", UartMotor::dumpRaw ? "ON" : "OFF");
    } else if(strcmp(cmd, "S") == 0) {
        StepDirMotor::stop("使用者命令");
    } else if(strcmp(cmd, "V") == 0) {
        if(argc < 2 || !parseFloatArg(argv[1], &f) || f <= 0.0f) {
            Serial.println("用法：V <rpm>");
            return;
        }
        StepDirMotor::speedRpm = f;
        StepDirMotor::printStatus();
    } else if(strcmp(cmd, "K") == 0) {
        if(argc < 2 || !parseFloatArg(argv[1], &f) || f < 0.0f) {
            Serial.println("用法：K <steps/s²>（0 = 不加減速）");
            return;
        }
        StepDirMotor::accelSps2 = f;
        StepDirMotor::printStatus();
    } else if(strcmp(cmd, "c") == 0) {
        if(argc < 2 || !parseFloatArg(argv[1], &f) || f < 1.0f) {
            Serial.println("用法：c <steps/rev>（例：c 200 或 c 3200）");
            return;
        }
        StepDirMotor::stepsPerRev = f;
        StepDirMotor::printStatus();
    } else if(strcmp(cmd, "n") == 0) {
        StepDirMotor::enActiveLow = !StepDirMotor::enActiveLow;
        StepDirMotor::applyEnablePin();
        StepDirMotor::printStatus();
    } else if(strcmp(cmd, "D") == 0) {
        StepDirMotor::dirInvert = !StepDirMotor::dirInvert;
        StepDirMotor::printStatus();
    } else {
        Serial.printf("未知命令 '%s'（h 看說明）\n", cmd);
    }
}

// ============================================================================
// Arduino 進入點
// ============================================================================
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    delay(300);

    Serial.println("\n\n=== stepper_combined_test ===");
    Serial.printf("UART 馬達: TX=GP%u RX=GP%u @115200\n", Pin::UART_TX, Pin::UART_RX);
    Serial.printf("STEP/DIR: STEP=GP%u DIR=GP%u EN=GP%u\n", Pin::STEP, Pin::DIR,
                  Pin::EN);

    // 先讓 STEP/DIR 的 PIO SM 取得 PIO0，再啟動 SerialPIO 的 UART SM
    StepDirMotor::init();
    UartMotor::init();

    printAllStatus();
    Serial.println("[READY] 輸入 h 看命令說明");
}

void loop() {
    static char line[96];
    static uint8_t len = 0;

    while(Serial.available() > 0) {
        const int c = Serial.read();
        if(c < 0)
            break;
        if(c == '\n' || c == '\r') {
            if(len > 0) {
                line[len] = '\0';
                handleCommand(line);
                len = 0;
            }
        } else if(len < sizeof(line) - 1) {
            line[len++] = static_cast<char>(c);
        }
    }

    StepDirMotor::update();
    UartMotor::poll();

    // 心跳：每 1 秒翻一次板載 LED（確認 loop 還活著）
    static uint32_t lastLedMs = 0;
    static bool ledOn = false;
    if(millis() - lastLedMs >= 1000) {
        lastLedMs = millis();
        ledOn = !ledOn;
        digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
    }
}
