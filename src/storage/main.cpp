// ============================================================================
// Storage — Slave 節點 (RP2350)：只有伺服馬達（PCA9685）
//
// 架構（參考 src/test/mechanism 的機構測試，移除 trigger 步進馬達）：
//   Core 0 (loop)：   SlaveComms::update() — 收 Hub 控制幀（硬體 UART0）
//   Core 1 (loop1)：  SlaveComms::runCore1() — 命令驅動 + idleHook 跑機構序列
//
// 動作邏輯：
//   - 上電 / E-Stop / 序列結束 → 所有「已啟用」通道停在 HOME 角度（預設 30°）
//   - 收到 trigger 上緣（0 → 1，對應 Hub 端 ch7 < 0）→ 全部通道做「一次」循環：
//       CH0 → OUT(120°)，停留 dwellMs → 回 HOME(30°)，等 settleMs
//       CH1 → ... 依此類推，做完所有已啟用通道 = 一次 loop，然後停在 HOME
//   - trigger 持續為 1 不會重複觸發；必須放開（回 0）再打一次才會再跑一輪
//
// 通道校正（角度零點 / 行程 / 反向 / 啟用）直接沿用 EEPROM 內容
// （與 pca9685_angle / mechanism_test 同一份，MAGIC = "PCA4"）。
// 沒有 EEPROM 設定時使用預設值：16 通道全啟用、600~2400µs、0~180°、正向。
//
// 通訊：
//   Hub PIO SM2 (GP6 TX, GP7 RX) ↔ Storage UART0 (TX=GP0, RX=GP1)
//   921600 baud, COBS 封幀 + CRC16
//
// 控制資料 (StoragePayload)：
//   trigger      (uint8) — 0/1，上緣觸發「全部通道跑一次循環」
//   motorChannel (uint8) — 保留欄位，本版不使用
//   sw           (uint8) — 保留欄位，本版不使用
//
// 硬體配置：
//   PCA9685: I2C1 (GP2=SDA, GP3=SCL), 位址 0x40, 50Hz
//   伺服馬達：PCA9685 通道 0~15（實際接了哪幾顆由 EEPROM 的 enabled 決定）
//   ※ 本節點已移除 trigger 步進馬達（GP4/GP5/GP6 不再使用）
// ============================================================================
#include "SlaveComms.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <cmath>
#include <cstring>
#include <hardware/i2c.h>
#include <pico/stdlib.h>

// ============================================================================
// 腳位 / 常數
// ============================================================================
namespace Pin {
constexpr uint I2C_SDA = 2;
constexpr uint I2C_SCL = 3;
constexpr uint I2C_HZ = 100000;
} // namespace Pin

namespace Pca {
constexpr uint8_t ADDR_DEFAULT = 0x40;
constexpr uint8_t MODE1 = 0x00;
constexpr uint8_t MODE2 = 0x01;
constexpr uint8_t LED0_ON_L = 0x06;
constexpr uint8_t PRE_SCALE = 0xFE;
constexpr uint8_t CHAN_STEP = 4;
constexpr uint8_t CHANNEL_COUNT = 16;
constexpr uint8_t MODE1_AI = 0x20;
constexpr uint8_t MODE1_SLEEP = 0x10;
constexpr uint8_t MODE1_RESTART = 0x80;
constexpr uint8_t MODE2_OUTDRV = 0x04;
constexpr uint32_t PERIOD_US = 20000; // 50Hz
constexpr uint32_t COUNT_SPAN = 4096;
} // namespace Pca

namespace Servo {
constexpr float ANGLE_MIN = 0.0f;
constexpr float ANGLE_MAX = 180.0f;
constexpr uint16_t DEFAULT_MIN_US = 600;
constexpr uint16_t DEFAULT_MAX_US = 2400;
constexpr uint16_t ABS_MIN_US = 300;
constexpr uint16_t ABS_MAX_US = 3000;
constexpr uint16_t MIN_SPAN_US = 100;
} // namespace Servo

// ============================================================================
// 機構序列參數（與 mechanism_test 預設相同）
// ============================================================================
namespace SeqCfg {
constexpr float HOME_DEG = 30.0f;  // 待機位置
constexpr float OUT_DEG = 120.0f;  // 推出位置
constexpr uint32_t DWELL_MS = 1000;  // OUT 停留
constexpr uint32_t SETTLE_MS = 500;  // 回 HOME 後等待，再換下一顆
} // namespace SeqCfg

// ============================================================================
// 每通道校正（EEPROM 格式必須與 pca9685_angle / mechanism_test 完全一致）
// ============================================================================
struct ChannelLimit {
    uint16_t minUs;
    uint16_t maxUs;
    uint8_t minDeg;
    uint8_t maxDeg;
    uint8_t enabled;
    uint8_t invert;
};

namespace Limits {
static ChannelLimit channels[Pca::CHANNEL_COUNT];

constexpr int EEPROM_SIZE = 512;
constexpr int EEPROM_ADDR = 0;
constexpr uint32_t MAGIC = 0x50434134; // "PCA4"

struct Persist {
    uint32_t magic;
    uint16_t minUs[Pca::CHANNEL_COUNT];
    uint16_t maxUs[Pca::CHANNEL_COUNT];
    uint8_t minDeg[Pca::CHANNEL_COUNT];
    uint8_t maxDeg[Pca::CHANNEL_COUNT];
    uint8_t enabled[Pca::CHANNEL_COUNT];
    uint8_t invert[Pca::CHANNEL_COUNT];
    uint32_t checksum;
};

static uint32_t checksumOf(const Persist &p) {
    const uint8_t *raw = reinterpret_cast<const uint8_t *>(&p);
    uint32_t sum = 0;
    for(size_t i = 0; i < sizeof(Persist) - sizeof(uint32_t); i++)
        sum += raw[i];
    return sum;
}

static bool validPulse(uint16_t minUs, uint16_t maxUs) {
    return minUs >= Servo::ABS_MIN_US && maxUs <= Servo::ABS_MAX_US &&
           maxUs >= static_cast<uint16_t>(minUs + Servo::MIN_SPAN_US);
}

static ChannelLimit defaultChannel() {
    ChannelLimit c = {};
    c.minUs = Servo::DEFAULT_MIN_US;
    c.maxUs = Servo::DEFAULT_MAX_US;
    c.minDeg = 0;
    c.maxDeg = 180;
    c.enabled = 1;
    c.invert = 0;
    return c;
}

static bool load() {
    Persist p = {};
    EEPROM.get(EEPROM_ADDR, p);
    if(p.magic != MAGIC || p.checksum != checksumOf(p)) {
        for(uint8_t i = 0; i < Pca::CHANNEL_COUNT; i++)
            channels[i] = defaultChannel();
        return false;
    }
    for(uint8_t i = 0; i < Pca::CHANNEL_COUNT; i++) {
        ChannelLimit l = {};
        l.minUs = p.minUs[i];
        l.maxUs = p.maxUs[i];
        l.minDeg = p.minDeg[i];
        l.maxDeg = p.maxDeg[i];
        l.enabled = (p.enabled[i] != 0) ? 1 : 0;
        l.invert = (p.invert[i] != 0) ? 1 : 0;
        const bool ok = validPulse(l.minUs, l.maxUs) && l.minDeg <= l.maxDeg &&
                        l.maxDeg <= 180 && l.enabled <= 1 && l.invert <= 1;
        channels[i] = ok ? l : defaultChannel();
    }
    return true;
}
} // namespace Limits

// ============================================================================
// I2C（含重試與 bus recovery）
// ============================================================================
static i2c_inst_t *sI2c = nullptr;
static uint8_t sAddr = Pca::ADDR_DEFAULT;
static uint32_t sI2cErrors = 0;
static constexpr uint8_t I2C_RETRY_MAX = 3;
static constexpr uint32_t I2C_TIMEOUT_US = 10000;

/// SDA 腳位所屬 I2C 週邊（GP2 → i2c1，與硬體接線一致）
static i2c_inst_t *selectI2c(uint sdaPin) {
    return ((sdaPin / 2) % 2 == 0) ? i2c0 : i2c1;
}

static void i2cBusInit(uint32_t hz) {
    sI2c = selectI2c(Pin::I2C_SDA);
    i2c_init(sI2c, hz);
    gpio_set_function(Pin::I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(Pin::I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(Pin::I2C_SDA);
    gpio_pull_up(Pin::I2C_SCL);
    busy_wait_us(10);
}

static void i2cBusRecover() {
    i2c_deinit(sI2c);
    gpio_set_function(Pin::I2C_SDA, GPIO_FUNC_SIO);
    gpio_set_dir(Pin::I2C_SDA, GPIO_IN);
    gpio_pull_up(Pin::I2C_SDA);
    gpio_set_function(Pin::I2C_SCL, GPIO_FUNC_SIO);
    gpio_set_dir(Pin::I2C_SCL, GPIO_OUT);
    gpio_put(Pin::I2C_SCL, 1);
    busy_wait_us(5);
    for(uint8_t i = 0; i < 9; i++) {
        gpio_put(Pin::I2C_SCL, 0);
        busy_wait_us(5);
        gpio_put(Pin::I2C_SCL, 1);
        busy_wait_us(5);
    }
    gpio_set_dir(Pin::I2C_SDA, GPIO_OUT);
    gpio_put(Pin::I2C_SDA, 0);
    busy_wait_us(5);
    gpio_put(Pin::I2C_SCL, 1);
    busy_wait_us(5);
    gpio_put(Pin::I2C_SDA, 1);
    busy_wait_us(5);
    i2cBusInit(Pin::I2C_HZ);
    busy_wait_us(200);
}

static bool rawWrite(uint8_t addr, const uint8_t *buf, size_t len) {
    for(uint8_t attempt = 0; attempt < I2C_RETRY_MAX; attempt++) {
        if(i2c_write_timeout_us(sI2c, addr, buf, len, false, I2C_TIMEOUT_US) ==
           static_cast<int>(len))
            return true;
        sI2cErrors++;
        if(attempt + 1 < I2C_RETRY_MAX)
            i2cBusRecover();
    }
    return false;
}

static bool rawRead(uint8_t addr, uint8_t reg, uint8_t *out, size_t len) {
    for(uint8_t attempt = 0; attempt < I2C_RETRY_MAX; attempt++) {
        if(i2c_write_timeout_us(sI2c, addr, &reg, 1, true, I2C_TIMEOUT_US) == 1 &&
           i2c_read_timeout_us(sI2c, addr, out, len, false, I2C_TIMEOUT_US) ==
               static_cast<int>(len))
            return true;
        sI2cErrors++;
        if(attempt + 1 < I2C_RETRY_MAX)
            i2cBusRecover();
    }
    return false;
}

static bool regWrite(uint8_t reg, uint8_t val) {
    const uint8_t buf[2] = {reg, val};
    return rawWrite(sAddr, buf, sizeof(buf));
}

// ============================================================================
// 角度 → 脈衝（含每通道零點 / 行程 / 反向 / 實體工作範圍）
// ============================================================================
static float clampAngle(float deg) {
    if(deg < Servo::ANGLE_MIN)
        return Servo::ANGLE_MIN;
    if(deg > Servo::ANGLE_MAX)
        return Servo::ANGLE_MAX;
    return deg;
}

static float logicalToPhysical(uint8_t ch, float logicalDeg) {
    if(ch >= Pca::CHANNEL_COUNT)
        ch = 0;
    logicalDeg = clampAngle(logicalDeg);
    const ChannelLimit &l = Limits::channels[ch];
    float phys = l.invert ? (Servo::ANGLE_MAX - logicalDeg) : logicalDeg;
    if(phys < static_cast<float>(l.minDeg))
        phys = static_cast<float>(l.minDeg);
    if(phys > static_cast<float>(l.maxDeg))
        phys = static_cast<float>(l.maxDeg);
    return phys;
}

static uint16_t angleToPulseUs(uint8_t ch, float deg) {
    const ChannelLimit &l = Limits::channels[ch];
    const float span = static_cast<float>(l.maxUs - l.minUs);
    const float phys = logicalToPhysical(ch, deg);
    return static_cast<uint16_t>(static_cast<float>(l.minUs) +
                                 span * phys / Servo::ANGLE_MAX + 0.5f);
}

static uint16_t pulseUsToCount(uint16_t pulseUs) {
    return static_cast<uint16_t>(
        (static_cast<uint32_t>(pulseUs) * Pca::COUNT_SPAN + Pca::PERIOD_US / 2) /
        Pca::PERIOD_US);
}

static bool writeChannelCount(uint8_t ch, uint16_t count) {
    if(ch >= Pca::CHANNEL_COUNT)
        return false;
    uint8_t buf[5];
    buf[0] = static_cast<uint8_t>(Pca::LED0_ON_L + ch * Pca::CHAN_STEP);
    buf[1] = 0x00;
    buf[2] = 0x00;
    if(count == 0) {
        buf[3] = 0x00;
        buf[4] = 0x10; // OFF full → 輸出恆低
    } else {
        buf[3] = static_cast<uint8_t>(count & 0xFF);
        buf[4] = static_cast<uint8_t>((count >> 8) & 0x0F);
    }
    return rawWrite(sAddr, buf, sizeof(buf));
}

static bool setAngle(uint8_t ch, float deg) {
    return writeChannelCount(ch, pulseUsToCount(angleToPulseUs(ch, deg)));
}

static void setChannelOff(uint8_t ch) { writeChannelCount(ch, 0); }

static void allChannelsOff() {
    for(uint8_t ch = 0; ch < Pca::CHANNEL_COUNT; ch++)
        setChannelOff(ch);
}

static bool channelEnabled(uint8_t ch) {
    return ch < Pca::CHANNEL_COUNT && Limits::channels[ch].enabled != 0;
}

// ============================================================================
// PCA9685 初始化（50Hz）
// ============================================================================
static bool pcaInit() {
    uint8_t probe = 0;
    if(rawRead(sAddr, Pca::MODE1, &probe, 1) == false) {
        return false;
    }
    regWrite(Pca::MODE1, Pca::MODE1_SLEEP);
    regWrite(Pca::PRE_SCALE, 121); // 25MHz / (4096 × 50Hz) - 1
    regWrite(Pca::MODE1, Pca::MODE1_AI);
    busy_wait_us(500);
    regWrite(Pca::MODE2, Pca::MODE2_OUTDRV);
    busy_wait_us(500);
    regWrite(Pca::MODE1,
             static_cast<uint8_t>(Pca::MODE1_AI | Pca::MODE1_RESTART));
    busy_wait_us(500);
    regWrite(Pca::MODE1, Pca::MODE1_AI);
    busy_wait_us(500);
    allChannelsOff();
    return true;
}

// ============================================================================
// 機構序列（非阻塞狀態機，全部在 Core 1 執行）
//
//   trigger 上緣 → 依序把每顆「已啟用」通道做 OUT → dwell → HOME → settle，
//   全部做完一次就停（不重複），結束時所有通道都在 HOME。
// ============================================================================
namespace ServoSeq {
static float homeDeg = SeqCfg::HOME_DEG;
static float outDeg = SeqCfg::OUT_DEG;
static uint32_t dwellMs = SeqCfg::DWELL_MS;
static uint32_t settleMs = SeqCfg::SETTLE_MS;

static bool running = false;
static uint8_t steps[Pca::CHANNEL_COUNT] = {0};
static uint8_t stepCount = 0;
static uint8_t stepIdx = 0;

enum Phase : uint8_t { PHASE_NONE = 0, PHASE_OUT, PHASE_DWELL, PHASE_SETTLE };
static Phase phase = PHASE_NONE;
static uint32_t phaseMs = 0;
static uint32_t cycleStartMs = 0;
static uint32_t cyclesDone = 0;

static void goHomeAll() {
    for(uint8_t ch = 0; ch < Pca::CHANNEL_COUNT; ch++) {
        if(channelEnabled(ch))
            setAngle(ch, homeDeg);
        else
            setChannelOff(ch);
    }
}

static void buildSteps() {
    stepCount = 0;
    for(uint8_t ch = 0; ch < Pca::CHANNEL_COUNT; ch++) {
        if(channelEnabled(ch))
            steps[stepCount++] = ch;
    }
    stepIdx = 0;
}

/// 停止：所有通道回 HOME（E-Stop 與 trigger 放開都走這條）
static void stop() {
    const bool wasRunning = running;
    running = false;
    phase = PHASE_NONE;
    goHomeAll();
    if(wasRunning) {
        const int32_t h = static_cast<int32_t>(lroundf(homeDeg * 10.0f));
        Serial.printf("[STORAGE][SEQ] 停止，全部回 %ld.%u°\n",
                      static_cast<long>(h / 10), static_cast<unsigned>(h % 10));
    }
}

/// 跑一次完整循環（先停止當前動作，全部先回 HOME 再依序推出）
static void startOnce() {
    stop();
    buildSteps();
    if(stepCount == 0) {
        Serial.println("[STORAGE][SEQ] 沒有已啟用的通道（EEPROM 校正全部關閉）");
        return;
    }
    running = true;
    phase = PHASE_OUT;
    phaseMs = millis();
    cycleStartMs = phaseMs;
    Serial.printf("[STORAGE][SEQ] 開始一次循環，共 %u 顆：", stepCount);
    for(uint8_t i = 0; i < stepCount; i++) {
        Serial.printf(" CH%u", steps[i]);
        setAngle(steps[i], homeDeg);
    }
    Serial.println();
}

/// idleHook 呼叫：推進狀態機
static void update() {
    if(!running)
        return;
    const uint32_t now = millis();
    switch(phase) {
    case PHASE_OUT: {
        const uint8_t ch = steps[stepIdx];
        setAngle(ch, outDeg);
        const int32_t o = static_cast<int32_t>(lroundf(outDeg * 10.0f));
        Serial.printf("[STORAGE][SEQ] CH%u → %ld.%u° (%u/%u)\n", ch,
                      static_cast<long>(o / 10), static_cast<unsigned>(o % 10),
                      static_cast<unsigned>(stepIdx + 1),
                      static_cast<unsigned>(stepCount));
        phase = PHASE_DWELL;
        phaseMs = now;
        break;
    }
    case PHASE_DWELL:
        if(now - phaseMs >= dwellMs) {
            const uint8_t ch = steps[stepIdx];
            setAngle(ch, homeDeg);
            const int32_t h = static_cast<int32_t>(lroundf(homeDeg * 10.0f));
            Serial.printf("[STORAGE][SEQ] CH%u → %ld.%u°\n", ch,
                          static_cast<long>(h / 10),
                          static_cast<unsigned>(h % 10));
            phase = PHASE_SETTLE;
            phaseMs = now;
        }
        break;
    case PHASE_SETTLE:
        if(now - phaseMs >= settleMs) {
            stepIdx++;
            if(stepIdx >= stepCount) {
                running = false;
                phase = PHASE_NONE;
                cyclesDone++;
                goHomeAll();
                Serial.printf("[STORAGE][SEQ] 循環完成（%lu ms），全部回 HOME\n",
                              static_cast<unsigned long>(now - cycleStartMs));
            } else {
                phase = PHASE_OUT;
            }
        }
        break;
    default:
        break;
    }
}

static bool isRunning() { return running; }
static uint8_t currentChannel() {
    return (running && stepIdx < stepCount) ? steps[stepIdx] : 0xFF;
}
static uint8_t progress() { return running ? stepIdx : stepCount; }
static uint8_t total() { return stepCount; }
static uint32_t doneCount() { return cyclesDone; }
} // namespace ServoSeq

// ============================================================================
// 控制邏輯
// ============================================================================
namespace MotorControl {
// trigger 上緣偵測（Core 1）
static bool prevTrigger = false;

static void init() {
    EEPROM.begin(Limits::EEPROM_SIZE);
    const bool loaded = Limits::load();

    i2cBusInit(Pin::I2C_HZ);
    const bool ok = pcaInit();
    ServoSeq::stop(); // 上電先全部回 HOME
    prevTrigger = false;

    Serial.printf("[STORAGE] 通道校正%s\n", loaded ? "已從 EEPROM 載入"
                                                   : "使用預設值（16 通道）");
    Serial.printf("[STORAGE] PCA9685 0x%02X %s | HOME=%.0f° OUT=%.0f° "
                  "dwell=%lums settle=%lums\n",
                  sAddr, ok ? "OK 50Hz" : "無回應！",
                  static_cast<double>(SeqCfg::HOME_DEG),
                  static_cast<double>(SeqCfg::OUT_DEG),
                  static_cast<unsigned long>(SeqCfg::DWELL_MS),
                  static_cast<unsigned long>(SeqCfg::SETTLE_MS));
    if(sI2cErrors) {
        Serial.printf("[STORAGE] I2C 錯誤累計 %lu\n",
                      static_cast<unsigned long>(sI2cErrors));
    }
}

/// 收到 Hub 命令：trigger 上緣 → 全部通道跑一次循環
static void executeMotion(const ChassisSnapshot &cmd) {
    const bool trigger = (cmd.trigger != 0);
    if(trigger && !prevTrigger) {
        ServoSeq::startOnce();
    }
    prevTrigger = trigger;
}

static void estop() {
    ServoSeq::stop();
    prevTrigger = false;
}
} // namespace MotorControl

// ============================================================================
// SlaveApp 回呼
// ============================================================================
static uint8_t tmFrameSeq = 0;       // 遙測幀序號
static MoveTelemetry pendingTm = {}; // Core 1 寫入 → Core 0 發送

static void onStorageEstop() { MotorControl::estop(); }

static void onStorageCommand(const ChassisSnapshot &cmd) {
    MotorControl::executeMotion(cmd);

    // 構建遙測：currentRps[0] = 目前通道, [1] = 進度, [2] = 總數,
    //             [3] = 已完成循環次數
    pendingTm = {};
    pendingTm.sync = 0xA5;
    pendingTm.msgType = MsgType::TELEMETRY;
    pendingTm.frameSeq = tmFrameSeq++;
    pendingTm.targetID = TargetID::HUB;
    pendingTm.currentRps[0] = static_cast<float>(ServoSeq::currentChannel());
    pendingTm.currentRps[1] = static_cast<float>(ServoSeq::progress());
    pendingTm.currentRps[2] = static_cast<float>(ServoSeq::total());
    pendingTm.currentRps[3] = static_cast<float>(ServoSeq::doneCount());
    pendingTm.targetRps[0] = ServoSeq::isRunning() ? 1.0f : 0.0f;
    pendingTm.mode = RobotMode::AUTO;
    pendingTm.estopFlag = 0;
    pendingTm.timestamp = time_us_64();
    pendingTm.crc16 = 0;
    pendingTm.crc16 =
        SignalUtils::calculateCrc(reinterpret_cast<const uint8_t *>(&pendingTm),
                                  sizeof(MoveTelemetry) - 2);
}

// 前向宣告：storageCommandWrapper 定義在 slave 之後
static void storageCommandWrapper(const ChassisSnapshot &cmd);

static SlaveApp storageApp = {
    .targetID = TargetID::STORAGE,
    .onCommand = storageCommandWrapper, // wrapper: onStorageCommand + 遙測發送
    .onEstop = onStorageEstop,
};

static SlaveComms slave(&storageApp);

static void storageCommandWrapper(const ChassisSnapshot &cmd) {
    onStorageCommand(cmd);
    slave.prepareTelemetry(pendingTm);
}

// ============================================================================
// Core 1 idle hook — 推進伺服序列狀態機
// ============================================================================
static void storageIdleHook() { ServoSeq::update(); }

// ============================================================================
// 狀態 LED（Core 0）
//   未連線 → 長亮；序列執行中 → 快閃（100ms）；閒置 → 慢閃（1000ms）
// ============================================================================
static void updateStatusLed() {
    static uint32_t lastBlinkMs = 0;
    static bool ledOn = false;

    if(slave.validCmdCount() == 0) {
        digitalWrite(LED_BUILTIN, HIGH);
        return;
    }
    const uint32_t now = millis();
    const uint32_t interval = ServoSeq::isRunning() ? 100 : 1000;
    if(now - lastBlinkMs >= interval) {
        lastBlinkMs = now;
        ledOn = !ledOn;
        digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
    }
}

// ============================================================================
// Core 0
// ============================================================================
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // 開機先亮

    Serial.begin(115200);
    delay(200);
    Serial.println("[STORAGE] boot: 只有伺服馬達（PCA9685 I2C1 GP2/GP3）");

    slave.begin(); // 硬體 UART0 (GP0/GP1) → Hub
    slave.setIdleHook(storageIdleHook);

    Serial.println("[STORAGE] 等待 Hub 命令（trigger 上緣 → 全部通道跑一次）");
}

void loop() {
    slave.update();
    updateStatusLed();

    // 500ms 監控輸出
    static uint32_t lastPrint = 0;
    const uint32_t nowMs = millis();
    if(nowMs - lastPrint >= 500) {
        lastPrint = nowMs;
        if(slave.isEstopActive()) {
            Serial.printf("[STORAGE] ** E-STOP ** sinceCmd=%ums 伺服已回 HOME\n",
                          slave.msSinceLastValidCmd());
        } else {
            Serial.printf("[STORAGE] sinceCmd=%ums cnt=%u | seq=%s %u/%u ch=%d "
                          "cycles=%lu | i2cErr=%lu\n",
                          slave.msSinceLastValidCmd(),
                          static_cast<unsigned>(slave.validCmdCount()),
                          ServoSeq::isRunning() ? "RUN" : "IDLE",
                          static_cast<unsigned>(ServoSeq::progress()),
                          static_cast<unsigned>(ServoSeq::total()),
                          static_cast<int>(ServoSeq::currentChannel()),
                          static_cast<unsigned long>(ServoSeq::doneCount()),
                          static_cast<unsigned long>(sI2cErrors));
        }
    }
}

// ============================================================================
// Core 1
// ============================================================================
void setup1() { MotorControl::init(); }

void loop1() { slave.runCore1(); }
