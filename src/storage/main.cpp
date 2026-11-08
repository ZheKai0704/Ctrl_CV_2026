// ============================================================================
// Storage — Slave 節點 (RP2350)：只有伺服馬達（PCA9685）
//
// 架構（參考 src/test/mechanism 的機構測試，移除 trigger 步進馬達）：
//   Core 0 (loop)：   SlaveComms::update() — 收 Hub 控制幀（硬體 UART0）
//   Core 1 (loop1)：  SlaveComms::runCore1() — 命令驅動 + idleHook 跑機構序列
//
// 動作邏輯：
//   - 上電 / E-Stop → 所有「已啟用」通道停在 HOME 角度（預設 30°）
//   - 收到 trigger 上緣（0 → 1，對應 Hub 端 rc[8] < 0）
//     → 每按一次只做「一個動作」，做完等下一次 trigger
//     動作順序：g2×5 → g5×5 → g1×5 → g6×5 → g0×5 → g7×5（共 30 次）
//     每個動作 = 組內每顆「已啟用」通道依序 OUT(120°) → dwell → HOME(30°) → settle
//     **該組的最後一輪（第 5 次）**驗證完後，整組一起開到 OUT(120°) 並維持開啟，
//     不再回 HOME；之前已開啟的組也會一直保持。下一次完整循環（回到第 1 動）
//     開始前才全部回 HOME。
//   - trigger 持續為 1 不會重複觸發；一次上緣 = 一個動作，須放開再按下一動
//   - 上電/第一個命令只記錄開關狀態，不會動作；必須先看到 trigger=0，
//     之後的 0 → 1 才算一次按鈕（避免開機時開關已壓著就直接跑第一動）
//
// 通道校正（角度零點 / 行程 / 反向 / 啟用）直接沿用 EEPROM 內容
// （與 pca9685_angle / mechanism_test 同一份，MAGIC = "PCA4"）。
// 沒有 EEPROM 設定時使用預設值：16 通道全啟用、600~2400µs、0~180°、正向。
// PCA9685 奇數通道（CH1/3/5/7/9/11/13/15）與 mechanism_test 一致，由韌體強制反向
// （與 EEPROM 的 invert 以 OR 合併，不會二次反向）。
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
//   PCA9685: I2C0 (GP4=SDA, GP5=SCL), 位址 0x40, 50Hz
//   伺服馬達：PCA9685 通道 0~15（實際接了哪幾顆由 EEPROM 的 enabled 決定）
//   ※ 本節點已移除 trigger 步進馬達（GP6 未使用；GP4/GP5 現在是 I2C SDA/SCL）
// ============================================================================
#include "SlaveComms.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <hardware/i2c.h>
#include <pico/stdlib.h>

// ============================================================================
// 腳位 / 常數
// ============================================================================
namespace Pin {
constexpr uint I2C_SDA = 4;
constexpr uint I2C_SCL = 5;
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
// 機構左右成對安裝，PCA9685 奇數通道（CH1/3/5/7/9/11/13/15）實體方向相反。
// 與 mechanism_test 相同：硬體層強制反向，與 EEPROM invert 採 OR 合併。
constexpr bool MIRROR_ODD_CHANNELS = true;
} // namespace Servo

// ============================================================================
// 機構序列參數（與 mechanism_test 預設相同）
// ============================================================================
namespace SeqCfg {
constexpr float HOME_DEG = 30.0f;   // 待機位置
constexpr float OUT_DEG = 100.0f;   // 推出位置
constexpr uint32_t DWELL_MS = 1000; // OUT 停留
constexpr uint32_t SETTLE_MS = 500; // 回 HOME 後等待，再換下一顆

// trigger 一次要做動的「組」順序（組 n = CH(2n), CH(2n+1)）
// 每組連續作動 REPEAT_PER_GROUP 次，每次 = 組內每顆依序 OUT → HOME。
// 按一次 trigger 只執行「一個動作」（一組的一次），全部共 30 次。
constexpr uint8_t GROUP_ORDER[] = {2, 5, 1, 6, 0, 7};
constexpr uint8_t GROUP_ORDER_COUNT =
    sizeof(GROUP_ORDER) / sizeof(GROUP_ORDER[0]);
constexpr uint8_t REPEAT_PER_GROUP = 5;
constexpr uint8_t ACTION_COUNT = GROUP_ORDER_COUNT * REPEAT_PER_GROUP;
constexpr uint8_t GROUP_CHANNELS = 2;
constexpr uint8_t STEPS_PER_ACTION = GROUP_CHANNELS; // 一次動作最多兩顆
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

// 實際是否反向：奇數通道硬體鏡像，或 EEPROM 校正指定反向
static bool effectiveInvert(uint8_t ch) {
    if(ch >= Pca::CHANNEL_COUNT)
        return false;
    if(Servo::MIRROR_ODD_CHANNELS && (ch & 1u))
        return true;
    return channels[ch].invert != 0;
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

/// SDA 腳位所屬 I2C 週邊（GP4 → i2c0，與 mechanism_test 相同）
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
    float phys =
        Limits::effectiveInvert(ch) ? (Servo::ANGLE_MAX - logicalDeg) : logicalDeg;
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
// Core 1 → Core 0 日誌佇列
//
// ⚠ AGENTS.md 規定 Serial 只給 Core 0。Core 1 直接 Serial.printf() 在 USB CDC
//    TX FIFO 滿時會阻塞（實測最長 1 秒），會把機構序列卡在半途、看起來像
//    「seq=RUN 但馬達不動」。因此 Core 1 只把短字串丟進環形佇列，
//    由 Core 0 的 loop() 統一把它印出來（LogQ::drain()）。
// ============================================================================
namespace LogQ {
constexpr uint8_t SLOTS = 32;    // 佇列格數
constexpr uint8_t LINE_LEN = 96; // 單行長度
static char buf[SLOTS][LINE_LEN];
static volatile uint8_t head = 0; // Core 0 讀取位置
static volatile uint8_t tail = 0; // Core 1 寫入位置
static volatile uint32_t dropped = 0;

static uint8_t nextIdx(uint8_t i) {
    return static_cast<uint8_t>((i + 1) % SLOTS);
}

/// Core 1 呼叫：寫入一行（不阻塞；佇列滿就丟棄並計數）
static void push(const char *text) {
    const uint8_t t = tail;
    if(nextIdx(t) == head) { // 滿了
        dropped++;
        return;
    }
    char *dst = buf[t];
    uint8_t i = 0;
    for(; text[i] != '\0' && i < LINE_LEN - 1; i++)
        dst[i] = text[i];
    dst[i] = '\0';
    tail = nextIdx(t);
}

/// Core 1 呼叫：格式化後寫入一行（只支援整數格式，newlib-nano 無浮點 printf）
static void pushf(const char *fmt, ...) {
    char tmp[LINE_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    push(tmp);
}

/// Core 0 呼叫：把佇列裡累積的訊息印出來
static void drain() {
    while(head != tail) {
        Serial.println(buf[head]);
        head = nextIdx(head);
    }
}
} // namespace LogQ

// ============================================================================
// 機構序列（非阻塞狀態機，全部在 Core 1 執行）
//
//   每收到一次 trigger 上緣只做「一個動作」，做完就停，等下一次 trigger。
//   動作順序：g2×5 → g5×5 → g1×5 → g6×5 → g0×5 → g7×5（共 30 次）
//   每個動作 = 組內每顆「已啟用」通道依序 OUT → dwell → HOME → settle。
//   該組的最後一輪（第 REPEAT_PER_GROUP 次）多做一段：整組 → OUT 並維持開啟。
// ============================================================================
namespace ServoSeq {
static float homeDeg = SeqCfg::HOME_DEG;
static float outDeg = SeqCfg::OUT_DEG;
static uint32_t dwellMs = SeqCfg::DWELL_MS;
static uint32_t settleMs = SeqCfg::SETTLE_MS;

static constexpr uint8_t kStepsMax = SeqCfg::STEPS_PER_ACTION;

static bool running = false;
static uint8_t steps[kStepsMax] = {0};
static uint8_t stepCount = 0;
static uint8_t stepIdx = 0;

// 序列進度：下一次 trigger 要做的動作（0 ~ ACTION_COUNT-1）
static uint8_t nextAction = 0;
static uint8_t actionsDone = 0;
static uint8_t currentAction = 0;
static uint8_t currentGroup = 0;
static uint8_t currentRep = 0;

enum Phase : uint8_t { PHASE_NONE = 0,
                       PHASE_OUT,
                       PHASE_DWELL,
                       PHASE_SETTLE,
                       PHASE_OPEN };
static Phase phase = PHASE_NONE;
static uint32_t phaseMs = 0;
static uint32_t cycleStartMs = 0;
static uint32_t cyclesDone = 0;
// 序列期間 I2C 寫入失敗次數（PCA9685 沒 ACK / 接線問題一目了然）
static uint32_t writeErrors = 0;
// 全部已啟用通道是否都在 HOME。用於讓 stop() 可重複呼叫但只寫一次 I2C：
// E-Stop 期間 runCore1() 每圈都會呼叫 onEstop()，若每次都跑 goHomeAll()
// 就是每圈 16 次 I2C 寫入（每次可能阻塞數 ms）→ 整個系統卡住。
static bool atHome = false;

static void goHomeAll() {
    for(uint8_t ch = 0; ch < Pca::CHANNEL_COUNT; ch++) {
        if(channelEnabled(ch))
            setAngle(ch, homeDeg);
        else
            setChannelOff(ch);
    }
    atHome = true;
}

// 動作編號 → 組別 / 該組第幾次
static uint8_t groupOfAction(uint8_t action) {
    return SeqCfg::GROUP_ORDER[action / SeqCfg::REPEAT_PER_GROUP];
}
static uint8_t repOfAction(uint8_t action) {
    return static_cast<uint8_t>(action % SeqCfg::REPEAT_PER_GROUP + 1);
}

/// 建立「某一組的一次」要動的通道（跳過停用通道）
static bool buildAction(uint8_t group) {
    stepCount = 0;
    stepIdx = 0;
    for(uint8_t k = 0; k < SeqCfg::GROUP_CHANNELS; k++) {
        const uint8_t ch = static_cast<uint8_t>(group * SeqCfg::GROUP_CHANNELS + k);
        if(ch < Pca::CHANNEL_COUNT && channelEnabled(ch) && stepCount < kStepsMax)
            steps[stepCount++] = ch;
    }
    return stepCount > 0;
}

/// 停止：所有通道回 HOME（E-Stop / 重新開始動作時）
/// 可重複呼叫：已經全在 HOME 就不再寫 I2C（避免 E-Stop 每圈洪水）。
static void stop() {
    const bool wasRunning = running;
    running = false;
    phase = PHASE_NONE;
    if(!atHome)
        goHomeAll();
    if(wasRunning) {
        const int32_t h = static_cast<int32_t>(lroundf(homeDeg * 10.0f));
        LogQ::pushf("[STORAGE][SEQ] 停止，全部回 %ld.%u°",
                    static_cast<long>(h / 10), static_cast<unsigned>(h % 10));
    }
}

/// 把本動作會用到的通道放回 HOME（不動已維持開啟的其他組）
static void homeActionChannels() {
    for(uint8_t i = 0; i < stepCount; i++)
        setAngle(steps[i], homeDeg);
}

/// 執行「下一個動作」（一次 trigger 只呼叫一次）
static void startNext() {
    // 新的完整循環（第 1 動）開始前才全部回 HOME；其餘只歸零本動作的通道，
    // 讓前面組「維持開啟」的狀態保持住。
    const bool newCycle = (nextAction == 0);
    running = false;
    phase = PHASE_NONE;
    if(newCycle)
        goHomeAll();
    uint8_t action = nextAction;
    for(uint8_t tried = 0; tried < SeqCfg::ACTION_COUNT; tried++) {
        const uint8_t group = groupOfAction(action);
        if(buildAction(group)) {
            if(!newCycle)
                homeActionChannels();
            currentAction = action;
            currentGroup = group;
            currentRep = repOfAction(action);
            running = true;
            phase = PHASE_OUT;
            phaseMs = millis();
            cycleStartMs = phaseMs;
            // 一行把「第幾動 + 組 + 次數 + 通道」寫進佇列，由 Core 0 印出
            char line[LogQ::LINE_LEN];
            int n = snprintf(line, sizeof(line),
                             "[STORAGE][SEQ] 第 %u/%u 動：g%u %u/%u，通道",
                             static_cast<unsigned>(action + 1),
                             static_cast<unsigned>(SeqCfg::ACTION_COUNT),
                             static_cast<unsigned>(group),
                             static_cast<unsigned>(currentRep),
                             static_cast<unsigned>(SeqCfg::REPEAT_PER_GROUP));
            for(uint8_t i = 0; i < stepCount && n > 0 &&
                               n < static_cast<int>(sizeof(line));
                i++)
                n += snprintf(line + n, sizeof(line) - static_cast<size_t>(n),
                              " CH%u", steps[i]);
            LogQ::push(line);
            return;
        }
        action = static_cast<uint8_t>((action + 1) % SeqCfg::ACTION_COUNT);
    }
    nextAction = 0;
    LogQ::push("[STORAGE][SEQ] 沒有已啟用的通道（EEPROM 校正全部關閉）");
}

/// 動作結束：keepOpen = true 代表通道已開到 OUT，不回 HOME
static void finishAction(bool keepOpen, uint32_t now) {
    running = false;
    phase = PHASE_NONE;
    cyclesDone++;
    actionsDone++;
    nextAction = static_cast<uint8_t>((currentAction + 1) % SeqCfg::ACTION_COUNT);
    // 只收起本動作的通道；已「維持開啟」的其他組必須保持原狀
    if(!keepOpen)
        homeActionChannels();
    const uint32_t elapsed = now - cycleStartMs;
    if(keepOpen) {
        LogQ::pushf("[STORAGE][SEQ] 第 %u/%u 動完成（%lu ms），g%u 維持開啟，"
                    "等下一次 trigger",
                    static_cast<unsigned>(currentAction + 1),
                    static_cast<unsigned>(SeqCfg::ACTION_COUNT),
                    static_cast<unsigned long>(elapsed),
                    static_cast<unsigned>(currentGroup));
    } else {
        LogQ::pushf("[STORAGE][SEQ] 第 %u/%u 動完成（%lu ms），"
                    "全部回 HOME，等下一次 trigger",
                    static_cast<unsigned>(currentAction + 1),
                    static_cast<unsigned>(SeqCfg::ACTION_COUNT),
                    static_cast<unsigned long>(elapsed));
    }
}

/// idleHook 呼叫：推進狀態機
static void update() {
    if(!running)
        return;
    const uint32_t now = millis();
    switch(phase) {
    case PHASE_OUT: {
        const uint8_t ch = steps[stepIdx];
        if(!setAngle(ch, outDeg))
            writeErrors++;
        atHome = false;
        const int32_t o = static_cast<int32_t>(lroundf(outDeg * 10.0f));
        LogQ::pushf("[STORAGE][SEQ] g%u %u/%u CH%u → %ld.%u° (step %u/%u)",
                    static_cast<unsigned>(currentGroup),
                    static_cast<unsigned>(currentRep),
                    static_cast<unsigned>(SeqCfg::REPEAT_PER_GROUP), ch,
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
            if(!setAngle(ch, homeDeg))
                writeErrors++;
            const int32_t h = static_cast<int32_t>(lroundf(homeDeg * 10.0f));
            LogQ::pushf("[STORAGE][SEQ] CH%u → %ld.%u°", ch,
                        static_cast<long>(h / 10),
                        static_cast<unsigned>(h % 10));
            phase = PHASE_SETTLE;
            phaseMs = now;
        }
        break;
    case PHASE_SETTLE:
        if(now - phaseMs >= settleMs) {
            stepIdx++;
            if(stepIdx < stepCount) {
                phase = PHASE_OUT;
            } else if(currentRep >= SeqCfg::REPEAT_PER_GROUP) {
                // 這一組的最後一輪：驗證完再把整組開到 OUT 並維持開啟
                stepIdx = 0;
                phase = PHASE_OPEN;
                phaseMs = now;
            } else {
                finishAction(false, now);
            }
        }
        break;
    case PHASE_OPEN: {
        const int32_t o = static_cast<int32_t>(lroundf(outDeg * 10.0f));
        for(uint8_t i = 0; i < stepCount; i++) {
            if(!setAngle(steps[i], outDeg))
                writeErrors++;
            atHome = false;
            LogQ::pushf("[STORAGE][SEQ] g%u 最後一輪 CH%u → %ld.%u°"
                        "（開啟，維持）",
                        static_cast<unsigned>(currentGroup),
                        static_cast<unsigned>(steps[i]),
                        static_cast<long>(o / 10),
                        static_cast<unsigned>(o % 10));
        }
        finishAction(true, now);
        break;
    }
    default:
        break;
    }
}

static bool isRunning() { return running; }
static uint8_t currentChannel() {
    return (running && stepIdx < stepCount) ? steps[stepIdx] : 0xFF;
}
static uint8_t progress() {
    return running ? static_cast<uint8_t>(currentAction + 1) : actionsDone;
}
static uint8_t total() { return SeqCfg::ACTION_COUNT; }
static uint32_t doneCount() { return cyclesDone; }
static uint32_t writeErrorCount() { return writeErrors; }
static void resetSequence() {
    nextAction = 0;
    actionsDone = 0;
}
} // namespace ServoSeq

// ============================================================================
// 控制邏輯
// ============================================================================
// Core 1 初始化結果旗標 → Core 0 負責印訊息（Serial 只能在 Core 0 使用）
static volatile bool sPcaInitDone = false;
static volatile bool sPcaInitOk = false;
static volatile bool sCalLoaded = false;

namespace MotorControl {
// trigger 上緣偵測（Core 1）
static bool prevTrigger = false;
// 是否已看過第一個有效命令。上電時若 trigger 已經是 1（開關壓著），
// 不可當成「上緣」直接啟動，必須先放開（0）再按（0→1）才算一次。
static bool triggerLatched = false;

static void init() {
    EEPROM.begin(Limits::EEPROM_SIZE);
    const bool loaded = Limits::load();

    i2cBusInit(Pin::I2C_HZ);
    const bool ok = pcaInit();
    ServoSeq::stop(); // 上電先全部回 HOME
    prevTrigger = false;
    triggerLatched = false;

    // ⚠ 本函式在 Core 1 (setup1) 執行，禁止呼叫 Serial。
    //    狀態存成旗標，交由 Core 0 的 loop() 印出。
    sCalLoaded = loaded;
    sPcaInitOk = ok;
    sPcaInitDone = true;
}

/// 收到 Hub 命令：trigger 上緣 → 做下一個動作（一次只做一動）
static void executeMotion(const ChassisSnapshot &cmd) {
    const bool trigger = (cmd.trigger != 0);

    // 第一個命令只記錄目前開關狀態，不動作。
    if(!triggerLatched) {
        triggerLatched = true;
        prevTrigger = trigger;
        return;
    }

    if(trigger && !prevTrigger && !ServoSeq::isRunning()) {
        ServoSeq::startNext();
    }
    prevTrigger = trigger;
}

static void estop() {
    ServoSeq::stop();
    ServoSeq::resetSequence();
    prevTrigger = false;
    triggerLatched = false; // E-Stop 解除後，先放開 trigger 才會再做動作
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
    Serial.println("[STORAGE] boot: 只有伺服馬達（PCA9685 I2C0 GP4/GP5）");

    slave.begin(); // 硬體 UART0 (GP0/GP1) → Hub
    slave.setIdleHook(storageIdleHook);

    Serial.println("[STORAGE] 等待 Hub 命令（trigger 上緣 → 全部通道跑一次）");
}

void loop() {
    slave.update();
    updateStatusLed();

    // Core 1 累積的日誌（序列訊息）在這裡印出
    LogQ::drain();

    // Core 1 初始化完成後，由 Core 0 印一次設定摘要（Serial 只能在 Core 0）
    static bool initPrinted = false;
    if(!initPrinted && sPcaInitDone) {
        initPrinted = true;
        Serial.printf("[STORAGE] 通道校正%s\n",
                      sCalLoaded ? "已從 EEPROM 載入" : "使用預設值（16 通道）");
        // 注意：newlib-nano 沒開浮點 printf，這裡只用整數輸出
        Serial.printf("[STORAGE] PCA9685 0x%02X %s | HOME=%u° OUT=%u° "
                      "dwell=%lums settle=%lums\n",
                      sAddr, sPcaInitOk ? "OK 50Hz" : "無回應！",
                      static_cast<unsigned>(SeqCfg::HOME_DEG),
                      static_cast<unsigned>(SeqCfg::OUT_DEG),
                      static_cast<unsigned long>(SeqCfg::DWELL_MS),
                      static_cast<unsigned long>(SeqCfg::SETTLE_MS));

        // 逐通道 dump：確認 EEPROM 校正是否讓 HOME/OUT 變成同一個位置
        // （minDeg == maxDeg 或範圍太窄 → 脈寬一樣 → 馬達看起來「不動」）
        Serial.println("[STORAGE] ch en  us(min-max)  deg(min-max) inv "
                       "HOME/OUT(us)");
        for(uint8_t ch = 0; ch < Pca::CHANNEL_COUNT; ch++) {
            const ChannelLimit &l = Limits::channels[ch];
            const uint16_t hUs = angleToPulseUs(ch, SeqCfg::HOME_DEG);
            const uint16_t oUs = angleToPulseUs(ch, SeqCfg::OUT_DEG);
            Serial.printf("  %2u  %u  %4u-%4u  %3u-%3u  %u  %4u/%4u%s\n",
                          static_cast<unsigned>(ch),
                          static_cast<unsigned>(l.enabled),
                          static_cast<unsigned>(l.minUs),
                          static_cast<unsigned>(l.maxUs),
                          static_cast<unsigned>(l.minDeg),
                          static_cast<unsigned>(l.maxDeg),
                          static_cast<unsigned>(
                              Limits::effectiveInvert(ch) ? 1 : 0),
                          static_cast<unsigned>(hUs),
                          static_cast<unsigned>(oUs),
                          (l.enabled && hUs == oUs) ? "  <-- 動作無效" : "");
        }
        Serial.println("[STORAGE] ※ 若上表脈寬都正確但伺服不動：檢查 PCA9685 V+ "
                       "伺服電源、GND 共地、OE 是否拉低");
    }

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
                          "cycles=%lu | i2cErr=%lu seqErr=%lu drop=%lu\n",
                          slave.msSinceLastValidCmd(),
                          static_cast<unsigned>(slave.validCmdCount()),
                          ServoSeq::isRunning() ? "RUN" : "IDLE",
                          static_cast<unsigned>(ServoSeq::progress()),
                          static_cast<unsigned>(ServoSeq::total()),
                          static_cast<int>(ServoSeq::currentChannel()),
                          static_cast<unsigned long>(ServoSeq::doneCount()),
                          static_cast<unsigned long>(sI2cErrors),
                          static_cast<unsigned long>(
                              ServoSeq::writeErrorCount()),
                          static_cast<unsigned long>(LogQ::dropped));
        }
    }
}

// ============================================================================
// Core 1
// ============================================================================
void setup1() { MotorControl::init(); }

void loop1() { slave.runCore1(); }
