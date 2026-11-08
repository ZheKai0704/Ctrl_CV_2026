// ============================================================================
// mechanism_test — 機構動作測試 (RP2350 + PCA9685)
//
// 行為（依觸發順序逐組動作）：
//   1. 開機 / 停止時：所有「已啟用」通道停在 HOME 角度（預設 30°）
//   2. 按下啟動：依觸發順序 g2 → g5 → g1 → g6 → g0 → g7
//      （組 n = CH(2n), CH(2n+1)；序列未列到的 g3、g4 排在最後）
//      每一組做兩段：
//        a) 逐顆驗證行程：CH(2n) 30° → 120° → 停 1 秒 → 30°
//                         CH(2n+1) 30° → 120° → 停 1 秒 → 30°
//        b) 整組開啟：CH(2n) → 120°、CH(2n+1) → 120° 並保持
//      這組就維持開啟，等 SETTLE(0.5s) 再換下一組；後面的組同樣開啟並保持
//      全部做完 = 一個循環（可選擇重複），結束時所有組都維持開啟，
//      要回 HOME 必須按 `q`（或 `m` 重設角度）
//   3. 通道分組（每 2 顆一組）：(0,1) (2,3) (4,5) ... (14,15)
//      `g <n>` 只測第 n 組
//
// 重要：
//   - 角度使用「邏輯角度」，與 pca9685_angle 相同慣例；
//     每通道的 0° 零點、行程、反向、實體工作範圍、啟用狀態
//     都直接沿用 EEPROM 內的設定（與角度測試工具同一份，MAGIC=PCA4）。
//   - 奇數通道（CH1/3/5/7/9/11/13/15）因機構左右對稱安裝，
//     韌體強制反向（與 EEPROM invert 以 OR 合併，不會二次反向）。
//   - 停用通道（目前 CH6~9）會自動跳過，不會被驅動。
//   - 全程非阻塞狀態機，Serial 隨時可下命令。
//
// 接線：
//   GP4=SDA, GP5=SCL, PCA9685 VCC=3.3V, GND 共地, V+=伺服電源, OE 拉低
//
// Serial 命令（115200，※大小寫有別）：
//   s            執行一次完整循環（依觸發順序 g2→g5→g1→g6→g0→g7）
//   S            重複執行完整循環（再按 S 或 q 停止）
//   g <0~7>      只測第 n 組：(0,1)=0, (2,3)=1, ... (14,15)=7
//   q            停止（所有通道回 HOME）
//   m <home> <out>   設定角度（預設 30 120）
//   t <dwellMs> <settleMs>  設定停留 / 換組前等待（預設 1000 500）
//   p            顯示狀態與設定
//   l            列出各通道校正設定
//   x            緊急關閉全部輸出
//   r            重新開機
//   h            說明
// ============================================================================
#include <Arduino.h>
#include <EEPROM.h>
#include <hardware/i2c.h>
#include <math.h>
#include <pico/stdlib.h>
#include <stdlib.h>
#include <string.h>

// ============================================================================
// 硬體 / 常數（與 pca9685_angle 相同）
// ============================================================================
namespace Pin {
constexpr uint I2C_SDA = 4;
constexpr uint I2C_SCL = 5;
constexpr uint I2C_HZ = 100000;
constexpr uint I2C_SCAN_HZ = 100000;
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
constexpr uint32_t PERIOD_US = 20000;
constexpr uint32_t COUNT_SPAN = 4096;
constexpr uint8_t SCAN_FIRST = 0x08;
constexpr uint8_t SCAN_LAST = 0x77;
} // namespace Pca

namespace Servo {
constexpr float ANGLE_MIN = 0.0f;
constexpr float ANGLE_MAX = 180.0f;
constexpr uint16_t DEFAULT_MIN_US = 600;
constexpr uint16_t DEFAULT_MAX_US = 2400;
constexpr uint16_t ABS_MIN_US = 300;
constexpr uint16_t ABS_MAX_US = 3000;
constexpr uint16_t MIN_SPAN_US = 100;
// 機構左右成對安裝，奇數通道（CH1/3/5/7/9/11/13/15）實體方向相反。
// 這裡在硬體層強制反向，與 EEPROM 的 invert 採 OR 合併（不會二次反向）。
constexpr bool MIRROR_ODD_CHANNELS = true;
} // namespace Servo

// ============================================================================
// 每通道校正（EEPROM 格式必須與 pca9685_angle 完全一致）
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
constexpr uint32_t MAGIC = 0x50434134; // "PCA4"（與角度工具共用）

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
// I2C
// ============================================================================
static i2c_inst_t *sI2c = nullptr;
static uint8_t sAddr = Pca::ADDR_DEFAULT;
static uint32_t sI2cErrors = 0;
static constexpr uint8_t I2C_RETRY_MAX = 3;
static constexpr uint32_t I2C_TIMEOUT_US = 10000;

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

static bool regRead(uint8_t reg, uint8_t *out, size_t len) {
    return rawRead(sAddr, reg, out, len);
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

static float physicalToLogical(uint8_t ch, float physDeg) {
    if(ch >= Pca::CHANNEL_COUNT)
        ch = 0;
    if(Limits::effectiveInvert(ch))
        physDeg = Servo::ANGLE_MAX - physDeg;
    return clampAngle(physDeg);
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
        buf[4] = 0x10;
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
// 初始化 / 設定輸出
// ============================================================================
static uint8_t i2cScan(bool verbose) {
    uint8_t found = 0;
    if(verbose)
        Serial.printf("[SCAN] SDA=GP%u SCL=GP%u @ %uHz\n", Pin::I2C_SDA,
                      Pin::I2C_SCL, Pin::I2C_SCAN_HZ);
    for(uint8_t addr = Pca::SCAN_FIRST; addr <= Pca::SCAN_LAST; addr++) {
        uint8_t dummy = 0;
        if(i2c_read_timeout_us(sI2c, addr, &dummy, 1, false, I2C_TIMEOUT_US / 2) >=
           0) {
            found++;
            if(verbose)
                Serial.printf("  0x%02X%s\n", addr,
                              (addr >= 0x40 && addr <= 0x7F) ? "  <-- PCA9685?"
                                                             : "");
        }
        busy_wait_us(200);
    }
    if(verbose && found == 0)
        Serial.print("[SCAN] 找不到裝置 → 檢查 VCC/GND/上拉/接線\n");
    return found;
}

static bool pcaInit() {
    uint8_t probe = 0;
    if(i2c_read_timeout_us(sI2c, sAddr, &probe, 1, false, I2C_TIMEOUT_US) < 0) {
        Serial.printf("[INIT] 0x%02X 無回應\n", sAddr);
        return false;
    }
    regWrite(Pca::MODE1, Pca::MODE1_SLEEP);
    regWrite(Pca::PRE_SCALE, 121);
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
    Serial.print("[INIT] OK 50Hz\n");
    return true;
}

// ============================================================================
// 序列狀態機
// ============================================================================
namespace Seq {
static float homeDeg = 30.0f;
static float outDeg = 120.0f;
static uint32_t dwellMs = 1000;  // OUT 停留
static uint32_t settleMs = 500;   // 回 HOME 後/換組前的等待

// 觸發順序（與 src/storage/main.cpp 的 SeqCfg::GROUP_ORDER 相同）：
//   組 n = CH(2n), CH(2n+1) → g2 → g5 → g1 → g6 → g0 → g7
// 完整循環（'s'）依此順序走；'g' 不帶參數時也依此順序換下一組。
// 順序表未列到的組（g3、g4）排在最後，仍可用 'g 3' / 'g 4' 單獨測試。
static constexpr uint8_t GROUP_ORDER[] = {2, 5, 1, 6, 0, 7};
static constexpr uint8_t GROUP_ORDER_COUNT =
    sizeof(GROUP_ORDER) / sizeof(GROUP_ORDER[0]);
static constexpr uint8_t GROUP_COUNT = Pca::CHANNEL_COUNT / 2;
static constexpr uint8_t GROUP_CHANNELS = 2;

static bool running = false;
static bool repeating = false;
static uint8_t steps[Pca::CHANNEL_COUNT] = {0};
static uint8_t stepGroup[Pca::CHANNEL_COUNT] = {0}; // 每個 step 所屬組別
static uint8_t stepCount = 0;
static uint8_t nextGroup = GROUP_ORDER[0]; // 'g' 不帶參數時使用，依 GROUP_ORDER 換組
static uint8_t stepIdx = 0;

// 每一組的動作流程：
//   [組內每顆] OUT → dwell → HOME → settle    ← 逐顆驗證行程
//   然後 [整組] 全部 → OUT（開啟）並保持       ← 這組就維持開啟，不再回 HOME
//   → 下一組
enum Phase : uint8_t { PHASE_NONE = 0,
                       PHASE_OUT,
                       PHASE_DWELL,
                       PHASE_HOME,
                       PHASE_SETTLE,
                       PHASE_OPEN,
                       PHASE_GAP };
static Phase phase = PHASE_NONE;
static uint32_t phaseMs = 0;
static uint32_t cycleStartMs = 0;

static void goHomeAll() {
    for(uint8_t ch = 0; ch < Pca::CHANNEL_COUNT; ch++) {
        if(channelEnabled(ch))
            setAngle(ch, homeDeg);
        else
            setChannelOff(ch);
    }
}

static void stop(); // 前向宣告

static bool groupInOrder(uint8_t group) {
    for(uint8_t i = 0; i < GROUP_ORDER_COUNT; i++)
        if(GROUP_ORDER[i] == group)
            return true;
    return false;
}

/// 把某一組內「已啟用」的通道附加到 steps（停用通道自動跳過）
static void appendGroup(uint8_t group) {
    for(uint8_t k = 0; k < GROUP_CHANNELS; k++) {
        const uint8_t ch = static_cast<uint8_t>(group * GROUP_CHANNELS + k);
        if(ch < Pca::CHANNEL_COUNT && channelEnabled(ch) &&
           stepCount < Pca::CHANNEL_COUNT) {
            stepGroup[stepCount] = group;
            steps[stepCount++] = ch;
        }
    }
}

/// 'g' 不帶參數時的下一組：依 GROUP_ORDER 前進，最後回到第一組
static uint8_t nextGroupAfter(uint8_t group) {
    for(uint8_t i = 0; i < GROUP_ORDER_COUNT; i++)
        if(GROUP_ORDER[i] == group)
            return GROUP_ORDER[(i + 1) % GROUP_ORDER_COUNT];
    return GROUP_ORDER[0];
}

/// 完整循環：先依觸發順序（g2→g5→g1→g6→g0→g7），補上順序表外的組
static void buildAllSteps() {
    stepCount = 0;
    for(uint8_t i = 0; i < GROUP_ORDER_COUNT; i++)
        appendGroup(GROUP_ORDER[i]);
    for(uint8_t g = 0; g < GROUP_COUNT; g++)
        if(!groupInOrder(g))
            appendGroup(g);
    stepIdx = 0;
}

static bool buildGroupSteps(uint8_t group) {
    stepCount = 0;
    appendGroup(group);
    stepIdx = 0;
    return stepCount > 0;
}

static void start(bool repeat) {
    stop();
    buildAllSteps();
    if(stepCount == 0) {
        Serial.print("[SEQ] 沒有已啟用的通道\n");
        return;
    }
    repeating = repeat;
    running = true;
    phase = PHASE_OUT;
    phaseMs = millis();
    cycleStartMs = phaseMs;
    Serial.printf("[SEQ] 開始%s循環，共 %u 顆：", repeating ? "重複" : "單次",
                  stepCount);
    for(uint8_t i = 0; i < stepCount; i++) {
        Serial.printf(" CH%u", steps[i]);
        setAngle(steps[i], homeDeg); // 先確保在 HOME
    }
    Serial.print("\n");
}

static bool startGroup(uint8_t group) {
    stop();
    nextGroup = nextGroupAfter(group);
    if(!buildGroupSteps(group)) {
        Serial.printf("[SEQ] 第 %u 組（CH%u/CH%u）都沒有啟用\n", group,
                      static_cast<unsigned>(group * 2),
                      static_cast<unsigned>(group * 2 + 1));
        return false;
    }
    repeating = false;
    running = true;
    phase = PHASE_OUT;
    phaseMs = millis();
    cycleStartMs = phaseMs;
    Serial.printf("[SEQ] 測試第 %u 組（CH%u/CH%u）：", group,
                  static_cast<unsigned>(group * 2),
                  static_cast<unsigned>(group * 2 + 1));
    for(uint8_t i = 0; i < stepCount; i++) {
        Serial.printf(" CH%u", steps[i]);
        setAngle(steps[i], homeDeg);
    }
    Serial.print("\n");
    return true;
}

static void stop() {
    const bool wasRunning = running;
    running = false;
    phase = PHASE_NONE;
    goHomeAll();
    if(wasRunning) {
        const int32_t h = static_cast<int32_t>(lroundf(homeDeg * 10.0f));
        Serial.printf("[SEQ] 停止，全部回 %ld.%u°\n",
                      static_cast<long>(h / 10), static_cast<unsigned>(h % 10));
    }
}

static void finishCycle() {
    if(repeating) {
        stepIdx = 0;
        phase = PHASE_OUT;
        phaseMs = millis();
        cycleStartMs = phaseMs;
        Serial.print("[SEQ] 循環完成，重新開始（各組續維持 OUT）\n");
        return;
    }
    // 完成後「不」回 HOME：全部維持在 OUT（開啟）狀態，需 'q' / 'm' 才會回 HOME
    running = false;
    phase = PHASE_NONE;
    Serial.printf("[SEQ] 循環完成（%lu ms），各組維持 OUT，全部回 HOME 請按 q\n",
                  static_cast<unsigned long>(millis() - cycleStartMs));
}

static void printMove(uint8_t ch, float deg, const char *suffix) {
    const int32_t t = static_cast<int32_t>(lroundf(deg * 10.0f));
    Serial.printf("[SEQ] CH%u → %ld.%u°%s\n", ch, static_cast<long>(t / 10),
                  static_cast<unsigned>(t % 10), suffix);
}

/// 這個 step 是不是所屬組的最後一顆
static bool lastOfGroup(uint8_t idx) {
    return (static_cast<uint8_t>(idx + 1) >= stepCount) ||
           (stepGroup[idx + 1] != stepGroup[idx]);
}

/// 整組切到 OUT 並保持（開啟），回傳開啟了幾顆
static uint8_t openGroup(uint8_t group) {
    uint8_t n = 0;
    for(uint8_t i = 0; i < stepCount; i++) {
        if(stepGroup[i] != group)
            continue;
        setAngle(steps[i], outDeg);
        printMove(steps[i], outDeg, "（開啟）");
        n++;
    }
    return n;
}

static void update() {
    if(!running)
        return;
    const uint32_t now = millis();
    switch(phase) {
    case PHASE_OUT: {
        const uint8_t ch = steps[stepIdx];
        setAngle(ch, outDeg);
        printMove(ch, outDeg, "");
        phase = PHASE_DWELL;
        phaseMs = now;
        break;
    }
    case PHASE_DWELL:
        if(now - phaseMs >= dwellMs) {
            setAngle(steps[stepIdx], homeDeg);
            printMove(steps[stepIdx], homeDeg, "");
            phase = PHASE_HOME;
            phaseMs = now;
        }
        break;
    case PHASE_HOME:
        if(now - phaseMs >= settleMs) {
            if(lastOfGroup(stepIdx)) {
                // 組內每顆都驗過了 → 整組一起開到 OUT 並保持
                phase = PHASE_OPEN;
            } else {
                stepIdx++;
                phase = PHASE_OUT;
            }
            phaseMs = now;
        }
        break;
    case PHASE_OPEN: {
        const uint8_t group = stepGroup[stepIdx];
        const uint8_t n = openGroup(group);
        Serial.printf("[SEQ] 第 %u 組完成：%u 顆維持開啟\n",
                      static_cast<unsigned>(group), static_cast<unsigned>(n));
        phase = PHASE_GAP;
        phaseMs = now;
        break;
    }
    case PHASE_GAP:
        if(now - phaseMs >= settleMs) {
            if(static_cast<uint8_t>(stepIdx + 1) >= stepCount) {
                finishCycle();
            } else {
                stepIdx++;
                phase = PHASE_OUT;
            }
        }
        break;
    default:
        break;
    }
}
} // namespace Seq

// ============================================================================
// 命令列
// ============================================================================
static int32_t degToTenths(float deg) {
    return static_cast<int32_t>(lroundf(deg * 10.0f));
}

static void printDeg(const char *label, float deg) {
    const int32_t t = degToTenths(deg);
    Serial.printf("%s%ld.%u°", label, static_cast<long>(t / 10),
                  static_cast<unsigned>(t % 10));
}

static void printStatus() {
    Serial.print("[STATUS] ");
    printDeg("home=", Seq::homeDeg);
    Serial.print(" ");
    printDeg("out=", Seq::outDeg);
    Serial.printf(" dwell=%lums settle=%lums\n",
                  static_cast<unsigned long>(Seq::dwellMs),
                  static_cast<unsigned long>(Seq::settleMs));
    Serial.print("[STATUS] enabled:");
    uint8_t n = 0;
    for(uint8_t ch = 0; ch < Pca::CHANNEL_COUNT; ch++) {
        if(channelEnabled(ch)) {
            Serial.printf(" CH%u", ch);
            n++;
        }
    }
    Serial.printf("%s（%u 顆）\n", n ? "" : " （無）", n);
    Serial.printf("[STATUS] 下一組 g → %u（CH%u/CH%u）\n", Seq::nextGroup,
                  static_cast<unsigned>(Seq::nextGroup * 2),
                  static_cast<unsigned>(Seq::nextGroup * 2 + 1));
    if(Seq::running) {
        Serial.printf("[STATUS] 執行中 %s，第 %u/%u 顆 CH%u\n",
                      Seq::repeating ? "重複循環" : "單次循環",
                      static_cast<unsigned>(Seq::stepIdx + 1),
                      static_cast<unsigned>(Seq::stepCount),
                      Seq::steps[Seq::stepIdx]);
    } else {
        Serial.print("[STATUS] 閒置（保持目前位置；q 全部回 HOME）\n");
    }
    if(sI2cErrors)
        Serial.printf("[STATUS] I2C 累計錯誤 %lu\n",
                      static_cast<unsigned long>(sI2cErrors));
}

static void printLimits() {
    Serial.print("[LIMIT] ch  en  dir  0°(us) 180°(us) 實體角度\n");
    for(uint8_t ch = 0; ch < Pca::CHANNEL_COUNT; ch++) {
        const ChannelLimit &l = Limits::channels[ch];
        Serial.printf("[LIMIT] %2u  %s  %s  %5u   %5u   %3u~%3u°\n", ch,
                      l.enabled ? "on " : "OFF",
                      Limits::effectiveInvert(ch) ? "rev" : "fwd",
                      l.minUs, l.maxUs, l.minDeg, l.maxDeg);
    }
}

static void printHelp() {
    Serial.print(
        "\n=== 機構測試 ===\n"
        "  s            完整循環（依觸發順序 g2→g5→g1→g6→g0→g7）\n"
        "               每組：逐顆 30→120→30 驗證，再整組開到 120 並保持\n"
        "  S            重複循環（再按 q 停止）\n"
        "  g [0~7]      測第 n 組（(0,1)=0 ... (14,15)=7）；不給參數 = 依觸發順序下一組\n"
        "  q            停止，全部回 HOME\n"
        "  m <home> <out>       設定角度（預設 30 120）\n"
        "  t <dwell> <settle>   設定 OUT 停留 / 回位與換組等待 ms（預設 1000 500）\n"
        "  p            狀態\n"
        "  l            各通道校正設定\n"
        "  x            緊急關閉全部輸出\n"
        "  r            重新開機\n"
        "  h            本說明\n\n");
}

static bool parseLong(const char *s, long *out) {
    if(s == nullptr || *s == '\0')
        return false;
    char *end = nullptr;
    const long v = strtol(s, &end, 0);
    if(end == s)
        return false;
    *out = v;
    return true;
}

static bool parseFloat(const char *s, float *out) {
    if(s == nullptr || *s == '\0')
        return false;
    char *end = nullptr;
    const float v = strtof(s, &end);
    if(end == s)
        return false;
    *out = v;
    return true;
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
    long a = 0, b = 0;
    float f = 0.0f;

    if(strcmp(cmd, "h") == 0 || strcmp(cmd, "H") == 0 ||
       strcmp(cmd, "?") == 0) {
        printHelp();
    } else if(strcmp(cmd, "s") == 0) {
        Seq::start(false);
    } else if(strcmp(cmd, "S") == 0) {
        Seq::start(true);
    } else if(strcmp(cmd, "g") == 0) {
        if(argc < 2) {
            Serial.printf("[SEQ] 下一組 = %u\n", Seq::nextGroup);
            Seq::startGroup(Seq::nextGroup);
            return;
        }
        if(!parseLong(argv[1], &a) || a < 0 || a > 7) {
            Serial.print("用法：g [0~7]（不給參數=依觸發順序下一組）\n");
            return;
        }
        Seq::startGroup(static_cast<uint8_t>(a));
    } else if(strcmp(cmd, "q") == 0 || strcmp(cmd, "Q") == 0) {
        Seq::stop();
    } else if(strcmp(cmd, "m") == 0 || strcmp(cmd, "M") == 0) {
        if(argc < 3 || !parseFloat(argv[1], &f)) {
            Serial.print("用法：m <home> <out>（例：m 30 120）\n");
            return;
        }
        float outDeg = 0.0f;
        if(!parseFloat(argv[2], &outDeg) || f < 0.0f || outDeg > 180.0f) {
            Serial.print("用法：m <home> <out>（0~180）\n");
            return;
        }
        Seq::homeDeg = clampAngle(f);
        Seq::outDeg = clampAngle(outDeg);
        Seq::stop();
        printStatus();
    } else if(strcmp(cmd, "t") == 0 || strcmp(cmd, "T") == 0) {
        if(argc < 3 || !parseLong(argv[1], &a) || !parseLong(argv[2], &b) ||
           a < 0 || b < 0) {
            Serial.print("用法：t <dwellMs> <settleMs>\n");
            return;
        }
        Seq::dwellMs = static_cast<uint32_t>(a);
        Seq::settleMs = static_cast<uint32_t>(b);
        printStatus();
    } else if(strcmp(cmd, "p") == 0 || strcmp(cmd, "P") == 0) {
        printStatus();
    } else if(strcmp(cmd, "l") == 0 || strcmp(cmd, "L") == 0) {
        printLimits();
    } else if(strcmp(cmd, "x") == 0 || strcmp(cmd, "X") == 0) {
        Seq::running = false;
        Seq::phase = Seq::PHASE_NONE;
        allChannelsOff();
        Serial.print("[EMERGENCY] 全部輸出關閉\n");
    } else if(strcmp(cmd, "r") == 0 || strcmp(cmd, "R") == 0) {
        Seq::goHomeAll();
        Serial.print("[SYS] 重新開機\n");
        delay(50);
        watchdog_reboot(0, 0, 0);
    } else {
        Serial.printf("未知命令 '%s'（h 看說明）\n", cmd);
    }
}

// ============================================================================
// Arduino 進入點
// ============================================================================
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.begin(115200);
    delay(300);

    EEPROM.begin(Limits::EEPROM_SIZE);
    const bool loaded = Limits::load();

    i2cBusInit(Pin::I2C_HZ);
    Serial.print("\n\n=== 機構測試 (mechanism test) ===\n");
    Serial.printf("SDA=GP%u SCL=GP%u addr=0x%02X\n", Pin::I2C_SDA, Pin::I2C_SCL,
                  sAddr);
    Serial.printf("[CAL] 通道校正%s\n", loaded ? "已從 EEPROM 載入" : "使用預設值");

    (void)i2cScan(true);
    uint8_t probe = 0;
    const bool ok =
        i2c_read_timeout_us(sI2c, sAddr, &probe, 1, false, I2C_TIMEOUT_US) >= 0;
    if(ok)
        pcaInit();
    else
        Serial.print("[INIT] 未偵測到 PCA9685\n");

    if(ok) {
        Seq::goHomeAll();
        Serial.print("[READY] 所有已啟用通道已在 HOME 位置\n");
    }
    printStatus();
    printHelp();
}

void loop() {
    static char line[64];
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

    Seq::update();

    // LED：執行中快閃、閒置慢閃
    static uint32_t lastBlinkMs = 0;
    static bool ledOn = false;
    const uint32_t nowMs = millis();
    const uint32_t interval = Seq::running ? 100 : 1000;
    if(nowMs - lastBlinkMs >= interval) {
        lastBlinkMs = nowMs;
        ledOn = !ledOn;
        digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
    }
}
