// ============================================================================
// pca9685_angle_test — PCA9685 伺服「角度 ↔ 脈衝」測試 (RP2350)
//
// ★ 每通道獨立端點（避免伺服撞機械極限而卡住/發熱/嗡嗡叫）
//    - 預設端點：0° = 600μs、180° = 2400μs（比 500/2500 保守，相容性較好）
//    - 可用 'ma <ch> <minUs> <maxUs>' 逐通道校正，設定會存進 EEPROM，重開機保留
//    - 'A <deg>' 會依照每通道自己的端點換算，所以同一角度不會有人頂死點
//
// 對應公式（以該通道端點為準，50Hz / 20ms 週期）：
//   pulseUs  = minUs + (maxUs - minUs) × deg / 180
//   offCount = round(pulseUs × 4096 / 20000)
//
//   例如端點 600~2400：
//     deg     pulseUs    offCount
//       0°      600 μs       123
//      45°      1050 μs      215
//      90°      1500 μs      307
//     135°      1950 μs      400
//     180°      2400 μs      492
//
// 接線：
//   RP2350 GP4 → PCA9685 SDA      RP2350 GP5 → PCA9685 SCL
//   PCA9685 VCC → 3.3V            GND 全部共地
//   PCA9685 V+  → 伺服獨立電源（5~6V），OE 拉低
//   SDA/SCL 需 4.7k 上拉（模組多已內建）
//
// ⚠️ 伺服馬達無法被偵測。若某顆伺服在某角度持續發出嗡嗡聲/發熱，
//    代表它已到機械極限：立刻下 'x' 全部關閉，然後用 'c' 找安全端點，
//    再用 'ma' 寫入該通道的安全範圍。
//
// Serial 命令（115200，※大小寫有別）：
//   h / H / ?            說明
//   s / S                掃描 I2C 匯流排
//   i / I                初始化 PCA9685（50Hz）
//   l / L                列出 16 通道端點設定
//   d [ch]               該通道 0~180°（每 15°）對應表
//   v [ch]               0~180° 寫入→讀回驗證（預設 CH0）
//   a <ch> <deg>         指定通道轉到角度（0~180，可小數）
//   A <deg>              全部通道各自換算後轉到該角度
//   c <ch> <us>          直接輸出脈寬（忽略端點限制，校正找極限用）
//   p <ch>               讀回該通道目前脈寬/角度
//   l                    端點一覽
//   m <minUs> <maxUs>    一次設定全部通道端點（例：m 600 2400）
//   ma <ch> <minUs> <maxUs>  設定單通道端點並存檔
//   mr <ch>              單通道端點回復預設
//   mR                   全部通道端點回復預設
//   w <ch> [step] [ms]   單通道 0↔180° 來回掃描（依該通道端點）
//   W [step] [ms]        全通道輪流掃描（每通道 1.5 秒）
//   q                    停止掃描
//   o <ch> / x           關閉單通道 / 全部關閉（卡住時先用這個）
//   n <addr>             變更 PCA9685 位址
// ============================================================================
#include <Arduino.h>
#include <EEPROM.h>
#include <hardware/i2c.h>
#include <math.h>
#include <pico/stdlib.h>
#include <stdlib.h>
#include <string.h>

// ============================================================================
// 硬體 / 常數
// ============================================================================
namespace Pin {
constexpr uint I2C_SDA = 4; // GP4
constexpr uint I2C_SCL = 5; // GP5
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

// 安全預設：0° = 600μs，180° = 2400μs（每度 10μs）
constexpr uint16_t DEFAULT_MIN_US = 600;
constexpr uint16_t DEFAULT_MAX_US = 2400;
constexpr uint16_t DEFAULT_SPAN_US = DEFAULT_MAX_US - DEFAULT_MIN_US;

// 允許設定範圍（避免誤輸入爆衝）
constexpr uint16_t ABS_MIN_US = 300;
constexpr uint16_t ABS_MAX_US = 3000;
constexpr uint16_t MIN_SPAN_US = 100;

constexpr float TABLE_STEP_DEG = 15.0f;
constexpr float VERIFY_STEP_DEG = 15.0f;
} // namespace Servo

// ============================================================================
// 每通道端點（EEPROM 持久化）
// ============================================================================
struct ChannelLimit {
    uint16_t minUs;   // 0° 的脈寬（各通道機械零點）
    uint16_t maxUs;   // 180° 的脈寬（= 0° + 行程）
    uint8_t minDeg;   // 允許的工作角度下限（例如 10）
    uint8_t maxDeg;   // 允許的工作角度上限（通常 180）
    uint8_t enabled;  // 0 = 此通道不動作（A / W 會跳過）
    uint8_t invert;   // 1 = 角度反向（0° 與 180° 對調）
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
    // 不含最後的 checksum 欄位
    for(size_t i = 0; i < sizeof(Persist) - sizeof(uint32_t); i++)
        sum += raw[i];
    return sum;
}

static bool validPulse(uint16_t minUs, uint16_t maxUs) {
    return minUs >= Servo::ABS_MIN_US && maxUs <= Servo::ABS_MAX_US &&
           maxUs >= static_cast<uint16_t>(minUs + Servo::MIN_SPAN_US);
}

static bool valid(const ChannelLimit &l) {
    return validPulse(l.minUs, l.maxUs) && l.minDeg <= l.maxDeg &&
           l.maxDeg <= 180 && l.invert <= 1 && l.enabled <= 1;
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

static void resetAll() {
    for(uint8_t i = 0; i < Pca::CHANNEL_COUNT; i++)
        channels[i] = defaultChannel();
}

static bool load() {
    Persist p = {};
    EEPROM.get(EEPROM_ADDR, p);
    if(p.magic != MAGIC || p.checksum != checksumOf(p)) {
        resetAll();
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
        channels[i] = valid(l) ? l : defaultChannel();
    }
    return true;
}

static bool save() {
    Persist p = {};
    p.magic = MAGIC;
    for(uint8_t i = 0; i < Pca::CHANNEL_COUNT; i++) {
        p.minUs[i] = channels[i].minUs;
        p.maxUs[i] = channels[i].maxUs;
        p.minDeg[i] = channels[i].minDeg;
        p.maxDeg[i] = channels[i].maxDeg;
        p.enabled[i] = channels[i].enabled;
        p.invert[i] = channels[i].invert;
    }
    p.checksum = checksumOf(p);
    EEPROM.put(EEPROM_ADDR, p);
    return EEPROM.commit();
}

/// 設定 0°/180° 脈寬（不改工作角度與啟用狀態）
static bool setPulses(uint8_t ch, uint16_t minUs, uint16_t maxUs) {
    if(ch >= Pca::CHANNEL_COUNT || !validPulse(minUs, maxUs))
        return false;
    channels[ch].minUs = minUs;
    channels[ch].maxUs = maxUs;
    return true;
}

/// 只改 0° 零點，維持原本行程
static bool setZero(uint8_t ch, uint16_t zeroUs) {
    if(ch >= Pca::CHANNEL_COUNT)
        return false;
    const uint16_t span = static_cast<uint16_t>(channels[ch].maxUs - channels[ch].minUs);
    return setPulses(ch, zeroUs, static_cast<uint16_t>(zeroUs + span));
}

/// 只改行程（180° - 0°）
static bool setSpan(uint8_t ch, uint16_t spanUs) {
    if(ch >= Pca::CHANNEL_COUNT)
        return false;
    return setPulses(ch, channels[ch].minUs,
                     static_cast<uint16_t>(channels[ch].minUs + spanUs));
}

static bool setDegRange(uint8_t ch, uint8_t minDeg, uint8_t maxDeg) {
    if(ch >= Pca::CHANNEL_COUNT || minDeg > maxDeg || maxDeg > 180)
        return false;
    channels[ch].minDeg = minDeg;
    channels[ch].maxDeg = maxDeg;
    return true;
}

static void setEnabled(uint8_t ch, bool en) {
    if(ch < Pca::CHANNEL_COUNT)
        channels[ch].enabled = en ? 1 : 0;
}

/// 角度反向（0° 與 180° 對調）
static void setInvert(uint8_t ch, bool inv) {
    if(ch < Pca::CHANNEL_COUNT)
        channels[ch].invert = inv ? 1 : 0;
}

static void resetChannel(uint8_t ch) {
    if(ch < Pca::CHANNEL_COUNT)
        channels[ch] = defaultChannel();
}
} // namespace Limits

// ============================================================================
// I2C
// ============================================================================
static i2c_inst_t *sI2c = nullptr;
static uint8_t sAddr = Pca::ADDR_DEFAULT;

static uint32_t sI2cErrors = 0;
static uint32_t sI2cRetries = 0;
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
        if(attempt + 1 < I2C_RETRY_MAX) {
            sI2cRetries++;
            i2cBusRecover();
        }
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
        if(attempt + 1 < I2C_RETRY_MAX) {
            sI2cRetries++;
            i2cBusRecover();
        }
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
// 角度 ↔ 脈衝 ↔ offCount（依通道端點）
// ============================================================================
static float clampAngle(float deg) {
    if(deg < Servo::ANGLE_MIN)
        return Servo::ANGLE_MIN;
    if(deg > Servo::ANGLE_MAX)
        return Servo::ANGLE_MAX;
    return deg;
}

/// 邏輯角度 → 實際（實體）角度：反向時 0°↔180° 對調
static float logicalToPhysical(uint8_t ch, float logicalDeg) {
    if(ch >= Pca::CHANNEL_COUNT)
        ch = 0;
    logicalDeg = clampAngle(logicalDeg);
    float phys = Limits::channels[ch].invert
                     ? (Servo::ANGLE_MAX - logicalDeg)
                     : logicalDeg;
    // 工作角度限制的是「實體」角度（機械安全範圍）
    const ChannelLimit &l = Limits::channels[ch];
    if(phys < static_cast<float>(l.minDeg))
        phys = static_cast<float>(l.minDeg);
    if(phys > static_cast<float>(l.maxDeg))
        phys = static_cast<float>(l.maxDeg);
    return phys;
}

/// 實際（實體）角度 → 邏輯角度
static float physicalToLogical(uint8_t ch, float physDeg) {
    if(ch >= Pca::CHANNEL_COUNT)
        ch = 0;
    if(Limits::channels[ch].invert)
        physDeg = Servo::ANGLE_MAX - physDeg;
    return clampAngle(physDeg);
}

/// 該通道邏輯角度允許的範圍（給掃描/驗證用）
static void logicalRange(uint8_t ch, float *lo, float *hi) {
    if(ch >= Pca::CHANNEL_COUNT)
        ch = 0;
    const ChannelLimit &l = Limits::channels[ch];
    if(l.invert) {
        *lo = Servo::ANGLE_MAX - static_cast<float>(l.maxDeg);
        *hi = Servo::ANGLE_MAX - static_cast<float>(l.minDeg);
    } else {
        *lo = static_cast<float>(l.minDeg);
        *hi = static_cast<float>(l.maxDeg);
    }
}

static bool channelEnabled(uint8_t ch) {
    return ch < Pca::CHANNEL_COUNT && Limits::channels[ch].enabled != 0;
}

/// 該通道的角度 → 脈衝寬度（μs）
static uint16_t angleToPulseUs(uint8_t ch, float deg) {
    if(ch >= Pca::CHANNEL_COUNT)
        ch = 0;
    const ChannelLimit &l = Limits::channels[ch];
    const float span = static_cast<float>(l.maxUs - l.minUs);
    // 先轉成實體角度（反向時對調），再依實體工作範圍夾住
    const float phys = logicalToPhysical(ch, deg);
    return static_cast<uint16_t>(static_cast<float>(l.minUs) +
                                 span * phys / Servo::ANGLE_MAX + 0.5f);
}

/// 該通道的脈衝寬度 → 角度（近似，顯示用）
static float pulseUsToAngle(uint8_t ch, uint16_t us) {
    if(ch >= Pca::CHANNEL_COUNT)
        ch = 0;
    const ChannelLimit &l = Limits::channels[ch];
    const int32_t span = static_cast<int32_t>(l.maxUs) - l.minUs;
    if(span <= 0)
        return 0.0f;
    float phys = (static_cast<float>(us) - l.minUs) * Servo::ANGLE_MAX /
                 static_cast<float>(span);
    if(phys < 0.0f)
        phys = 0.0f;
    else if(phys > Servo::ANGLE_MAX)
        phys = Servo::ANGLE_MAX;
    return physicalToLogical(ch, phys);
}

static uint16_t pulseUsToCount(uint16_t pulseUs) {
    if(pulseUs > Pca::PERIOD_US)
        pulseUs = static_cast<uint16_t>(Pca::PERIOD_US);
    return static_cast<uint16_t>(
        (static_cast<uint32_t>(pulseUs) * Pca::COUNT_SPAN + Pca::PERIOD_US / 2) /
        Pca::PERIOD_US);
}

static uint16_t countToPulseUs(uint16_t count) {
    return static_cast<uint16_t>(static_cast<uint32_t>(count) *
                                 Pca::PERIOD_US / Pca::COUNT_SPAN);
}

/// 角度 → 十分之一度（整數；Serial.printf 不支援 %f）
static int32_t degToTenths(float deg) {
    return static_cast<int32_t>(lroundf(deg * 10.0f));
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
        buf[4] = 0x10; // OFF full → 恆低
    } else {
        buf[3] = static_cast<uint8_t>(count & 0xFF);
        buf[4] = static_cast<uint8_t>((count >> 8) & 0x0F);
    }
    return rawWrite(sAddr, buf, sizeof(buf));
}

static bool readChannelCount(uint8_t ch, uint16_t *on, uint16_t *off) {
    if(ch >= Pca::CHANNEL_COUNT)
        return false;
    uint8_t rb[4] = {0, 0, 0, 0};
    const uint8_t reg =
        static_cast<uint8_t>(Pca::LED0_ON_L + ch * Pca::CHAN_STEP);
    if(!regRead(reg, rb, sizeof(rb)))
        return false;
    if(on)
        *on = static_cast<uint16_t>(rb[0] | ((rb[1] & 0x0F) << 8));
    if(off)
        *off = static_cast<uint16_t>(rb[2] | ((rb[3] & 0x0F) << 8));
    return true;
}

static bool setAngle(uint8_t ch, float deg) {
    return writeChannelCount(ch, pulseUsToCount(angleToPulseUs(ch, deg)));
}

/// 直接寫入脈寬（忽略端點限制，校正用）
static bool setRawPulseUs(uint8_t ch, uint16_t us) {
    return writeChannelCount(ch, pulseUsToCount(us));
}

static bool setChannelOff(uint8_t ch) { return writeChannelCount(ch, 0); }

static void allChannelsOff() {
    for(uint8_t ch = 0; ch < Pca::CHANNEL_COUNT; ch++)
        setChannelOff(ch);
}

// ============================================================================
// 初始化
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
    if(verbose) {
        if(found == 0)
            Serial.print("[SCAN] 找不到任何裝置 → 檢查 VCC/GND/上拉/接線\n");
        else
            Serial.printf("[SCAN] 共 %u 個裝置\n", found);
    }
    return found;
}

static bool pcaInit() {
    uint8_t probe = 0;
    if(i2c_read_timeout_us(sI2c, sAddr, &probe, 1, false, I2C_TIMEOUT_US) < 0) {
        Serial.printf("[INIT] 0x%02X 無回應（先下 s 掃描）\n", sAddr);
        return false;
    }

    regWrite(Pca::MODE1, Pca::MODE1_SLEEP);
    regWrite(Pca::PRE_SCALE, 121); // 25MHz / (4096×50) - 1
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
    Serial.print("[INIT] OK 50Hz, 16 通道已關閉\n");
    return true;
}

// ============================================================================
// 端點一覽 / 角度表 / 驗證
// ============================================================================
static void printLimits() {
    Serial.print("[LIMIT] ch  en  dir   0°(us)  180°(us)  工作角度\n");
    for(uint8_t ch = 0; ch < Pca::CHANNEL_COUNT; ch++) {
        const ChannelLimit &l = Limits::channels[ch];
        Serial.printf("[LIMIT] %2u  %s  %s  %5u    %5u    %3u~%3u°\n", ch,
                      l.enabled ? "on " : "OFF", l.invert ? "rev" : "fwd",
                      l.minUs, l.maxUs, l.minDeg, l.maxDeg);
    }
}

static void printAngleTable(uint8_t ch) {
    if(ch >= Pca::CHANNEL_COUNT)
        return;
    const ChannelLimit &l = Limits::channels[ch];
    Serial.printf("[TABLE] CH%u 端點：0°=%u us, 180°=%u us, 工作 %u~%u°, %s%s\n",
                  ch, l.minUs, l.maxUs, l.minDeg, l.maxDeg,
                  l.invert ? "反向" : "正向",
                  l.enabled ? "" : "（已停用）");
    Serial.print("[TABLE]   deg    pulseUs   offCount\n");
    float lo = 0.0f, hi = 0.0f;
    logicalRange(ch, &lo, &hi);
    for(float deg = lo; deg <= hi + 0.001f; deg += Servo::TABLE_STEP_DEG) {
        const uint16_t us = angleToPulseUs(ch, deg);
        const int32_t t = degToTenths(deg);
        Serial.printf("[TABLE] %5ld.%u    %5u      %5u\n",
                      static_cast<long>(t / 10), static_cast<unsigned>(t % 10),
                      us, pulseUsToCount(us));
    }
}

static void verifyAngleMapping(uint8_t ch) {
    if(ch >= Pca::CHANNEL_COUNT) {
        Serial.print("[VERIFY] 通道超出範圍\n");
        return;
    }
    const uint32_t errBefore = sI2cErrors;
    uint8_t passCount = 0;
    uint8_t total = 0;
    bool monotonic = true;
    uint16_t lastOff = 0;
    const ChannelLimit &l = Limits::channels[ch];

    Serial.printf("[VERIFY] CH%u 端點 %u~%u us, 工作 %u~%u°（每 %d°）%s\n", ch,
                  l.minUs, l.maxUs, l.minDeg, l.maxDeg,
                  static_cast<int>(Servo::VERIFY_STEP_DEG),
                  l.enabled ? "" : "  ※此通道已停用");
    Serial.print("[VERIFY]  deg  expectUs  expectCnt  gotCnt  gotUs  result\n");

    float loDeg = 0.0f, hiDeg = 0.0f;
    logicalRange(ch, &loDeg, &hiDeg);
    for(float deg = loDeg; deg <= hiDeg + 0.001f;
        deg += Servo::VERIFY_STEP_DEG) {
        const uint16_t expectUs = angleToPulseUs(ch, deg);
        const uint16_t expectCnt = pulseUsToCount(expectUs);

        bool ok = false;
        uint16_t gotOff = 0;
        for(uint8_t attempt = 0; attempt < 3 && !ok; attempt++) {
            ok = setAngle(ch, deg);
            busy_wait_us(300);

            uint16_t gotOn = 0;
            ok = ok && readChannelCount(ch, &gotOn, &gotOff);
            ok = ok && gotOn == 0 && gotOff == expectCnt;
            if(!ok && attempt < 2) {
                sI2cRetries++;
                busy_wait_us(500);
            }
        }

        // 反向通道的 offCount 應該隨角度遞減
        if(deg > loDeg + 0.001f) {
            if(l.invert ? (gotOff >= lastOff) : (gotOff <= lastOff))
                monotonic = false;
        }
        lastOff = gotOff;

        const int32_t t = degToTenths(deg);
        Serial.printf("[VERIFY] %5ld.%u  %6u     %6u   %6u  %5u  %s\n",
                      static_cast<long>(t / 10), static_cast<unsigned>(t % 10),
                      expectUs, expectCnt, gotOff, countToPulseUs(gotOff),
                      ok ? "PASS" : "FAIL");
        total++;
        if(ok)
            passCount++;
    }

    allChannelsOff();
    Serial.printf("[VERIFY] RESULT %s %u/%u, 單調%s=%s, I2C 錯誤 +%lu\n",
                  passCount == total ? "PASS" : "FAIL", passCount, total,
                  l.invert ? "遞減" : "遞增", monotonic ? "YES" : "NO",
                  static_cast<unsigned long>(sI2cErrors - errBefore));
}

// ============================================================================
// 掃描（依各通道端點）
// ============================================================================
namespace Sweep {
static bool active = false;
static bool allMode = false;
static uint8_t ch = 0;
static uint8_t allCh = 0;
static float angle = Servo::ANGLE_MIN;
static float dir = 1.0f;
static float stepDeg = 2.0f;
static float loDeg = Servo::ANGLE_MIN;
static float hiDeg = Servo::ANGLE_MAX;
static uint32_t intervalMs = 20;
static uint32_t hopMs = 1500;
static uint32_t lastMs = 0;
static uint32_t lastHopMs = 0;

static void start(uint8_t channel, float step, uint32_t ms) {
    if(!channelEnabled(channel)) {
        Serial.printf("[SWEEP] CH%u 已停用（用 'me %u 1' 啟用）\n", channel,
                      channel);
        return;
    }
    active = true;
    allMode = false;
    ch = channel;
    logicalRange(ch, &loDeg, &hiDeg);
    angle = loDeg;
    dir = 1.0f;
    stepDeg = (step > 0.05f) ? step : 2.0f;
    intervalMs = (ms > 0) ? ms : 20;
    lastMs = millis();
    allChannelsOff();
    const int32_t t = degToTenths(stepDeg);
    const int32_t tlo = degToTenths(loDeg);
    const int32_t thi = degToTenths(hiDeg);
    Serial.printf("[SWEEP] CH%u 邏輯 %ld.%u~%ld.%u°（%u~%u us），每步 "
                  "%ld.%u°，%lums（q 停止）\n",
                  ch, static_cast<long>(tlo / 10), static_cast<unsigned>(tlo % 10),
                  static_cast<long>(thi / 10), static_cast<unsigned>(thi % 10),
                  angleToPulseUs(ch, loDeg), angleToPulseUs(ch, hiDeg),
                  static_cast<long>(t / 10), static_cast<unsigned>(t % 10),
                  static_cast<unsigned long>(intervalMs));
}

static void startAll(float step, uint32_t ms) {
    active = true;
    allMode = true;
    allCh = 0;
    // 找第一個已啟用通道，並套用其邏輯範圍
    for(uint8_t i = 0; i < Pca::CHANNEL_COUNT; i++) {
        allCh = i;
        if(channelEnabled(i))
            break;
    }
    logicalRange(allCh, &loDeg, &hiDeg);
    angle = loDeg;
    dir = 1.0f;
    stepDeg = (step > 0.05f) ? step : 4.0f;
    intervalMs = (ms > 0) ? ms : 20;
    lastMs = millis();
    lastHopMs = millis();
    allChannelsOff();
    Serial.printf("[SWEEP] 全通道輪流（只跑已啟用通道，每通道 %lums，q 停止）\n",
                  static_cast<unsigned long>(hopMs));
}

static void stop() {
    if(!active)
        return;
    active = false;
    allChannelsOff();
    Serial.print("[SWEEP] 停止，全部通道關閉\n");
}

static void update() {
    if(!active)
        return;
    const uint32_t now = millis();
    if(now - lastMs < intervalMs)
        return;
    lastMs = now;

    angle += dir * stepDeg;
    if(angle >= hiDeg) {
        angle = hiDeg;
        dir = -1.0f;
    } else if(angle <= loDeg) {
        angle = loDeg;
        dir = 1.0f;
    }

    if(!allMode) {
        setAngle(ch, angle);
        return;
    }

    setAngle(allCh, angle);
    if(now - lastHopMs >= hopMs) {
        lastHopMs = now;
        setChannelOff(allCh);
        // 找下一個已啟用的通道
        for(uint8_t i = 0; i < Pca::CHANNEL_COUNT; i++) {
            allCh = static_cast<uint8_t>((allCh + 1) % Pca::CHANNEL_COUNT);
            if(channelEnabled(allCh))
                break;
        }
        logicalRange(allCh, &loDeg, &hiDeg);
        angle = loDeg;
        dir = 1.0f;
        Serial.printf("[SWEEP] → CH%u\n", static_cast<unsigned>(allCh));
    }
}
} // namespace Sweep

// ============================================================================
// 微步走查：一次只改一點脈寬，讓使用者聽/看伺服何時不再抖動
//   t <ch> <fromUs> <toUs> [stepUs] [holdMs]
// ============================================================================
namespace Walk {
static bool active = false;
static uint8_t ch = 0;
static uint16_t us = 0;
static uint16_t toUs = 0;
static int16_t stepUs = 10;
static uint32_t holdMs = 800;
static uint32_t lastMs = 0;

static void start(uint8_t channel, uint16_t fromUs, uint16_t toUs_, int16_t step,
                  uint32_t hold) {
    active = true;
    ch = channel;
    us = fromUs;
    toUs = toUs_;
    stepUs = (step == 0) ? 10 : step;
    holdMs = (hold == 0) ? 800 : hold;
    lastMs = millis();
    allChannelsOff();
    setRawPulseUs(ch, us);
    Serial.printf("[WALK] CH%u %u→%u us，每步 %d us，每步停留 %lums（q 停止）\n",
                  ch, us, toUs, static_cast<int>(stepUs),
                  static_cast<unsigned long>(holdMs));
    Serial.printf("[WALK] CH%u 現在 %u us\n", ch, us);
}

static void stop() {
    if(!active)
        return;
    active = false;
    allChannelsOff();
    Serial.print("[WALK] 停止，全部通道關閉（用 ma 寫入安全端點）\n");
}

static void update() {
    if(!active)
        return;
    const uint32_t now = millis();
    if(now - lastMs < holdMs)
        return;
    lastMs = now;

    if((stepUs > 0 && us + stepUs > toUs) || (stepUs < 0 && us + stepUs < toUs)) {
        us = toUs;
    } else {
        us = static_cast<uint16_t>(static_cast<int32_t>(us) + stepUs);
    }
    setRawPulseUs(ch, us);
    Serial.printf("[WALK] CH%u 現在 %u us\n", ch, us);
    if(us == toUs) {
        active = false;
        Serial.print("[WALK] 到達終點（q 以外可用 ma 寫入端點）\n");
    }
}
} // namespace Walk

// ============================================================================
// 命令列
// ============================================================================
static void printHelp() {
    Serial.print(
        "\n=== PCA9685 角度測試 (0~180°)  ※指令大小寫有別 ===\n"
        "  s                    掃描 I2C\n"
        "  i                    初始化 PCA9685 (50Hz)\n"
        "  l                    列出 16 通道設定（啟用/零點/行程/角度範圍）\n"
        "  d [ch]               角度對應表（依該通道端點，預設 CH0）\n"
        "  v [ch]               0~180° 寫入→讀回驗證（預設 CH0）\n"
        "  a <ch> <deg>         指定通道轉到角度（可小數）\n"
        "  A <deg>              全部通道（各自端點換算）\n"
        "  c <ch> <us>          直接輸出脈寬（忽略端點，校正用）\n"
        "  p <ch>               讀回該通道目前脈寬/角度\n"
        "  m <minUs> <maxUs>    設定全部通道端點\n"
        "  ma <ch> <minUs> <maxUs>  設定單通道 0°/180° 脈寬（自動存檔）\n"
        "  mz <ch> <zeroUs>     只改 0° 零點（維持行程）\n"
        "  ms <ch> <spanUs>     只改行程（180°-0°）\n"
        "  md <ch> <minDeg> <maxDeg>  設定實體工作角度（例：md 1 10 180）\n"
        "  me <ch> <0|1>        啟用/停用該通道（停用後 A/W 會跳過）\n"
        "  mi <ch> <0|1>        角度反向（1 = 0° 與 180° 對調）\n"
        "  mr <ch>              單通道回復預設端點\n"
        "  mR                   全部回復預設端點\n"
        "  t <ch> <fromUs> <toUs> [stepUs] [holdMs]\n"
        "                       微步走查脈寬（找抖動/異音的安全端點）\n"
        "  w <ch> [step] [ms]   單通道 0↔180° 來回掃描\n"
        "  W [step] [ms]        全通道輪流掃描（找伺服用）\n"
        "  q                    停止掃描\n"
        "  o <ch> / x           關閉單通道 / 全部關閉（卡住先用 x）\n"
        "  n <addr>             變更 PCA9685 位址\n"
        "  r                    重新開機（驗證端點是否存檔）\n"
        "  h                    本說明\n"
        "\n預設端點 600~2400us（安全範圍）。伺服在某角度嗡嗡叫 =\n"
        "已到機械極限，請下 x 關閉，再用 c 找安全值、ma 寫入。\n\n");
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

static void printChannelPulse(uint8_t ch) {
    uint16_t on = 0, off = 0;
    if(!readChannelCount(ch, &on, &off)) {
        Serial.printf("[CH%u] 讀取失敗\n", ch);
        return;
    }
    if(off == 0) {
        Serial.printf("[CH%u] 關閉（on=%u off=0）\n", ch, on);
        return;
    }
    const uint16_t us = countToPulseUs(off);
    const float deg = pulseUsToAngle(ch, us);
    const int32_t t = degToTenths(deg);
    const ChannelLimit &l = Limits::channels[ch];
    Serial.printf("[CH%u] %s %s on=%u off=%u → %u us ≈ %ld.%u°（0°=%u, "
                  "行程=%u, 工作 %u~%u°）\n",
                  ch, l.enabled ? "on " : "OFF", l.invert ? "rev" : "fwd", on, off, us,
                  static_cast<long>(t / 10), static_cast<unsigned>(t % 10),
                  l.minUs, static_cast<unsigned>(l.maxUs - l.minUs), l.minDeg,
                  l.maxDeg);
}

static void handleCommand(char *line) {
    char *argv[6] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    uint8_t argc = 0;
    char *save = nullptr;
    for(char *tok = strtok_r(line, " \t\r\n", &save); tok != nullptr && argc < 6;
        tok = strtok_r(nullptr, " \t\r\n", &save)) {
        argv[argc++] = tok;
    }
    if(argc == 0)
        return;

    // 大小寫有別：'a' 單通道 / 'A' 全通道、'w' 單通道掃描 / 'W' 全通道掃描、
    // 'm' 全通道端點 / 'ma' 單通道端點 / 'mr' / 'mR'
    const char *cmd = argv[0];
    long a = 0, b = 0, c2 = 0;
    float f = 0.0f;

    if(strcmp(cmd, "h") == 0 || strcmp(cmd, "H") == 0 || strcmp(cmd, "?") == 0) {
        printHelp();
    } else if(strcmp(cmd, "s") == 0 || strcmp(cmd, "S") == 0) {
        i2cBusInit(Pin::I2C_SCAN_HZ);
        i2cScan(true);
        i2cBusInit(Pin::I2C_HZ);
    } else if(strcmp(cmd, "i") == 0 || strcmp(cmd, "I") == 0) {
        if(!pcaInit())
            Serial.print("[INIT] 失敗\n");
    } else if(strcmp(cmd, "l") == 0 || strcmp(cmd, "L") == 0) {
        printLimits();
    } else if(strcmp(cmd, "d") == 0 || strcmp(cmd, "D") == 0) {
        uint8_t ch = 0;
        if(argc >= 2 && parseLong(argv[1], &a) && a >= 0 &&
           a < Pca::CHANNEL_COUNT)
            ch = static_cast<uint8_t>(a);
        printAngleTable(ch);
    } else if(strcmp(cmd, "v") == 0 || strcmp(cmd, "V") == 0) {
        uint8_t ch = 0;
        if(argc >= 2 && parseLong(argv[1], &a) && a >= 0 &&
           a < Pca::CHANNEL_COUNT)
            ch = static_cast<uint8_t>(a);
        else if(argc >= 2) {
            Serial.print("用法：v <ch 0~15>\n");
            return;
        }
        verifyAngleMapping(ch);
    } else if(strcmp(cmd, "a") == 0) {
        if(argc < 3 || !parseLong(argv[1], &a) || !parseFloat(argv[2], &f) ||
           a < 0 || a >= Pca::CHANNEL_COUNT) {
            Serial.print("用法：a <ch 0~15> <deg 0~180>\n");
            return;
        }
        Walk::stop();
        Sweep::stop();
        const uint8_t ch = static_cast<uint8_t>(a);
        if(!channelEnabled(ch)) {
            Serial.printf("[CH%u] 已停用（用 'me %u 1' 啟用）\n", ch, ch);
            return;
        }
        if(setAngle(ch, f)) {
            const float achieved =
                physicalToLogical(ch, logicalToPhysical(ch, f));
            const int32_t t = degToTenths(achieved);
            Serial.printf("[CH%u] %ld.%u° → %u us (offCount=%u)\n", ch,
                          static_cast<long>(t / 10), static_cast<unsigned>(t % 10),
                          angleToPulseUs(ch, f),
                          pulseUsToCount(angleToPulseUs(ch, f)));
        } else {
            Serial.print("[CH] 寫入失敗\n");
        }
    } else if(strcmp(cmd, "A") == 0) {
        if(argc < 2 || !parseFloat(argv[1], &f)) {
            Serial.print("用法：A <deg 0~180>\n");
            return;
        }
        Walk::stop();
        Sweep::stop();
        f = clampAngle(f);
        uint8_t applied = 0, skipped = 0;
        for(uint8_t ch = 0; ch < Pca::CHANNEL_COUNT; ch++) {
            if(!channelEnabled(ch)) {
                skipped++;
                continue;
            }
            setAngle(ch, f);
            applied++;
        }
        const int32_t t = degToTenths(f);
        Serial.printf("[ALL] %ld.%u°（依各通道端點/工作範圍換算）已套用 %u 顆%s\n",
                      static_cast<long>(t / 10), static_cast<unsigned>(t % 10),
                      applied,
                      skipped ? "，停用通道已跳過" : "");
    } else if(strcmp(cmd, "c") == 0 || strcmp(cmd, "C") == 0) {
        if(argc < 3 || !parseLong(argv[1], &a) || !parseLong(argv[2], &b) ||
           a < 0 || a >= Pca::CHANNEL_COUNT || b < Servo::ABS_MIN_US ||
           b > Servo::ABS_MAX_US) {
            Serial.printf("用法：c <ch 0~15> <us %u~%u>\n", Servo::ABS_MIN_US,
                          Servo::ABS_MAX_US);
            return;
        }
        Sweep::stop();
        const uint8_t ch = static_cast<uint8_t>(a);
        setRawPulseUs(ch, static_cast<uint16_t>(b));
        Serial.printf("[CH%u] 原始脈寬 %ld us（offCount=%u）\n", ch, b,
                      pulseUsToCount(static_cast<uint16_t>(b)));
    } else if(strcmp(cmd, "p") == 0 || strcmp(cmd, "P") == 0) {
        if(argc < 2 || !parseLong(argv[1], &a) || a < 0 ||
           a >= Pca::CHANNEL_COUNT) {
            Serial.print("用法：p <ch 0~15>\n");
            return;
        }
        printChannelPulse(static_cast<uint8_t>(a));
    } else if(strcmp(cmd, "m") == 0 || strcmp(cmd, "M") == 0) {
        if(argc < 3 || !parseLong(argv[1], &a) || !parseLong(argv[2], &b)) {
            Serial.print("用法：m <minUs> <maxUs>（例：m 600 2400）\n");
            return;
        }
        bool ok = true;
        for(uint8_t ch = 0; ch < Pca::CHANNEL_COUNT; ch++)
            ok = Limits::setPulses(ch, static_cast<uint16_t>(a),
                                   static_cast<uint16_t>(b)) &&
                 ok;
        if(!ok) {
            Serial.printf("[CAL] 失敗：需 %u <= min < max <= %u\n",
                          Servo::ABS_MIN_US, Servo::ABS_MAX_US);
            return;
        }
        Limits::save();
        Serial.printf("[CAL] 全部通道端點 = %ld~%ld us（已存檔）\n", a, b);
    } else if(strcmp(cmd, "ma") == 0) {
        if(argc < 4 || !parseLong(argv[1], &a) || !parseLong(argv[2], &b) ||
           !parseLong(argv[3], &c2) || a < 0 || a >= Pca::CHANNEL_COUNT) {
            Serial.print("用法：ma <ch 0~15> <minUs> <maxUs>\n");
            return;
        }
        if(!Limits::setPulses(static_cast<uint8_t>(a), static_cast<uint16_t>(b),
                              static_cast<uint16_t>(c2))) {
            Serial.printf("[CAL] 失敗：需 %u <= min < max <= %u\n",
                          Servo::ABS_MIN_US, Servo::ABS_MAX_US);
            return;
        }
        Limits::save();
        Serial.printf("[CAL] CH%ld 端點 = %ld~%ld us（已存檔）\n", a, b, c2);
    } else if(strcmp(cmd, "mz") == 0) {
        // 設定 0° 零點（維持原有行程）
        if(argc < 3 || !parseLong(argv[1], &a) || !parseLong(argv[2], &b) ||
           a < 0 || a >= Pca::CHANNEL_COUNT) {
            Serial.print("用法：mz <ch 0~15> <zeroUs>\n");
            return;
        }
        if(!Limits::setZero(static_cast<uint8_t>(a), static_cast<uint16_t>(b))) {
            Serial.printf("[CAL] 失敗：零點+行程需在 %u~%u us 內\n",
                          Servo::ABS_MIN_US, Servo::ABS_MAX_US);
            return;
        }
        Limits::save();
        Serial.printf("[CAL] CH%ld 0°=%ld us, 180°=%u us（已存檔）\n", a, b,
                      Limits::channels[a].maxUs);
    } else if(strcmp(cmd, "ms") == 0) {
        // 設定行程（180° - 0°）
        if(argc < 3 || !parseLong(argv[1], &a) || !parseLong(argv[2], &b) ||
           a < 0 || a >= Pca::CHANNEL_COUNT) {
            Serial.print("用法：ms <ch 0~15> <spanUs>\n");
            return;
        }
        if(!Limits::setSpan(static_cast<uint8_t>(a), static_cast<uint16_t>(b))) {
            Serial.printf("[CAL] 失敗：行程需 >= %u 且 180° 不超過 %u us\n",
                          Servo::MIN_SPAN_US, Servo::ABS_MAX_US);
            return;
        }
        Limits::save();
        Serial.printf("[CAL] CH%ld 行程=%ld us, 0°=%u, 180°=%u（已存檔）\n", a, b,
                      Limits::channels[a].minUs, Limits::channels[a].maxUs);
    } else if(strcmp(cmd, "md") == 0) {
        // 設定工作角度範圍
        if(argc < 4 || !parseLong(argv[1], &a) || !parseLong(argv[2], &b) ||
           !parseLong(argv[3], &c2) || a < 0 || a >= Pca::CHANNEL_COUNT ||
           b < 0 || c2 > 180 || b > c2) {
            Serial.print("用法：md <ch 0~15> <minDeg> <maxDeg>（0~180）\n");
            return;
        }
        Limits::setDegRange(static_cast<uint8_t>(a), static_cast<uint8_t>(b),
                            static_cast<uint8_t>(c2));
        Limits::save();
        Serial.printf("[CAL] CH%ld 工作角度 %ld~%ld°（已存檔）\n", a, b, c2);
    } else if(strcmp(cmd, "mi") == 0) {
        // 角度反向（0° ↔ 180°）
        if(argc < 3 || !parseLong(argv[1], &a) || !parseLong(argv[2], &b) ||
           a < 0 || a >= Pca::CHANNEL_COUNT) {
            Serial.print("用法：mi <ch 0~15> <0|1>（1=反向）\n");
            return;
        }
        Limits::setInvert(static_cast<uint8_t>(a), b != 0);
        Limits::save();
        Serial.printf("[CAL] CH%ld %s（已存檔）\n", a,
                      b ? "角度反向（0°↔180°）" : "角度正向");
    } else if(strcmp(cmd, "me") == 0) {
        // 啟用/停用通道
        if(argc < 3 || !parseLong(argv[1], &a) || !parseLong(argv[2], &b) ||
           a < 0 || a >= Pca::CHANNEL_COUNT) {
            Serial.print("用法：me <ch 0~15> <0|1>\n");
            return;
        }
        Limits::setEnabled(static_cast<uint8_t>(a), b != 0);
        Limits::save();
        if(b == 0)
            setChannelOff(static_cast<uint8_t>(a));
        Serial.printf("[CAL] CH%ld %s（已存檔）\n", a,
                      b ? "啟用" : "停用並關閉輸出");
    } else if(strcmp(cmd, "mr") == 0) {
        if(argc < 2 || !parseLong(argv[1], &a) || a < 0 ||
           a >= Pca::CHANNEL_COUNT) {
            Serial.print("用法：mr <ch 0~15>\n");
            return;
        }
        Limits::resetChannel(static_cast<uint8_t>(a));
        Limits::save();
        Serial.printf("[CAL] CH%ld 回復預設（0°=%u, 180°=%u, 0~180°, 啟用）\n", a,
                      Servo::DEFAULT_MIN_US, Servo::DEFAULT_MAX_US);
    } else if(strcmp(cmd, "mR") == 0) {
        Limits::resetAll();
        Limits::save();
        Serial.printf("[CAL] 全部通道回復預設（0°=%u, 180°=%u, 0~180°, 啟用）\n",
                      Servo::DEFAULT_MIN_US, Servo::DEFAULT_MAX_US);
    } else if(strcmp(cmd, "w") == 0) {
        if(argc < 2 || !parseLong(argv[1], &a) || a < 0 ||
           a >= Pca::CHANNEL_COUNT) {
            Serial.print("用法：w <ch 0~15> [stepDeg] [intervalMs]\n");
            return;
        }
        float step = 2.0f;
        long ms = 20;
        if(argc >= 3 && parseFloat(argv[2], &f))
            step = f;
        if(argc >= 4 && parseLong(argv[3], &b))
            ms = b;
        Sweep::start(static_cast<uint8_t>(a), step, static_cast<uint32_t>(ms));
    } else if(strcmp(cmd, "W") == 0) {
        float step = 4.0f;
        long ms = 20;
        if(argc >= 2 && parseFloat(argv[1], &f))
            step = f;
        if(argc >= 3 && parseLong(argv[2], &b))
            ms = b;
        Sweep::startAll(step, static_cast<uint32_t>(ms));
    } else if(strcmp(cmd, "t") == 0 || strcmp(cmd, "T") == 0) {
        if(argc < 4 || !parseLong(argv[1], &a) || !parseLong(argv[2], &b) ||
           !parseLong(argv[3], &c2) || a < 0 || a >= Pca::CHANNEL_COUNT ||
           b < Servo::ABS_MIN_US || b > Servo::ABS_MAX_US ||
           c2 < Servo::ABS_MIN_US || c2 > Servo::ABS_MAX_US) {
            Serial.printf("用法：t <ch> <fromUs> <toUs> [stepUs] [holdMs]（%u~%u）\n",
                          Servo::ABS_MIN_US, Servo::ABS_MAX_US);
            return;
        }
        int16_t step = (c2 >= b) ? 10 : -10;
        long hold = 800;
        long stepArg = 0;
        if(argc >= 5 && parseLong(argv[4], &stepArg) && stepArg != 0)
            step = static_cast<int16_t>((c2 >= b) ? stepArg : -stepArg);
        if(argc >= 6 && !parseLong(argv[5], &hold))
            hold = 800;
        Sweep::stop();
        Walk::start(static_cast<uint8_t>(a), static_cast<uint16_t>(b),
                    static_cast<uint16_t>(c2), step,
                    static_cast<uint32_t>(hold));
    } else if(strcmp(cmd, "q") == 0 || strcmp(cmd, "Q") == 0) {
        Walk::stop();
        Sweep::stop();
    } else if(strcmp(cmd, "o") == 0 || strcmp(cmd, "O") == 0) {
        if(argc < 2 || !parseLong(argv[1], &a) || a < 0 ||
           a >= Pca::CHANNEL_COUNT) {
            Serial.print("用法：o <ch 0~15>\n");
            return;
        }
        Sweep::stop();
        setChannelOff(static_cast<uint8_t>(a));
        Serial.printf("[CH%ld] 關閉\n", a);
    } else if(strcmp(cmd, "x") == 0 || strcmp(cmd, "X") == 0) {
        Walk::stop();
        Sweep::stop();
        allChannelsOff();
        Serial.print("[ALL] 全部關閉\n");
    } else if(strcmp(cmd, "r") == 0 || strcmp(cmd, "R") == 0) {
        allChannelsOff();
        Serial.print("[SYS] 重新開機（測試 EEPROM 端點是否保留）\n");
        delay(50);
        watchdog_reboot(0, 0, 0);
    } else if(strcmp(cmd, "n") == 0 || strcmp(cmd, "N") == 0) {
        if(argc < 2 || !parseLong(argv[1], &a) || a < Pca::SCAN_FIRST ||
           a > Pca::SCAN_LAST) {
            Serial.print("用法：n <addr>（例：n 0x40）\n");
            return;
        }
        sAddr = static_cast<uint8_t>(a);
        Serial.printf("[ADDR] 目標改為 0x%02X（請重新下 i）\n", sAddr);
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
    Serial.print("\n\n=== PCA9685 角度測試 (0~180°) ===\n");
    Serial.printf("SDA=GP%u SCL=GP%u bus=i2c%u addr=0x%02X\n", Pin::I2C_SDA,
                  Pin::I2C_SCL, sI2c == i2c1 ? 1u : 0u, sAddr);
    Serial.printf("[CAL] 端點%s，預設 %u~%u us（l 看各通道）\n",
                  loaded ? "已從 EEPROM 載入" : "使用預設值",
                  Servo::DEFAULT_MIN_US, Servo::DEFAULT_MAX_US);

    (void)i2cScan(true);
    uint8_t probe = 0;
    if(i2c_read_timeout_us(sI2c, sAddr, &probe, 1, false, I2C_TIMEOUT_US) >= 0)
        pcaInit();
    else
        Serial.print("[AUTO] 未偵測到 PCA9685，請確認接線後下 s / i\n");

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

    Sweep::update();
    Walk::update();

    static uint32_t lastBlinkMs = 0;
    static bool ledOn = false;
    const uint32_t nowMs = millis();
    if(nowMs - lastBlinkMs >= 500) {
        lastBlinkMs = nowMs;
        ledOn = !ledOn;
        digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
    }
}
