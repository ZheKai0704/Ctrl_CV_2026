// ============================================================================
// pca9685_test — PCA9685 (16 通道 PWM / 伺服驅動板) 連線與掃描測試 (RP2350)
//
// 用途：
//   1. 掃描 I2C 匯流排，確認 PCA9685 是否在線（ACK）
//   2. 讀寫暫存器驗證 I2C 連線品質（MODE1 / MODE2 / PRE_SCALE）
//   3. 逐通道輸出伺服脈衝，用肉眼確認「哪幾個通道真的有接伺服馬達」
//
// ⚠️ 重要觀念：
//   PCA9685 只能偵測「PCA9685 這顆 IC 本身」，**無法偵測伺服馬達是否存在**。
//   伺服馬達只是被動吃 PWM 訊號，沒有任何回報線。因此「搜尋伺服馬達」只能
//   靠掃描 I2C 找到 PCA9685，再逐通道輸出脈衝，用眼睛看哪一顆會動。
//   本程式提供 `w`（單通道掃描）與 `W`（全通道輪流掃描）來做這件事。
//
// 接線（依需求：SDA=GP4, SCL=GP5）：
//   RP2350 GP4 ── SDA ── PCA9685 SDA
//   RP2350 GP5 ── SCL ── PCA9685 SCL
//   PCA9685 VCC → 3.3V（邏輯電源）
//   PCA9685 V+  → 伺服馬達獨立 5~6V 電源（不要用 RP2350 的 5V 拉 10 顆伺服）
//   GND 全部共地（RP2350 / PCA9685 / 伺服電源）
//   SDA/SCL 需外接 4.7kΩ 上拉到 3.3V（模組多半已內建；沒內建就要加）
//   OE 腳位拉低（致能輸出），部分模組已接地
//
// RP2350 I2C 對應（本檔會依 SDA 腳位自動選匯流排）：
//   I2C0: GP0/1, GP4/5, GP8/9, GP12/13, ...
//   I2C1: GP2/3, GP6/7, GP10/11, ...
//   → SDA=GP4 落在 I2C0
//
// Serial 命令（115200）：
//   s            掃描 I2C 匯流排（列出所有 ACK 的位址）
//   i            初始化 PCA9685（MODE1/PRE_SCALE=50Hz/MODE2）
//   r            讀回暫存器（MODE1/MODE2/PRE_SCALE + CH0 設定）
//   t            讀寫回測（MODE2 寫入→讀回比對）
//   v            全 16 通道寫入→讀回驗證（自動化測試用）
//   c <ch> <us>  指定通道輸出脈衝（ch 0~15, us 500~2500）
//   a <us>       全部 16 通道輸出同一脈衝
//   o <ch>       關閉單一通道輸出（恆低）
//   x            全部通道關閉
//   w <ch>       單通道來回掃描（500↔2500μs），觀察該顆伺服
//   W            全通道輪流掃描（每通道 2 秒），找出有接伺服的位置
//   q            停止掃描
//   n <addr>     變更 PCA9685 位址（十進位或 0x 開頭十六進位）
//   h / ?        說明
// ============================================================================
#include <Arduino.h>
#include <ctype.h>
#include <hardware/i2c.h>
#include <pico/stdlib.h>
#include <stdlib.h>
#include <string.h>

// ============================================================================
// 腳位 / 參數
// ============================================================================
namespace Pin {
constexpr uint I2C_SDA = 4; // GP4
constexpr uint I2C_SCL = 5; // GP5
constexpr uint I2C_HZ = 100000;  // Standard Mode（線長/上拉不足時較穩）
constexpr uint I2C_SCAN_HZ = 100000; // 掃描時用低速較穩
} // namespace Pin

namespace Pca {
constexpr uint8_t ADDR_DEFAULT = 0x40;

constexpr uint8_t MODE1 = 0x00;
constexpr uint8_t MODE2 = 0x01;
constexpr uint8_t LED0_ON_L = 0x06;
constexpr uint8_t PRE_SCALE = 0xFE;
constexpr uint8_t CHAN_STEP = 4; // 每通道 4 bytes: ON_L/ON_H/OFF_L/OFF_H
constexpr uint8_t CHANNEL_COUNT = 16;

// MODE1 bits
constexpr uint8_t MODE1_RESTART = 0x80;
constexpr uint8_t MODE1_AI = 0x20;
constexpr uint8_t MODE1_SLEEP = 0x10;
// MODE2 bits
constexpr uint8_t MODE2_OUTDRV = 0x04; // totem-pole（伺服建議）

constexpr uint32_t PWM_FREQ_HZ = 50;
constexpr uint32_t PERIOD_US = 20000;
constexpr uint16_t COUNTER_MAX = 4095;

// 伺服脈衝範圍
constexpr uint16_t PULSE_MIN_US = 500;
constexpr uint16_t PULSE_MAX_US = 2500;
constexpr uint16_t PULSE_NEUTRAL_US = 1500;

// I2C 掃描範圍
constexpr uint8_t SCAN_FIRST = 0x08;
constexpr uint8_t SCAN_LAST = 0x77;
} // namespace Pca

// ============================================================================
// I2C 底層
// ============================================================================
static i2c_inst_t *sI2c = nullptr;
static uint8_t sAddr = Pca::ADDR_DEFAULT;
static uint sSdaPin = Pin::I2C_SDA;
static uint sSclPin = Pin::I2C_SCL;

/// RP2350：每 4 個腳位一組輪替 I2C0 / I2C1
static i2c_inst_t *selectI2c(uint sdaPin) {
    return ((sdaPin / 2) % 2 == 0) ? i2c0 : i2c1;
}

static void i2cBusInit(uint32_t hz) {
    sI2c = selectI2c(sSdaPin);
    i2c_init(sI2c, hz);
    gpio_set_function(sSdaPin, GPIO_FUNC_I2C);
    gpio_set_function(sSclPin, GPIO_FUNC_I2C);
    gpio_pull_up(sSdaPin);
    gpio_pull_up(sSclPin);
    busy_wait_us(10);
}

// ——— I2C 統計（測試結尾印出，用來判斷匯流排是否穩定）———
static uint32_t sI2cErrors = 0;   // 失敗次數
static uint32_t sI2cRetries = 0;  // 重試次數
static constexpr uint8_t I2C_RETRY_MAX = 3;
// 所有 I2C 交易都設逾時：避免 Slave 卡住 SDA 時 while(true) 無限阻塞
static constexpr uint32_t I2C_TIMEOUT_US = 10000;

/// 匯流排電位檢查（'b' 命令）：把 SDA/SCL 當一般 GPIO 讀，判斷接線狀態
///   SDA=1 SCL=1 → 閒置正常（有上拉、沒短路），但沒有裝置 ACK
///   SDA=0 或 SCL=0 → 被拉死（短路 / 沒接 VCC / 上拉缺失 / OE 接錯）
static void busLevelCheck() {
    i2c_deinit(sI2c);
    const uint pins[2] = {sSdaPin, sSclPin};
    for(uint8_t i = 0; i < 2; i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_up(pins[i]);
    }
    busy_wait_us(2000);
    const int sdaPu = gpio_get(sSdaPin);
    const int sclPu = gpio_get(sSclPin);

    // 再用下拉判斷「上拉夠不夠力」：外部 4.7k 會壓過內部 ~50k 下拉
    gpio_set_pulls(sSdaPin, false, true);
    busy_wait_us(2000);
    const int sdaPd = gpio_get(sSdaPin);
    gpio_set_pulls(sSclPin, false, true);
    busy_wait_us(2000);
    const int sclPd = gpio_get(sSclPin);

    Serial.printf("[BUS] idle with pull-up : SDA=%d SCL=%d\n", sdaPu, sclPu);
    Serial.printf("[BUS] with pull-down    : SDA=%d SCL=%d (1 = 有外部上拉)\n",
                  sdaPd, sclPd);
    if(!sdaPu || !sclPu)
        Serial.print("[BUS] 判定：線被拉在低電位 → 檢查 VCC/GND/短路/上拉\n");
    else if(!sdaPd && !sclPd)
        Serial.print("[BUS] 判定：只有內部弱上拉（建議外接 4.7k）\n");
    else
        Serial.print("[BUS] 判定：上拉正常，線路閒置 high（裝置未 ACK）\n");

    i2cBusInit(Pin::I2C_HZ);
}

/// 匯流排救援：Slave 卡住 SDA 時，手動打 9 個 clock + STOP 後重新初始化
static void i2cBusRecover() {
    i2c_deinit(sI2c);

    gpio_set_function(sSdaPin, GPIO_FUNC_SIO);
    gpio_set_dir(sSdaPin, GPIO_IN);
    gpio_pull_up(sSdaPin);
    gpio_set_function(sSclPin, GPIO_FUNC_SIO);
    gpio_set_dir(sSclPin, GPIO_OUT);
    gpio_put(sSclPin, 1);
    busy_wait_us(5);

    for(uint8_t i = 0; i < 9; i++) {
        gpio_put(sSclPin, 0);
        busy_wait_us(5);
        gpio_put(sSclPin, 1);
        busy_wait_us(5);
    }
    // 手動 STOP：SCL 高 → SDA 低 → SDA 高
    gpio_set_dir(sSdaPin, GPIO_OUT);
    gpio_put(sSdaPin, 0);
    busy_wait_us(5);
    gpio_put(sSclPin, 1);
    busy_wait_us(5);
    gpio_put(sSdaPin, 1);
    busy_wait_us(5);

    i2cBusInit(Pin::I2C_HZ);
    busy_wait_us(200);
}

/// 原始寫入（失敗自動救援 + 重試）
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

/// 原始讀取（repeated start，失敗自動救援 + 重試）
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

/// 寫單一暫存器
static bool regWrite(uint8_t addr, uint8_t reg, uint8_t val) {
    const uint8_t buf[2] = {reg, val};
    return rawWrite(addr, buf, sizeof(buf));
}

/// 從暫存器連續讀取 len bytes（使用 repeated start）
static bool regRead(uint8_t addr, uint8_t reg, uint8_t *out, size_t len) {
    return rawRead(addr, reg, out, len);
}

/// 讀回直到內容符合（用來吸收偶發的讀取錯誤）
static bool regReadUntil(uint8_t addr, uint8_t reg, uint8_t *out, size_t len,
                         uint8_t attempts = I2C_RETRY_MAX) {
    for(uint8_t i = 0; i < attempts; i++) {
        if(regRead(addr, reg, out, len))
            return true;
    }
    return false;
}

// ============================================================================
// I2C 掃描
// ============================================================================
/// @return 找到的裝置數量
static uint8_t i2cScan(bool verbose) {
    uint8_t found = 0;
    if(verbose)
        Serial.printf("[SCAN] bus i2c%u  SDA=GP%u SCL=GP%u @ %uHz\n",
                      sI2c == i2c1 ? 1u : 0u, sSdaPin, sSclPin,
                      Pin::I2C_SCAN_HZ);

    for(uint8_t addr = Pca::SCAN_FIRST; addr <= Pca::SCAN_LAST; addr++) {
        uint8_t dummy = 0;
        // 讀 1 byte；有 ACK 就代表該位址有裝置
        if(i2c_read_timeout_us(sI2c, addr, &dummy, 1, false, I2C_TIMEOUT_US / 2) >= 0) {
            found++;
            if(verbose) {
                Serial.printf("  0x%02X", addr);
                if(addr >= 0x40 && addr <= 0x7F)
                    Serial.print("  <-- 可能是 PCA9685 / 同族 PWM IC");
                else
                    Serial.print("  (其他 I2C 裝置)");
                Serial.print("\n");
            }
        }
        busy_wait_us(200);
    }

    if(verbose) {
        if(found == 0)
            Serial.print("[SCAN] 匯流排上沒有任何裝置 → 檢查接線 / 上拉電阻 / 電源\n");
        else
            Serial.printf("[SCAN] 共找到 %u 個裝置\n", found);
    }
    return found;
}

// ============================================================================
// PCA9685 操作
// ============================================================================
static uint16_t pulseUsToOffCount(uint16_t pulseUs) {
    if(pulseUs > Pca::PERIOD_US)
        pulseUs = Pca::PERIOD_US;
    return static_cast<uint16_t>(
        (static_cast<uint32_t>(pulseUs) * Pca::COUNTER_MAX) / Pca::PERIOD_US);
}

/// 設定單通道脈衝寬度；pulseUs == 0 → 恆低（關閉）
static bool setChannelPulseUs(uint8_t ch, uint16_t pulseUs) {
    if(ch >= Pca::CHANNEL_COUNT)
        return false;

    uint8_t buf[5];
    buf[0] = static_cast<uint8_t>(Pca::LED0_ON_L + ch * Pca::CHAN_STEP);
    buf[1] = 0x00; // ON_L
    buf[2] = 0x00; // ON_H
    if(pulseUs == 0) {
        buf[3] = 0x00;
        buf[4] = 0x10; // OFF full → 永遠 off
    } else {
        const uint16_t off = pulseUsToOffCount(pulseUs);
        buf[3] = static_cast<uint8_t>(off & 0xFF);
        buf[4] = static_cast<uint8_t>((off >> 8) & 0x0F);
    }
    return rawWrite(sAddr, buf, sizeof(buf));
}

static void allChannelsOff() {
    for(uint8_t ch = 0; ch < Pca::CHANNEL_COUNT; ch++)
        setChannelPulseUs(ch, 0);
}

static void printRegisters() {
    uint8_t mode1 = 0, mode2 = 0, preScale = 0, ch0[4] = {0, 0, 0, 0};
    const bool okM1 = regRead(sAddr, Pca::MODE1, &mode1, 1);
    const bool okM2 = regRead(sAddr, Pca::MODE2, &mode2, 1);
    const bool okPs = regRead(sAddr, Pca::PRE_SCALE, &preScale, 1);
    const bool okCh =
        regRead(sAddr, Pca::LED0_ON_L, ch0, sizeof(ch0));

    if(!okM1 || !okM2 || !okPs) {
        Serial.printf("[REG] 讀取失敗（0x%02X 無回應）\n", sAddr);
        return;
    }

    Serial.printf("[REG] addr=0x%02X\n", sAddr);
    Serial.printf("  MODE1      = 0x%02X  (SLEEP=%u AI=%u RESTART=%u)\n", mode1,
                  (mode1 & Pca::MODE1_SLEEP) ? 1u : 0u,
                  (mode1 & Pca::MODE1_AI) ? 1u : 0u,
                  (mode1 & Pca::MODE1_RESTART) ? 1u : 0u);
    Serial.printf("  MODE2      = 0x%02X  (OUTDRV=%u)\n", mode2,
                  (mode2 & Pca::MODE2_OUTDRV) ? 1u : 0u);
    const uint32_t osc = 25000000u;
    const uint32_t freq =
        osc / (static_cast<uint32_t>(Pca::COUNTER_MAX + 1) * (preScale + 1u));
    Serial.printf("  PRE_SCALE  = %u  → 約 %u Hz\n", preScale, freq);
    if(okCh) {
        const uint16_t on = static_cast<uint16_t>(ch0[0] | ((ch0[1] & 0x0F) << 8));
        const uint16_t off = static_cast<uint16_t>(ch0[2] | ((ch0[3] & 0x0F) << 8));
        Serial.printf("  CH0 on=%u off=%u  (~%u us)\n", on, off,
                      static_cast<unsigned>(off * Pca::PERIOD_US /
                                            (Pca::COUNTER_MAX + 1)));
    }
}

/// 初始化 PCA9685：50Hz、auto-increment、totem-pole、全部通道關閉
static bool pcaInit() {
    uint8_t probe = 0;
    if(i2c_read_timeout_us(sI2c, sAddr, &probe, 1, false, I2C_TIMEOUT_US) < 0) {
        Serial.printf("[INIT] 0x%02X 無回應 → 先確認掃描是否找到裝置\n", sAddr);
        return false;
    }

    // 進 sleep（才能改 PRE_SCALE），同時清掉 auto-increment
    if(!regWrite(sAddr, Pca::MODE1, Pca::MODE1_SLEEP)) {
        Serial.print("[INIT] 寫 MODE1 失敗\n");
        return false;
    }

    // PRE_SCALE = round(25MHz / (4096 × 50Hz)) - 1 = 121
    constexpr uint8_t preScale = 121;
    if(!regWrite(sAddr, Pca::PRE_SCALE, preScale)) {
        Serial.print("[INIT] 寫 PRE_SCALE 失敗\n");
        return false;
    }

    // 離開 sleep + auto-increment
    if(!regWrite(sAddr, Pca::MODE1, Pca::MODE1_AI)) {
        Serial.print("[INIT] 離開 sleep 失敗\n");
        return false;
    }
    busy_wait_us(500); // 震盪器穩定

    // totem-pole 輸出（伺服建議）
    regWrite(sAddr, Pca::MODE2, Pca::MODE2_OUTDRV);
    busy_wait_us(500);

    // 重啟 PWM（清除 RESTART）— 之後再設一次 MODE1 確認 AI 仍開
    regWrite(sAddr, Pca::MODE1,
             static_cast<uint8_t>(Pca::MODE1_AI | Pca::MODE1_RESTART));
    busy_wait_us(500);
    regWrite(sAddr, Pca::MODE1, Pca::MODE1_AI);
    busy_wait_us(500);

    allChannelsOff();

    Serial.printf("[INIT] OK：addr=0x%02X, 50Hz, 16 通道全部關閉\n", sAddr);
    printRegisters();
    return true;
}

/// 暫存器讀寫回測（MODE2 寫入反相值 → 讀回比對）
static void readBackTest() {
    uint8_t original = 0;
    if(!regRead(sAddr, Pca::MODE2, &original, 1)) {
        Serial.print("[TEST] 讀 MODE2 失敗\n");
        return;
    }

    const uint8_t patterns[2] = {
        static_cast<uint8_t>(original ^ Pca::MODE2_OUTDRV),
        original,
    };
    uint8_t pass = 0;
    for(uint8_t i = 0; i < 2; i++) {
        const uint8_t expect = patterns[i];
        if(!regWrite(sAddr, Pca::MODE2, expect)) {
            Serial.printf("[TEST] 第 %u 次寫入失敗\n", i + 1u);
            continue;
        }
        busy_wait_us(200);
        uint8_t got = 0;
        if(regRead(sAddr, Pca::MODE2, &got, 1) && got == expect)
            pass++;
        else
            Serial.printf("[TEST] 第 %u 次不符：寫 0x%02X 讀 0x%02X\n", i + 1u,
                          expect, got);
    }
    Serial.printf("[TEST] MODE2 讀寫回測 %s\n",
                  pass == 2 ? "PASS" : "FAIL（I2C 連線不穩／線太長／上拉不足）");
}

/// 連續讀同一通道，統計讀取穩定度（除錯用，'G <ch> <count>' 命令）
static void readStressTest(uint8_t ch, uint32_t count) {
    if(ch >= Pca::CHANNEL_COUNT || count == 0)
        return;
    const uint8_t regAddr = static_cast<uint8_t>(Pca::LED0_ON_L + ch * Pca::CHAN_STEP);
    uint32_t mismatches = 0;
    uint32_t firstOff = 0xFFFF;
    uint16_t lastOff = 0;
    for(uint32_t i = 0; i < count; i++) {
        uint8_t rb[4] = {0, 0, 0, 0};
        const bool ok = regRead(sAddr, regAddr, rb, sizeof(rb));
        const uint16_t off = static_cast<uint16_t>(rb[2] | ((rb[3] & 0x0F) << 8));
        lastOff = off;
        if(!ok)
            mismatches++;
    }
    uint8_t ref[4] = {0, 0, 0, 0};
    regRead(sAddr, regAddr, ref, sizeof(ref));
    const uint16_t refOff = static_cast<uint16_t>(ref[2] | ((ref[3] & 0x0F) << 8));
    for(uint32_t i = 0; i < count; i++) {
        uint8_t rb[4] = {0, 0, 0, 0};
        if(!regRead(sAddr, regAddr, rb, sizeof(rb)) ||
           static_cast<uint16_t>(rb[2] | ((rb[3] & 0x0F) << 8)) != refOff)
            mismatches++;
    }
    firstOff = refOff;
    Serial.printf("[STRESS] CH%u reg=0x%02X count=%lu expected_off=%u mismatch=%lu "
                  "last=%u\n",
                  ch, regAddr, static_cast<unsigned long>(count), firstOff,
                  static_cast<unsigned long>(mismatches), lastOff);
}

/// 讀取單通道 4 個暫存器（除錯用，'g <ch>' 命令）
static void dumpChannel(uint8_t ch) {
    if(ch >= Pca::CHANNEL_COUNT)
        return;
    uint8_t rb[4] = {0, 0, 0, 0};
    const uint8_t regAddr = static_cast<uint8_t>(Pca::LED0_ON_L + ch * Pca::CHAN_STEP);
    const bool ok = regRead(sAddr, regAddr, rb, sizeof(rb));
    const uint16_t on = static_cast<uint16_t>(rb[0] | ((rb[1] & 0x0F) << 8));
    const uint16_t off = static_cast<uint16_t>(rb[2] | ((rb[3] & 0x0F) << 8));
    Serial.printf("[DUMP] CH%02u reg=0x%02X on=%u off=%u rb=%02X %02X %02X %02X ok=%u\n",
                  ch, regAddr, on, off, rb[0], rb[1], rb[2], rb[3], ok ? 1u : 0u);
}

/// 全 16 通道寫入 → 讀回比對（自動化測試用，'v' 命令）
static void verifyAllChannels(uint32_t delayUs = 300) {
    uint8_t passCount = 0;
    const uint32_t errBefore = sI2cErrors;
    const uint32_t retryBefore = sI2cRetries;
    Serial.printf("[VERIFY] write→read 延遲 %lu us\n", static_cast<unsigned long>(delayUs));

    for(uint8_t ch = 0; ch < Pca::CHANNEL_COUNT; ch++) {
        // 每通道給一個不同脈寬，方便確認真的是寫到該通道
        const uint16_t us =
            static_cast<uint16_t>(1000 + ch * 50); // 1000, 1050, ... 1750
        const uint16_t expectOff = pulseUsToOffCount(us);
        const uint8_t regAddr =
            static_cast<uint8_t>(Pca::LED0_ON_L + ch * Pca::CHAN_STEP);

        bool ok = false;
        uint8_t attempts = 0;
        uint8_t rb[4] = {0, 0, 0, 0};
        uint16_t off = 0;
        for(; attempts < 4 && !ok; attempts++) {
            const bool wok = setChannelPulseUs(ch, us);
            busy_wait_us(delayUs); // 給 PCA9685 內部時脈一點時間

            rb[0] = rb[1] = rb[2] = rb[3] = 0;
            const bool rok = regRead(sAddr, regAddr, rb, sizeof(rb));
            const uint16_t on = static_cast<uint16_t>(rb[0] | ((rb[1] & 0x0F) << 8));
            off = static_cast<uint16_t>(rb[2] | ((rb[3] & 0x0F) << 8));
            ok = wok && rok && on == 0 && off == expectOff;
            if(!ok && attempts + 1 < 4) {
                sI2cRetries++;
                busy_wait_us(500);
            }
        }

        Serial.printf(
            "[VERIFY] CH%02u reg=0x%02X %4uus expect=%4u got=%4u rb=%02X %02X %02X %02X "
            "tries=%u %s\n",
            ch, regAddr, us, expectOff, off, rb[0], rb[1], rb[2], rb[3],
            static_cast<unsigned>(attempts), ok ? "PASS" : "FAIL");
        if(ok)
            passCount++;
    }
    allChannelsOff();
    Serial.printf("[VERIFY] RESULT %s %u/%u  (I2C 錯誤 +%lu, 重試 +%lu)\n",
                  passCount == Pca::CHANNEL_COUNT ? "PASS" : "FAIL", passCount,
                  static_cast<unsigned>(Pca::CHANNEL_COUNT),
                  static_cast<unsigned long>(sI2cErrors - errBefore),
                  static_cast<unsigned long>(sI2cRetries - retryBefore));
}

// ============================================================================
// 掃描（sweep）狀態機 — 用來肉眼找伺服
// ============================================================================
namespace Sweep {
enum Mode : uint8_t { NONE = 0, SINGLE, ALL };
static Mode mode = NONE;
static uint8_t singleCh = 0;
static int8_t allCh = 0;
static uint16_t pulseUs = Pca::PULSE_MIN_US;
static int16_t dir = 1;
static uint32_t lastStepMs = 0;
static uint32_t lastHopMs = 0;

constexpr uint32_t STEP_MS = 20;    // 50Hz 週期
constexpr uint16_t STEP_US = 25;    // 每步 25μs → 500~2500 約 1.6 秒
constexpr uint32_t HOP_MS = 2000;   // 全通道模式每通道停留 2 秒

static void startSingle(uint8_t ch) {
    mode = SINGLE;
    singleCh = ch;
    pulseUs = Pca::PULSE_MIN_US;
    dir = 1;
    lastStepMs = millis();
    for(uint8_t i = 0; i < Pca::CHANNEL_COUNT; i++)
        setChannelPulseUs(i, 0);
    Serial.printf("[SWEEP] 單通道 CH%u 來回掃描中（q 停止）\n", ch);
}

static void startAll() {
    mode = ALL;
    allCh = 0;
    pulseUs = Pca::PULSE_MIN_US;
    dir = 1;
    lastStepMs = millis();
    lastHopMs = millis();
    for(uint8_t i = 0; i < Pca::CHANNEL_COUNT; i++)
        setChannelPulseUs(i, 0);
    Serial.print("[SWEEP] 全通道輪流掃描中（每通道 2 秒，q 停止）\n");
}

static void stop() {
    if(mode == NONE)
        return;
    mode = NONE;
    allChannelsOff();
    Serial.print("[SWEEP] 停止，全部通道關閉\n");
}

static void update() {
    if(mode == NONE)
        return;
    const uint32_t now = millis();
    if(now - lastStepMs < STEP_MS)
        return;
    lastStepMs = now;

    const int32_t next = static_cast<int32_t>(pulseUs) + dir * STEP_US;
    if(next >= Pca::PULSE_MAX_US) {
        pulseUs = Pca::PULSE_MAX_US;
        dir = -1;
    } else if(next <= Pca::PULSE_MIN_US) {
        pulseUs = Pca::PULSE_MIN_US;
        dir = 1;
    } else {
        pulseUs = static_cast<uint16_t>(next);
    }

    if(mode == SINGLE) {
        setChannelPulseUs(singleCh, pulseUs);
        return;
    }

    // ALL：一次只驅動一個通道，時間到就換下一顆
    setChannelPulseUs(static_cast<uint8_t>(allCh), pulseUs);
    if(now - lastHopMs >= HOP_MS) {
        lastHopMs = now;
        setChannelPulseUs(static_cast<uint8_t>(allCh), 0);
        allCh = static_cast<int8_t>((allCh + 1) % Pca::CHANNEL_COUNT);
        pulseUs = Pca::PULSE_MIN_US;
        dir = 1;
        Serial.printf("[SWEEP] → CH%u\n", static_cast<unsigned>(allCh));
    }
}
} // namespace Sweep

// ============================================================================
// 命令列
// ============================================================================
static void printHelp() {
    Serial.print(
        "\n=== PCA9685 測試  ※指令大小寫有別 ===\n"
        "  b            匯流排電位檢查（SDA/SCL 是否有上拉/被拉死）\n"
        "  j            交換 SDA/SCL 重新掃描（診斷接線反接）\n"
        "  s            掃描 I2C 匯流排\n"
        "  i            初始化 PCA9685（50Hz）\n"
        "  r            讀回暫存器\n"
        "  t            MODE2 讀寫回測\n"
        "  v [delay]    全 16 通道寫入→讀回驗證（可選 write→read 延遲 us）\n"
        "  g <ch>       讀取單通道暫存器（除錯用）\n"
        "  k <ch> <us>  逐 byte 寫入單通道（除錯用）\n"
        "  c <ch> <us>  指定通道脈衝（ch 0~15, us 500~2500）\n"
        "  a <us>       全部通道同一脈衝\n"
        "  o <ch>       關閉單通道\n"
        "  x            全部關閉\n"
        "  w <ch>       單通道來回掃描\n"
        "  W            全通道輪流掃描\n"
        "  q            停止掃描\n"
        "  n <addr>     換 PCA9685 位址（0x40 / 64）\n"
        "  h            本說明\n"
        "提示：伺服馬達無法被偵測，只能掃描 I2C 找到 PCA9685，\n"
        "      再用 w / W 看哪一顆會動。\n\n");
}

static bool parseLong(const char *s, long *out) {
    if(s == nullptr || *s == '\0')
        return false;
    char *end = nullptr;
    const long v = strtol(s, &end, 0); // 支援 0x 前綴
    if(end == s)
        return false;
    *out = v;
    return true;
}

static void handleCommand(char *line) {
    // 切出最多 3 個 token
    char *argv[3] = {nullptr, nullptr, nullptr};
    uint8_t argc = 0;
    char *save = nullptr;
    for(char *tok = strtok_r(line, " \t\r\n", &save); tok != nullptr && argc < 3;
        tok = strtok_r(nullptr, " \t\r\n", &save)) {
        argv[argc++] = tok;
    }
    if(argc == 0)
        return;

    // 注意：大小寫有別（'w' 單通道掃描 / 'W' 全通道掃描、'g' 讀取 / 'G' 壓力測試）
    const char cmd = argv[0][0];
    long a = 0, b = 0;

    switch(cmd) {
    case 'h':
    case 'H':
    case '?':
        printHelp();
        break;

    case 'b':
    case 'B':
        busLevelCheck();
        break;

    case 'j':
    case 'J': {
        // 交換 SDA/SCL 再掃描：若找到裝置就是接線反了
        const uint tmp = sSdaPin;
        sSdaPin = sSclPin;
        sSclPin = tmp;
        i2c_deinit(sI2c);
        i2cBusInit(Pin::I2C_SCAN_HZ);
        Serial.printf("[SWAP] 以 SDA=GP%u SCL=GP%u 重新掃描\n", sSdaPin, sSclPin);
        const uint8_t n = i2cScan(true);
        // 還原原本接線
        sSdaPin = Pin::I2C_SDA;
        sSclPin = Pin::I2C_SCL;
        i2c_deinit(sI2c);
        i2cBusInit(Pin::I2C_HZ);
        if(n > 0)
            Serial.print("[SWAP] ★ 找到裝置 → SDA/SCL 接反了，請對調接線\n");
        else
            Serial.print("[SWAP] 交換後仍找不到 → 不是接反，檢查 VCC/GND/模組電源\n");
        break;
    }


    case 's':
    case 'S':
        i2cBusInit(Pin::I2C_SCAN_HZ);
        i2cScan(true);
        i2cBusInit(Pin::I2C_HZ);
        break;

    case 'i':
    case 'I':
        if(!pcaInit())
            Serial.print("[INIT] 失敗\n");
        break;

    case 'r':
    case 'R':
        printRegisters();
        break;

    case 't':
    case 'T':
        readBackTest();
        break;

    case 'v':
    case 'V':
        if(argc >= 2 && parseLong(argv[1], &a) && a >= 0 && a <= 100000)
            verifyAllChannels(static_cast<uint32_t>(a));
        else
            verifyAllChannels();
        break;

    case 'g':
        if(argc < 2 || !parseLong(argv[1], &a) || a < 0 ||
           a >= Pca::CHANNEL_COUNT) {
            Serial.print("用法：g <ch 0~15>\n");
            break;
        }
        dumpChannel(static_cast<uint8_t>(a));
        break;

    case 'G':
        if(argc < 3 || !parseLong(argv[1], &a) || !parseLong(argv[2], &b) || a < 0 ||
           a >= Pca::CHANNEL_COUNT || b <= 0 || b > 100000) {
            Serial.print("用法：G <ch 0~15> <count>\n");
            break;
        }
        readStressTest(static_cast<uint8_t>(a), static_cast<uint32_t>(b));
        break;

    case 'k': { // 逐 byte 寫入（不用 auto-increment）除錯用
        if(argc < 3 || !parseLong(argv[1], &a) || !parseLong(argv[2], &b) ||
           a < 0 || a >= Pca::CHANNEL_COUNT || b < 0 || b > Pca::PERIOD_US) {
            Serial.print("用法：k <ch 0~15> <us 0~20000>\n");
            break;
        }
        const uint8_t ch = static_cast<uint8_t>(a);
        const uint16_t off = pulseUsToOffCount(static_cast<uint16_t>(b));
        const uint8_t base = static_cast<uint8_t>(Pca::LED0_ON_L + ch * Pca::CHAN_STEP);
        for(uint8_t i = 0; i < 4; i++) {
            const uint8_t vals[4] = {0x00, 0x00, static_cast<uint8_t>(off & 0xFF),
                                     static_cast<uint8_t>((off >> 8) & 0x0F)};
            regWrite(sAddr, static_cast<uint8_t>(base + i), vals[i]);
            busy_wait_us(200);
        }
        Serial.printf("[K] CH%u 逐 byte 寫入 off=%u\n", ch, off);
        dumpChannel(ch);
        break;
    }

    case 'c':
    case 'C':
        if(argc < 3 || !parseLong(argv[1], &a) || !parseLong(argv[2], &b) ||
           a < 0 || a >= Pca::CHANNEL_COUNT || b < 0 ||
           b > Pca::PERIOD_US) {
            Serial.print("用法：c <ch 0~15> <us 0~20000>\n");
            break;
        }
        Sweep::stop();
        if(setChannelPulseUs(static_cast<uint8_t>(a), static_cast<uint16_t>(b)))
            Serial.printf("[CH] CH%ld = %ld us\n", a, b);
        else
            Serial.print("[CH] 寫入失敗（PCA9685 無回應？先下 i 初始化）\n");
        break;

    case 'a':
        if(argc < 2 || !parseLong(argv[1], &a) || a < 0 ||
           a > Pca::PERIOD_US) {
            Serial.print("用法：a <us 0~20000>\n");
            break;
        }
        Sweep::stop();
        for(uint8_t ch = 0; ch < Pca::CHANNEL_COUNT; ch++)
            setChannelPulseUs(ch, static_cast<uint16_t>(a));
        Serial.printf("[ALL] 16 通道 = %ld us\n", a);
        break;

    case 'o':
    case 'O':
        if(argc < 2 || !parseLong(argv[1], &a) || a < 0 ||
           a >= Pca::CHANNEL_COUNT) {
            Serial.print("用法：o <ch 0~15>\n");
            break;
        }
        Sweep::stop();
        setChannelPulseUs(static_cast<uint8_t>(a), 0);
        Serial.printf("[CH] CH%ld 關閉\n", a);
        break;

    case 'x':
    case 'X':
        Sweep::stop();
        allChannelsOff();
        Serial.print("[ALL] 全部關閉\n");
        break;

    case 'w':
        if(argc < 2 || !parseLong(argv[1], &a) || a < 0 ||
           a >= Pca::CHANNEL_COUNT) {
            Serial.print("用法：w <ch 0~15>\n");
            break;
        }
        Sweep::startSingle(static_cast<uint8_t>(a));
        break;

    case 'W':
        Sweep::startAll();
        break;

    case 'q':
    case 'Q':
        Sweep::stop();
        break;

    case 'n':
    case 'N':
        if(argc < 2 || !parseLong(argv[1], &a) || a < Pca::SCAN_FIRST ||
           a > Pca::SCAN_LAST) {
            Serial.print("用法：n <addr>  例：n 0x40\n");
            break;
        }
        sAddr = static_cast<uint8_t>(a);
        Serial.printf("[ADDR] 目標位址改為 0x%02X（請重新下 i）\n", sAddr);
        break;

    default:
        Serial.printf("未知命令 '%c'（h 看說明）\n", cmd);
        break;
    }
}

// ============================================================================
// Arduino 進入點（單核測試程式，不需要 setup1/loop1）
// ============================================================================
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.begin(115200);
    delay(300); // 等 USB CDC 主機連上

    i2cBusInit(Pin::I2C_HZ);

    Serial.print("\n\n=== PCA9685 連線測試 ===\n");
    Serial.printf("SDA=GP%u  SCL=GP%u  bus=i2c%u  addr=0x%02X\n", sSdaPin,
                  sSclPin, sI2c == i2c1 ? 1u : 0u, sAddr);

    // 開機自動：先掃描，找到 0x40 就自動初始化
    (void)i2cScan(true);
    uint8_t probe = 0;
    if(i2c_read_timeout_us(sI2c, sAddr, &probe, 1, false, I2C_TIMEOUT_US) >= 0) {
        pcaInit();
    } else {
        Serial.printf("[AUTO] 0x%02X 沒回應，略過初始化；"
                      "確認接線後輸入 s 再輸入 i\n",
                      sAddr);
    }
    printHelp();
}

void loop() {
    // 非阻塞讀取一行命令
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

    // 心跳 LED：若燈不閃代表韌體卡住（例如 I2C 阻塞）
    static uint32_t lastBlinkMs = 0;
    static bool ledOn = false;
    const uint32_t nowMs = millis();
    if(nowMs - lastBlinkMs >= 500) {
        lastBlinkMs = nowMs;
        ledOn = !ledOn;
        digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
    }
}
