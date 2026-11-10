#pragma once
#include <cstddef>
#include <cstdint>
#include <hardware/i2c.h>
#include <pico/stdlib.h>

// ============================================================================
// Pca9685 — PCA9685 16 通道 PWM 低階驅動（I2C，含重試與 bus recovery）
//
// 為什麼抽出來：
//   Storage 的 PCA9685 驅動（I2C 初始化、重試、bus recovery、暫存器寫入、
//   50Hz prescale、脈寬 → count 換算）是「晶片層」，與機構無關；而角度 →
//   脈寬的校正（Limits / effectiveInvert / logicalToPhysical / angleToPulseUs /
//   Persist）是「機構參數」。兩者混在同一個 main.cpp 讓機構校正的 EEPROM 版面
//   容易被誤以為可以隨驅動一起改。這裡只搬晶片層，機構校正留在 storage。
//
// 來源：src/storage/main.cpp 的 namespace Pca 常數 + I2C 區塊 + pcaInit()
//       （暫存器初始化順序、121 prescale、I2C_RETRY_MAX=3、I2C_TIMEOUT_US=10000
//        與 recovery 的 9 個 clock 脈衝全部逐字沿用）。
//
// 使用限制：
//   - 所有 I2C 呼叫都會 busy_wait / 逾時等待（最長 10ms × 3 次重試），屬阻塞
//     呼叫。只可在低頻路徑使用（本專案在 Core 1 的機構狀態機裡一動作一次）。
//   - busRecover() 會 i2c_deinit + 重設腳位為 SIO 再拉 9 個 clock，期間該 I2C
//     週邊不可有其他使用者。
//   - 一個實例對應「一組 SDA/SCL + 一個位址」；begin() 之後 sdaPin/sclPin/hz
//     即固定（busRecover() 會用同一組參數重新初始化）。
// ============================================================================

// ============================================================================
// PCA9685 暫存器 / 時序常數（原 src/storage/main.cpp namespace Pca）
// ============================================================================
namespace Pca9685Reg {
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
constexpr uint32_t COUNT_SPAN = 4096; // 12-bit
constexpr uint8_t PRE_SCALE_50HZ = 121; // 25MHz / (4096 × 50Hz) - 1
} // namespace Pca9685Reg

class Pca9685 {
  public:
    Pca9685() = default;

    /// SDA 腳位所屬的 I2C 週邊（GP4 → i2c0，與 mechanism_test 相同）
    static i2c_inst_t *selectI2c(uint sdaPin) {
        return ((sdaPin / 2) % 2 == 0) ? i2c0 : i2c1;
    }

    /// 初始化：I2C 週邊 + 50Hz prescale。
    /// @param deferOutputs true = 只做 prescale/設定，通道輸出維持未啟動
    ///        （晶片留在 SLEEP），由 wakeOutputs() 再啟動。
    ///        伺服機構應優先用這個模式：SLEEP 期間先把每顆要的角度寫入，
    ///        醒來時伺服直接到位，不會先「全通道 full-OFF（輸出恆低）」
    ///        讓伺服失去脈衝 → 上電亂轉 / 不鎖定。
    /// @return false = PCA9685 沒 ACK（沒接線 / 位址錯 / 匯流排卡死）
    bool begin(uint sdaPin, uint sclPin, uint32_t hz, uint8_t addr,
               bool deferOutputs = false) {
        sdaPin_ = sdaPin;
        sclPin_ = sclPin;
        hz_ = hz;
        addr_ = addr;
        errorCount_ = 0;
        i2cBusInit();

        uint8_t probe = 0;
        if(!rawRead(addr_, Pca9685Reg::MODE1, &probe, 1)) {
            return false;
        }

        // PRESCALE 只能在 SLEEP 時寫（寫完振盪器需 500µs 穩定）
        regWrite(Pca9685Reg::MODE1, Pca9685Reg::MODE1_SLEEP);
        regWrite(Pca9685Reg::PRE_SCALE, Pca9685Reg::PRE_SCALE_50HZ);
        regWrite(Pca9685Reg::MODE2, Pca9685Reg::MODE2_OUTDRV);
        if(deferOutputs) {
            // 保持 SLEEP：輸出未啟動，外部伺服看不到任何脈衝，
            // 等 wakeOutputs() 時才開始送（這時通道資料已經寫好了）。
            regWrite(Pca9685Reg::MODE1,
                     static_cast<uint8_t>(Pca9685Reg::MODE1_AI |
                                          Pca9685Reg::MODE1_SLEEP));
            busy_wait_us(500);
            return true;
        }

        regWrite(Pca9685Reg::MODE1, Pca9685Reg::MODE1_AI);
        busy_wait_us(500);
        regWrite(Pca9685Reg::MODE1, static_cast<uint8_t>(Pca9685Reg::MODE1_AI |
                                                        Pca9685Reg::MODE1_RESTART));
        busy_wait_us(500);
        regWrite(Pca9685Reg::MODE1, Pca9685Reg::MODE1_AI);
        busy_wait_us(500);
        allChannelsOff();
        return true;
    }

    /// 從 SLEEP 醒來開始送 PWM（begin(..., true) 之後呼叫）。
    /// 呼叫前所有通道的 ON/OFF 值都應該已寫好，伺服就會直接落在目標位置。
    void wakeOutputs() {
        regWrite(Pca9685Reg::MODE1, Pca9685Reg::MODE1_AI);
        busy_wait_us(500);
        regWrite(Pca9685Reg::MODE1, static_cast<uint8_t>(Pca9685Reg::MODE1_AI |
                                                        Pca9685Reg::MODE1_RESTART));
        busy_wait_us(500);
        regWrite(Pca9685Reg::MODE1, Pca9685Reg::MODE1_AI);
        busy_wait_us(500);
    }

    /// 原始 I2C 寫入（含重試；失敗自動嘗試 busRecover）
    bool rawWrite(uint8_t addr, const uint8_t *buf, size_t len) {
        for(uint8_t attempt = 0; attempt < I2C_RETRY_MAX; attempt++) {
            if(i2c_write_timeout_us(sI2c_, addr, buf, len, false,
                                    I2C_TIMEOUT_US) == static_cast<int>(len))
                return true;
            errorCount_++;
            if(attempt + 1 < I2C_RETRY_MAX)
                busRecover();
        }
        return false;
    }

    /// 原始 I2C 讀取（先寫 1 byte 暫存器位址，再 repeated-start 讀 len bytes）
    bool rawRead(uint8_t addr, uint8_t reg, uint8_t *out, size_t len) {
        for(uint8_t attempt = 0; attempt < I2C_RETRY_MAX; attempt++) {
            if(i2c_write_timeout_us(sI2c_, addr, &reg, 1, true, I2C_TIMEOUT_US) ==
                   1 &&
               i2c_read_timeout_us(sI2c_, addr, out, len, false, I2C_TIMEOUT_US) ==
                   static_cast<int>(len))
                return true;
            errorCount_++;
            if(attempt + 1 < I2C_RETRY_MAX)
                busRecover();
        }
        return false;
    }

    /// 寫單一暫存器（reg, val）
    bool regWrite(uint8_t reg, uint8_t val) {
        const uint8_t buf[2] = {reg, val};
        return rawWrite(addr_, buf, sizeof(buf));
    }

    /// 讀單一暫存器（失敗回 0xFF，呼叫端應自行判斷）
    uint8_t readReg(uint8_t reg) {
        uint8_t v = 0xFF;
        (void)rawRead(addr_, reg, &v, 1);
        return v;
    }

    /// 讀回通道目前的 ON/OFF 設定（診斷用：確認寫入真的落地）
    /// @param offCount 12-bit OFF 目標值（有效脈寬 ≈ count × 20000 / 4096 µs）
    /// @param fullOff  OFF_H bit4：輸出恆低（完全沒有脈衝 → 伺服務不會通電/不鎖定）
    /// @param fullOn   ON_H bit4：輸出恆高（沒有任何脈衝邊緣 → 伺服務會往單邊撞）
    bool readChannel(uint8_t ch, uint16_t &offCount, bool &fullOff, bool &fullOn) {
        if(ch >= Pca9685Reg::CHANNEL_COUNT)
            return false;
        uint8_t b[4] = {0, 0, 0, 0};
        if(!rawRead(addr_,
                    static_cast<uint8_t>(Pca9685Reg::LED0_ON_L +
                                          ch * Pca9685Reg::CHAN_STEP),
                    b, sizeof(b)))
            return false;
        fullOn = (b[1] & 0x10) != 0;
        fullOff = (b[3] & 0x10) != 0;
        offCount = static_cast<uint16_t>(((static_cast<uint16_t>(b[3]) & 0x0F) << 8) |
                                         b[2]);
        return true;
    }

    /// MODE1 的 SLEEP 位是否已清除（= 振盪器真的有在跑、有在送 PWM）
    bool isAwake() { return (readReg(Pca9685Reg::MODE1) & Pca9685Reg::MODE1_SLEEP) == 0; }

    /// 指定通道輸出 count（0 = 恆低；4096 由 count==0 的特例代表 full OFF）
    bool writeChannelCount(uint8_t ch, uint16_t count) {
        if(ch >= Pca9685Reg::CHANNEL_COUNT)
            return false;
        uint8_t buf[5];
        buf[0] = static_cast<uint8_t>(Pca9685Reg::LED0_ON_L +
                                     ch * Pca9685Reg::CHAN_STEP);
        buf[1] = 0x00; // ON_L
        buf[2] = 0x00; // ON_H
        if(count == 0) {
            buf[3] = 0x00;
            buf[4] = 0x10; // OFF full → 輸出恆低
        } else {
            buf[3] = static_cast<uint8_t>(count & 0xFF);
            buf[4] = static_cast<uint8_t>((count >> 8) & 0x0F);
        }
        return rawWrite(addr_, buf, sizeof(buf));
    }

    /// SDA 被從機拉死時的匯流排復原：deinit → 手動 9 個 SCL clock → STOP → 重新 init
    void busRecover() {
        i2c_deinit(sI2c_);
        gpio_set_function(sdaPin_, GPIO_FUNC_SIO);
        gpio_set_dir(sdaPin_, GPIO_IN);
        gpio_pull_up(sdaPin_);
        gpio_set_function(sclPin_, GPIO_FUNC_SIO);
        gpio_set_dir(sclPin_, GPIO_OUT);
        gpio_put(sclPin_, 1);
        busy_wait_us(5);
        for(uint8_t i = 0; i < 9; i++) {
            gpio_put(sclPin_, 0);
            busy_wait_us(5);
            gpio_put(sclPin_, 1);
            busy_wait_us(5);
        }
        gpio_set_dir(sdaPin_, GPIO_OUT);
        gpio_put(sdaPin_, 0);
        busy_wait_us(5);
        gpio_put(sclPin_, 1);
        busy_wait_us(5);
        gpio_put(sdaPin_, 1);
        busy_wait_us(5);
        i2cBusInit();
        busy_wait_us(200);
    }

    /// 累計 I2C 失敗次數（每次重試失敗 +1；供序列監控診斷接線）
    uint32_t errorCount() const { return errorCount_; }

    /// 脈寬 (µs) → PCA9685 count（12-bit，50Hz 週期 20000µs）
    static uint16_t pulseUsToCount(uint16_t pulseUs) {
        return static_cast<uint16_t>(
            (static_cast<uint32_t>(pulseUs) * Pca9685Reg::COUNT_SPAN +
             Pca9685Reg::PERIOD_US / 2) /
            Pca9685Reg::PERIOD_US);
    }

  private:
    static constexpr uint8_t I2C_RETRY_MAX = 3;
    static constexpr uint32_t I2C_TIMEOUT_US = 10000;

    void i2cBusInit() {
        sI2c_ = selectI2c(sdaPin_);
        i2c_init(sI2c_, hz_);
        gpio_set_function(sdaPin_, GPIO_FUNC_I2C);
        gpio_set_function(sclPin_, GPIO_FUNC_I2C);
        gpio_pull_up(sdaPin_);
        gpio_pull_up(sclPin_);
        busy_wait_us(10);
    }

    void allChannelsOff() {
        for(uint8_t ch = 0; ch < Pca9685Reg::CHANNEL_COUNT; ch++)
            writeChannelCount(ch, 0);
    }

    i2c_inst_t *sI2c_ = nullptr;
    uint sdaPin_ = 0;
    uint sclPin_ = 0;
    uint32_t hz_ = 0;
    uint8_t addr_ = Pca9685Reg::ADDR_DEFAULT;
    uint32_t errorCount_ = 0;
};
