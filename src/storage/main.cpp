// ============================================================================
// Storage — Slave 節點 (RP2350)：只有伺服馬達（PCA9685）
//
// 架構（參考 src/test/mechanism 的機構測試，移除 trigger 步進馬達）：
//   Core 0 (loop)：   SlaveComms::update() — 收 Hub 控制幀（硬體 UART0）
//   Core 1 (loop1)：  SlaveComms::runCore1() — 命令驅動 + idleHook 跑機構序列
//
// 動作邏輯：
//   - 角度設定：每顆通道各自一組 {起始角度, 結束角度, 反向}，見 SeqCfg::PROFILE。
//     reverse=false → 動作 start→end（平常停在 startDeg）
//     reverse=true  → 動作 end→start（平常停在 endDeg）
//   - 上電 / E-Stop → 所有「已啟用」通道停在自己的待機角度（預設 30°）
//   - 收到 trigger 上緣（0 → 1，對應 Hub 端 rc[8] < 0）
//     → 每按一次只做「一個動作」，做完等下一次 trigger
//     動作順序：g2×5 → g5×5 → g1×5 → g6×5 → g0×5 → g7×5（共 30 次）
//     每個動作 = 組內每顆「已啟用」通道依序 作動角度 → dwell → 待機角度 → settle
//     （角度各自獨立，預設 待機 30° ⇄ 作動 100°）
//     **該組的最後一輪（第 5 次）**驗證完後，整組一起開到作動角度並維持開啟，
//     不再回 HOME；之前已開啟的組也會一直保持。下一次完整循環（回到第 1 動）
//     開始前才全部回 HOME。
//   - trigger 持續為 1 不會重複觸發；一次上緣 = 一個動作，須放開再按下一動
//   - 追加動作：上面主序列的「一個動作」做完後，CH6 再從待機 75° → 30°
//     （停留 POST_DWELL_MS）→ 回 75°，然後才等下一次 trigger。
//     （CH6 的角度同樣是 SeqCfg::PROFILE[6]，預設 {75, 30, false}）
//     CH6 不在 GROUP_ORDER 內，上電、E-Stop、以及新一輪完整循環開始時
//     都停在自己的待機角度（主序列其餘通道停在各自待機角度）。
//     CH6 不論 EEPROM 的 enabled 為何都會被驅動（POST_FORCE_ENABLE），
//     否則它只會收到 full-OFF（輸出恆低＝沒脈衝）→ 伺服不鎖定一直抖。
//   - 上電順序：PCA9685 先停在 SLEEP（不送 PWM）→ 寫好所有通道目標角度
//     → 再 wakeOutputs()。伺服上電就直接落在待機位置，不會先掉位再回位。
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
//   row          (int8)  — 水平位置（-97~97）→ STEP/DIR/EN 步進
//   motorChannel (uint8) — 保留欄位，本版不使用
//   sw           (uint8) — 保留欄位，本版不使用
//
// 硬體配置：
//   PCA9685: I2C0 (GP4=SDA, GP5=SCL), 位址 0x40, 50Hz
//   伺服馬達：PCA9685 通道 0~15（實際接了哪幾顆由 EEPROM 的 enabled 決定）
//   水平步進（移自 Shoot，腳位同 src/test/stepdir_test）：
//     STEP=GP17（PIO0 SM 產生脈衝）、DIR=GP16、EN=GP18（低電位致能）
//     行程限制：以 0 步為中間，前 / 後各最多 2400 步（RowStepper::STEP_LIMIT_*）
//   無刷電調（ESC）：GP13 硬體 PWM 50Hz，1000µs = 0%、2000µs = 100%
//     上電先送 0% 完成 arming（ARM_DELAY_MS），再爬升到 EscOutput::SPEED_PCT
//   ※ 本節點已移除 trigger 步進馬達（GP6 未使用；GP4/GP5 現在是 I2C SDA/SCL）
// ============================================================================
#include "SlaveComms.h"
#include "step_pulse.pio.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <EepromStore.h>
#include <MathUtils.h>
#include <Pca9685.h>
#include <PioStepRate.h>
#include <SlaveLog.h>
#include <StatusLed.h>
#include <cmath>
#include <cstdio>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h> // time_us_64()（遙測時間戳）

// ============================================================================
// 腳位 / 常數
// ============================================================================
namespace Pin {
constexpr uint I2C_SDA = 4;
constexpr uint I2C_SCL = 5;
constexpr uint I2C_HZ = 100000;

// ——— 水平 STEP/DIR/EN 步進（腳位同 src/test/stepdir_test） ———
constexpr uint ROW_STEP = 17; // → 驅動器 PUL（PIO0 SM 產生）
constexpr uint ROW_DIR = 16;  // → 驅動器 DIR
constexpr uint ROW_EN = 18;   // → 驅動器 ENA（低電位致能）

// ——— 無刷電調（ESC）油門訊號 ———
constexpr uint ESC_PWM = 13; // GP13 → ESC 訊號線（硬體 PWM，50Hz）
} // namespace Pin

// Core 1 → Core 0 除錯日誌（SPSC 佇列）：Core 1 不能直接 Serial，改由 loop() drain 印出
static SlaveLog storageLog;

// ============================================================================
// ESC（單向油門）：硬體 PWM 持續送 50Hz 訊號
//   1000µs = 0%（停轉）、2000µs = 100%
//
// Arming 程序（解決 Pico 與 ESC 開機時間不同步 → ESC 拒轉的問題）：
//   ESC 上電時必須先收到中性訊號（1000µs）一段時間才會完成 arm / 節流校正，
//   之後才接受油門。若上電瞬間直接收到非 0% 的油門，ESC 會判定訊號無效而
//   拒絕旋轉。因此流程固定為：
//     1. init()  → 先送 0%（1000µs），不論目標轉速是多少
//     2. update()（由 Core 0 loop() 每圈呼叫）→ 送滿 ARM_DELAY_MS 的 0%
//     3. 送滿後在 RAMP_MS 內由 0% 線性爬升到 setPercent() 指定的目標
//   這樣就算 ESC 比 Pico 晚通電，也一定先看到足夠久的中性訊號才收到油門。
// ============================================================================
namespace EscOutput {
constexpr uint32_t TICK_HZ = 1000000; // 1 tick = 1µs
constexpr uint16_t PWM_WRAP = 19999;  // 20000 ticks = 20ms → 50Hz
constexpr uint16_t PULSE_ZERO_US = 1000;
constexpr uint16_t PULSE_FULL_US = 2000;
constexpr uint8_t SPEED_PCT = 30; // 目標固定速度（先 0% → arming 後爬升到此）

// Arming 時間：上電後維持 0% 這麼久，ESC 才算完成 arm
// （一般 BLHeli / HobbyWing 電調約需 1~3 秒的中性訊號；抓寬一點保險）
constexpr uint32_t ARM_DELAY_MS = 3000;
// 0% → 目標轉速的線性爬升時間（太陡 ESC 也可能當成異常訊號）
constexpr uint32_t RAMP_MS = 2000;

static uint16_t pulseUsFor(uint8_t percent) {
    if(percent > 100) {
        percent = 100;
    }
    return static_cast<uint16_t>(
        PULSE_ZERO_US +
        (static_cast<uint32_t>(PULSE_FULL_US - PULSE_ZERO_US) * percent) / 100u);
}

static void writePulse(uint16_t us) {
    if(us < PULSE_ZERO_US) {
        us = PULSE_ZERO_US;
    }
    if(us > PULSE_FULL_US) {
        us = PULSE_FULL_US;
    }
    pwm_set_gpio_level(Pin::ESC_PWM, us); // level = 高電位 tick 數 = µs
}

// 目前輸出的實際百分比（爬升過程中的瞬間值）
static uint8_t sCurrentPct = 0;
// 目標百分比（arming 完成後要爬到的值）
static uint8_t sTargetPct = 0;
// init() 的時間戳，用來計算 arming 是否已送滿
static uint32_t sInitMs = 0;

void init(uint8_t percent) {
    gpio_set_function(Pin::ESC_PWM, GPIO_FUNC_PWM);
    const float div = static_cast<float>(clock_get_hz(clk_sys)) /
                      static_cast<float>(TICK_HZ);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, div);
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    const uint slice = pwm_gpio_to_slice_num(Pin::ESC_PWM);
    pwm_init(slice, &cfg, true);
    // 一律先送 0%（中性訊號），不論目標轉速；等 arming 完成再由 update() 爬升
    sTargetPct = percent;
    sCurrentPct = 0;
    sInitMs = millis();
    writePulse(pulseUsFor(0));
}

/// Core 0 loop() 每圈呼叫：arming 等待 + 0% → 目標轉速的線性爬升
void update(uint32_t nowMs) {
    if(sCurrentPct >= sTargetPct) {
        if(sCurrentPct != sTargetPct) {
            sCurrentPct = sTargetPct;
            writePulse(pulseUsFor(sCurrentPct));
        }
        return;
    }
    if(nowMs < sInitMs + ARM_DELAY_MS)
        return; // 還在送 0% 等 ESC arm
    const uint32_t sinceArm = nowMs - (sInitMs + ARM_DELAY_MS);
    if(sinceArm >= RAMP_MS) {
        sCurrentPct = sTargetPct;
    } else {
        sCurrentPct = static_cast<uint8_t>(
            (static_cast<uint32_t>(sTargetPct) * sinceArm) / RAMP_MS);
    }
    writePulse(pulseUsFor(sCurrentPct));
}

uint8_t currentPercent() { return sCurrentPct; }

/// 直接下目標轉速：不跳變，交給 update() 線性爬升（若已在爬升中則更新終點）
void setPercent(uint8_t percent) {
    if(percent > 100)
        percent = 100;
    sTargetPct = percent;
}
} // namespace EscOutput

// PCA9685 暫存器 / 時序常數集中在 lib/Pca9685（Pca9685Reg），
// 本檔只保留機構層：腳位、校正、序列。

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
// ---------------------------------------------------------------------------
// 每顆伺服自己的角度設定（16 通道各自獨立，改這裡就好）
//
//   startDeg  起始角度
//   endDeg    結束角度
//   reverse   false = 動作 startDeg → endDeg（平常停在 startDeg）
//             true  = 動作 endDeg → startDeg（平常停在 endDeg）
//
// 例：{30, 120, false} → 平常 30°，trigger 打到 120° 再回 30°
//     {30, 120, true}  → 平常 120°，trigger 打到 30° 再回 120°
// 也就是說「兩端角度」由 startDeg/endDeg 決定，
// 「哪一邊是待機、往哪個方向轉」由 reverse 決定，兩者互不干擾。
// ---------------------------------------------------------------------------
struct ServoProfile {
    float startDeg;
    float endDeg;
    bool reverse;
};

constexpr ServoProfile PROFILE[Pca9685Reg::CHANNEL_COUNT] = {
    /* CH0  */ {45.0f, 120.0f, false},
    /* CH1  */ {30.0f, 120.0f, false},
    /* CH2  */ {25.0f, 110.0f, false},
    /* CH3  */ {33.0f, 120.0f, false},
    /* CH4  */ {30.0f, 110.0f, false},
    /* CH5  */ {30.0f, 110.0f, false},
    /* CH6  */ {90.0f, 25.0f, false}, // 追加動作那顆：平常 75° → 打到 30°
    /* CH7  */ {30.0f, 110.0f, false},
    /* CH8  */ {30.0f, 110.0f, false},
    /* CH9  */ {30.0f, 110.0f, false},
    /* CH10 */ {25.0f, 110.0f, false},
    /* CH11 */ {20.0f, 110.0f, false},
    /* CH12 */ {30.0f, 110.0f, false},
    /* CH13 */ {30.0f, 120.0f, false},
    /* CH14 */ {15.0f, 110.0f, false},
    /* CH15 */ {30.0f, 110.0f, false},
};

/// 待機（平常停住）角度：reverse=false → startDeg，reverse=true → endDeg
inline float restDeg(uint8_t ch) {
    const ServoProfile &p = PROFILE[ch < Pca9685Reg::CHANNEL_COUNT ? ch : 0];
    return p.reverse ? p.endDeg : p.startDeg;
}

/// 作動（trigger 打出去）角度：reverse=false → endDeg，reverse=true → startDeg
inline float activeDeg(uint8_t ch) {
    const ServoProfile &p = PROFILE[ch < Pca9685Reg::CHANNEL_COUNT ? ch : 0];
    return p.reverse ? p.startDeg : p.endDeg;
}

constexpr uint32_t DWELL_MS = 500; // 作動角度停留
constexpr uint32_t SETTLE_MS = 500; // 回待機角度後等待，再換下一顆

// trigger 一次要做動的「組」順序（組 n = CH(2n), CH(2n+1)）
// 每組連續作動 REPEAT_PER_GROUP 次，每次 = 組內每顆依序 OUT → HOME。
// 按一次 trigger 只執行「一個動作」（一組的一次），全部共 30 次。
constexpr uint8_t GROUP_ORDER[] = {2, 5, 6, 7, 1, 0};
// 個別組的 dwell 覆寫：GROUP_ORDER 中值為 0 或 1 的組停留較久
// （組 0 = CH0/CH1、組 1 = CH2/CH3）
constexpr uint32_t GROUP0_DWELL_MS = 1000;
constexpr uint32_t GROUP1_DWELL_MS = 1000;
constexpr uint8_t GROUP_ORDER_COUNT =
    sizeof(GROUP_ORDER) / sizeof(GROUP_ORDER[0]);
constexpr uint8_t REPEAT_PER_GROUP = 5;
constexpr uint8_t ACTION_COUNT = GROUP_ORDER_COUNT * REPEAT_PER_GROUP;
constexpr uint8_t GROUP_CHANNELS = 2;
constexpr uint8_t STEPS_PER_ACTION = GROUP_CHANNELS; // 一次動作最多兩顆

// 主序列做完後的「追加動作」：單顆伺服抽動一次
//   CH6 不在 GROUP_ORDER 內，不會與主序列搶同一顆馬達。
//   角度也走同一份 PROFILE 表（預設 PROFILE[6] = {75, 30, false}）：
//   平常停在 restDeg(6) = 75°，主序列做完 → 打到 activeDeg(6) = 30°
//   → 停留 POST_DWELL_MS → 回 75°，然後才等下一個 trigger。
constexpr uint8_t POST_CHANNEL = 6;
constexpr uint32_t POST_DWELL_MS = 700; // 停在作動角度的時間
// CH6 一律要驅動：它不在主序列內，常常是 EEPROM 校正裡沒被 enable 的那顆。
// 若照 EEPROM 的 enabled 跳過，它就只會被寫成 full-OFF（輸出恆低 = 沒有脈衝），
// 實際上有接伺服的∏就是「不動作 + 一直嗡嗡抖」的來源。
// 所以預設強制驅動（仍套用該通道的 min/max µs、角度範圍、反向校正）。
constexpr bool POST_FORCE_ENABLE = true;

// EEPROM 裡 enabled=0 的通道要不要也「停在自己的待機角度」？
// 0 的輸出是 full-OFF（`OFF_H` bit4）＝ 輸出恆低、完全沒有脈衝，伺服收不到
// 訊號就不會鎖定（看起來像「沒通電」、會抖）。只要該通道實際有接伺服，就
// 應該給它一個合法脈寬；沒接伺服的通道多送一條脈衝也無害。
constexpr bool DRIVE_DISABLED_CHANNELS = true;
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
static ChannelLimit channels[Pca9685Reg::CHANNEL_COUNT];

constexpr int EEPROM_SIZE = 512;
constexpr int EEPROM_ADDR = 0;
constexpr uint32_t MAGIC = 0x50434134; // "PCA4"

// EEPROM 版面（欄位順序、MAGIC="PCA4"、512 bytes）必須與 pca9685_angle /
// mechanism_test 位元等價，車上現有的校正資料才不會被清掉。驗證（magic +
// 逐 byte 加總 checksum）交給 lib/EepromStore，版面本身留在本檔。
struct Persist {
    uint32_t magic;
    uint16_t minUs[Pca9685Reg::CHANNEL_COUNT];
    uint16_t maxUs[Pca9685Reg::CHANNEL_COUNT];
    uint8_t minDeg[Pca9685Reg::CHANNEL_COUNT];
    uint8_t maxDeg[Pca9685Reg::CHANNEL_COUNT];
    uint8_t enabled[Pca9685Reg::CHANNEL_COUNT];
    uint8_t invert[Pca9685Reg::CHANNEL_COUNT];
    uint32_t checksum;
};

// 實際是否反向：奇數通道硬體鏡像，或 EEPROM 校正指定反向
static bool effectiveInvert(uint8_t ch) {
    if(ch >= Pca9685Reg::CHANNEL_COUNT)
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
    if(!EepromStore::load(MAGIC, EEPROM_ADDR, p)) {
        for(uint8_t i = 0; i < Pca9685Reg::CHANNEL_COUNT; i++)
            channels[i] = defaultChannel();
        return false;
    }
    for(uint8_t i = 0; i < Pca9685Reg::CHANNEL_COUNT; i++) {
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
// PCA9685 晶片層（I2C 初始化、重試、bus recovery、50Hz prescale、脈寬 → count、
// 通道輸出）全部在 lib/Pca9685；本檔只保留機構層的角度 ↔ 脈寬換算。
// 注意：所有 Pca9685 呼叫都是阻塞式的（I2C 逾時 10ms × 3 次重試、bus
//   recovery 另加 ~200µs），因此只在低頻路徑（Core 1 機構狀態機一動作一次）使用。
// ============================================================================
static Pca9685 sPca;

// ============================================================================
// 角度 → 脈衝（含每通道零點 / 行程 / 反向 / 實體工作範圍）
// ============================================================================
static float clampAngle(float deg) {
    return MathUtils::clampf(deg, Servo::ANGLE_MIN, Servo::ANGLE_MAX);
}

static float logicalToPhysical(uint8_t ch, float logicalDeg) {
    if(ch >= Pca9685Reg::CHANNEL_COUNT)
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

/// 該角度是否會被 EEPROM 的 minDeg/maxDeg 夾住（表示命令到不了，伺服可能頂死點）
static bool angleClampedByRange(uint8_t ch, float deg) {
    if(ch >= Pca9685Reg::CHANNEL_COUNT)
        return false;
    const ChannelLimit &l = Limits::channels[ch];
    float phys = clampAngle(deg);
    if(Limits::effectiveInvert(ch))
        phys = Servo::ANGLE_MAX - phys;
    return phys < static_cast<float>(l.minDeg) ||
           phys > static_cast<float>(l.maxDeg);
}

// ============================================================================
// 每通道「目前送出的目標」記錄（自我修復用）
//
// 伺服一旦收到合法脈寬就會一直保持，理論上不需重寫；但通電瞬間的 I2C/供電
// 不稳定、或伺服大電流造成電壓下陷時，PCA9685 暫存器可能沒真的寫進去，
// 伺服就卡在錯的位置一直抖，直到下一次寫入才恢復（對應「通電後大機率不正常、
// 過幾分鐘才自己好」）。所以這裡記下每通道最後一次「理當長什麼樣」，
// 由 holdRefresh() 定期讀回比對，不一致就重寫。
// ============================================================================
static uint16_t chTargetCount[Pca9685Reg::CHANNEL_COUNT] = {0};
static bool chTargetOff[Pca9685Reg::CHANNEL_COUNT] = {false};
static bool chTargetValid[Pca9685Reg::CHANNEL_COUNT] = {false};
// 通電時逐通道錯開寫入（µs）：避免 16 顆伺服同時啟動的湧浪電流
static uint32_t sInterChannelGapUs = 0;

static bool setAngle(uint8_t ch, float deg) {
    const uint16_t count = Pca9685::pulseUsToCount(angleToPulseUs(ch, deg));
    const bool ok = sPca.writeChannelCount(ch, count);
    if(ch < Pca9685Reg::CHANNEL_COUNT) {
        chTargetCount[ch] = count;
        chTargetOff[ch] = false;
        chTargetValid[ch] = true;
    }
    if(sInterChannelGapUs != 0)
        busy_wait_us(sInterChannelGapUs);
    return ok;
}

static void setChannelOff(uint8_t ch) {
    sPca.writeChannelCount(ch, 0);
    if(ch < Pca9685Reg::CHANNEL_COUNT) {
        chTargetCount[ch] = 0;
        chTargetOff[ch] = true;
        chTargetValid[ch] = true;
    }
    if(sInterChannelGapUs != 0)
        busy_wait_us(sInterChannelGapUs);
}

static bool channelEnabled(uint8_t ch) {
    if(ch >= Pca9685Reg::CHANNEL_COUNT)
        return false;
    if(SeqCfg::POST_FORCE_ENABLE && ch == SeqCfg::POST_CHANNEL)
        return true; // 追加動作那顆不論 EEPROM 是否 enable 都要驅動
    return Limits::channels[ch].enabled != 0;
}

/// EEPROM 對該通道的原始設定是否啟用（不套 POST_FORCE_ENABLE，僅供診斷）
static bool channelEnabledInEeprom(uint8_t ch) {
    return ch < Pca9685Reg::CHANNEL_COUNT && Limits::channels[ch].enabled != 0;
}

// ============================================================================
// 機構序列（非阻塞狀態機，全部在 Core 1 執行）
//
//   每收到一次 trigger 上緣只做「一個動作」，做完就停，等下一次 trigger。
//   動作順序：g2×5 → g5×5 → g1×5 → g6×5 → g0×5 → g7×5（共 30 次）
//   每個動作 = 組內每顆「已啟用」通道依序 OUT → dwell → HOME → settle。
//   該組的最後一輪（第 REPEAT_PER_GROUP 次）多做一段：整組 → OUT 並維持開啟。
// ============================================================================
namespace ServoSeq {
// 待機 / 作動角度是「每顆通道各自」的（見 SeqCfg::PROFILE），
// 這裡只留時間參數；角度一律透過 SeqCfg::restDeg() / activeDeg() 取得。
static uint32_t dwellMs = SeqCfg::DWELL_MS;
static uint32_t settleMs = SeqCfg::SETTLE_MS;

static constexpr uint8_t kStepsMax = SeqCfg::STEPS_PER_ACTION;

static bool running = false;
static uint8_t steps[kStepsMax] = {0};
static uint8_t stepCount = 0;
static uint8_t stepIdx = 0;

// 追加動作（CH6 抽動）狀態
//   postStep 0 = 已打到作動角度，等停留時間到
//   postStep 1 = 已回待機角度，下一圈 update() 收尾
static bool postPending = false;
static uint8_t postStep = 0;

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

/// 追加動作用的那顆伺服平常停在自己的待機角度（CH6 預設 75°），
/// 不跟主序列一起動作。
/// 上電、E-Stop、新一輪循環開始都靠這支把 CH6 擺回待機位置。
static void restPostChannel() {
    if(channelEnabled(SeqCfg::POST_CHANNEL))
        setAngle(SeqCfg::POST_CHANNEL, SeqCfg::restDeg(SeqCfg::POST_CHANNEL));
}

/// 全部回待機：主序列通道 → 自己的待機角度；追加動作那顆 → 自己的待機角度
/// CH6 絕不寫 full-OFF（那會讓伺服收不到脈衝而抖動/不鎖定）；
/// EEPROM 停用的通道也一樣停在 HOME（除非關掉 DRIVE_DISABLED_CHANNELS）。
static void goHomeAll() {
    for(uint8_t ch = 0; ch < Pca9685Reg::CHANNEL_COUNT; ch++) {
        if(ch == SeqCfg::POST_CHANNEL)
            continue; // 交給 restPostChannel()，避免同一顆連擺兩次
        if(channelEnabled(ch) || SeqCfg::DRIVE_DISABLED_CHANNELS)
            setAngle(ch, SeqCfg::restDeg(ch));
        else
            setChannelOff(ch);
    }
    restPostChannel();
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
        if(ch < Pca9685Reg::CHANNEL_COUNT && channelEnabled(ch) && stepCount < kStepsMax)
            steps[stepCount++] = ch;
    }
    return stepCount > 0;
}

/// 停止：所有通道回 HOME（E-Stop / 重新開始動作時）
/// 可重複呼叫：已經全在 HOME 就不再寫 I2C（避免 E-Stop 每圈洪水）。
/// @param force true = 不管 atHome 旗標，一律重寫一次待機位置
static void stop(bool force = false) {
    running = false;
    postPending = false;
    postStep = 0;
    phase = PHASE_NONE;
    if(force || !atHome)
        goHomeAll(); // 全部回各自待機角度
}

/// 把本動作會用到的通道放回各自待機角度（不動已維持開啟的其他組）
static void homeActionChannels() {
    for(uint8_t i = 0; i < stepCount; i++)
        setAngle(steps[i], SeqCfg::restDeg(steps[i]));
}

/// 執行「下一個動作」（一次 trigger 只呼叫一次）
static void startNext() {
    // 新的完整循環（第 1 動）開始前才全部回 HOME；其餘只歸零本動作的通道，
    // 讓前面組「維持開啟」的狀態保持住。
    const bool newCycle = (nextAction == 0);
    running = false;
    postPending = false;
    postStep = 0;
    phase = PHASE_NONE;
    if(newCycle)
        goHomeAll(); // 全部回各自待機角度
    uint8_t action = nextAction;
    for(uint8_t tried = 0; tried < SeqCfg::ACTION_COUNT; tried++) {
        const uint8_t group = groupOfAction(action);
        if(buildAction(group)) {
            if(!newCycle)
                homeActionChannels();
            currentAction = action;
            currentGroup = group;
            currentRep = repOfAction(action);
            // 組 0 / 組 1 停留時間覆寫，其餘組用預設 DWELL_MS
            dwellMs = (group == 0)   ? SeqCfg::GROUP0_DWELL_MS
                      : (group == 1) ? SeqCfg::GROUP1_DWELL_MS
                                     : SeqCfg::DWELL_MS;
            running = true;
            phase = PHASE_OUT;
            phaseMs = millis();
            cycleStartMs = phaseMs;
            return;
        }
        action = static_cast<uint8_t>((action + 1) % SeqCfg::ACTION_COUNT);
    }
    nextAction = 0;
}

/// 主序列做完後的追加動作：CH6 待機角度 → 作動角度，停留後再回待機角度
static void startPost(uint32_t now) {
    if(!channelEnabled(SeqCfg::POST_CHANNEL))
        return; // 只有 POST_CHANNEL 超出範圍才會到這裡（預設強制驅動）
    if(!setAngle(SeqCfg::POST_CHANNEL, SeqCfg::activeDeg(SeqCfg::POST_CHANNEL)))
        writeErrors++;
    atHome = false;
    postPending = true;
    postStep = 0;
    // 追加動作期間維持 running，避免連按 trigger 插隊到動作中間
    running = true;
    phase = PHASE_NONE;
    phaseMs = now;
}

/// 追加動作狀態機（idleHook 呼叫）
static void updatePost(uint32_t now) {
    if(postStep == 0) {
        if(now - phaseMs < SeqCfg::POST_DWELL_MS)
            return;
        if(!setAngle(SeqCfg::POST_CHANNEL, SeqCfg::restDeg(SeqCfg::POST_CHANNEL)))
            writeErrors++;
        postStep = 1;
        phaseMs = now;
        return;
    }
    // postStep == 1：已回待機角度，追加動作結束，等下一個 trigger
    postPending = false;
    running = false;
    phase = PHASE_NONE;
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
    } else {
    }
}

/// idleHook 呼叫：推進狀態機
static void update() {
    const uint32_t now = millis();
    if(postPending) {
        updatePost(now);
        return;
    }
    if(!running)
        return;
    switch(phase) {
    case PHASE_OUT: {
        const uint8_t ch = steps[stepIdx];
        const float act = SeqCfg::activeDeg(ch); // 該通道自己的作動角度
        if(!setAngle(ch, act))
            writeErrors++;
        atHome = false;
        const int32_t o = static_cast<int32_t>(lroundf(act * 10.0f));
        phase = PHASE_DWELL;
        phaseMs = now;
        break;
    }
    case PHASE_DWELL:
        if(now - phaseMs >= dwellMs) {
            const uint8_t ch = steps[stepIdx];
            const float rest = SeqCfg::restDeg(ch); // 該通道自己的待機角度
            if(!setAngle(ch, rest))
                writeErrors++;
            const int32_t h = static_cast<int32_t>(lroundf(rest * 10.0f));
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
                startPost(now); // 主序列做完 → 追加 CH6 抽動
            }
        }
        break;
    case PHASE_OPEN: {
        for(uint8_t i = 0; i < stepCount; i++) {
            const float act = SeqCfg::activeDeg(steps[i]);
            if(!setAngle(steps[i], act))
                writeErrors++;
            atHome = false;
            const int32_t o = static_cast<int32_t>(lroundf(act * 10.0f));
        }
        finishAction(true, now);
        startPost(now); // 主序列做完 → 追加 CH6 抽動
        break;
    }
    default:
        break;
    }
}

static bool isRunning() { return running || postPending; }
/// 追加動作目前是否正停在作動角度
static bool postHoldingTrig() { return postPending && postStep == 0; }
static uint8_t currentChannel() {
    if(postPending)
        return SeqCfg::POST_CHANNEL;
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
static volatile bool sPcaAwake = false;

// PCA9685 讀回快照：由 Core 1 在 init 時填好，Core 0 只負責印。
// ⚠️ Pca9685 的 I2C 不是 thread-safe，所有 I2C 存取（含讀回）只能在 Core 1，
//    因此 Core 0 絕對不可以直接呼叫 sPca.*。
struct PcaSnapshot {
    uint8_t mode1;
    uint8_t prescale;
    uint16_t offCount[Pca9685Reg::CHANNEL_COUNT];
    uint8_t fullOffMask; // bit n = CHn 輸出恆低（沒有脈衝）
    uint8_t fullOnMask;  // bit n = CHn 輸出恆高
    uint8_t readOkMask;  // bit n = CHn 讀回成功
};
static PcaSnapshot sSnap = {};

namespace MotorControl {
// trigger 上緣偵測（Core 1）
static bool prevTrigger = false;
// 是否已看過第一個有效命令。上電時若 trigger 已經是 1（開關壓著），
// 不可當成「上緣」直接啟動，必須先放開（0）再按（0→1）才算一次。
static bool triggerLatched = false;

static void init() {
    EEPROM.begin(Limits::EEPROM_SIZE);
    const bool loaded = Limits::load();

    // ⚠ 上電順序：
    //   1. PCA9685 只做 50Hz prescale，晶片留在 SLEEP（輸出還沒啟動）
    //   2. 在 SLEEP 期間把每顆伺服該在的位置寫進通道暫存器
    //      （主序列與 CH6 都寫自己的待機角度）
    //   3. wakeOutputs() 才開始送 PWM → 伺服上電直接落在待機位置
    // 舊順序（begin() 先把 16 通道全關 = 輸出恆低）會讓伺服先失去脈衝，
    // 掉位完才被寫入位置，看起來就像「開機亂轉到錯的位置」。
    const bool ok = sPca.begin(Pin::I2C_SDA, Pin::I2C_SCL, Pin::I2C_HZ,
                               Pca9685Reg::ADDR_DEFAULT, true);
    // 通電時逐通道錯開 25ms：避免 16 顆伺服同時啟動的湧浪電流把電壓拉低
    // （電壓一低，伺服就不鎖定、抖動、停到錯的位置）。
    // 必須在寫位置前就先打開，寫完再關掉（執行期不想要這些延遲）。
    sInterChannelGapUs = 25000;
    ServoSeq::stop(true); // 全部寫各自待機角度（SLEEP 期間寫入）
    // 就算 probe 沒 ACK 也試著唤一次：寧可多寫幾個失敗的 I2C，
    // 也不要讓晶片卡在 SLEEP（那會全車伺服都沒脈衝 = 「沒通電」）。
    sPca.wakeOutputs();
    sPcaAwake = sPca.isAwake();
    if(!sPcaAwake) {
        // 唤不醒（MODE1.SLEEP 没清）→ 退回傳統初始化：直接醒著設 50Hz，
        // 再把待機位置重下一次，避免整台伺服完全沒有 PWM。
        (void)sPca.begin(Pin::I2C_SDA, Pin::I2C_SCL, Pin::I2C_HZ,
                         Pca9685Reg::ADDR_DEFAULT, false);
        ServoSeq::stop(true); // 傳統初始化會把通道全關，這裡強制重寫位置
        sPcaAwake = sPca.isAwake();
    }
    sInterChannelGapUs = 0; // 執行期不再錯開（序列保持硬時序）

    // 讀回自檢（只能在 Core 1 做）：把晶片內實際 OFF 值抄成快照，
    // 給 Core 0 開機時印出，用來分辨「校正值錯」還是「I2C 根本沒寫進去」。
    sSnap.mode1 = sPca.readReg(Pca9685Reg::MODE1);
    sSnap.prescale = sPca.readReg(Pca9685Reg::PRE_SCALE);
    sSnap.fullOffMask = 0;
    sSnap.fullOnMask = 0;
    sSnap.readOkMask = 0;
    for(uint8_t ch = 0; ch < Pca9685Reg::CHANNEL_COUNT; ch++) {
        uint16_t off = 0;
        bool fullOff = false;
        bool fullOn = false;
        if(!sPca.readChannel(ch, off, fullOff, fullOn))
            continue;
        sSnap.readOkMask = static_cast<uint8_t>(sSnap.readOkMask | (1u << ch));
        sSnap.offCount[ch] = off;
        if(fullOff)
            sSnap.fullOffMask = static_cast<uint8_t>(sSnap.fullOffMask | (1u << ch));
        if(fullOn)
            sSnap.fullOnMask = static_cast<uint8_t>(sSnap.fullOnMask | (1u << ch));
    }
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

static uint32_t sHoldRepaired = 0; // 累計讀回不一致並重寫的通道數（診斷用）

// 守護刷新（Core 1，每 HOLD_REFRESH_MS 一次）：
//   1. 讀回每通道晶片實際 OFF 值，跟「理當送出」的值比對
//   2. 不一致 / full-OFF / 讀回失敗 → 直接重寫一次（自我修復）
// 用途：通電瞬間 I2C 寫不進去、或伺服大電流把電壓拉低害晶片設值亂掉時，
//       一秒內就會自動拉到正確位置，不用等下一次 trigger。
// 若一直重複印「修復」，代表 I2C 或伺服供電不穩（硬件問題）。
static void holdRefresh() {
    static constexpr uint32_t HOLD_REFRESH_MS = 1000;
    static uint32_t lastMs = 0;
    const uint32_t now = millis();
    if(now - lastMs < HOLD_REFRESH_MS)
        return;
    lastMs = now;

    uint8_t repaired = 0;
    for(uint8_t ch = 0; ch < Pca9685Reg::CHANNEL_COUNT; ch++) {
        if(!chTargetValid[ch])
            continue;
        uint16_t off = 0;
        bool fullOff = false;
        bool fullOn = false;
        const uint16_t want = chTargetOff[ch] ? 0 : chTargetCount[ch];
        const bool ok = sPca.readChannel(ch, off, fullOff, fullOn);
        const bool match =
            ok && !fullOn && (want == 0 ? fullOff : (!fullOff && off == want));
        if(match)
            continue;
        // 重寫（讀回失敗也重寫：可能只是讀被干擾，寫一次不損失）
        (void)sPca.writeChannelCount(ch, want);
        if(ok)
            repaired++;
    }
    sHoldRepaired += repaired;
}

/// 讀回不一致而重寫的累計次數（給 Core 0 的監控行用：0 = 晶片設定一直正常）
static uint32_t repairCount() { return sHoldRepaired; }

// CH6 追加動作的執行期自檢（Core 1，每 2s 一次）：
// 讀回晶片實際 OFF 值，只在「跟預期不符 / 沒有脈衝」時才印 log，不洗版。
// 用途：分辨 CH6 到底是「軔體沒寫進去」還是「伺服的機械極限/校正不對」。
static void pollPostChannel() {
    static uint32_t lastMs = 0;
    const uint32_t now = millis();
    if(now - lastMs < 2000)
        return;
    lastMs = now;
    uint16_t off = 0;
    bool fullOff = false;
    bool fullOn = false;
    if(!sPca.readChannel(SeqCfg::POST_CHANNEL, off, fullOff, fullOn)) {
        return;
    }
    // 該通道自己的待機 / 作動角度（來自 SeqCfg::PROFILE）
    const float wantDeg = ServoSeq::postHoldingTrig()
                              ? SeqCfg::activeDeg(SeqCfg::POST_CHANNEL)
                              : SeqCfg::restDeg(SeqCfg::POST_CHANNEL);
    const uint16_t wantUs = angleToPulseUs(SeqCfg::POST_CHANNEL, wantDeg);
    const uint16_t wantCount = Pca9685::pulseUsToCount(wantUs);
    const uint16_t actualUs = static_cast<uint16_t>(
        static_cast<uint32_t>(off) * Pca9685Reg::PERIOD_US / Pca9685Reg::COUNT_SPAN);
    const bool bad = fullOff || fullOn || off + 4 < wantCount ||
                     off > wantCount + 4;
    if(!bad)
        return;
}
} // namespace MotorControl

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
namespace AxisConfig {
// 輸入範圍：-97 ~ +97（超過限幅，保留機構餘裕）
constexpr float INPUT_LIMIT = 97.0f;
} // namespace AxisConfig

namespace RowStepper {
// 行程限制（步）：0 = 中間位置；正向 = front，負向 = back
// rc[3] 的 -97~+97 直接線性映射到 -2400~+2400（±97 對應±2400）
constexpr int32_t STEP_LIMIT_FRONT = 2400;
constexpr int32_t STEP_LIMIT_BACK = 2400;
constexpr float STEP_PER_INPUT =
    static_cast<float>(STEP_LIMIT_FRONT) / AxisConfig::INPUT_LIMIT;
// 方向：rc 正值 → 正步數（DIR=1）。若實機方向相反改成 -1.0f。
constexpr float INPUT_SIGN = -1.0f;
// 速度 / 加速度：
//   短行程（±2400 步）的實際峰值由加速度限制，不是 SPEED_SPS：
//     peak = min(SPEED_SPS, sqrt(2 × ACCEL_SPS2 × 步數))
//   例：2400 步、ACCEL=120000 → peak≈24000 sps、來回約 0.28 秒。
//   若要在 2400 步內真的跑到 30000 sps，ACCEL 需 ≥ 187500。
constexpr float SPEED_SPS = 30000.0f;      // 到位過程最高脈衝速率
constexpr float ACCEL_SPS2 = 120000.0f;    // 加速度 steps/s²（越大越衝）
constexpr float MIN_SPS = 330.0f;          // 起步/收尾速率（PIO clkdiv 上限對應 ~327sps）
constexpr uint32_t DIR_SETTLE_US = 20;     // DIR 建立時間
constexpr uint32_t RAMP_INTERVAL_US = 100; // 加減速更新週期（100µs）

// ---- 干擾防護 ----
// 開機靜默期：前 BOOT_HOLD_MS 忽略 row 動作（PCA9685 初始化 + ESC arming 這段
// 供電最不穩，指令/雜訊造成的起動看起來就像「開機自轉」）。目標值照樣記錄，
// 靜默期結束後才補走到位。
constexpr uint32_t BOOT_HOLD_MS = 4000;
// row 輸入死區（-97~97 刻度）：微抖不動，避免搖桿/雜訊造成微量轉動
constexpr float INPUT_DEADBAND = 3.0f;
// 小於此步數的位移差直接忽略（約滿行程 1.7%）：E-Stop 回補估算誤差、
// 單幀雜訊 row 值跳動，都不會再讓馬達「跳一點位置」
constexpr int32_t MIN_MOVE_STEPS = 40;
// E-Stop 觸發次數（診斷用：rc[4] 開關時若一直增加，代表 ESC 湧浪
// 把 UART 幀打掉 >100ms → E-Stop，是供電/EMI 問題不是遙輯問題）
static volatile uint32_t sEstopCount = 0;

// ---- row 目標雜訊濾波 ----
// 實測：無刷運轉時 rc[3] 命令會高頻振盪 ±10~30（每幀都變、正負交替），
// 每個雜訊值直接成新目標就會造成 ±數百步的偏移自轉。
// EMA 低通：高頻振盪被平均掉（振幅 < INPUT_DEADBAND → 歸零），
// 真實搖桿操作是持續同方向的，只延遲約 60ms（@100Hz）。
constexpr float ROW_EMA_ALPHA = 0.15f;
static float sFiltInput = 0.0f;   // EMA 後的輸入（-97~97）
static bool sFiltInit = false;    // 首次命令直接貼齊，不等 EMA 收斂

static int32_t refSteps = 0;        // 目前位置估算（steps，0 = 上電位置）
static bool sHoldAfterEstop = false; // E-Stop 後不自動補走，等新 row 命令才動
static bool sInEstop = false;        // 目前是否處在 E-Stop 中（計數去重用）
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
// 每步的 PIO 週期數（set[3] 4 + set[1] 2 + jmp 1）：這是 step_pulse.pio 的程式知識，
// 留在本檔；速率 → clkdiv 的換算走 PioStepRate::clkdivForSps()。
constexpr float kCyclesPerStep = 7.0f; // set[3] 4 + set[1] 2 + jmp 1
static float sClkSysHz = 150000000.0f;

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
    if(input > AxisConfig::INPUT_LIMIT) {
        input = AxisConfig::INPUT_LIMIT;
    }
    if(input < -AxisConfig::INPUT_LIMIT) {
        input = -AxisConfig::INPUT_LIMIT;
    }
    // 輸入死區：|input| < INPUT_DEADBAND → 0（搖桿微抖/單幀雜訊不動）
    if(fabsf(input) < INPUT_DEADBAND)
        input = 0.0f;
    // -97~+97 → -2400~+2400 steps
    return static_cast<int32_t>(roundf(input * STEP_PER_INPUT * INPUT_SIGN));
}

void init() {
    sClkSysHz = static_cast<float>(clock_get_hz(clk_sys));

    // 安全狀態先行：STEP/DIR/EN 三支腳在上電後立刻輸出確定電位。
    // （先前 STEP 在 PIO 設定完成前是浮接輸入，驅動器光耦合輸入吃到雜訊
    //   就會產生脈衝 → 「開機自轉」。）
    gpio_init(Pin::ROW_STEP);
    gpio_set_dir(Pin::ROW_STEP, GPIO_OUT);
    gpio_put(Pin::ROW_STEP, 0); // STEP 閒置 = 低

    gpio_init(Pin::ROW_DIR);
    gpio_set_dir(Pin::ROW_DIR, GPIO_OUT);
    gpio_put(Pin::ROW_DIR, 0);

    gpio_init(Pin::ROW_EN);
    gpio_set_dir(Pin::ROW_EN, GPIO_OUT);
    gpio_put(Pin::ROW_EN, 0); // 致能（低電位）

    int smIdx = pio_claim_unused_sm(sPio, false);
    if(smIdx < 0) {
        pioReady = false;
        // PIO 取得失敗也保持 STEP 由 CPU 驅動為低，絕不浮接
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
    storageLog.printf("[STEP] 移動 %ld 步 (pos %ld → %ld) dir=%s",
                      static_cast<long>(steps > 0 ? steps : -steps),
                      static_cast<long>(refSteps),
                      static_cast<long>(planTargetSteps),
                      curDir > 0 ? "+" : "-");

    if(pioReady) {
        pio_sm_set_enabled(sPio, sSm, false);
        pio_sm_clear_fifos(sPio, sSm);
        pio_sm_restart(sPio, sSm);
        pio_sm_exec(sPio, sSm, pio_encode_jmp(sOffset));
        pio_interrupt_clear(sPio, kIrqFlag);
        pio_sm_set_clkdiv(
            sPio, sSm,
            PioStepRate::clkdivForSps(sClkSysHz, kCyclesPerStep, curSps));
        pio_sm_put(sPio, sSm, static_cast<uint32_t>(steps > 0 ? steps : -steps));
        pio_sm_set_enabled(sPio, sSm, true);
    }
}

/// 設定目標（input -97~97）。Core 1 context。
void setTarget(float input) {
    if(input > AxisConfig::INPUT_LIMIT) {
        input = AxisConfig::INPUT_LIMIT;
    }
    if(input < -AxisConfig::INPUT_LIMIT) {
        input = -AxisConfig::INPUT_LIMIT;
    }

    // EMA 低通濾波：無刷運轉時 rc[3] 的高頻雜訊在此被平均掉
    if(!sFiltInit) {
        sFiltInit = true;
        sFiltInput = input; // 第一個命令直接貼齊，不等 EMA 收斂
    } else {
        sFiltInput += (input - sFiltInput) * ROW_EMA_ALPHA;
    }

    int32_t s = inputToSteps(sFiltInput);
    // 收到「不同的」新目標 → 解除 E-Stop 後的補走鎖定
    // （同一個值每控制幀都會重送，不算新命令）
    if(s != targetSteps)
        sHoldAfterEstop = false;
    // 行程限制：以中間（0 步）為原點，前 / 後各最多 2400 步
    if(s > STEP_LIMIT_FRONT)
        s = STEP_LIMIT_FRONT;
    if(s < -STEP_LIMIT_BACK)
        s = -STEP_LIMIT_BACK;
    targetSteps = s;
}

/// 目前命令目標 / 位置估算（Core 1 context，供日誌與遙測讀取）
int32_t targetStepsValue() { return targetSteps; }
int32_t refStepsValue() { return refSteps; }
float filteredInputValue() { return sFiltInput; } // 診斷：濾波後的 row 輸入
bool isMoving() { return moving; }

/// idleHook：推進水平軸
void update() {
    if(!moving) {
        // E-Stop 後鎖定：等新的 row 命令（不同目標值）才解除
        if(sHoldAfterEstop) {
            return;
        }
        int32_t diff = targetSteps - refSteps;
        if(diff == 0) {
            return; // 已到位，零開銷
        }
        // 小於 MIN_MOVE_STEPS 的位移差直接忽略：E-Stop 回補估算誤差
        // 或雜訊 row 值只差幾十步時，不起動（看起來像「被干擾跳一點」的來源）
        if(diff > -MIN_MOVE_STEPS && diff < MIN_MOVE_STEPS) {
            refSteps = targetSteps; // 視為到位，位置估算直接貼齊目標
            return;
        }
        // 開機靜默期：目標照樣記錄，但不動（供電最不穩的這段不起動）
        if(millis() < BOOT_HOLD_MS) {
            return;
        }
        startSegment(diff);
        return;
    }

    if(pioReady && pio_interrupt_get(sPio, kIrqFlag)) {
        pio_interrupt_clear(sPio, kIrqFlag);
        moving = false;
        refSteps = planTargetSteps;
        curSps = 0.0f;
        storageLog.printf("[STEP] 到位 pos=%ld", static_cast<long>(refSteps));
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
        pio_sm_set_clkdiv(
            sPio, sSm,
            PioStepRate::clkdivForSps(sClkSysHz, kCyclesPerStep, curSps));
    }
}

/// E-Stop：立即停止脈衝。
/// ⚠ 不再自動補走原位：ESC 湘浪/EMI 會讓 UART 反覆掉幀 → E-Stop 風暴，
///   若解除後自動補走，馬達會陷入「轉 → E-Stop 停 → 補走 → 又 E-Stop」
///   的循環，看起來就是無刷一轉步進就自轉。改為：E-Stop 後位置凍結，
///   等收到「不同的」新 row 目標才會再動（setTarget 解除 sHoldAfterEstop）。
void estop() {
    haltPulses();
    if(moving) {
        refSteps += (curDir > 0 ? 1 : -1) * static_cast<int32_t>(estSteps + 0.5f);
        storageLog.printf("[STEP] E-Stop 停止 pos=%ld", static_cast<long>(refSteps));
    }
    moving = false;
    curSps = 0.0f;
    estSteps = 0.0f;
    sHoldAfterEstop = true;
}

int32_t positionSteps() { return moving ? refSteps : targetSteps; }
} // namespace RowStepper

// ============================================================================
// SlaveApp 回呼
// ============================================================================
static uint8_t tmFrameSeq = 0;       // 遙測幀序號
static MoveTelemetry pendingTm = {}; // Core 1 寫入 → Core 0 發送

static void refreshTelemetry(); // 定義於下方

static void onStorageEstop() {
    // ⚠️ 本函式可能由 Core 0（SlaveComms::update() 的 100ms 逾時、或收到 ESTOP
    //    廣播）與 Core 1（runCore1() 的 E-Stop 分支）兩邊呼叫。
    //    SlaveComms 的 estopActive_ 分支本來就會在 Core 1 每圈呼叫 onEstop，
    //    所以 Core 0 這邊的呼叫是冗餘的，而且若在 Core 0 執行
    //    ServoSeq::stop() / goHomeAll() 會造成：
    //      1. Core 0 做阻塞式 I2C（每次最長 ~60ms × 17 通道），與 Core 1
    //         idleHook 的 I2C（holdRefresh / pollPostChannel / 序列寫入）並發
    //         → 同一個 DW_apb_i2c 被兩顆核心同時驅動，匯流排可能整個卡死
    //         （兩邊同時 busRecover 更會 i2c_deinit 與對方交易互踩）。
    //      2. PCA9685 收到半截交易 → 暫存器被寫入垃圾（最壞 latch 到
    //         FULL_ON，伺服撞機械極限堵轉）。
    //      3. ServoSeq / chTarget* 等非原子共享狀態被雙核同時改寫。
    //    因此 Core 0 只負責 ESC（無 I2C / 無共享狀態），其餘一律等 Core 1
    //    自己的 E-Stop 分支處理。
    if(get_core_num() == 0) {
        EscOutput::setPercent(0); // E-Stop → ESC 停轉（純 PWM 暫存器，Core 0 安全）
        return;
    }

    // ---- 以下只在 Core 1 執行 ----
    // E-Stop 邊緣觸發（runCore1 的 E-Stop 分支每圈都會呼叫 onEstop，
    // 不去重的話計數會暴增）。onStorageCommand 收到有效幀時清鎖存。
    if(!RowStepper::sInEstop) {
        RowStepper::sInEstop = true;
        RowStepper::sEstopCount++;
        static uint32_t sLastEstopLogMs = 0;
        const uint32_t nowMs = millis();
        if(nowMs - sLastEstopLogMs > 1000) {
            sLastEstopLogMs = nowMs;
            storageLog.printf("[STEP] E-Stop #%lu pos=%ld (無刷轉動時頻繁出現 = "
                              "ESC 湘浪打掉 UART/供電不穩)",
                              static_cast<unsigned long>(RowStepper::sEstopCount),
                              static_cast<long>(RowStepper::refStepsValue()));
        }
    }
    MotorControl::estop();
    EscOutput::setPercent(0); // E-Stop → ESC 停轉
    RowStepper::estop();      // 步進：只由 Core 1 操作 PIO
    // ⚠️ E-Stop 期間 runCore1() 只會一直呼叫 onEstop，**idleHook 不會被呼叫**，
    //    所以守護刷新與遙測保活要在這裡也跑一次，否則長時間 E-Stop 時：
    //    - 晶片設定若被干擾就沒人修（伺服就把位置結在錯的地方）
    //    - 遙測斷 500ms → Hub 會送 REBOOT 把 Storage 重開機
    MotorControl::holdRefresh();
    refreshTelemetry();
}

static void buildTelemetry(); // 定義於下方

static void onStorageCommand(const ChassisSnapshot &cmd) {
    // 收到有效控制幀 → E-Stop 已解除，清鎖存（下一次 E-Stop 再計一次）
    RowStepper::sInEstop = false;
    MotorControl::executeMotion(cmd);
    // 無刷 ESC（GP13）：sw=1 → 轉（爬升到 SPEED_PCT），sw=0 → 停（0%）
    EscOutput::setPercent(cmd.sw ? EscOutput::SPEED_PCT : 0);
    RowStepper::setTarget(cmd.row); // 水平：STEP/DIR 脈衝
    // 只在 row 數值變動時記一行（控制幀約 100Hz，不能每幀印）
    static int16_t sLastRow = 1000;
    if(cmd.row != sLastRow) {
        sLastRow = static_cast<int16_t>(cmd.row);
        // 同時印原始與濾波後值（×10 定點，避免 printf 浮點支援問題）
        storageLog.printf("[ROW] 命令 %d 濾後 %d", static_cast<int>(cmd.row),
                          static_cast<int>(RowStepper::filteredInputValue() * 10.0f));
    }
    buildTelemetry();
}

/// 構建遙測：currentRps[0] = 目前通道, [1] = 進度, [2] = 總數,
///             [3] = 已完成循環次數
static void buildTelemetry() {
    pendingTm = {};
    pendingTm.currentRps[0] = static_cast<float>(ServoSeq::currentChannel());
    pendingTm.currentRps[1] = static_cast<float>(ServoSeq::progress());
    pendingTm.currentRps[2] = static_cast<float>(ServoSeq::total());
    pendingTm.currentRps[3] = static_cast<float>(ServoSeq::doneCount());
    pendingTm.targetRps[0] = ServoSeq::isRunning() ? 1.0f : 0.0f;
    // 水平步進狀態借放 cmdVx/cmdVy/cmdOmega（Storage 不使用車身速度欄位）
    pendingTm.cmdVx = static_cast<float>(RowStepper::targetStepsValue());
    pendingTm.cmdVy = static_cast<float>(RowStepper::refStepsValue());
    pendingTm.cmdOmega = RowStepper::isMoving() ? 1.0f : 0.0f;
    pendingTm.mode = RobotMode::AUTO;
    pendingTm.estopFlag = 0;
    // ⚠ 遙測標頭（sync/msgType/frameSeq/targetID）、timestamp 與 CRC 由
    //   SignalUtils::finalizeTelemetry 統一填寫；手寫漏填 sync 或 CRC 會讓
    //   Hub 端驗證失敗而整包丟棄（序列監控只看到 "CRC mismatch"）。
    //   CRC 計算範圍不含 crc16 自身；timestamp 維持 us 計時的低 32 bits。
    SignalUtils::finalizeTelemetry(
        pendingTm, tmFrameSeq++,
        static_cast<uint32_t>(time_us_64() & 0xFFFFFFFFULL));
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
// 遙測保活（Core 1，每 40ms）：即使沒有新命令也要持續刷新遙測。
// ⚠️ 若只在收到命令時才送遙測，Hub 一旦 500ms 沒收到就會送 REBOOT，
//    Storage 會反覆重開機：伺服每次重開都回到 SLEEP→寫位置→唤回，
//    看起來就是「亂轉、有些沒通電、追加動作永遠做不完」。
static void refreshTelemetry() {
    static uint32_t lastMs = 0;
    const uint32_t now = millis();
    if(now - lastMs < 40)
        return;
    lastMs = now;
    buildTelemetry();
    slave.prepareTelemetry(pendingTm);
}
static void storageIdleHook() {
    RowStepper::update(); // 水平：PIO 脈衝 + 加減速
    ServoSeq::update();
    MotorControl::pollPostChannel();
    MotorControl::holdRefresh();
    refreshTelemetry();
}

// ============================================================================
// 狀態 LED（Core 0）
//   未連線 → 長亮；序列執行中 → 快閃（100ms）；閒置 → 慢閃（1000ms）
//   相位推進交給 lib/StatusLed，判定條件留在本檔。
// ============================================================================
namespace LedCfg {
constexpr uint32_t ACTIVE_ON_MS = 100;
constexpr uint32_t ACTIVE_OFF_MS = 100;
constexpr uint32_t IDLE_ON_MS = 1000;
constexpr uint32_t IDLE_OFF_MS = 1000;
} // namespace LedCfg

enum class LedState : uint8_t { BOOT = 0,
                                IDLE = 1,
                                ACTIVE = 2 };

// 亮滅圖樣表：索引 = LedState 的數值（相位推進交給 StatusLed）
//   BOOT   → 長亮（剛開機、從未收到有效幀）
//   IDLE   → 已連線但序列閒置，亮/滅各 1000ms（慢閃）
//   ACTIVE → 機構序列執行中，亮/滅各 100ms（快閃）
static const StatusLed::Pattern kLedPatterns[] = {
    {0, 0, true},                                         // BOOT：長亮
    {LedCfg::IDLE_ON_MS, LedCfg::IDLE_OFF_MS, false},     // IDLE
    {LedCfg::ACTIVE_ON_MS, LedCfg::ACTIVE_OFF_MS, false}, // ACTIVE
};
static constexpr uint8_t LED_PATTERN_COUNT =
    static_cast<uint8_t>(sizeof(kLedPatterns) / sizeof(kLedPatterns[0]));
static StatusLed statusLed(LED_BUILTIN, kLedPatterns, LED_PATTERN_COUNT);

static LedState currentLedState() {
    if(slave.validCmdCount() == 0)
        return LedState::BOOT; // 開機至今從未收到有效幀
    return ServoSeq::isRunning() ? LedState::ACTIVE : LedState::IDLE;
}

static void updateStatusLed() {
    // 狀態切換 → 重置相位並立刻點亮；長亮狀態持續 HIGH；其餘亮滿 onMs 熄、
    // 熄滿 offMs 亮（非阻塞，見 StatusLed::update）。
    statusLed.update(static_cast<uint8_t>(currentLedState()), millis());
}

// ============================================================================
// Core 0
// ============================================================================
void setup() {
    // 步進腳位安全狀態最先設定（STEP/DIR/EN 上電立刻有確定電位，不浮接）
    RowStepper::init();                    // 水平 STEP/DIR/EN（PIO0 SM）
    EscOutput::init(0); // ESC GP13 先維持 0%（停轉），等 Hub sw=1 才爬升轉速
    statusLed.begin();
    digitalWrite(LED_BUILTIN, HIGH); // 開機先亮（BOOT 狀態）

    Serial.begin(115200);
    delay(200);
    Serial.println("[STORAGE] boot");
    Serial.printf("[ESC] GP%u 50Hz arming %lums（先送 0%%），之後 %lums 內爬升到 %u%%\n",
                  static_cast<unsigned>(Pin::ESC_PWM),
                  static_cast<unsigned long>(EscOutput::ARM_DELAY_MS),
                  static_cast<unsigned long>(EscOutput::RAMP_MS),
                  static_cast<unsigned>(EscOutput::SPEED_PCT));

    slave.begin(); // 硬體 UART0 (GP0/GP1) → Hub
    slave.setIdleHook(storageIdleHook);
}

void loop() {
    slave.update();
    updateStatusLed();
    storageLog.drain(Serial);
    EscOutput::update(millis()); // ESC arming 等待 + 0% → 目標轉速爬升

    // PCA9685 初始化結果（Core 1 完成後印一次）
    static bool pcaPrinted = false;
    if(!pcaPrinted && sPcaInitDone) {
        pcaPrinted = true;
        Serial.printf("[PCA] 0x%02X %s | SLEEP=%s MODE1=0x%02X PRE_SCALE=%u\n",
                      Pca9685Reg::ADDR_DEFAULT, sPcaInitOk ? "OK" : "無回應!",
                      sPcaAwake ? "否(輸出中)" : "是(無脈衝!)",
                      static_cast<unsigned>(sSnap.mode1),
                      static_cast<unsigned>(sSnap.prescale));
    }

    // 每 2 秒一行運作狀態：PCA / 步進 / ESC
    static uint32_t lastStatusMs = 0;
    const uint32_t nowMs = millis();
    if(nowMs - lastStatusMs >= 2000) {
        lastStatusMs = nowMs;
        Serial.printf("[STATUS] PCA err=%lu repair=%lu | STEP pos=%ld tgt=%ld %s | "
                      "ESC %u%% | Hub cmd=%lu | estop=%lu\n",
                      static_cast<unsigned long>(sPca.errorCount()),
                      static_cast<unsigned long>(MotorControl::repairCount()),
                      static_cast<long>(RowStepper::refStepsValue()),
                      static_cast<long>(RowStepper::targetStepsValue()),
                      RowStepper::isMoving() ? "移動中" : "停止",
                      static_cast<unsigned>(EscOutput::currentPercent()),
                      static_cast<unsigned long>(slave.validCmdCount()),
                      static_cast<unsigned long>(RowStepper::sEstopCount));
    }
}

// ============================================================================
// Core 1
// ============================================================================
void setup1() { MotorControl::init(); }

void loop1() { slave.runCore1(); }
