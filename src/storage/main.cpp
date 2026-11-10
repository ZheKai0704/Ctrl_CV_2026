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
#include <EepromStore.h>
#include <MathUtils.h>
#include <Pca9685.h>
#include <SlaveLog.h>
#include <StatusLed.h>
#include <cmath>
#include <cstdio>
#include <pico/stdlib.h> // time_us_64()（遙測時間戳）

// ============================================================================
// 腳位 / 常數
// ============================================================================
namespace Pin {
constexpr uint I2C_SDA = 4;
constexpr uint I2C_SCL = 5;
constexpr uint I2C_HZ = 100000;
} // namespace Pin

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
    /* CH0  */ {30.0f, 110.0f, false},
    /* CH1  */ {30.0f, 100.0f, false},
    /* CH2  */ {30.0f, 110.0f, false},
    /* CH3  */ {30.0f, 110.0f, false},
    /* CH4  */ {30.0f, 110.0f, false},
    /* CH5  */ {30.0f, 110.0f, false},
    /* CH6  */ {80.0f, 30.0f, false}, // 追加動作那顆：平常 75° → 打到 30°
    /* CH7  */ {30.0f, 110.0f, false},
    /* CH8  */ {30.0f, 110.0f, false},
    /* CH9  */ {30.0f, 110.0f, false},
    /* CH10 */ {30.0f, 110.0f, false},
    /* CH11 */ {30.0f, 110.0f, false},
    /* CH12 */ {30.0f, 110.0f, false},
    /* CH13 */ {30.0f, 120.0f, false},
    /* CH14 */ {30.0f, 110.0f, false},
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

constexpr uint32_t DWELL_MS = 1000; // 作動角度停留
constexpr uint32_t SETTLE_MS = 500; // 回待機角度後等待，再換下一顆

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

// 主序列做完後的「追加動作」：單顆伺服抽動一次
//   CH6 不在 GROUP_ORDER 內，不會與主序列搶同一顆馬達。
//   角度也走同一份 PROFILE 表（預設 PROFILE[6] = {75, 30, false}）：
//   平常停在 restDeg(6) = 75°，主序列做完 → 打到 activeDeg(6) = 30°
//   → 停留 POST_DWELL_MS → 回 75°，然後才等下一個 trigger。
constexpr uint8_t POST_CHANNEL = 6;
constexpr uint32_t POST_DWELL_MS = 1000; // 停在作動角度的時間
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
// Core 1 → Core 0 日誌（lib/SlaveLog 的環形佇列，本檔持有唯一實例）
//
// ⚠ AGENTS.md 規定 Serial 只給 Core 0。Core 1 直接 Serial.printf() 在 USB CDC
//    TX FIFO 滿時會阻塞（實測最長 1 秒），會把機構序列卡在半途、看起來像
//    「seq=RUN 但馬達不動」。因此 Core 1 只把短字串丟進環形佇列，
//    由 Core 0 的 loop() 統一把它印出來（drain(Serial)）。
// ============================================================================
static SlaveLog storageLog;

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
    const bool wasRunning = running || postPending;
    running = false;
    postPending = false;
    postStep = 0;
    phase = PHASE_NONE;
    if(force || !atHome)
        goHomeAll(); // 全部回各自待機角度
    if(wasRunning) {
        // E-Stop / 停止：主序列全部回各自待機角度，追加那顆也回自己的待機角度
        const int32_t p =
            static_cast<int32_t>(lroundf(SeqCfg::restDeg(SeqCfg::POST_CHANNEL) * 10.0f));
        storageLog.printf("[STORAGE][SEQ] 停止，全部回各自待機角度，CH%u 停在 %ld.%u°",
                          static_cast<unsigned>(SeqCfg::POST_CHANNEL),
                          static_cast<long>(p / 10), static_cast<unsigned>(p % 10));
    }
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
            running = true;
            phase = PHASE_OUT;
            phaseMs = millis();
            cycleStartMs = phaseMs;
            // 一行把「第幾動 + 組 + 次數 + 通道」寫進佇列，由 Core 0 印出
            char line[SlaveLog::LINE_LEN];
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
            storageLog.push(line);
            return;
        }
        action = static_cast<uint8_t>((action + 1) % SeqCfg::ACTION_COUNT);
    }
    nextAction = 0;
    storageLog.push("[STORAGE][SEQ] 沒有已啟用的通道（EEPROM 校正全部關閉）");
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
    storageLog.printf("[STORAGE][SEQ] 追加：CH%u → %u°（停留 %lums 後回 %u°）",
                      static_cast<unsigned>(SeqCfg::POST_CHANNEL),
                      static_cast<unsigned>(
                          lroundf(SeqCfg::activeDeg(SeqCfg::POST_CHANNEL))),
                      static_cast<unsigned long>(SeqCfg::POST_DWELL_MS),
                      static_cast<unsigned>(
                          lroundf(SeqCfg::restDeg(SeqCfg::POST_CHANNEL))));
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
        storageLog.printf("[STORAGE][SEQ] 追加：CH%u → %u°（回待機）",
                          static_cast<unsigned>(SeqCfg::POST_CHANNEL),
                          static_cast<unsigned>(
                              lroundf(SeqCfg::restDeg(SeqCfg::POST_CHANNEL))));
        return;
    }
    // postStep == 1：已回待機角度，追加動作結束，等下一個 trigger
    postPending = false;
    running = false;
    phase = PHASE_NONE;
    storageLog.printf("[STORAGE][SEQ] 追加完成，CH%u 停在 %u°，等下一次 trigger",
                      static_cast<unsigned>(SeqCfg::POST_CHANNEL),
                      static_cast<unsigned>(
                          lroundf(SeqCfg::restDeg(SeqCfg::POST_CHANNEL))));
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
        storageLog.printf("[STORAGE][SEQ] 第 %u/%u 動完成（%lu ms），g%u 維持開啟，"
                          "等下一次 trigger",
                          static_cast<unsigned>(currentAction + 1),
                          static_cast<unsigned>(SeqCfg::ACTION_COUNT),
                          static_cast<unsigned long>(elapsed),
                          static_cast<unsigned>(currentGroup));
    } else {
        storageLog.printf("[STORAGE][SEQ] 第 %u/%u 動完成（%lu ms），"
                          "全部回 HOME，等下一次 trigger",
                          static_cast<unsigned>(currentAction + 1),
                          static_cast<unsigned>(SeqCfg::ACTION_COUNT),
                          static_cast<unsigned long>(elapsed));
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
        storageLog.printf("[STORAGE][SEQ] g%u %u/%u CH%u → %ld.%u° (step %u/%u)",
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
            const float rest = SeqCfg::restDeg(ch); // 該通道自己的待機角度
            if(!setAngle(ch, rest))
                writeErrors++;
            const int32_t h = static_cast<int32_t>(lroundf(rest * 10.0f));
            storageLog.printf("[STORAGE][SEQ] CH%u → %ld.%u°", ch,
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
            storageLog.printf("[STORAGE][SEQ] g%u 最後一輪 CH%u → %ld.%u°"
                              "（開啟，維持）",
                              static_cast<unsigned>(currentGroup),
                              static_cast<unsigned>(steps[i]),
                              static_cast<long>(o / 10),
                              static_cast<unsigned>(o % 10));
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
    uint8_t readFail = 0;
    uint8_t badCh = 0xFF;
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
        badCh = ch;
        if(ok) {
            repaired++;
        } else {
            readFail++;
        }
    }
    if(repaired == 0 && readFail == 0)
        return;
    sHoldRepaired += repaired;
    storageLog.printf("[STORAGE][HOLD] 讀回不一致→已重寫 %u 通道（讀回失敗 %u，累計 %lu）"
                      " 例如 CH%u",
                      static_cast<unsigned>(repaired),
                      static_cast<unsigned>(readFail),
                      static_cast<unsigned long>(sHoldRepaired),
                      static_cast<unsigned>(badCh));
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
        storageLog.printf("[STORAGE][CH%u] 讀回失敗（I2C 沒 ACK）",
                          static_cast<unsigned>(SeqCfg::POST_CHANNEL));
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
    storageLog.printf("[STORAGE][CH%u] 異常：want %u°=%uµs(count %u)，晶片實測 %uµs"
                      "(count %u)%s%s",
                      static_cast<unsigned>(SeqCfg::POST_CHANNEL),
                      static_cast<unsigned>(wantDeg), static_cast<unsigned>(wantUs),
                      static_cast<unsigned>(wantCount),
                      static_cast<unsigned>(actualUs), static_cast<unsigned>(off),
                      fullOff ? " full-OFF(沒脈衝)" : "",
                      fullOn ? " full-ON(恆高)" : "");
}
} // namespace MotorControl

// ============================================================================
// SlaveApp 回呼
// ============================================================================
static uint8_t tmFrameSeq = 0;       // 遙測幀序號
static MoveTelemetry pendingTm = {}; // Core 1 寫入 → Core 0 發送

static void refreshTelemetry(); // 定義於下方

static void onStorageEstop() {
    MotorControl::estop();
    // ⚠️ E-Stop 期間 runCore1() 只會一直呼叫 onEstop，**idleHook 不會被呼叫**，
    //    所以守護刷新與遙測保活要在這裡也跑一次，否則長時間 E-Stop 時：
    //    - 晶片設定若被干擾就沒人修（伺服就把位置結在錯的地方）
    //    - 遙測斷 500ms → Hub 會送 REBOOT 把 Storage 重開機
    MotorControl::holdRefresh();
    refreshTelemetry();
}

static void buildTelemetry(); // 定義於下方

static void onStorageCommand(const ChassisSnapshot &cmd) {
    MotorControl::executeMotion(cmd);
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
    statusLed.begin();
    digitalWrite(LED_BUILTIN, HIGH); // 開機先亮（BOOT 狀態）

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
    storageLog.drain(Serial);

    // Core 1 初始化完成後，由 Core 0 印一次設定摘要（Serial 只能在 Core 0）
    static bool initPrinted = false;
    if(!initPrinted && sPcaInitDone) {
        initPrinted = true;
        Serial.printf("[STORAGE] 通道校正%s\n",
                      sCalLoaded ? "已從 EEPROM 載入" : "使用預設值（16 通道）");
        // 注意：newlib-nano 沒開浮點 printf，這裡只用整數輸出（度數用 x10 表示）
        Serial.printf("[STORAGE] PCA9685 0x%02X %s | SLEEP=%s MODE1=0x%02X "
                      "PRE_SCALE=%u | dwell=%lums settle=%lums\n",
                      Pca9685Reg::ADDR_DEFAULT, sPcaInitOk ? "OK" : "無回應！",
                      sPcaAwake ? "已清除(有在送PWM)" : "仍為1(沒脈衝！)",
                      static_cast<unsigned>(sSnap.mode1),
                      static_cast<unsigned>(sSnap.prescale),
                      static_cast<unsigned long>(SeqCfg::DWELL_MS),
                      static_cast<unsigned long>(SeqCfg::SETTLE_MS));

        // 逐通道 dump：EEPROM 校正期望值 vs PCA9685 晶片內部實際值對照。
        //   最後兩欄是「晶片真的寫進去的 OFF count / 換算 µs」：
        //   - 跟前面待機/作動脈寬不同 → I2C 寫入沒落地（接線/上拉/干擾）
        //   - full-OFF（無脈衝） → 伺服沒訊號（看起來像「沒通電」、不鎖定）
        //   - full-ON            → 訊號恆高，伺服會往單邊撞
        Serial.println("[STORAGE] ch en us(min-max) deg(min-max) inv | "
                       "start→end rev | 待機/作動us | 晶片實際 off(count/µs) / 診斷");
        for(uint8_t ch = 0; ch < Pca9685Reg::CHANNEL_COUNT; ch++) {
            const ChannelLimit &l = Limits::channels[ch];
            const SeqCfg::ServoProfile &pf = SeqCfg::PROFILE[ch];
            const float restF = SeqCfg::restDeg(ch);
            const float actF = SeqCfg::activeDeg(ch);
            const int32_t sT = static_cast<int32_t>(lroundf(pf.startDeg * 10.0f));
            const int32_t eT = static_cast<int32_t>(lroundf(pf.endDeg * 10.0f));
            const uint16_t hUs = angleToPulseUs(ch, restF);
            const uint16_t oUs = angleToPulseUs(ch, actF);
            const bool hClamp = angleClampedByRange(ch, restF);
            const bool oClamp = angleClampedByRange(ch, actF);
            Serial.printf("  %2u  %u  %4u-%4u  %3u-%3u  %u | %4ld.%u→%4ld.%u %s | "
                          "%4u/%4u",
                          static_cast<unsigned>(ch),
                          static_cast<unsigned>(l.enabled),
                          static_cast<unsigned>(l.minUs),
                          static_cast<unsigned>(l.maxUs),
                          static_cast<unsigned>(l.minDeg),
                          static_cast<unsigned>(l.maxDeg),
                          static_cast<unsigned>(
                              Limits::effectiveInvert(ch) ? 1 : 0),
                          static_cast<long>(sT / 10), static_cast<unsigned>(sT % 10),
                          static_cast<long>(eT / 10), static_cast<unsigned>(eT % 10),
                          pf.reverse ? "R" : "F",
                          static_cast<unsigned>(hUs),
                          static_cast<unsigned>(oUs));
            if((sSnap.readOkMask & (1u << ch)) == 0) {
                Serial.println(" | 讀回失敗（I2C）");
                continue;
            }
            const uint16_t offCount = sSnap.offCount[ch];
            if((sSnap.fullOffMask & (1u << ch)) != 0)
                Serial.print(" | full-OFF 無脈衝（伺服不會通電/不鎖定）");
            else if((sSnap.fullOnMask & (1u << ch)) != 0)
                Serial.print(" | full-ON 恆高（伺服會往單邊撞死）");
            else
                Serial.printf(" | %4u/%4uus", static_cast<unsigned>(offCount),
                              static_cast<unsigned>(offCount *
                                                    Pca9685Reg::PERIOD_US /
                                                    Pca9685Reg::COUNT_SPAN));
            if(hClamp || oClamp)
                Serial.printf("  <-- %s%s被 EEPROM 角度範圍夾住（命令到不了，會頂死點）",
                              hClamp ? "待機 " : "", oClamp ? "作動 " : "");
            else if(l.enabled && hUs == oUs)
                Serial.print("  <-- 待機=作動（動作無效，檢查 start/end）");
            Serial.println();
        }
        Serial.println("[STORAGE] ※ 若上表脈寬都正確但伺服不動：檢查 PCA9685 V+ "
                       "伺服電源、GND 共地、OE 是否拉低");

        // 追加動作（CH6）診斷：EEPROM 沒 enable 也照樣驅動，但脈寬/角度範圍
        // 若被夾住，伺服就會「不動」或頂在極限「嗡嗡拌」。
        {
            const uint8_t ps = SeqCfg::POST_CHANNEL;
            const ChannelLimit &l = Limits::channels[ps];
            const float pRest = SeqCfg::restDeg(ps);
            const float pAct = SeqCfg::activeDeg(ps);
            Serial.printf("[STORAGE] 追加 CH%u：EEPROM enabled=%u%s | "
                          "待機 %u°=%uus / 作動 %u°=%uus | dwell %lums\n",
                          static_cast<unsigned>(ps),
                          static_cast<unsigned>(l.enabled),
                          channelEnabledInEeprom(ps) ? "" : "（本版強制驅動）",
                          static_cast<unsigned>(lroundf(pRest)),
                          static_cast<unsigned>(angleToPulseUs(ps, pRest)),
                          static_cast<unsigned>(lroundf(pAct)),
                          static_cast<unsigned>(angleToPulseUs(ps, pAct)),
                          static_cast<unsigned long>(SeqCfg::POST_DWELL_MS));
            if(!channelEnabledInEeprom(ps))
                Serial.printf("[STORAGE] ※ CH%u 在 EEPROM 是停用（enabled=0）；"
                              "本版有強制驅動，但校正值（%u-%u° / %u-%uuS）"
                              "仍是預設/未校正值，請用 pca9685_angle 校正後再寫回\n",
                              static_cast<unsigned>(ps),
                              static_cast<unsigned>(l.minDeg),
                              static_cast<unsigned>(l.maxDeg),
                              static_cast<unsigned>(l.minUs),
                              static_cast<unsigned>(l.maxUs));
            if(angleClampedByRange(ps, pRest) ||
               angleClampedByRange(ps, pAct))
                Serial.printf("[STORAGE] ※ CH%u 的 EEPROM 角度範圍 %u-%u° 會把 "
                              "%u°/%u° 夾住 → 伺服會頂到極限（嗡嗡叫、不鎖定），"
                              "請用 pca9685_angle 的 'ma 6 <minUs> <maxUs>' 重新校正\n",
                              static_cast<unsigned>(ps),
                              static_cast<unsigned>(l.minDeg),
                              static_cast<unsigned>(l.maxDeg),
                              static_cast<unsigned>(lroundf(pRest)),
                              static_cast<unsigned>(lroundf(pAct)));
        }

        // 已停用通道目前的處理：DRIVE_DISABLED_CHANNELS=true 時仍會停在 HOME，
        // 只是不參與 trigger 序列；false 時則是 full-OFF（沒有脈衝 → 不鎖定）。
        {
            bool anyOff = false;
            for(uint8_t ch = 0; ch < Pca9685Reg::CHANNEL_COUNT; ch++) {
                if(channelEnabledInEeprom(ch) || ch == SeqCfg::POST_CHANNEL)
                    continue;
                if(!anyOff) {
                    Serial.print("[STORAGE] ※ EEPROM 未啟用通道：");
                    anyOff = true;
                }
                Serial.printf(" CH%u", static_cast<unsigned>(ch));
            }
            if(anyOff)
                Serial.printf(" → 不參與 trigger 序列，目前%s\n",
                              SeqCfg::DRIVE_DISABLED_CHANNELS
                                  ? "仍停在各自待機角度保持鎖定"
                                  : "輸出 full-OFF（沒有脈衝，會抖、不鎖定）");
        }
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
                          "cycles=%lu | i2cErr=%lu seqErr=%lu hold=%lu drop=%lu\n",
                          slave.msSinceLastValidCmd(),
                          static_cast<unsigned>(slave.validCmdCount()),
                          ServoSeq::isRunning() ? "RUN" : "IDLE",
                          static_cast<unsigned>(ServoSeq::progress()),
                          static_cast<unsigned>(ServoSeq::total()),
                          static_cast<int>(ServoSeq::currentChannel()),
                          static_cast<unsigned long>(ServoSeq::doneCount()),
                          static_cast<unsigned long>(sPca.errorCount()),
                          static_cast<unsigned long>(
                              ServoSeq::writeErrorCount()),
                          static_cast<unsigned long>(MotorControl::repairCount()),
                          static_cast<unsigned long>(storageLog.dropped()));
        }
    }
}

// ============================================================================
// Core 1
// ============================================================================
void setup1() { MotorControl::init(); }

void loop1() { slave.runCore1(); }
