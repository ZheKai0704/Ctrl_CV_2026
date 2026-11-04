// ============================================================================
// move_pinmap — Move 板「腳位自動對應測試」v2  (RP2350，獨立程式)
//
// 為什麼是 v2：
//   v1 靠互動指令 + 「上下拉普查跳過被驅動腳」+ 25% 出力，實測會漏掉腳位
//   （外部下拉/上拉會讓普查誤判，25% 也可能推不動馬達）。v2 改成：
//     - 開機 3 秒後「全自動」跑完，完全不需要打字
//     - 先印出全部腳位的原始電位普查（含 GP0/GP1，方便對照實際接線）
//     - 出力改用「斜坡 18% → 45%」，確保克服靜摩擦
//     - PWM 腳：逐腳試轉，用「其他腳有沒有邊緣跳動」判定，不預設任何假設
//     - 編碼器：試轉時跳動最多的兩腳 = 該輪 A/B（並印出去噪比）
//     - DIR 腳：對每個已找到的 PWM，逐一候選腳拉 HIGH，看有號正交計數是否反向
//     - 最後產出「可直接貼回 src/move/main.cpp 的 namespace Pin」區塊
//
// 已知的舊板對應（本程式以此為預期值，用來標示「相同 / 不同」）：
//   FR: PWM=6  DIR=7  ENC=14,15
//   FL: PWM=9  DIR=8  ENC=16,17
//   BR: PWM=11 DIR=10 ENC=20,21
//   BL: PWM=12 DIR=13 ENC=18,19
//
// ### 安全 ###
//   - 每個 PWM 只轉 ~0.85 秒，最高 45%（可用 scan 45 提高）
//   - 任何時候按 Enter 可中止；所有腳位結束後一律回「輸入 + pull-down」
//   - 不驅動 GP0/GP1（UART0，避免與 Hub 對打）
//
// Serial 命令（115200）：
//   run [duty%]   重新跑一次全自動對應（預設 45%）
//   dump          印出腳位原始電位普查
//   enc           每秒列出各腳邊緣數（手動轉輪確認編碼器）
//   quad <a> <b>  即時有號正交計數
//   spin <pin> <duty>   單腳 PWM 輸出（Enter 停）
//   dir <pin> <0|1>     單腳固定輸出（測 DIR / EN）
//   rel           全部釋放回「輸入 + pull-down」
//   pwmoff        停止所有 PWM
//   help
// ============================================================================
#include <Arduino.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>

// ============================================================================
// 參數
// ============================================================================
namespace Cfg {
constexpr uint16_t PWM_WRAP = 1000;      // 20kHz（clkdiv 6.25，與正式韌體相同）
constexpr float SCAN_DUTY = 45.0f;       // 掃描出力（%）
constexpr float SCAN_RAMP_DUTY = 18.0f;  // 起步斜坡（克服靜摩擦用）
constexpr uint32_t RAMP_MS = 150;
constexpr uint32_t SAMPLE_MS = 700;
constexpr uint32_t DIR_SAMPLE_MS = 450;
constexpr uint16_t EDGE_MIN = 20;        // 判定「有在跳」的最小邊緣數
constexpr int32_t SIGN_MIN = 12;         // 判定「有轉」的最小有號計數
} // namespace Cfg

// 候選腳位（GP0/GP1 = UART0，不列入；GP23~25 板內保留）
static const uint8_t kCandidates[] = {
    2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14,
    15, 16, 17, 18, 19, 20, 21, 22, 26, 27, 28, 29,
};
static constexpr size_t kNumCand = sizeof(kCandidates) / sizeof(kCandidates[0]);

// 舊板（預期）對應，用來標示相同/不同，也用來為輪子命名
struct Expected {
    const char *name;
    uint8_t pwm;
    uint8_t dir;
    uint8_t encA;
};
static const Expected kExpected[4] = {
    {"FR", 6, 7, 14},
    {"FL", 9, 8, 16},
    {"BR", 10, 11, 20},
    {"BL", 12, 13, 18},
};

// ============================================================================
// 狀態
// ============================================================================
static int8_t g_gpioToIdx[32];
static uint32_t g_maskAll = 0;
static uint32_t g_pwmSliceInit = 0;
static bool g_pullUp = false; // 監看模式的內部上下拉
static bool g_abort = false;
static bool g_started = false;
// 「持續拉高」的腳（例如驅動板的 EN/SLEEP）。掃描時不會被重設成輸入。
static uint32_t g_holdMask = 0;
static int8_t g_enPin = -1; // 自動偵測到的致能腳

static void pollAbort();
static void dirBaseline(uint pwm, uint encA, uint encB, bool baseHigh);

// 正交解碼表（同 src/move/main.cpp，索引 = [prev][cur]，bit0=A bit1=B）
static const int8_t kQuadTable[4][4] = {
    {0, +1, -1, 0},
    {-1, 0, 0, +1},
    {+1, 0, 0, -1},
    {0, -1, +1, 0},
};

// ============================================================================
// 基本工具
// ============================================================================
static bool isCandidate(uint pin) {
    return pin < 32 && g_gpioToIdx[pin] >= 0;
}

static const char *pinName(uint pin) {
    static char buf[4][10];
    static int n = 0;
    char *p = buf[n];
    n = (n + 1) % 4;
    if(pin >= 32) snprintf(p, sizeof(buf[0]), "--");
    else snprintf(p, sizeof(buf[0]), "GP%u", static_cast<unsigned>(pin));
    return p;
}

static void pinToInput(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    if(g_pullUp)
        gpio_pull_up(pin);
    else
        gpio_pull_down(pin);
}

static void pinStrong(uint pin, bool level) {
    gpio_set_function(pin, GPIO_FUNC_SIO);
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, level);
}

static void releasePin(uint pin) {
    const uint slice = pwm_gpio_to_slice_num(pin);
    if(g_pwmSliceInit & (1u << slice)) pwm_set_gpio_level(pin, 0);
    gpio_set_function(pin, GPIO_FUNC_SIO);
    pinToInput(pin);
    gpio_pull_down(pin);
}

static void releaseAll() {
    g_holdMask = 0;
    for(size_t i = 0; i < kNumCand; i++)
        releasePin(kCandidates[i]);
    g_pwmSliceInit = 0;
}

static void holdPin(uint pin, bool level) {
    pinStrong(pin, level);
    if(level)
        g_holdMask |= 1u << pin;
    else
        g_holdMask &= ~(1u << pin);
}

static void allInputsExcept(uint skipMask) {
    for(size_t i = 0; i < kNumCand; i++) {
        const uint pin = kCandidates[i];
        if(skipMask & (1u << pin)) continue;
        if(g_holdMask & (1u << pin)) continue; // 保持拉高（EN 腳）
        pinToInput(pin);
        if(!g_pullUp) gpio_pull_down(pin);
    }
}

static void pwmClaim(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    const uint slice = pwm_gpio_to_slice_num(pin);
    if(!(g_pwmSliceInit & (1u << slice))) {
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, 6.25f);
        pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_FREE_RUNNING);
        pwm_config_set_wrap(&cfg, Cfg::PWM_WRAP);
        pwm_init(slice, &cfg, true);
        g_pwmSliceInit |= 1u << slice;
    }
    pwm_set_gpio_level(pin, 0);
}

static void pwmDuty(uint pin, float pct) {
    if(pct < 0.0f) pct = 0.0f;
    if(pct > 100.0f) pct = 100.0f;
    pwm_set_gpio_level(pin,
                       static_cast<uint16_t>(pct / 100.0f * (Cfg::PWM_WRAP + 1)));
}

// ============================================================================
// Serial（行導向、可中途中止）
// ============================================================================
static char g_line[96];
static size_t g_lineLen = 0;
static bool g_swallowLF = false; // 吃掉 CRLF 的 LF（否則會被誤判成「按 Enter 中止」）

static bool pollLine(char *out, size_t outLen) {
    while(Serial.available()) {
        const int c = Serial.read();
        if(c < 0) break;
        // CRLF: '\r' ends the command, the following '\n' must be ignored
        if(c == '\n' && g_swallowLF) {
            g_swallowLF = false;
            continue;
        }
        g_swallowLF = false;
        if(c == '\r' || c == '\n') {
            if(c == '\r') g_swallowLF = true;
            if(g_lineLen == 0) {
                g_abort = true; // 真的空行（第二次 Enter）才算中止
                continue;
            }
            g_line[g_lineLen] = '\0';
            strncpy(out, g_line, outLen - 1);
            out[outLen - 1] = '\0';
            g_lineLen = 0;
            return true;
        }
        if(g_lineLen + 1 < sizeof(g_line)) g_line[g_lineLen++] = static_cast<char>(c);
    }
    return false;
}

static void pollAbort() {
    char dummy[8];
    if(pollLine(dummy, sizeof(dummy))) g_abort = true;
}

static void drainSerial() {
    while(Serial.available()) Serial.read();
    g_lineLen = 0;
    g_swallowLF = false;
    g_abort = false;
}

// 等 ms，期間維持 heartbeat + 可中止
static void waitMs(uint32_t ms) {
    const uint32_t t0 = millis();
    uint32_t beat = 0;
    while(millis() - t0 < ms) {
        pollAbort();
        if(g_abort) break;
        if((++beat & 0x3FF) == 0) digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delayMicroseconds(50);
    }
}

// ============================================================================
// 邊緣計數 / 有號正交計數
// ============================================================================
static void countEdges(uint32_t mask, uint32_t durationMs, uint16_t *out) {
    memset(out, 0, sizeof(uint16_t) * kNumCand);
    uint32_t prev = gpio_get_all() & mask;
    const uint32_t t0 = millis();
    uint32_t spin = 0;
    while(millis() - t0 < durationMs) {
        if((++spin & 0x7FF) == 0) pollAbort();
        if(g_abort) break;
        const uint32_t now = gpio_get_all() & mask;
        uint32_t diff = now ^ prev;
        prev = now;
        while(diff) {
            const int bit = __builtin_ctz(diff);
            diff &= diff - 1;
            const int idx = g_gpioToIdx[bit];
            if(idx >= 0 && out[idx] < 0xFFFF) out[idx]++;
        }
    }
}

// 一次量測同時算出三種資訊，方便判斷「到底是有轉還是解碼錯」：
//   quad  : Gray 表（正式韌體同一套）
//   order : 邊緣順序法（獨立實作，用來交叉驗證 quad）
//   ea/eb : A、B 各自的邊緣次數（有跳但 quad=0 → 解碼/相位問題）
struct Measure {
    int32_t quad = 0;
    int32_t order = 0;
    uint16_t ea = 0, eb = 0;
    int32_t busy = 0;
};

static Measure measureAll(uint aPin, uint bPin, uint32_t ms) {
    Measure m;
    uint8_t prev = static_cast<uint8_t>((gpio_get(aPin) ? 1u : 0u) |
                                        ((gpio_get(bPin) ? 1u : 0u) << 1));
    bool prevA = prev & 1u, prevB = (prev >> 1) & 1u;
    const uint32_t t0 = millis();
    uint32_t spin = 0;
    while(millis() - t0 < ms) {
        if((++spin & 0x7FF) == 0) pollAbort();
        if(g_abort) break;
        const uint32_t now = gpio_get_all();
        const bool a = ((now >> aPin) & 1u) != 0;
        const bool b = ((now >> bPin) & 1u) != 0;
        const uint8_t cur = static_cast<uint8_t>((a ? 1u : 0u) | ((b ? 1u : 0u) << 1));
        if(cur != prev) {
            m.busy++;
            m.quad += kQuadTable[prev][cur];
            // 邊緣順序法：A 動 → A!=B 為正；B 動 → A==B 為正（正交編碼器的相位關係）
            if(a != prevA) {
                m.ea++;
                m.order += (a != b) ? +1 : -1;
            } else if(b != prevB) {
                m.eb++;
                m.order += (a == b) ? +1 : -1;
            }
            prev = cur;
            prevA = a;
            prevB = b;
        }
    }
    return m;
}

// 取「可用」的有號計數：quad 為 0 但 order 有值時（相位跳變被取樣吃掉）就用 order
static int32_t measureSigned(uint aPin, uint bPin, uint32_t ms) {
    const Measure m = measureAll(aPin, bPin, ms);
    if(m.quad != 0 || m.order == 0) return m.quad;
    return m.order;
}

// ============================================================================
// 0) 腳位普查：印出 pull-down / pull-up 兩次讀值（純診斷，不跳過任何腳）
// ============================================================================
static void censusDump() {
    Serial.println(F("\n--------------- 腳位原始電位普查 ---------------"));
    Serial.println(F("GP    下拉讀 上拉讀  判讀"));
    for(uint pin = 0; pin <= 29; pin++) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_down(pin);
        delayMicroseconds(1500);
        const bool low = gpio_get(pin);
        gpio_pull_up(pin);
        delayMicroseconds(1500);
        const bool high = gpio_get(pin);
        pinToInput(pin);
        gpio_pull_down(pin);

        const char *tag = "輸入/浮空";
        if(low && high)
            tag = "外部上拉 (或強 pull-up)";
        else if(!low && !high)
            tag = "外部下拉 (或強 pull-down)";
        else if(!low && high)
            tag = "浮空 (可安全輸出)";
        if(pin == 0 || pin == 1) tag = "UART0（本程式不驅動）";
        Serial.printf("GP%-2u   %d      %d     %s\n", static_cast<unsigned>(pin),
                      low ? 1 : 0, high ? 1 : 0, tag);
    }
    Serial.println(F("----------------------------------------------"));
}

// ============================================================================
// 1) 找 PWM 腳 + 該輪編碼器 A/B
// ============================================================================
struct WheelMap {
    int8_t pwm = -1;
    int8_t dir = -1;
    int8_t dirHigh = -1; // 讓方向反轉的 DIR 電位（0/1）
    int8_t encA = -1;
    int8_t encB = -1;
    int32_t signLow = 0;  // 基準（其他腳為 LOW）時的有號計數
    int32_t signHigh = 0; // DIR 觸發後的有號計數
    bool baseWasHigh = false; // 基準是否為「其他腳全 HIGH」
    int32_t baseSign = 0;     // 基準計數（原始）
    int8_t expected = -1; // 對應的舊板輪別（0..3）
    bool dirIsExpected = false;
    uint16_t edgesA = 0, edgesB = 0; // 掃描時量到的編碼器邊緣數（A/B 都有跳才成對）
};

static WheelMap g_wheels[6];
static size_t g_numWheels = 0;
static uint8_t g_claimed[32]; // 已被指派的腳位用途（只做記錄）

static int expectedOfPwm(uint pin) {
    for(int i = 0; i < 4; i++)
        if(kExpected[i].pwm == pin) return i;
    return -1;
}
static int expectedOfEnc(uint aPin) {
    for(int i = 0; i < 4; i++)
        if(kExpected[i].encA == aPin) return i;
    return -1;
}

// 讀回 PWM 硬體 level（cc 暫存器）— 證明真的有輸出，而不是被誰關掉
static uint16_t pwmHardwareLevel(uint pin) {
    const uint slice = pwm_gpio_to_slice_num(pin);
    const uint chan = pwm_gpio_to_channel(pin);
    const uint32_t cc = pwm_hw->slice[slice].cc;
    return static_cast<uint16_t>((cc >> (chan ? 16 : 0)) & 0xFFFF);
}

// 量測期間：感測腳一律 pull-up（open-drain / 開集極編碼器才讀得到）
static void sensePinsPullUp(uint skipMask) {
    for(size_t i = 0; i < kNumCand; i++) {
        const uint p = kCandidates[i];
        if(skipMask & (1u << p)) continue;
        if(g_holdMask & (1u << p)) continue;
        gpio_pull_up(p);
    }
}

// 對某一腳試轉，回報哪些腳在跳
static void spinAndSense(uint pin, float duty, uint16_t *edges) {
    const uint32_t mask = g_maskAll & ~(1u << pin);
    allInputsExcept(1u << pin);
    sensePinsPullUp(1u << pin);
    gpio_pull_down(pin);

    pwmClaim(pin);
    pwmDuty(pin, Cfg::SCAN_RAMP_DUTY);
    waitMs(Cfg::RAMP_MS);
    pwmDuty(pin, duty);
    countEdges(mask, Cfg::SAMPLE_MS, edges);
    pwmDuty(pin, 0.0f);
    releasePin(pin);
}

// 讓某腳跑 PWM（其他腳 LOW、編碼器上拉），回傳該輪的有號計數
static int32_t spinBaseline(uint pwm, uint encA, uint encB, float duty) {
    pwmClaim(pwm);
    pwmDuty(pwm, Cfg::SCAN_RAMP_DUTY);
    waitMs(Cfg::RAMP_MS);
    pwmDuty(pwm, duty);
    dirBaseline(pwm, encA, encB, false);
    const int32_t v = measureSigned(encA, encB, Cfg::DIR_SAMPLE_MS);
    pwmDuty(pwm, 0.0f);
    releasePin(pwm);
    return v;
}

// 馬達側腳位（舊板 PWM/DIR 都在 GP2~GP13）。先只掃這裡，
// 避免把「編碼器輸出」當馬達 PWM 硬推。
static bool inMotorZone(uint pin) { return pin >= 2 && pin <= 13; }

static void discoverMotors(float duty, bool wide, bool reset) {
    if(reset) {
        g_numWheels = 0;
        memset(g_claimed, 0, sizeof(g_claimed));
    }

    Serial.printf("\n[1/3] 逐腳試轉找 PWM（%s，斜坡 %d%% → %d%%，每腳 0.85s）\n",
                  wide ? "全腳位" : "GP2~GP13 馬達側",
                  static_cast<int>(Cfg::SCAN_RAMP_DUTY + 0.5f),
                  static_cast<int>(duty + 0.5f));
    uint16_t edges[kNumCand];

    for(size_t i = 0; i < kNumCand; i++) {
        if(g_abort) return;
        const uint pin = kCandidates[i];
        if(!wide && !inMotorZone(pin)) continue;
        if(g_claimed[pin]) continue; // 已指派（PWM/DIR/編碼器）

        spinAndSense(pin, duty, edges);

        // 取跳動最多的兩腳
        int iA = -1, iB = -1;
        for(size_t j = 0; j < kNumCand; j++) {
            if(edges[j] < Cfg::EDGE_MIN) continue;
            if(iA < 0 || edges[j] > edges[iA]) {
                iB = iA;
                iA = static_cast<int>(j);
            } else if(iB < 0 || edges[j] > edges[iB]) {
                iB = static_cast<int>(j);
            }
        }

        Serial.printf("  %s: ", pinName(pin));
        if(iA < 0) {
            Serial.println(F("無任何腳位跳動 → 不是馬達 PWM"));
            continue;
        }

        const uint encA = kCandidates[iA];
        const uint encB = (iB >= 0) ? kCandidates[iB] : 0xFF;

        // 兩個 A/B 應該邊緣數接近（去噪比）；否則只是雜訊
        bool pairOk = false;
        if(iB >= 0) {
            const uint16_t ca = edges[iA], cb = edges[iB];
            const uint16_t hi = ca > cb ? ca : cb;
            const uint16_t lo = ca > cb ? cb : ca;
            pairOk = (lo * 4 >= hi);
        }

        if(!pairOk) {
            Serial.printf("只有 %s 在跳（%u 次），未成對 → 略過\n", pinName(encA),
                          static_cast<unsigned>(edges[iA]));
            continue;
        }

        Serial.printf("★ 馬達轉動！編碼器 = %s / %s  (邊緣 %u / %u)%s\n",
                      pinName(encA), pinName(encB),
                      static_cast<unsigned>(edges[iA]),
                      static_cast<unsigned>(edges[iB]),
                      (expectedOfPwm(pin) >= 0) ? "  [與舊板相同]" : "  [腳位已改變]");

        // 同一組編碼器已經登錄過 → 兩隻腳都會讓這輪轉：
        //   真 PWM（其他腳 LOW）轉得多；DIR 被 20kHz/45% 方波推只會慢慢轉。
        //   所以保留「轉動量大」的那一腳當 PWM。
        int dupIdx = -1;
        for(size_t k = 0; k < g_numWheels; k++) {
            const WheelMap &w = g_wheels[k];
            if(w.encA == static_cast<int8_t>(encA) && w.encB == static_cast<int8_t>(encB))
                dupIdx = static_cast<int>(k);
        }
        if(dupIdx >= 0) {
            const int32_t nb = spinBaseline(pin, encA, encB, duty);
            WheelMap &old = g_wheels[dupIdx];
            if(abs(nb) > abs(old.baseSign)) {
                Serial.printf("     ↻ 這腳轉得更多（%+ld vs %+ld）→ 改用 %s 當 PWM，"
                              "%s 退回候選 DIR\n",
                              static_cast<long>(nb), static_cast<long>(old.baseSign),
                              pinName(pin), pinName(static_cast<uint>(old.pwm)));
                g_claimed[static_cast<uint>(old.pwm)] = 0;
                old.pwm = static_cast<int8_t>(pin);
                old.baseSign = nb;
                g_claimed[pin] = 1;
            } else {
                Serial.printf("     （同一組編碼器已登錄；本腳轉動量 %+ld 較小 → 視為 DIR 候選）\n",
                              static_cast<long>(nb));
            }
            continue;
        }

        WheelMap &w = g_wheels[g_numWheels++];
        w.pwm = static_cast<int8_t>(pin);
        w.encA = static_cast<int8_t>(encA);
        w.encB = static_cast<int8_t>(encB);
        w.edgesA = edges[iA];
        w.edgesB = edges[iB];
        w.expected = static_cast<int8_t>(expectedOfEnc(encA));
        if(w.expected < 0) w.expected = static_cast<int8_t>(expectedOfPwm(pin));
        g_claimed[pin] = 1;
        g_claimed[encA] = 2;
        g_claimed[encB] = 2;

        // 立即驗證：這腳真的「像 PWM」嗎？
        //   DIR 腳若被 20 kHz / 45% 的方波推，也會因為正反不對稱而慢慢轉，
        //   造成「DIR 被誤認成 PWM」。真正的 PWM 轉動量會遠大於此。
        const int32_t base = spinBaseline(pin, encA, encB, duty);
        w.baseSign = base;
        if(abs(base) < Cfg::SIGN_MIN) {
            Serial.printf("     ⚠ 有號計數只有 %+ld（幾乎沒動）→ 這腳可能不是 PWM\n",
                          static_cast<long>(base));
        } else if(abs(base) < Cfg::SIGN_MIN * 3) {
            Serial.printf("     ⚠ 有號計數 %+ld 偏小 → 有可能這腳其實是 DIR（真 PWM 在別隻）\n",
                          static_cast<long>(base));
        } else {
            Serial.printf("     驗證：DIR=LOW 時有號計數 %+ld ✓ 確認為 PWM\n",
                          static_cast<long>(base));
        }
    }

    Serial.printf("[1/3] 目前共 %u 組馬達/編碼器\n", static_cast<unsigned>(g_numWheels));
}

// ============================================================================
// 1b) 找致能腳（EN / SLEEP / STBY）：沒有任何輪子轉時才會跑
//     做法：逐一候選腳拉 HIGH，然後試轉「舊板的 4 個 PWM 腳」，看有沒有輪子動。
// ============================================================================
static int8_t huntEnable(float duty) {
    Serial.println(F("\n[1b] 找不到轉動 → 自動搜尋致能腳（EN/SLEEP），逐一拉 HIGH 試舊板 PWM"));
    uint16_t edges[kNumCand];

    for(size_t i = 0; i < kNumCand; i++) {
        if(g_abort) return -1;
        const uint q = kCandidates[i];
        holdPin(q, true);
        bool any = false;

        for(int w = 0; w < 4 && !any; w++) {
            const uint pwm = kExpected[w].pwm;
            if(!isCandidate(pwm) || pwm == q) continue;
            spinAndSense(pwm, duty, edges);
            for(size_t j = 0; j < kNumCand; j++) {
                if(edges[j] >= Cfg::EDGE_MIN) {
                    any = true;
                    break;
                }
            }
        }
        holdPin(q, false);
        pinToInput(q);
        gpio_pull_down(q);

        if(any) {
            Serial.printf("      → ★ %s 拉高時馬達才會轉：這就是致能腳（EN/SLEEP/STBY）\n",
                          pinName(q));
            g_enPin = static_cast<int8_t>(q);
            holdPin(q, true); // 後續全程保持拉高
            return g_enPin;
        }
        Serial.printf("      %s：無反應\n", pinName(q));
    }
    Serial.println(F("[1b] 仍未找到致能腳 → 可能馬達沒供電或驅動板故障"));
    return -1;
}

// ============================================================================
// 2) 找 DIR 腳 + 極性（逐一候選腳拉 HIGH，看有號計數是否反向）
// ============================================================================
// 把某輪以外的候選腳設成 baseHigh（true=HIGH / false=LOW）；編碼器永遠上拉
// 用「內部上/下拉」設定電位（弱驅動，不會硬尬外部 5V 訊號）
//   回傳：腳位實際讀到的電位是否符合要求（false = 被外部強驅動，拉不動）
static bool weakLevel(uint pin, bool high) {
    gpio_set_function(pin, GPIO_FUNC_SIO);
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    if(high)
        gpio_pull_up(pin);
    else
        gpio_pull_down(pin);
    delayMicroseconds(300);
    return (gpio_get(pin) != 0) == high;
}

static void dirBaseline(uint pwm, uint encA, uint encB, bool baseHigh) {
    for(size_t i = 0; i < kNumCand; i++) {
        const uint p = kCandidates[i];
        if(p == pwm || p == encA || p == encB) continue;
        if(g_holdMask & (1u << p)) continue; // 致能腳保持拉高
        pinToInput(p);
        if(baseHigh)
            gpio_pull_up(p);
        else
            gpio_pull_down(p);
    }
    // ⚠ 絕對不可碰 pwm 腳：它已經是 PWM function，改成輸入等於把馬達關掉
    //   （舊版這裡寫 pinToInput(pwm) 導致 DIR 搜尋時馬達完全不轉）
    gpio_pull_up(encA); // 編碼器讀取端一律上拉
    gpio_pull_up(encB);
}

// 找 DIR：讓馬達轉，再把某腳拉成「相反電位」，看有號計數是否反向。
// verbose=true 會逐腳印出計數（probe 指令用），auto 模式只找第一根。
static int8_t searchDir(WheelMap &w, float duty, bool verbose) {
    const uint pwm = static_cast<uint>(w.pwm);
    const uint encA = static_cast<uint>(w.encA);
    const uint encB = static_cast<uint>(w.encB);

    pwmClaim(pwm);
    pwmDuty(pwm, Cfg::SCAN_RAMP_DUTY);
    waitMs(Cfg::RAMP_MS);
    pwmDuty(pwm, duty);

    if(verbose)
        Serial.printf("     PWM 硬體輸出：slice%u chan%u cc=%u（wrap=1000 → 每 1000 約 1.0）\n",
                      static_cast<unsigned>(pwm_gpio_to_slice_num(pwm)),
                      static_cast<unsigned>(pwm_gpio_to_channel(pwm)),
                      static_cast<unsigned>(pwmHardwareLevel(pwm)));

    // 基準：先試「其他腳全 HIGH」（掃描時就是這個狀態，實際已知推得動），
    //       不行才退回「其他腳全 LOW」。
    struct BaseCase { bool high; const char *label; };
    const BaseCase cases[2] = {{true, "HIGH"}, {false, "LOW "}};
    Measure mb;
    int32_t base = 0;
    bool baseHigh = true;

    for(int c = 0; c < 2; c++) {
        dirBaseline(pwm, encA, encB, cases[c].high);
        mb = measureAll(encA, encB, Cfg::DIR_SAMPLE_MS);
        base = (mb.quad != 0 || mb.order == 0) ? mb.quad : mb.order;
        baseHigh = cases[c].high;
        if(verbose)
            Serial.printf("     基準(其他腳%s): quad=%+ld order=%+ld 邊緣%u/%u 跳變%ld%s\n",
                          cases[c].label, static_cast<long>(mb.quad),
                          static_cast<long>(mb.order), static_cast<unsigned>(mb.ea),
                          static_cast<unsigned>(mb.eb), static_cast<long>(mb.busy),
                          abs(base) >= Cfg::SIGN_MIN ? "  ← 用這個當基準" : "");
        if(abs(base) >= Cfg::SIGN_MIN) break;
    }

    w.baseSign = base;
    w.baseWasHigh = baseHigh;

    if(abs(base) < Cfg::SIGN_MIN) {
        pwmDuty(pwm, 0.0f);
        releasePin(pwm);
        if(verbose) {
            Serial.printf("     基準轉動量不足（quad=%+ld order=%+ld 跳變=%ld）\n",
                          static_cast<long>(mb.quad), static_cast<long>(mb.order),
                          static_cast<long>(mb.busy));
            if(mb.ea + mb.eb > 20)
                Serial.println(F("     → 編碼器有訊號但計數為 0：A/B 可能同相或接線異常"));
            else
                Serial.println(F("     → 編碼器沒訊號：這兩次基準下馬達都沒轉（DIR/EN 狀態問題）"));
        }
        return -1;
    }

    if(verbose)
        Serial.printf("     基準：其他腳全 %s，計數 %+ld\n", baseHigh ? "HIGH" : "LOW",
                      static_cast<long>(base));

    int8_t found = -1;
    int8_t swapCand = -1; // 「轉速大跳」的腳：它才是真 PWM，原本那隻其實是 DIR
    for(size_t i = 0; i < kNumCand; i++) {
        if(g_abort) break;
        const uint q = kCandidates[i];
        if(q == pwm || q == encA || q == encB) continue;
        if(g_holdMask & (1u << q)) continue;
        if(!verbose) {
            if(g_claimed[q] == 1) continue; // 別輪的 PWM
            if(g_claimed[q] == 2) continue; // 編碼器
            if(g_claimed[q] == 3) continue; // 別輪的 DIR
        }

        // 把 q 拉成「與基準相反」的電位（弱驅動：50k 上/下拉）
        const bool stuck = !weakLevel(q, !baseHigh) && verbose;
        waitMs(60);
        const Measure mq = measureAll(encA, encB, Cfg::DIR_SAMPLE_MS);
        const int32_t s = (mq.quad != 0 || mq.order == 0) ? mq.quad : mq.order;
        pinToInput(q);
        if(baseHigh)
            gpio_pull_up(q);
        else
            gpio_pull_down(q);

        const bool moved = abs(s) >= Cfg::SIGN_MIN;
        const bool reversed = moved && ((base > 0 && s < 0) || (base < 0 && s > 0));
        // 同向但轉速大跳（>3 倍）→ 這腳是「PWM 輸入」被我們當成常 HIGH 灌進去，
        //   也就是原本認定的 PWM/DIR 角色其實互換了。
        const bool muchFaster = moved && !reversed && abs(s) * 2 > abs(base) * 5;
        if(muchFaster && swapCand < 0) swapCand = static_cast<int8_t>(q);

        if(verbose) {
            char tag[32] = "";
            if(g_claimed[q] == 1)
                snprintf(tag, sizeof(tag), " [別輪PWM]");
            else if(g_claimed[q] == 2)
                snprintf(tag, sizeof(tag), " [編碼器]");
            else if(g_claimed[q] == 3)
                snprintf(tag, sizeof(tag), " [別輪DIR]");
            Serial.printf("     %s 拉成 %s → %+6ld（q%+ld o%+ld e%u/%u） %s%s%s\n", pinName(q),
                          baseHigh ? "LOW " : "HIGH", static_cast<long>(s),
                          static_cast<long>(mq.quad), static_cast<long>(mq.order),
                          static_cast<unsigned>(mq.ea), static_cast<unsigned>(mq.eb),
                          !moved ? "沒動"
                                 : (reversed ? "★ 反向 ← DIR"
                                             : (muchFaster ? "★ 轉速大跳 ← 這才是 PWM" : "同向")),
                          tag, stuck ? " [拉不動/外部強驅動]" : "");
        }

        if(reversed && found < 0) {
            found = static_cast<int8_t>(q);
            w.signLow = base;
            w.signHigh = s;
            if(!verbose) break; // auto 模式找到就停
        }
    }

    pwmDuty(pwm, 0.0f);
    releasePin(pwm);

    if(found < 0 && swapCand >= 0) {
        // 角色互換：swapCand 才是 PWM，原本的 pwm 其實是 DIR
        const uint8_t oldPwm = w.pwm;
        Serial.printf("     ↻ 角色互換：%s 才是 PWM，%s 其實是 DIR\n", pinName(swapCand),
                      pinName(oldPwm));
        g_claimed[oldPwm] = 3;
        g_claimed[swapCand] = 1;
        w.pwm = swapCand;
        w.dir = static_cast<int8_t>(oldPwm);
        w.baseWasHigh = baseHigh;
        w.dirIsExpected = (w.expected >= 0 &&
                           kExpected[w.expected].dir == static_cast<uint>(oldPwm));
        if(g_holdMask & (1u << oldPwm)) g_holdMask &= ~(1u << oldPwm);
        return w.dir;
    }

    if(found >= 0) {
        w.dir = found;
        w.dirIsExpected = (w.expected >= 0 &&
                           kExpected[w.expected].dir == static_cast<uint>(found));
        g_claimed[found] = 3;
        if(g_holdMask & (1u << found)) g_holdMask &= ~(1u << found);
    }
    return found;
}

// ============================================================================
// 3) 報表 + 可貼回 main.cpp 的區塊
// ============================================================================
static const char *wheelName(const WheelMap &w, char *buf, size_t len) {
    if(w.expected >= 0)
        snprintf(buf, len, "%s", kExpected[w.expected].name);
    else
        snprintf(buf, len, "?#%d", static_cast<int>(&w - g_wheels) + 1);
    return buf;
}

static void printReport() {
    char nm[16];
    Serial.println(F("\n================== 自動對應結果 =================="));
    for(size_t i = 0; i < g_numWheels; i++) {
        const WheelMap &w = g_wheels[i];
        Serial.printf("%-3s PWM=%s  DIR=%s  ENC=%s/%s  (邊緣 %u/%u)\n",
                      wheelName(w, nm, sizeof(nm)),
                      pinName(static_cast<uint>(w.pwm)),
                      w.dir < 0 ? "?(未找到)" : pinName(static_cast<uint>(w.dir)),
                      pinName(static_cast<uint>(w.encA)),
                      pinName(static_cast<uint>(w.encB)),
                      static_cast<unsigned>(w.edgesA),
                      static_cast<unsigned>(w.edgesB));
        if(w.dir >= 0) {
            Serial.printf("      基準（其他腳 %s）%+ld → DIR觸發 %+ld（%s）\n",
                          w.baseWasHigh ? "HIGH" : "LOW ",
                          static_cast<long>(w.signLow), static_cast<long>(w.signHigh),
                          w.dirIsExpected ? "DIR 腳與舊板相同" : "DIR 腳已改變");
        } else {
            Serial.printf("      ⚠ DIR 未測到（基準計數 %+ld）→ 請跑 probe %u %u %u\n",
                          static_cast<long>(w.baseSign),
                          static_cast<unsigned>(w.pwm), static_cast<unsigned>(w.encA),
                          static_cast<unsigned>(w.encB));
        }
    }

    // 缺失 / 重複提示
    for(int i = 0; i < 4; i++) {
        bool used = false;
        for(size_t k = 0; k < g_numWheels; k++)
            if(g_wheels[k].expected == i) used = true;
        if(!used)
            Serial.printf("⚠ 舊板的 %s（PWM=%u）沒有對應到任何轉動 → 該輪可能接錯/沒電/卡住\n",
                          kExpected[i].name, static_cast<unsigned>(kExpected[i].pwm));
    }

    if(g_enPin >= 0)
        Serial.printf("⚠ 致能腳：%s 必須拉高驅動板才會出力（正式韌體要自行處理）\n",
                      pinName(static_cast<uint>(g_enPin)));

    Serial.println(F("\n------------- 可直接貼回 src/move/main.cpp -------------"));
    for(int i = 0; i < 4; i++) {
        const WheelMap *w = nullptr;
        for(size_t k = 0; k < g_numWheels; k++)
            if(g_wheels[k].expected == i) w = &g_wheels[k];

        const uint pwm = w ? static_cast<uint>(w->pwm) : kExpected[i].pwm;
        const bool dirOk = (w && w->dir >= 0);
        const uint dir = dirOk ? static_cast<uint>(w->dir) : kExpected[i].dir;
        // PIO 一次讀 2 個「相鄰」腳，所以 base 一律取較小腳
        const uint e1 = w ? static_cast<uint>(w->encA) : kExpected[i].encA;
        const uint e2 = w ? static_cast<uint>(w->encB) : static_cast<uint>(kExpected[i].encA + 1);
        const uint enc = e1 < e2 ? e1 : e2;

        Serial.printf("constexpr uint MOTOR_%s_PWM = %u;\n", kExpected[i].name, pwm);
        if(dirOk)
            Serial.printf("constexpr uint MOTOR_%s_DIR = %u;\n", kExpected[i].name, dir);
        else
            Serial.printf("constexpr uint MOTOR_%s_DIR = %u;   // ⚠ 未測到，這是舊板值，請用 probe 確認！\n",
                          kExpected[i].name, dir);
        Serial.printf("constexpr uint MOTOR_%s_ENC = %u;   // A=%u B=%u\n", kExpected[i].name,
                      enc, e1, e2);
        if(!w) {
            Serial.printf("// ↑ %s 未測到轉動，以上保留舊板預設值\n", kExpected[i].name);
        } else if(e2 - e1 == 1 || e1 - e2 == 1) {
            if(e1 > e2)
                Serial.printf("// ↑ %s：A/B 與舊板相反（A=%u,B=%u）→ kEncReadPolarity 要取反\n",
                              kExpected[i].name, e1, e2);
        } else {
            Serial.printf("// ⚠ %s 的 A=%u / B=%u 不相鄰！quad_phase.pio 的 `in pins,2` 只能讀相鄰腳，"
                          "需改寫 encoderInit\n",
                          kExpected[i].name, e1, e2);
        }
    }
    // 完全沒對上舊板輪別的輪子
    for(size_t k = 0; k < g_numWheels; k++) {
        const WheelMap &w = g_wheels[k];
        if(w.expected >= 0) continue;
        Serial.printf("// 未命名輪（請看哪顆輪子在轉）：PWM=%u DIR=%d ENC=%d/%d\n",
                      static_cast<unsigned>(w.pwm), static_cast<int>(w.dir),
                      static_cast<int>(w.encA), static_cast<int>(w.encB));
    }
    Serial.println(F("-------------------------------------------------------"));
    if(g_numWheels == 0) {
        Serial.println(F("結果為空，請檢查："));
        Serial.println(F("  1) 馬達電源（VM）是否有電？驅動板 VCC/EN 是否要拉高？"));
        Serial.println(F("  2) 用 dump 看腳位普查，把結果貼給我"));
        Serial.println(F("  3) 用 dir <pin> 1 逐一拉高猜致能腳，再 run 一次"));
    } else {
        Serial.println(F("方向極性：DIR=0/1 的計數正負如上（+ 代表 A 領先 B）。"));
        Serial.println(F("四輪符號若不一致，請對照 kWheelPolarity / kEncReadPolarity 調整。"));
    }
    Serial.println(F("========================================================="));
}

// ============================================================================
// 全自動主流程
// ============================================================================
static void runAutoMap(float duty) {
    if(duty <= 0.0f) duty = Cfg::SCAN_DUTY;
    if(duty > 70.0f) duty = 70.0f;
    g_abort = false;
    g_numWheels = 0;

    Serial.println(F("\n################ 自動腳位對應開始 ################"));
    censusDump();
    if(g_abort) {
        Serial.println(F("[中止] 已停止，腳位全部釋放"));
        releaseAll();
        return;
    }

    discoverMotors(duty, false, true);
    if(g_abort) {
        releaseAll();
        Serial.println(F("[中止] 已停止，腳位全部釋放"));
        return;
    }

    // 若幾乎沒找到，先找致能腳再重掃一次（很多新板需要 EN 拉高）
    if(g_numWheels < 2) {
        if(huntEnable(duty) >= 0) {
            Serial.println(F("[1b] 致能腳已拉高，重新掃描…"));
            discoverMotors(duty, false, true);
        }
    }

    // 還是不足 → 把掃描範圍擴大到 GP14 以上（此時可能推到編碼器輸出，風險自負）
    if(g_numWheels < 4) {
        Serial.println(F("\n[1c] 馬達側只找到部分 → 擴大掃描 GP14 以上"));
        Serial.println(F("     ⚠ 此區多半是編碼器輸入，若某腳正被編碼器驅動，硬推可能傷到編碼器"));
        Serial.println(F("     按 Enter 可跳過這一步；否則 2 秒後繼續"));
        waitMs(2000);
        if(!g_abort) discoverMotors(duty, true, false);
    }
    if(g_abort) {
        releaseAll();
        Serial.println(F("[中止] 已停止，腳位全部釋放"));
        return;
    }

    Serial.printf("\n[2/3] 找 DIR 腳（每輪逐一候選腳拉 HIGH，看轉向是否反轉）\n");
    for(size_t i = 0; i < g_numWheels; i++) {
        if(g_abort) break;
        char nm[16];
        Serial.printf("  %s（PWM=%s）搜尋 DIR…\n", wheelName(g_wheels[i], nm, sizeof(nm)),
                      pinName(static_cast<uint>(g_wheels[i].pwm)));
        const int8_t d = searchDir(g_wheels[i], duty, false);
        if(d >= 0)
            Serial.printf("    → DIR = %s\n", pinName(static_cast<uint>(d)));
        else
            Serial.println(F("    → 找不到會反轉的腳（DIR 腳可能不在此候選表，或該輪卡住）"));
    }

    releaseAll();
    Serial.println(F("\n[3/3] 完成，腳位已全部釋放（輸入 + pull-down）"));
    printReport();
    g_abort = false;
}

// ============================================================================
// 互動指令
// ============================================================================
static void printHelp() {
    Serial.println(F("\n==== move_pinmap v2 ===="));
    Serial.println(F("  run [duty%]        重新跑全自動對應（預設 45%）"));
    Serial.println(F("  dump               腳位原始電位普查"));
    Serial.println(F("  enc                每秒列出各腳邊緣數（手動轉輪）"));
    Serial.println(F("  quad <a> <b>       即時有號正交計數"));
    Serial.println(F("  probe <pwm> <a> <b> [duty]  逐腳弱拉印出轉向計數，找/確認 DIR"));
    Serial.println(F("  roles <a> <b> <encA> <encB> 判斷兩隻腳誰是 PWM、誰是 DIR（duty 掃描）"));
    Serial.println(F("  verify             用目前量到的對應逐輪試轉 1 秒驗證（含 DIR 換向）"));
    Serial.println(F("  spin <pin> <duty>  單腳 PWM 輸出（Enter 停）"));
    Serial.println(F("  dir <pin> <0|1>    單腳固定輸出"));
    Serial.println(F("  rel / pwmoff       全部釋放 / 停止 PWM"));
    Serial.println(F("  pull <up|down>     監看模式的內部上下拉"));
    Serial.println(F("  Enter              中止目前動作"));
    Serial.println(F("========================"));
}

static void cmdEnc() {
    Serial.println(F("\n[監看] 每 1 秒統計各腳跳變次數；手動慢慢轉輪子。Enter 結束"));
    allInputsExcept(0);
    uint16_t edges[kNumCand];
    while(!g_abort) {
        countEdges(g_maskAll, 1000, edges);
        Serial.print(F("  "));
        bool any = false;
        for(size_t i = 0; i < kNumCand; i++) {
            if(edges[i] < 3) continue;
            Serial.printf("%s=%u ", pinName(kCandidates[i]),
                          static_cast<unsigned>(edges[i]));
            any = true;
        }
        if(!any) Serial.print(F("(無跳動)"));
        Serial.println();
    }
    allInputsExcept(0);
    g_abort = false;
}

static void cmdQuad(uint aPin, uint bPin) {
    if(!isCandidate(aPin) || !isCandidate(bPin) || aPin == bPin) {
        Serial.println(F("用法：quad <aPin> <bPin>（需不同且為候選腳）"));
        return;
    }
    pinToInput(aPin);
    gpio_pull_up(aPin);
    pinToInput(bPin);
    gpio_pull_up(bPin);
    Serial.printf("\n[正交] A=%s B=%s（Enter 結束）\n", pinName(aPin), pinName(bPin));
    int32_t count = 0;
    uint32_t last = millis();
    int32_t shown = 0;
    while(!g_abort) {
        pollAbort();
        count += measureSigned(aPin, bPin, 50);
        if(millis() - last >= 150) {
            last = millis();
            if(count != shown) {
                shown = count;
                Serial.printf("  count=%+ld\n", static_cast<long>(count));
            }
        }
    }
    releasePin(aPin);
    releasePin(bPin);
    g_abort = false;
}

// probe <pwm> <encA> <encB> [duty]：逐腳印出「拉成相反電位」的計數，找/確認 DIR
static void cmdProbe(uint pwm, uint encA, uint encB, float duty) {
    if(!isCandidate(pwm) || !isCandidate(encA) || !isCandidate(encB) ||
       pwm == encA || pwm == encB || encA == encB) {
        Serial.println(F("用法：probe <pwm> <encA> <encB> [duty%]"));
        return;
    }
    WheelMap w;
    w.pwm = static_cast<int8_t>(pwm);
    w.encA = static_cast<int8_t>(encA);
    w.encB = static_cast<int8_t>(encB);
    Serial.printf("\n[probe] PWM=%s ENC=%s/%s duty=%d%%\n", pinName(pwm), pinName(encA),
                  pinName(encB), static_cast<int>(duty + 0.5f));

    // 先單純數「邊緣」，確認 A/B 兩相都有訊號（與 signed 計數無關）
    {
        const uint32_t mask = (1u << encA) | (1u << encB);
        uint16_t e2[kNumCand];
        allInputsExcept(1u << pwm);
        sensePinsPullUp(1u << pwm);
        pwmClaim(pwm);
        pwmDuty(pwm, Cfg::SCAN_RAMP_DUTY);
        waitMs(Cfg::RAMP_MS);
        pwmDuty(pwm, duty);
        countEdges(mask, 400, e2);
        pwmDuty(pwm, 0.0f);
        releasePin(pwm);
        Serial.printf("     編碼器邊緣測試：%s=%u / %s=%u 次%s\n", pinName(encA),
                      static_cast<unsigned>(e2[g_gpioToIdx[encA]]), pinName(encB),
                      static_cast<unsigned>(e2[g_gpioToIdx[encB]]),
                      (e2[g_gpioToIdx[encA]] || e2[g_gpioToIdx[encB]]) ? ""
                                                                     : "  ⚠ 完全沒訊號");
    }
    const int8_t d = searchDir(w, duty, true);
    Serial.printf("[probe] 結論：%s\n",
                  d < 0 ? "找不到會反轉的腳（DIR 可能在候選表外，或馬達沒轉）"
                        : pinName(static_cast<uint>(d)));
    if(d >= 0) {
        Serial.printf("[probe] 建議：constexpr uint MOTOR_?_DIR = %u;  "
                      "（DIR 電位 %u 會反轉）\n",
                      static_cast<unsigned>(d), w.baseWasHigh ? 0u : 1u);
    }
    g_abort = false;
}

// verify：用「目前量到的對應」逐輪試轉，DIR 低/高各一次，印編碼器增減量
static void cmdVerify() {
    if(g_numWheels == 0) {
        Serial.println(F("還沒有任何對應資料，請先 run 一次"));
        return;
    }
    Serial.println(F("\n[verify] 逐輪試轉：DIR=LOW 0.7s、DIR=HIGH 0.7s，比較有號計數"));
    for(size_t i = 0; i < g_numWheels; i++) {
        const WheelMap &w = g_wheels[i];
        const uint pwm = static_cast<uint>(w.pwm);
        const uint encA = static_cast<uint>(w.encA);
        const uint encB = static_cast<uint>(w.encB);
        char nm[16];
        Serial.printf("  %s PWM=%s", wheelName(w, nm, sizeof(nm)), pinName(pwm));

        pwmClaim(pwm);
        pwmDuty(pwm, Cfg::SCAN_RAMP_DUTY);
        waitMs(Cfg::RAMP_MS);
        pwmDuty(pwm, Cfg::SCAN_DUTY);

        int32_t d[2];
        for(int lvl = 0; lvl < 2; lvl++) {
            dirBaseline(pwm, encA, encB, false); // 其他腳 LOW
            if(w.dir >= 0) pinStrong(static_cast<uint>(w.dir), lvl == 1);
            waitMs(120);
            d[lvl] = measureSigned(encA, encB, 700);
        }
        if(w.dir >= 0) {
            pinToInput(static_cast<uint>(w.dir));
            gpio_pull_down(static_cast<uint>(w.dir));
        }
        pwmDuty(pwm, 0.0f);
        releasePin(pwm);

        Serial.printf("  DIR低 %+6ld / DIR高 %+6ld", static_cast<long>(d[0]),
                      static_cast<long>(d[1]));
        if(abs(d[0]) < 10 && abs(d[1]) < 10)
            Serial.println(F("  ⚠ 沒轉！PWM 或這輪有問題"));
        else if(w.dir < 0)
            Serial.println(F("  （DIR 未知，兩次同向是正常的）"));
        else if((d[0] > 0 && d[1] < 0) || (d[0] < 0 && d[1] > 0))
            Serial.println(F("  ✓ DIR 換向正常、編碼器正常"));
        else
            Serial.println(F("  ⚠ DIR 沒換向（這腳不是這輪的 DIR？）"));
    }
    releaseAll();
    g_abort = false;
}

// roles <pinA> <pinB> <encA> <encB>
// 決定「這兩隻腳哪一隻是 PWM、哪一隻是 DIR」：
//   把 X 當 PWM 跑 20% / 60%（另一隻用弱下拉/弱上拉），看轉速是否隨 duty 成正比。
//   轉速隨 duty 變 → X 是 PWM；另一隻換電位會讓方向反轉 → 另一隻是 DIR。
// 一次 roles 測試：其他腳設成 othersHigh → 設 otherPin → PWM 起轉 → 穩定後量 measureMs
//   時間刻意拉長（起轉 400ms + 穩定 600ms + 量 1200ms），因為短時間馬達常常還沒轉起來。
static int32_t rolesRun(uint pwmPin, uint otherPin, bool otherHigh, bool othersHigh,
                        uint encA, uint encB, float duty, uint32_t measureMs) {
    for(size_t i = 0; i < kNumCand; i++) {
        const uint p = kCandidates[i];
        if(p == pwmPin || p == otherPin) continue;
        weakLevel(p, othersHigh);
    }
    // 只對「本次要測的那一隻腳」用真 3.3V 硬推（driver 輸入端，安全）
    pinStrong(otherPin, otherHigh);
    pwmClaim(pwmPin);
    pwmDuty(pwmPin, Cfg::SCAN_RAMP_DUTY);
    waitMs(400);
    pwmDuty(pwmPin, duty);
    waitMs(600);
    const int32_t v = measureSigned(encA, encB, measureMs);
    pwmDuty(pwmPin, 0.0f);
    releasePin(pwmPin);
    pinToInput(otherPin);
    gpio_pull_down(otherPin);
    return v;
}

// roles <pinA> <pinB> <encA> <encB> [duty%]
// 兩隻腳輪流當 PWM、另一隻輪流 LOW/HIGH，並各試「其他腳 LOW / HIGH」兩種背景。
// 判斷：速度會隨狀態改變方向 → 那隻是 DIR；哪隻腳當 PWM 才會轉 → 那隻是 PWM。
static void cmdRoles(uint a, uint b, uint encA, uint encB, float duty) {
    if(!isCandidate(a) || !isCandidate(b) || !isCandidate(encA) || !isCandidate(encB) ||
       a == b || encA == encB) {
        Serial.println(F("用法：roles <pinA> <pinB> <encA> <encB> [duty%]"));
        return;
    }
    if(duty < 10.0f) duty = 10.0f;
    if(duty > 70.0f) duty = 70.0f;

    Serial.printf("\n[roles] %s / %s 配對，編碼器 %s/%s，duty=%d%%"
                  "（每組 起轉400ms+穩定600ms+量1200ms）\n",
                  pinName(a), pinName(b), pinName(encA), pinName(encB),
                  static_cast<int>(duty + 0.5f));

    int32_t v[2][2][2]; // [哪隻當PWM][其他腳HIGH][other HIGH]
    for(int others = 0; others < 2; others++) {
        for(int which = 0; which < 2; which++) {
            const uint pwmPin = which ? b : a;
            const uint other = which ? a : b;
            const int32_t lo = rolesRun(pwmPin, other, false, others == 1, encA, encB, duty,
                                        Cfg::DIR_SAMPLE_MS * 3);
            const int32_t hi = rolesRun(pwmPin, other, true, others == 1, encA, encB, duty,
                                        Cfg::DIR_SAMPLE_MS * 3);
            v[which][others][0] = lo;
            v[which][others][1] = hi;
            Serial.printf("     其他腳=%s  %s當PWM → %s=LOW:%+8ld  %s=HIGH:%+8ld  %s\n",
                          others ? "HIGH" : "LOW ", pinName(pwmPin), pinName(other),
                          static_cast<long>(lo), pinName(other), static_cast<long>(hi),
                          (abs(lo) >= Cfg::SIGN_MIN * 3 && abs(hi) >= Cfg::SIGN_MIN * 3 &&
                           ((lo > 0 && hi < 0) || (lo < 0 && hi > 0)))
                              ? "★ 反向 → 這隻是 DIR"
                              : "");
        }
    }

    // 判斷
    auto flip = [&](int which) {
        bool any = false;
        for(int others = 0; others < 2; others++) {
            const int32_t x = v[which][others][0], y = v[which][others][1];
            if(abs(x) >= Cfg::SIGN_MIN * 3 && abs(y) >= Cfg::SIGN_MIN * 3 &&
               ((x > 0 && y < 0) || (x < 0 && y > 0)))
                any = true;
        }
        return any;
    };
    auto moves = [&](int which) {
        int32_t best = 0;
        for(int others = 0; others < 2; others++)
            for(int lvl = 0; lvl < 2; lvl++)
                if(abs(v[which][others][lvl]) > abs(best)) best = v[which][others][lvl];
        return best;
    };
    const int32_t moveA = moves(0), moveB = moves(1); // a當PWM / b當PWM 的最大計數
    const bool aIsPwm = abs(moveA) >= abs(moveB);
    const bool aFlips = flip(0), bFlips = flip(1);

    Serial.printf("[roles] 最大計數：%s當PWM=%+ld、%s當PWM=%+ld\n", pinName(a),
                  static_cast<long>(moveA), pinName(b), static_cast<long>(moveB));
    Serial.printf("[roles] 反向：%s=%s、%s=%s\n", pinName(a), aFlips ? "是" : "否",
                  pinName(b), bFlips ? "是" : "否");

    if(abs(moveA) < Cfg::SIGN_MIN && abs(moveB) < Cfg::SIGN_MIN) {
        Serial.println(F("[roles] 兩隻腳都不會轉 → 這對不是這輪的馬達腳（或馬達沒電）"));
    } else if(aIsPwm) {
        Serial.printf("[roles] → PWM = %s，DIR = %s\n", pinName(a),
                      (bFlips || !aFlips) ? pinName(b) : "?（換電位沒反轉）");
    } else {
        Serial.printf("[roles] → PWM = %s，DIR = %s\n", pinName(b),
                      (aFlips || !bFlips) ? pinName(a) : "?（換電位沒反轉）");
    }
    releaseAll();
    g_abort = false;
}

static void cmdSpin(uint pin, float duty) {
    if(!isCandidate(pin)) {
        Serial.println(F("該腳不在候選表內"));
        return;
    }
    pwmClaim(pin);
    pwmDuty(pin, duty);
    Serial.printf("[spin] %s duty=%d%%；Enter 停止\n", pinName(pin),
                  static_cast<int>(duty + 0.5f));
    while(!g_abort) waitMs(100);
    pwmDuty(pin, 0.0f);
    releasePin(pin);
    g_abort = false;
}

static uint parsePin(const char *s, bool *ok) {
    if(!s) {
        *ok = false;
        return 0;
    }
    char *end = nullptr;
    const unsigned long v = strtoul(s, &end, 10);
    *ok = (end != s && *end == '\0' && v < 32);
    return static_cast<uint>(v);
}

static void handleLine(char *line) {
    char *cmd = strtok(line, " ");
    if(!cmd) return;
    char *a1 = strtok(nullptr, " ");
    char *a2 = strtok(nullptr, " ");
    bool ok = false;

    if(!strcmp(cmd, "run") || !strcmp(cmd, "scan")) {
        runAutoMap(a1 ? strtof(a1, nullptr) : Cfg::SCAN_DUTY);
    } else if(!strcmp(cmd, "dump")) {
        censusDump();
    } else if(!strcmp(cmd, "enc")) {
        cmdEnc();
    } else if(!strcmp(cmd, "quad")) {
        const uint a = parsePin(a1, &ok);
        const uint b = parsePin(a2, &ok);
        if(!ok)
            Serial.println(F("用法：quad <aPin> <bPin>"));
        else
            cmdQuad(a, b);
    } else if(!strcmp(cmd, "roles")) {
        char *a3 = strtok(nullptr, " ");
        char *a4 = strtok(nullptr, " ");
        bool ok2 = false, ok3 = false, ok4 = false;
        const uint pa = parsePin(a1, &ok);
        const uint pb = parsePin(a2, &ok2);
        const uint ea = parsePin(a3, &ok3);
        const uint eb = parsePin(a4, &ok4);
        char *a5 = strtok(nullptr, " ");
        if(!ok || !ok2 || !ok3 || !ok4)
            Serial.println(F("用法：roles <pinA> <pinB> <encA> <encB> [duty%]"));
        else
            cmdRoles(pa, pb, ea, eb, a5 ? strtof(a5, nullptr) : Cfg::SCAN_DUTY);
    } else if(!strcmp(cmd, "verify")) {
        cmdVerify();
    } else if(!strcmp(cmd, "probe")) {
        char *a3 = strtok(nullptr, " ");
        char *a4 = strtok(nullptr, " ");
        bool ok2 = false, ok3 = false;
        const uint pwm = parsePin(a1, &ok);
        const uint ea = parsePin(a2, &ok2);
        const uint eb = parsePin(a3, &ok3);
        if(!ok || !ok2 || !ok3)
            Serial.println(F("用法：probe <pwm> <encA> <encB> [duty%]"));
        else
            cmdProbe(pwm, ea, eb, a4 ? strtof(a4, nullptr) : Cfg::SCAN_DUTY);
    } else if(!strcmp(cmd, "spin")) {
        const uint pin = parsePin(a1, &ok);
        if(!ok)
            Serial.println(F("用法：spin <pin> <duty 0~100>"));
        else
            cmdSpin(pin, a2 ? strtof(a2, nullptr) : 30.0f);
    } else if(!strcmp(cmd, "dir")) {
        const uint pin = parsePin(a1, &ok);
        if(!ok || !a2) {
            Serial.println(F("用法：dir <pin> <0|1>"));
        } else {
            pinStrong(pin, a2[0] == '1');
            Serial.printf("[dir] %s = %s\n", pinName(pin), a2[0] == '1' ? "HIGH" : "LOW");
        }
    } else if(!strcmp(cmd, "pwmoff")) {
        releaseAll();
        Serial.println(F("[pwmoff] 全部 PWM 已停止"));
    } else if(!strcmp(cmd, "rel")) {
        releaseAll();
        Serial.println(F("[rel] 全部腳位已釋放（輸入 + pull-down）"));
    } else if(!strcmp(cmd, "pull")) {
        if(a1 && (!strcmp(a1, "up") || !strcmp(a1, "down"))) {
            g_pullUp = (a1[0] == 'u');
            Serial.printf("[pull] 監看模式改用 %s\n", g_pullUp ? "pull-up" : "pull-down");
        } else {
            Serial.println(F("用法：pull <up|down>"));
        }
    } else if(!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
        printHelp();
    } else {
        Serial.printf("未知命令：%s（help 看說明）\n", cmd);
    }
}

// ============================================================================
// setup / loop
// ============================================================================
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    const uint32_t t0 = millis();
    while(!Serial && millis() - t0 < 8000) delay(10);

    for(int i = 0; i < 32; i++) g_gpioToIdx[i] = -1;
    g_maskAll = 0;
    for(size_t i = 0; i < kNumCand; i++) {
        g_gpioToIdx[kCandidates[i]] = static_cast<int8_t>(i);
        g_maskAll |= 1u << kCandidates[i];
        pinToInput(kCandidates[i]);
        gpio_pull_down(kCandidates[i]);
    }

    Serial.println();
    Serial.println(F("=================================================="));
    Serial.println(F("  move_pinmap v2 — Move 板腳位自動對應"));
    Serial.println(F("  3 秒後自動開始（按 o 可改手動，按 Enter 中止）"));
    Serial.println(F("=================================================="));
    printHelp();

    // 3 秒倒數：可按任意鍵取消自動，改為手動
    bool autoRun = true;
    const uint32_t tWait = millis();
    while(millis() - tWait < 3000) {
        if(Serial.available()) {
            Serial.read();
            autoRun = false;
            break;
        }
        digitalWrite(LED_BUILTIN, ((millis() - tWait) / 150) & 1);
        delay(10);
    }
    digitalWrite(LED_BUILTIN, HIGH);

    if(autoRun) {
        drainSerial();
        runAutoMap(Cfg::SCAN_DUTY);
        Serial.println(F("\n完成。可輸入 run 重跑、dump 看腳位、help 看指令。"));
        drainSerial();
    } else {
        Serial.println(F("\n已取消自動模式，請輸入指令（help）。"));
    }
    g_started = true;
}

void loop() {
    static uint32_t lastBeat = 0;
    if(millis() - lastBeat > 500) {
        lastBeat = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    char line[96];
    if(pollLine(line, sizeof(line))) {
        g_abort = false;
        handleLine(line);
        g_abort = false;
    }
    delay(2);
    (void)g_started;
}
