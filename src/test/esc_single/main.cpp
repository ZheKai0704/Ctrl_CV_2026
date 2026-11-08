// ============================================================================
// esc_single — 單顆無刷馬達 (航模單向油門 ESC) 測試 (RP2350)
//
// 用途：
//   用 RP2350 硬體 PWM 驅動「一顆」航模單向油門電調，ESC 訊號接 GP13。
//   透過 USB Serial (115200) 設定出力 %，方便單獨驗證馬達轉向 / 推力。
//
// 訊號規格（標準航模 ESC）：
//   - 50Hz（週期 20ms），脈寬 1000~2000µs
//       0%   = 1000µs（熄火 / 最低點）
//       100% = 2000µs（全速）
//   - 上電必須先送 0%（1000µs）讓 ESC arming，本程式開機自動維持 3 秒。
//   - 單向電調無法用訊號反轉；要反轉請對調三相線任兩條。
//
// 接線：
//   RP2350 GP13 ──→ ESC 訊號線 (白/橘)
//   GND ─────────→ ESC 訊號地 (黑/棕)
//   ESC 電源接電池，馬達三相線接馬達。
//   ⚠️ 第一次測試請拆槳 / 固定馬達，並從 5% 起步確認方向。
//
// Serial 命令（115200，打完按 Enter；字首大小寫不拘）：
//   <0-100>     設定出力 %（走斜坡，不會瞬間跳）
//   0           緩降回 0%
//   X           立即斷油（真急停，唯一不經斜坡）
//   + / - [n]   微調（預設 5%，如 +10 / -3）
//   ACC <n>     加速率 %/s（1-400，預設 40）
//   DEC <n>     減速率 %/s（1-400，預設 60）
//   CAL         油門端點校準（務必拆槳，依畫面指示斷電重上）
//   R           讀目前狀態
//   h 或 ?      說明
// ============================================================================
#include <Arduino.h>
#include <hardware/clocks.h>
#include <hardware/pwm.h>
#include <cstring>
#include <pico/stdlib.h>

// ============================================================================
// 腳位 / PWM 參數
// ============================================================================
namespace Pin {
constexpr uint ESC_PWM = 14; // GP13 → ESC 訊號線
} // namespace Pin

namespace Esc {
constexpr uint32_t TICK_HZ = 1000000; // 1 tick = 1µs
constexpr uint16_t PWM_WRAP = 19999;  // 20000 ticks = 20ms → 50Hz
constexpr uint16_t PULSE_MIN = 1000;  // 0%
constexpr uint16_t PULSE_MAX = 2000;  // 100%
constexpr int16_t MAX_PCT = 100;

// 出力以 0.001% 為單位，避免低速率被整數截斷
constexpr int32_t OUT_SCALE = 1000;
constexpr int32_t OUT_MAX = MAX_PCT * OUT_SCALE;
constexpr int16_t RAMP_UP_PCT_S_DEF = 40;
constexpr int16_t RAMP_DOWN_PCT_S_DEF = 60;
constexpr int16_t RAMP_MIN_PCT_S = 1;
constexpr int16_t RAMP_MAX_PCT_S = 400;
constexpr uint32_t RAMP_TICK_US = 2000; // slew 更新週期 2ms
constexpr uint32_t ARM_WAIT_MS = 3000;  // 上電後維持 0% 等 ESC arming
} // namespace Esc

static int32_t s_out = 0; // 目前實際出力（0.001%）
static int16_t s_cmd = 0; // 目標出力 %
static int16_t s_rampUp = Esc::RAMP_UP_PCT_S_DEF;
static int16_t s_rampDown = Esc::RAMP_DOWN_PCT_S_DEF;

// ============================================================================
// PWM 底層
// ============================================================================
static void setupPwm(uint gpio) {
    const uint slice = pwm_gpio_to_slice_num(gpio);
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    // 依實際系統時脈算出 1MHz 的除頻，RP2040/RP2350 都能用
    const float div = static_cast<float>(clock_get_hz(clk_sys)) /
                      static_cast<float>(Esc::TICK_HZ);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, div);
    pwm_config_set_wrap(&cfg, Esc::PWM_WRAP);
    pwm_init(slice, &cfg, true);
}

static void setPulse(uint16_t us) {
    if(us < Esc::PULSE_MIN)
        us = Esc::PULSE_MIN;
    if(us > Esc::PULSE_MAX)
        us = Esc::PULSE_MAX;
    pwm_set_gpio_level(Pin::ESC_PWM, us); // level = 高電位 tick 數 = µs
}

static uint16_t outToPulse(int32_t out) {
    if(out < 0)
        out = 0;
    if(out > Esc::OUT_MAX)
        out = Esc::OUT_MAX;
    const int32_t span = Esc::PULSE_MAX - Esc::PULSE_MIN;
    return static_cast<uint16_t>(Esc::PULSE_MIN + out * span / Esc::OUT_MAX);
}

static int16_t outToPct(int32_t out) {
    if(out < 0)
        out = 0;
    if(out > Esc::OUT_MAX)
        out = Esc::OUT_MAX;
    return static_cast<int16_t>((out + Esc::OUT_SCALE / 2) / Esc::OUT_SCALE);
}

// 直接寫入（跳過斜坡）—— 只用於急停 X / 校準 / 開機 arming
static void hardSet(int16_t pct) {
    if(pct < 0)
        pct = 0;
    if(pct > Esc::MAX_PCT)
        pct = Esc::MAX_PCT;
    s_out = static_cast<int32_t>(pct) * Esc::OUT_SCALE;
    setPulse(outToPulse(s_out));
}

// 朝目標以 ACC/DEC 速率前進 dtUs
static void slewToward(int16_t targetPct, uint32_t dtUs) {
    int32_t target = static_cast<int32_t>(targetPct) * Esc::OUT_SCALE;
    if(target < 0)
        target = 0;
    if(target > Esc::OUT_MAX)
        target = Esc::OUT_MAX;

    const int32_t diff = target - s_out;
    if(diff == 0) {
        setPulse(outToPulse(s_out));
        return;
    }

    const int16_t ratePct = (diff > 0) ? s_rampUp : s_rampDown;
    const int32_t rate = static_cast<int32_t>(ratePct) * Esc::OUT_SCALE; // 0.001%/s
    int64_t step = static_cast<int64_t>(rate) * static_cast<int64_t>(dtUs) / 1000000;
    if(step < 1)
        step = 1; // 速率再低也要有進展

    if(diff > 0)
        s_out += (diff > step) ? static_cast<int32_t>(step) : diff;
    else
        s_out -= (-diff > step) ? static_cast<int32_t>(step) : -diff;

    if(s_out < 0)
        s_out = 0;
    if(s_out > Esc::OUT_MAX)
        s_out = Esc::OUT_MAX;
    setPulse(outToPulse(s_out));
}

// ============================================================================
// Serial 輸入（Enter 才送出）
// ============================================================================
static char s_line[16];
static uint8_t s_lineLen = 0;

static bool pollLine() {
    while(Serial.available() > 0) {
        const char ch = static_cast<char>(Serial.read());
        if(ch == '\n' || ch == '\r') {
            Serial.println();
            while(s_lineLen > 0 && (s_line[s_lineLen - 1] == ' ' ||
                                    s_line[s_lineLen - 1] == '\t'))
                s_lineLen--;
            if(s_lineLen == 0)
                continue;
            s_line[s_lineLen] = '\0';
            return true;
        }
        if(ch == 8 || ch == 127) { // Backspace
            if(s_lineLen > 0) {
                s_line[--s_lineLen] = '\0';
                Serial.print("\b \b");
            }
            continue;
        }
        if(s_lineLen < sizeof(s_line) - 1) {
            s_line[s_lineLen++] = ch;
            s_line[s_lineLen] = '\0';
            Serial.write(ch);
        }
    }
    return false;
}

static bool parseNum(const char *str, int16_t &val) {
    while(*str == ' ' || *str == '\t')
        str++;
    if(*str == '-')
        return false; // 單向油門不接受負值
    if(*str == '+')
        str++;
    if(*str == '\0')
        return false;
    long v = 0;
    while(*str >= '0' && *str <= '9') {
        v = v * 10 + (*str - '0');
        if(v > 32767)
            v = 32767;
        str++;
    }
    if(*str != '\0')
        return false;
    val = static_cast<int16_t>(v);
    return true;
}

// ============================================================================
// 命令
// ============================================================================
static void printStatus() {
    Serial.printf("   現在 %d%% (~%uµs)   目標 %d%%   ACC %d%%/s  DEC %d%%/s\r\n",
                  static_cast<int>(outToPct(s_out)),
                  static_cast<unsigned>(outToPulse(s_out)), static_cast<int>(s_cmd),
                  static_cast<int>(s_rampUp), static_cast<int>(s_rampDown));
}

static void printHelp() {
    Serial.println();
    Serial.println("=== 單顆無刷馬達 (單向油門 ESC) 測試 ===");
    Serial.println("   訊號: GP13, 50Hz, 1000µs=0%  2000µs=100%");
    Serial.println();
    Serial.println("  <0-100>   設定出力 %（斜坡趨近）");
    Serial.println("  0         緩降回 0%（熄火）");
    Serial.println("  X         立即斷油（真急停）");
    Serial.println("  + / - [n] 微調（預設 5%，如 +10 / -3）");
    Serial.println("  ACC <n>   加速率 %/s (1-400)");
    Serial.println("  DEC <n>   減速率 %/s (1-400)");
    Serial.println("  CAL       油門端點校準（先拆槳！）");
    Serial.println("  R         讀狀態");
    Serial.println("  h 或 ?    此說明");
    Serial.println();
    Serial.println("※ 上電後維持 0% 待 ESC arming。請由 5% 起步確認方向；");
    Serial.println("  單向無法用訊號反轉 → 對調三相線任兩條。");
    Serial.println();
}

// 油門端點校準：全油門 2000µs → 使用者斷電重上 → 回 0% 記低端點
static void runThrottleCal() {
    hardSet(0);
    s_cmd = 0;
    while(Serial.available())
        Serial.read();

    Serial.println();
    Serial.println("== 油門端點校準 (CAL) — 務必先拆槳 ==");
    Serial.println("  現把油門拉到 100% (2000µs)…");
    hardSet(Esc::MAX_PCT);
    Serial.println("  請在 6 秒內『關 ESC 電源 → 再上電』(會發出校準嗶聲)");
    for(int i = 6; i > 0; i--) {
        Serial.printf("   %ds ...\r\n", i);
        delay(1000);
    }
    Serial.println("  保持高油門 2s 記高端點…");
    delay(2000);
    hardSet(0);
    Serial.println("  回 0% (1000µs) 記低端點…");
    delay(1500);
    Serial.println("  校準完成，已回 0% 待命。");
    Serial.println();
}

static void handleCommand(const char *line) {
    Serial.println();

    const char first = line[0];
    if(first == '0') {
        s_cmd = 0;
        Serial.printf("緩降至 0%%（DEC %d %%/s）\r\n", static_cast<int>(s_rampDown));
        return;
    }
    if(first == '+' || first == '-') {
        int16_t delta = 5;
        int16_t parsed = 0;
        if(parseNum(&line[1], parsed) && parsed >= 0 && parsed <= 100)
            delta = parsed;
        int16_t target = static_cast<int16_t>(outToPct(s_out) +
                                              ((first == '+') ? delta : -delta));
        if(target < 0)
            target = 0;
        if(target > Esc::MAX_PCT)
            target = Esc::MAX_PCT;
        s_cmd = target;
        Serial.printf("微調 → %d%%\r\n", static_cast<int>(s_cmd));
        return;
    }

    if(line[0] >= '0' && line[0] <= '9') { // 純數字 → 出力 %
        int16_t pct = 0;
        if(!parseNum(line, pct)) {
            Serial.println("?? 數字無效，鍵入 'h'");
            return;
        }
        if(pct > Esc::MAX_PCT)
            pct = Esc::MAX_PCT;
        s_cmd = pct;
        Serial.printf("目標 %d%%（斜坡趨近中）\r\n", static_cast<int>(s_cmd));
        return;
    }

    // 文字命令（不分大小寫）
    char op[5] = {0};
    const char *p = line;
    unsigned i = 0;
    while(i < 4 && *p != '\0') {
        const char c = *p;
        if(!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')))
            break;
        op[i++] = (c >= 'a' && c <= 'z') ? static_cast<char>(c - 'a' + 'A') : c;
        p++;
    }

    if(strcmp(op, "H") == 0 || strcmp(op, "?") == 0) {
        printHelp();
        return;
    }
    if(strcmp(op, "R") == 0) {
        printStatus();
        return;
    }
    if(strcmp(op, "X") == 0) {
        s_cmd = 0;
        hardSet(0);
        Serial.println("急停：立即 0% (1000µs 熄火)");
        return;
    }
    if(strcmp(op, "CAL") == 0) {
        runThrottleCal();
        return;
    }
    if(strcmp(op, "ACC") == 0 || strcmp(op, "DEC") == 0) {
        int16_t rate = 0;
        if(!parseNum(p, rate) || rate < Esc::RAMP_MIN_PCT_S ||
           rate > Esc::RAMP_MAX_PCT_S) {
            Serial.printf("?? 速率須為 %d-%d %%/s\r\n",
                          static_cast<int>(Esc::RAMP_MIN_PCT_S),
                          static_cast<int>(Esc::RAMP_MAX_PCT_S));
            return;
        }
        if(strcmp(op, "ACC") == 0)
            s_rampUp = rate;
        else
            s_rampDown = rate;
        Serial.printf("ACC %d %%/s   DEC %d %%/s\r\n", static_cast<int>(s_rampUp),
                      static_cast<int>(s_rampDown));
        return;
    }

    Serial.println("?? 未知命令，鍵入 'h' 看用法");
}

// ============================================================================
// setup / loop
// ============================================================================
void setup() {
    setupPwm(Pin::ESC_PWM);
    hardSet(0); // 先確保 0% 才啟動序列，避免 ESC 亂解鎖
    s_cmd = 0;

    Serial.begin(115200);
    delay(150);
    Serial.println();
    Serial.println("[esc_single] 單顆無刷馬達測試 — ESC 訊號 = GP13");

    // ESC arming：維持 0% (1000µs) 數秒，不可在此期間抬油門
    Serial.printf("ESC arming 中，維持 0%% %us …\r\n",
                  static_cast<unsigned>(Esc::ARM_WAIT_MS / 1000));
    delay(Esc::ARM_WAIT_MS);

    printHelp();
    Serial.println("已就緒：0% (1000µs 熄火) 待命。");
}

void loop() {
    static uint32_t lastTickUs = micros();
    const uint32_t nowUs = micros();
    uint32_t dtUs = nowUs - lastTickUs;
    if(dtUs < Esc::RAMP_TICK_US) {
        yield();
        return;
    }
    if(dtUs > 100000)
        dtUs = 100000; // 過長夾住，避免跳級
    lastTickUs = nowUs;

    slewToward(s_cmd, dtUs);

    if(pollLine()) {
        handleCommand(s_line);
        s_lineLen = 0;
        s_line[0] = '\0';
    }
}
