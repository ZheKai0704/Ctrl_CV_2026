// ============================================================================
// esc_test — 單向無刷馬達 (Hobbywing 航模油門電調) 測試控制 (RP2350)
//
// 用途：
//   以 RP2350 硬體 PWM 驅動 2 顆「航模單向油門」電調，2450KV 馬達。
//   透過 USB Serial 設定每顆馬達的「出力 %」(僅順向油門)。適合直連燒機/
//   單獨驗證推力——無伺服控時。
//
// ⚠️ 責任範圍：這類航模電調(Skywalker)只有「油門大小」,沒有反向。
//   - 訊號＝週期 20ms(50Hz)、脈寬 1000~2000µs:
//       0%   = 1000µs(熄火 / 低速煞停點 —— 出力的最低點)
//       100% = 2000µs(全速油門)
//   - 上電需先看到「低電位(→1000µs)」電調才解鎖 arming;中途 0% 是熄火
//   - 「反轉」無法用訊號做:靠三相任兩條對調或換「可反轉」電控。本版單向。
//
// ✓ Pico2(150MHz) 校正：
//   本板 RP2350 兩核跑 150MHz(Earlephilhower 預設,非 RP2040 的 125MHz)。
//   clkdiv = 150 → PWM 切片時脈恰為 1MHz,每 tick = 1µs,wrap=19999 → 50Hz。
//   之前誤用 125(那是 RP2040/Pico1 值),等同切片 1.2MHz、脈寬全被拉長 20%。
//
// 接線（RP2350）：GP18→ESC1(馬達A)  GP19→ESC2(馬達B)；共地；三相線接好。
// RP2350 PWM:clkdiv=150, wrap=19999 → 1MHz 計數、50Hz(150MHz/150=1MHz)。

// Serial 命令（115200, 字首大小寫不拘；輸入後按 Enter 才會執行）：
//   A <0-100>   馬達 A 出力 %        B <0-100>   馬達 B 出力 %
//   ALL <0-100> 兩顆同時出力
//   CAL         油門端點校準(會先拉到全油門→再回 0 存端點;請遠離槳)。
//               ※ 校準前務必先關電,讓電調重新以…見說明印
//   0           兩顆緩降回 0%（熄火/低點）
//   X           兩顆立即斷油（真急停，唯一不經 slew 的命令）
//   ACC <%/s>   設定加速率（緩加速）    DEC <%/s>  設定減速率（緩減速）
//   + / -       兩顆微調（帶量）       R 讀狀態    ?/h 幫助
//
// 加減速：所有出力變化（含 A/B/ALL/0/微調）一律以 ACC/DEC 速率斜坡趨近，
//         不會瞬間跳到目標值，也不會瞬間停止。預設 ACC=40 %/s、DEC=60 %/s。
// ============================================================================
#include <Arduino.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>

// ============================================================================
// 腳位
// ============================================================================
namespace Pin {
constexpr uint MOTOR_A_PWM = 18; // GP18 → ESC1 信號 (馬達 A)
constexpr uint MOTOR_B_PWM = 19; // GP19 → ESC2 信號 (馬達 B)
} // namespace Pin

// ============================================================================
// 油門 / PWM 參數  (Pico2/RP2350 = 150MHz)
// ============================================================================
namespace Esc {
constexpr uint32_t PWM_CLKDIV = 150; // 150MHz/150 = 1MHz → 每 tick = 1µs
constexpr uint16_t PWM_WRAP = 19999; // 20ms → 50Hz
constexpr uint16_t PULSE_MIN = 1000; // 0%（出力的最低/熄火點 → 校準低端點）
constexpr uint16_t PULSE_MAX = 2000; // 100%（全速油門 → 校準高端點）
constexpr int16_t  MAX_PCT  = 100;   // 出力上限 %

// --- 緩加速 / 緩減速 (slew rate) ---
// 內部出力以 0.001% 為單位（OUT_SCALE），避免低速率時被整數截斷而永遠不動。
constexpr int32_t OUT_SCALE = 1000;             // 1% = 1000 單位
constexpr int32_t OUT_MAX   = MAX_PCT * OUT_SCALE;
constexpr int32_t RAMP_UP_PCT_S_DEF   = 40;     // 預設加速率 %/s
constexpr int32_t RAMP_DOWN_PCT_S_DEF = 60;     // 預設減速率 %/s
constexpr int32_t RAMP_MIN_PCT_S      = 1;      // 速率下限 %/s
constexpr int32_t RAMP_MAX_PCT_S      = 400;    // 速率上限 %/s
constexpr uint32_t RAMP_TICK_US       = 2000;   // slew 更新週期 2ms
} // namespace Esc

// 目前目標（未 slew 完成的即時值）+ 實際輸出
// s_outX 單位 = 0.001%（0..100000）
static int32_t s_outA = 0;
static int32_t s_outB = 0;
static int16_t s_cmdA = 0; // 目標 %
static int16_t s_cmdB = 0;

// 加減速率（單位 0.001%/s）
static int32_t s_rampUp   = Esc::RAMP_UP_PCT_S_DEF * Esc::OUT_SCALE;
static int32_t s_rampDown = Esc::RAMP_DOWN_PCT_S_DEF * Esc::OUT_SCALE;

static uint16_t outToPulse(int32_t out) {
    if(out < 0) out = 0;
    if(out > Esc::OUT_MAX) out = Esc::OUT_MAX;
    int32_t pulse = Esc::PULSE_MIN +
                    out * (int32_t)(Esc::PULSE_MAX - Esc::PULSE_MIN) / Esc::OUT_MAX;
    return (uint16_t)pulse;
}

static int16_t outToPct(int32_t out) {
    if(out < 0) out = 0;
    if(out > Esc::OUT_MAX) out = Esc::OUT_MAX;
    return static_cast<int16_t>((out + Esc::OUT_SCALE / 2) / Esc::OUT_SCALE);
}

static void setPulse(uint gpio, uint16_t us) { pwm_set_gpio_level(gpio, us); }

static void setupPwm(uint gpio) {
    uint slice = pwm_gpio_to_slice_num(gpio);
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    pwm_set_clkdiv(slice, Esc::PWM_CLKDIV);
    pwm_set_wrap(slice, Esc::PWM_WRAP);
    pwm_set_enabled(slice, true);
}

// 直接寫入某個出力（跳過 slew）——僅急停 X / 校準 CAL 使用
static void hardSet(uint gpio, int32_t &out, int16_t pct) {
    if(pct < 0) pct = 0;
    if(pct > Esc::MAX_PCT) pct = Esc::MAX_PCT;
    out = static_cast<int32_t>(pct) * Esc::OUT_SCALE;
    setPulse(gpio, outToPulse(out));
}

static void hardIdle(uint gpio, int32_t &out) { hardSet(gpio, out, 0); }

// 緩加速/緩減速：朝 target 以 dtUs 內允許的量逐步靠近（0.001% 單位）
static void slewToward(uint gpio, int32_t &out, int16_t targetPct, uint32_t dtUs) {
    int32_t target = static_cast<int32_t>(targetPct) * Esc::OUT_SCALE;
    if(target < 0) target = 0;
    if(target > Esc::OUT_MAX) target = Esc::OUT_MAX;

    int32_t diff = target - out;
    if(diff == 0) {
        setPulse(gpio, outToPulse(out));
        return;
    }

    // 依方向選加速率 / 減速率，算出這一 tick 允許的變化量
    int32_t rate = (diff > 0) ? s_rampUp : s_rampDown; // 0.001%/s
    int64_t step = static_cast<int64_t>(rate) * static_cast<int64_t>(dtUs) / 1000000;
    if(step < 1) step = 1; // 速率再低也要有進展（否則永遠動不了）

    if(diff > 0) {
        out += (diff > step) ? static_cast<int32_t>(step) : diff;
    } else {
        out -= (-diff > step) ? static_cast<int32_t>(step) : -diff;
    }
    if(out < 0) out = 0;
    if(out > Esc::OUT_MAX) out = Esc::OUT_MAX;
    setPulse(gpio, outToPulse(out));
}

// 速率設定（%/s → 0.001%/s）
static void setRampRate(int32_t &rate, int16_t pctPerSec) {
    if(pctPerSec < Esc::RAMP_MIN_PCT_S) pctPerSec = Esc::RAMP_MIN_PCT_S;
    if(pctPerSec > Esc::RAMP_MAX_PCT_S) pctPerSec = Esc::RAMP_MAX_PCT_S;
    rate = static_cast<int32_t>(pctPerSec) * Esc::OUT_SCALE;
}

// ============================================================================
// Serial 介面
// ============================================================================
static char s_line[16];
static uint8_t s_lineLen = 0;

// 只認 Enter（\r 或 \n）才送出命令；不會因為停手就自動送出。
static bool pollLine() {
    while(Serial.available() > 0) {
        char ch = static_cast<char>(Serial.read());
        if(ch == '\n' || ch == '\r') {
            Serial.println();
            while(s_lineLen > 0 && (s_line[s_lineLen - 1] == '\r' ||
                                    s_line[s_lineLen - 1] == ' ' ||
                                    s_line[s_lineLen - 1] == '\t'))
                s_lineLen--;
            if(s_lineLen == 0) continue; // 空行忽略，不自動印幫助
            s_line[s_lineLen] = '\0';
            return true;
        }
        if(ch == 8 || ch == 127) { // Backspace
            if(s_lineLen > 0) {
                s_lineLen--;
                s_line[s_lineLen] = '\0';
                Serial.print("\b \b"); // 回顯：退一格、塗白、再退一格
            }
            continue;
        }
        if(s_lineLen < sizeof(s_line) - 1) {
            s_line[s_lineLen++] = ch;
            s_line[s_lineLen] = '\0';
            Serial.write(ch); // 回顯輸入字元
        }
    }
    return false;
}

static bool parseNum(const char *str, int16_t &val) {
    if(*str == '-') return false; // 單向油門不接受負值
    if(*str == '+') str++;
    if(*str == '\0') return false;
    long v = 0;
    while(*str >= '0' && *str <= '9') { v = v * 10 + (*str - '0'); str++; }
    if(*str != '\0') return false;
    if(v > 32767) v = 32767;
    val = static_cast<int16_t>(v);
    return true;
}

// ============================================================================
// 命令執行
// ============================================================================
static void printHelp() {
    Serial.println();
    Serial.println("=== 單向無刷馬達 (Skywalker 油門) 測試 — RP2350@150MHz ===");
    Serial.println("   0% = 1000µs 熄火    100% = 2000µs 全速   50Hz");
    Serial.println("   時脈: clkdiv=150 → 每 tick 1µs (Pico2 150MHz)");
    Serial.println("   GP18→馬達A ESC1      GP19→馬達B ESC2");
    Serial.println();
    Serial.println("命令 (單位 %，0-100，字首大小寫不拘，打完按 Enter)：");
    Serial.println("  A <0-100>   馬達 A 出力");
    Serial.println("  B <0-100>   馬達 B 出力");
    Serial.println("  ALL <0-100> 兩顆同時出力");
    Serial.println("  CAL         油門端點校準(先拆槳,依畫面指示斷電重上)");
    Serial.println("  0           兩顆『緩降』回 0%（熄火）");
    Serial.println("  X           兩顆『立即』斷油（真急停，唯一不經斜坡）");
    Serial.println("  ACC <1-400> 加速率 %/s（緩加速）   DEC <1-400> 減速率 %/s");
    Serial.println("  + / - ε     兩顆微調 (帶量如 +5 / -10)");
    Serial.println("  R           讀目前出力 % / µs / 加減速率");
    Serial.println("  ? 或 h      此幫助");
    Serial.println();
    Serial.printf("  目前速率：ACC %d %%/s   DEC %d %%/s（所有出力變化皆走斜坡）\r\n",
                  (int)(s_rampUp / Esc::OUT_SCALE), (int)(s_rampDown / Esc::OUT_SCALE));
    Serial.println();
    Serial.println("※ 上電後預設 0%(1000µs→熄火),待電調辨識解鎖。請由低%(如5%)");
    Serial.println("  起步確認方向。單向無法由訊號反轉→靠三相任兩條對調。");
    Serial.println();
}

// 油門端點校準（標準航模電調:全程油門 2s→回 0%)
//   電調會把 100% 脈寬與 0% 脈寬記起來,消除各廠中立偏移。
//   安全:執行前必須已拆槳/無負載,並在提示下自行『斷電再上電』
//   使電調以『它在收全油門瞬間電力啟動』進入校準模式(多數 Skywalker)。
//   全程封鎖 Serial；不會有另一條爆轉意外,但會出現高油門2秒須注意。
// -------------------------------------------------------------------
static bool runThrottleCal() {
    // 收回命令立即回 0,避免殘餘（校準流程本身即需要直接跳變）
    hardIdle(Pin::MOTOR_A_PWM, s_outA);
    hardIdle(Pin::MOTOR_B_PWM, s_outB);
    s_cmdA = s_cmdB = 0;
    s_outA = s_outB = 0;

    // flush 殘留
    while(Serial.available()) Serial.read();

    Serial.println();
    Serial.println("== 油門端點校準 (CAL) — 務必先拆槳 ==");
    Serial.println("  原理:電調要在『開機當下收到全油門 2000µs』才進入");
    Serial.println("        校準→記住高/低端點。以下我會先把訊號拉全,");
    Serial.println("        你在這期間把電調【斷電再上電】。");
    Serial.println();
    Serial.println("  現把出力拉到 100% (2000µs)…");
    // 1) 先送全油門 —— 讓電調『以此開機』進入校準
    setPulse(Pin::MOTOR_A_PWM, Esc::PULSE_MAX);
    setPulse(Pin::MOTOR_B_PWM, Esc::PULSE_MAX);
    s_outA = s_outB = Esc::OUT_MAX;
    Serial.println("  請在接下來 6 秒內『關電調電源 → 再上電』");
    Serial.println("  (上電後若聽到『嗶嗶嗶』即進入校準;馬達不該轉)");
    for(int i = 6; i > 0; i--) {
        Serial.printf("   %ds ...\r\n", i);
        delay(1000);
    }
    // 2) 高端點停留(讓電調記好 max),通常一聲『嗶』
    Serial.println("  繼續高油門 2s 記高端點 …(期待『嗶』)");
    delay(2000);
    // 3) 回 0% 收低端點(當下電調原在高端,回低 = 設定 0% 邊界)
    setPulse(Pin::MOTOR_A_PWM, Esc::PULSE_MIN);
    setPulse(Pin::MOTOR_B_PWM, Esc::PULSE_MIN);
    s_outA = s_outB = 0;
    Serial.println("  回 0% (1000µs) 記低端點 …(期待『嗶嗶』)");
    delay(1500);
    Serial.println("  校準完成(設定即存於電調 EEPROM)。");
    Serial.println("  現回到 0% 待命。請低 % 起步確認段位/轉速正確。");
    Serial.println();
    return true;
}

// 抽出開頭字母字與其後數字起點：op ≤4 字、buf 需 ≥5。
// 例："ALL 30"→op"ALL", num="30"；"A10"→op"A", num="10"；"cal"→op"CAL"。
// 例："0"/"X"/"+/-" 無字母→op 空。
static char s_op[6];
static const char *s_num;

static void parseOp() {
    unsigned i = 0;
    const char *p = s_line;
    // 取連續字母(不分大小寫→存大寫)
    while(i < 4 && *p != '\0') {
        char c = *p;
        if(!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'))) break;
        s_op[i] = (c >= 'a' && c <= 'z') ? (char)(c - ('a' - 'A')) : c;
        i++;
        p++;
    }
    s_op[i] = '\0';
    // 跳過 op 後空白
    while(*p == ' ' || *p == '\t') p++;
    s_num = p; // 數字起點(或沒數字時指向 op 後位置/空)
}

// 字串比較(剛好長度)
static bool opEq(const char *op, const char *w) {
    while(*op != '\0' && *w != '\0') { if(*op != *w) return false; op++; w++; }
    return *op == '\0' && *w == '\0';
}

// 命令主入口 —— 以「字首單字(不區大小寫)+ 可選數字」分派,乾淨解決 A/ALL 歧義
static void handleCommand() {
    Serial.println();

    // 特殊純符號: 以 + / - / 0 開頭
    char first = s_line[0];
    if(first == '+' || first == '-' || first == '0') {
        if(first == '0') { // 整列以 0 開頭 → 緩降停(兩顆回0)
            int32_t maxOut = (s_outA > s_outB) ? s_outA : s_outB;
            s_cmdA = s_cmdB = 0;
            // s_rampDown 單位 0.001%/s、maxOut 單位 0.001% → 比值即秒數(×10 取一位)
            int32_t etaTenths = (s_rampDown > 0) ? (maxOut * 10) / s_rampDown : 0;
            Serial.printf("兩顆緩降至 0%%（DEC %d %%/s → 約 %d.%ds）\r\n",
                          (int)(s_rampDown / Esc::OUT_SCALE),
                          (int)(etaTenths / 10), (int)(etaTenths % 10));
            return;
        }
        // +/- 微調(帶量為正數,如 +5 加 / -10 減…);此直接當增量
        int16_t delta = 5, tmp;
        const char *np = &s_line[1];
        while(*np == ' ' || *np == '\t') np++;
        if(parseNum(np, tmp) && tmp >= 0 && tmp <= 100) delta = tmp;
        int16_t d = (first == '+') ? delta : -delta;
        s_cmdA = static_cast<int16_t>(outToPct(s_outA) + d);
        s_cmdB = static_cast<int16_t>(outToPct(s_outB) + d);
        if(s_cmdA < 0) s_cmdA = 0;
        if(s_cmdB < 0) s_cmdB = 0;
        Serial.printf("微調 → A %d%%  B %d%%（斜坡中）\r\n", (int)s_cmdA, (int)s_cmdB);
        return;
    }

    parseOp();
    if(s_op[0] == '\0') {
        if(s_lineLen == 0) { printHelp(); return; }
        Serial.println("?? 未知命令，鍵入 'h' 看用法");
        return;
    }

    // 純字面(無需數字)命令
    if(opEq(s_op, "H") || opEq(s_op, "?")) { printHelp(); return; }
    if(opEq(s_op, "R")) {
        Serial.printf("   A: %d%% (~%uµs)   B: %d%% (~%uµs)\r\n",
                      (int)outToPct(s_outA), (unsigned)outToPulse(s_outA),
                      (int)outToPct(s_outB), (unsigned)outToPulse(s_outB));
        Serial.printf("   目標 A %d%% / B %d%%   ACC %d %%/s  DEC %d %%/s\r\n",
                      (int)s_cmdA, (int)s_cmdB,
                      (int)(s_rampUp / Esc::OUT_SCALE),
                      (int)(s_rampDown / Esc::OUT_SCALE));
        return;
    }
    if(opEq(s_op, "X")) { // 兩顆立即斷油（真急停）
        s_cmdA = 0; s_cmdB = 0;
        hardIdle(Pin::MOTOR_A_PWM, s_outA);
        hardIdle(Pin::MOTOR_B_PWM, s_outB);
        Serial.println("急停：兩顆立即回 0%（1000µs 熄火，不經斜坡）");
        return;
    }
    if(opEq(s_op, "CAL") || opEq(s_op, "C")) {
        runThrottleCal();
        return;
    }

    if(opEq(s_op, "ACC") || opEq(s_op, "DEC")) {
        int16_t rate = 0;
        if(!parseNum(s_num, rate) || rate < Esc::RAMP_MIN_PCT_S ||
           rate > Esc::RAMP_MAX_PCT_S) {
            Serial.printf("?? 速率須為 %d-%d %%/s\r\n", (int)Esc::RAMP_MIN_PCT_S,
                          (int)Esc::RAMP_MAX_PCT_S);
            return;
        }
        if(opEq(s_op, "ACC")) setRampRate(s_rampUp, rate);
        else setRampRate(s_rampDown, rate);
        Serial.printf("加速率 ACC %d %%/s   減速率 DEC %d %%/s\r\n",
                      (int)(s_rampUp / Esc::OUT_SCALE),
                      (int)(s_rampDown / Esc::OUT_SCALE));
        return;
    }

    // 剩下帶數字的出力指令（A/B/ALL）
    int16_t pct = 0;
    if(!parseNum(s_num, pct)) {
        Serial.println("?? 數字無效／負值(單向)或指令不符，鍵入 'h'");
        return;
    }
    if(pct > Esc::MAX_PCT) pct = Esc::MAX_PCT;

    if(opEq(s_op, "A") || opEq(s_op, "1")) {
        s_cmdA = pct;
        Serial.printf("馬達A → %d%%（緩慢趨近中）\r\n", (int)s_cmdA);
    } else if(opEq(s_op, "B") || opEq(s_op, "2")) {
        s_cmdB = pct;
        Serial.printf("馬達B → %d%%（緩慢趨近中）\r\n", (int)s_cmdB);
    } else if(opEq(s_op, "ALL") || opEq(s_op, "L")) {
        s_cmdA = pct; s_cmdB = pct;
        Serial.printf("兩顆同→ %d%%（緩慢趨近中）\r\n", (int)pct);
    } else {
        Serial.println("?? 未知命令，鍵入 'h' 看用法");
    }
}


// ============================================================================
// setup / loop
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(150);
    Serial.println();
    Serial.println("[esc_test] 單向無刷馬達 (Skywalker 油門) 測試");

    setupPwm(Pin::MOTOR_A_PWM);
    setupPwm(Pin::MOTOR_B_PWM);

    // 上電即 0% → 1000µs→熄火;等待電調辨識解鎖
    hardIdle(Pin::MOTOR_A_PWM, s_outA);
    hardIdle(Pin::MOTOR_B_PWM, s_outB);
    s_cmdA = 0; s_cmdB = 0;

    // 給電調完成「上電低電位解鎖」的穩定時間(不可在此段抬油門)
    delay(3000);

    printHelp();
    Serial.println("已就緒：兩顆 0% (1000µs 熄火) 待命。");
}

void loop() {
    // 以固定週期量測 dt 後朝目標緩加速／緩減速推進
    static uint32_t lastTickUs = micros();
    uint32_t nowUs = micros();
    uint32_t dtUs = nowUs - lastTickUs;
    if(dtUs < Esc::RAMP_TICK_US) {
        yield();
        return;
    }
    if(dtUs > 100000) dtUs = 100000; // 過長（如 CAL 後）夾住，避免跳級
    lastTickUs = nowUs;

    slewToward(Pin::MOTOR_A_PWM, s_outA, s_cmdA, dtUs);
    slewToward(Pin::MOTOR_B_PWM, s_outB, s_cmdB, dtUs);

    if(pollLine()) {
        handleCommand();
        s_lineLen = 0;
        for(uint8_t i = 0; i < sizeof(s_line); i++) s_line[i] = '\0';
    }
}
