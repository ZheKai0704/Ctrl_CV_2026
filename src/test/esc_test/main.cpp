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

// Serial 命令（115200, 字首大小寫不拘）：
//   A <0-100>   馬達 A 出力 %        B <0-100>   馬達 B 出力 %
//   ALL <0-100> 兩顆同時出力
//   CAL         油門端點校準(會先拉到全油門→再回 0 存端點;請遠離槳)。
//               ※ 校準前務必先關電,讓電調重新以…見說明印
//   0           馬達回 0%（熄火/低點）   X = 兩顆急停回 0%（disarm）
//   + / -       兩顆微調（帶量）       R 讀狀態    ?/h 幫助
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
constexpr int16_t  RAMP_STEP_PCT = 4;    // slew 每 tick 步進
} // namespace Esc

// 目前目標（未 slew 完成的即時值）+ 實際輸出
static int16_t s_outA = 0; // 目前實際 PWM 對應 pct(0..100)
static int16_t s_outB = 0;
static int16_t s_cmdA = 0; // 目標
static int16_t s_cmdB = 0;

static uint16_t pctToPulse(int16_t pct) {
    if(pct < 0) pct = 0;
    if(pct > Esc::MAX_PCT) pct = Esc::MAX_PCT;
    int32_t pulse = Esc::PULSE_MIN +
                    (int32_t)pct * (int32_t)(Esc::PULSE_MAX - Esc::PULSE_MIN) /
                        (int32_t)Esc::MAX_PCT;
    return (uint16_t)pulse;
}

static void setPulse(uint gpio, uint16_t us) { pwm_set_gpio_level(gpio, us); }

static void setupPwm(uint gpio) {
    uint slice = pwm_gpio_to_slice_num(gpio);
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    pwm_set_clkdiv(slice, Esc::PWM_CLKDIV);
    pwm_set_wrap(slice, Esc::PWM_WRAP);
    pwm_set_enabled(slice, true);
}

// 立即回廠（寫真實 0% 低脈）任何 slew 外強制
static void hardIdle(uint gpio, int16_t &out) {
    out = 0;
    setPulse(gpio, Esc::PULSE_MIN);
}

// 單格 slew（朝 target, ≤ RAMP_STEP_PCT）
static void slewToward(uint gpio, int16_t &out, int16_t target) {
    int16_t diff = target - out;
    if(diff > Esc::RAMP_STEP_PCT) diff = Esc::RAMP_STEP_PCT;
    if(diff < -Esc::RAMP_STEP_PCT) diff = -Esc::RAMP_STEP_PCT;
    out += diff;
    if(out < 0) out = 0;
    if(out > Esc::MAX_PCT) out = Esc::MAX_PCT;
    setPulse(gpio, pctToPulse(out));
}

// ============================================================================
// Serial 介面
// ============================================================================
static char s_line[16];
static uint8_t s_lineLen = 0;

static bool pollLine() {
    static uint32_t lastByteMs = millis();
    while(Serial.available() > 0) {
        char ch = static_cast<char>(Serial.read());
        Serial.write(ch);
        if(ch == '\n' || ch == '\r') {
            while(s_lineLen > 0 && (s_line[s_lineLen - 1] == '\r' ||
                                    s_line[s_lineLen - 1] == ' '))
                s_lineLen--;
            return true;
        }
        if(s_lineLen < sizeof(s_line) - 1) s_line[s_lineLen++] = ch;
        lastByteMs = millis();
    }
    if(s_lineLen > 0 && (millis() - lastByteMs) > 300) return true;
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
    Serial.println("命令 (單位 %，0-100，字首大小寫不拘)：");
    Serial.println("  A <0-100>   馬達 A 出力");
    Serial.println("  B <0-100>   馬達 B 出力");
    Serial.println("  ALL <0-100> 兩顆同時出力");
    Serial.println("  CAL         油門端點校準(先拆槳,依畫面指示斷電重上)");
    Serial.println("  0 或 X      兩顆急停回 0%（disarm）");
    Serial.println("  + / - ε     兩顆微調 (帶量如 +5 / -10)");
    Serial.println("  R           讀目前出力 % / µs");
    Serial.println("  ? 或 h      此幫助");
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
    // 收回命令立即回 0,避免殘餘
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
    s_outA = s_outB = Esc::MAX_PCT;
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
        if(first == '0') { // 整列以 0 開頭 → 停(兩顆回0)
            s_cmdA = s_cmdB = 0;
            hardIdle(Pin::MOTOR_A_PWM, s_outA);
            hardIdle(Pin::MOTOR_B_PWM, s_outB);
            Serial.println("兩顆回 0%（1000µs 熄火）");
            return;
        }
        // +/- 微調(帶量為正數,如 +5 加 / -10 減…);此直接當增量
        int16_t delta = 5, tmp;
        const char *np = &s_line[1];
        while(*np == ' ' || *np == '\t') np++;
        if(parseNum(np, tmp) && tmp >= 0 && tmp <= 100) delta = tmp;
        int16_t d = (first == '+') ? delta : -delta;
        s_cmdA = static_cast<int16_t>(s_outA + d);
        s_cmdB = static_cast<int16_t>(s_outB + d);
        if(s_cmdA < 0) s_cmdA = 0;
        if(s_cmdB < 0) s_cmdB = 0;
        Serial.printf("微調 → A %d  B %d\r\n", (int)s_cmdA, (int)s_cmdB);
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
                      (int)s_outA, (unsigned)pctToPulse(s_outA),
                      (int)s_outB, (unsigned)pctToPulse(s_outB));
        return;
    }
    if(opEq(s_op, "X")) { // 兩顆急停去使能
        s_cmdA = 0; s_cmdB = 0;
        hardIdle(Pin::MOTOR_A_PWM, s_outA);
        hardIdle(Pin::MOTOR_B_PWM, s_outB);
        Serial.println("急停：兩顆回 0%（1000µs 熄火）");
        return;
    }
    if(opEq(s_op, "CAL") || opEq(s_op, "C")) {
        runThrottleCal();
        return;
    }

    // 剩下帶數字的出力指令（A/B/ALL）
    int16_t pct = 0;
    if(!parseNum(s_num, pct)) {
        Serial.println("?? 數字無效／負值(單向)或指令不符，鍵入 'h'");
        return;
    }

    if(opEq(s_op, "A") || opEq(s_op, "1")) {
        s_cmdA = pct;
        Serial.printf("馬達A → %d%%\r\n", (int)s_cmdA);
    } else if(opEq(s_op, "B") || opEq(s_op, "2")) {
        s_cmdB = pct;
        Serial.printf("馬達B → %d%%\r\n", (int)s_cmdB);
    } else if(opEq(s_op, "ALL") || opEq(s_op, "L")) {
        s_cmdA = pct; s_cmdB = pct;
        Serial.printf("兩顆同→ %d%%\r\n", (int)pct);
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
    // 以 slew 朝目標推進（單向：只往 0..100）
    slewToward(Pin::MOTOR_A_PWM, s_outA, s_cmdA);
    slewToward(Pin::MOTOR_B_PWM, s_outB, s_cmdB);

    if(pollLine()) {
        handleCommand();
        s_lineLen = 0;
        for(uint8_t i = 0; i < sizeof(s_line); i++) s_line[i] = '\0';
    }
    delay(2);
}
