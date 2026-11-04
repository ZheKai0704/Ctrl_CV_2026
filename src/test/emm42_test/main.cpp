// ============================================================================
// emm42_test — 張大頭 X42S 步進馬達（Emm 固件）診斷型控制測試 (RP2350)
//
// 用途：
//   用 RP2350 (Waveshare RP2350 Plus, Arduino-Pico) 直接以 UART 控制一顆
//   X42S 步進馬達，並在「角度控制沒反應」時把 TX 幀、RX 回覆、錯誤碼全部印出來。
//
// 協定（ZDT_X42S 第二代手冊 / Emm 固件）：
//   - 幀： [地址][功能碼][參數…][尾碼 0x6B]，地址預設 = 1
//   - 位置單位為「脈衝」：1.8°步進 ×16細分 = 3200 脈衝 = 360°
//   - 回覆碼：02=收到命令正確、E2=參數錯誤或堵轉保護、EE=格式錯誤、9F=到位
//
// 接線（RP2350 UART0）：GP0=TX→馬達 R/A/H 或 T/B/L，GP1=RX，115200，務必共地。
//
// 按鍵（USB Serial, 115200）：
//   't' 一鍵自我診斷（使能→讀位置→+90°→讀位置→-90°→讀位置，逐步印過程）
//   'e' 使能            'd' 去使能        'i' 讀使能狀態
//   'v' 設定速度(RPM)   'k' 設定加速度檔位(0-255)
//   'a' 設定絕對角度(°)  → 需先歸零/清零才有意義
//   'g' 設定角度(°)      → 由目前位置再轉（最常用、不依賴零位）
//   'h' 歸零(回絕對零位) 'z' 把目前位置清零(設為0°)
//   'r' 讀即時位置(°)    's' 立即停止      'c' 清除堵轉保護
//   '*' 掃描地址 1..255（目前波特率）      '#' 掃描波特率×地址
//   'F' 切換 FD 位置幀格式：13B(第二代，含運動模式) / 12B(V4.2 舊 Emm)
//   '?' 此幫助
// ============================================================================

// 最多 46360 最少 0

#include <Arduino.h>

// ============================================================================
// 腳位（UART0 → 與馬達通訊；Earlephilhower 的 Serial1 = HW UART0，預設 GP0/GP1）
// ============================================================================
namespace Pin {
constexpr uint MOTOR_UART_TX = 0; // GP0 → 馬達 T/B/L
constexpr uint MOTOR_UART_RX = 1; // GP1 ← 馬達 R/A/H
} // namespace Pin

// ============================================================================
// 協定常數（Emm 固件）
// ============================================================================
namespace X42S {
constexpr uint8_t PROTO_ADDR = 0x01; // 地址（預設 1）
constexpr uint8_t TAIL = 0x6B;       // 幀尾碼
// 功能碼
constexpr uint8_t FD_EN = 0xF3;       // 使能/去使能（輔助碼 AB）
constexpr uint8_t FD_POS = 0xFD;      // 位置模式控制（Emm）
constexpr uint8_t FD_STOP = 0xFE;     // 立即停止（輔助碼 98）
constexpr uint8_t FD_READPOS = 0x36;  // 讀即時位置
constexpr uint8_t FD_HOME = 0x9A;     // 觸發回零
constexpr uint8_t FD_CLEARPOS = 0x0A; // 把當前位置角度清零（輔助碼 6D）
constexpr uint8_t FD_CLOG = 0x0E;     // 解除堵轉（輔助碼 52）
constexpr uint8_t FD_STATUS = 0x3A;   // 讀使能狀態
// 輔助碼
constexpr uint8_t AX_EN = 0xAB;
constexpr uint8_t AX_STOP = 0x98;
constexpr uint8_t AX_CLEARPOS = 0x6D;
constexpr uint8_t AX_CLOG = 0x52;
// 回覆碼
constexpr uint8_t RP_OK = 0x02;    // 命令正確
constexpr uint8_t RP_PARAM = 0xE2; // 參數錯誤 / 堵轉保護中
constexpr uint8_t RP_FMT = 0xEE;   // 格式錯誤
constexpr uint8_t RP_DONE = 0x9F;  // 位置到位
} // namespace X42S

// Emm 位置換算：1.8°步進 ×16細分 = 3200 脈衝 / 圈 = 360°
constexpr uint16_t kPulsesPerRev = 3200;

// 可調參數（預設值保守，避免超出馬達上限被回 E2 而看起來「不會動」）
constexpr uint16_t kVelMinRpm = 1;
constexpr uint16_t kVelMaxRpm = 3000; // Emm42_V5.0 / X42S 標稱上限
static uint16_t gVelRpm = 600;        // 位置模式速度（RPM）
static uint8_t gAccStep = 100;        // 加速度檔位 0-255（0 = 無加減速）
// FD 位置幀格式：true = 13B（第二代，含「運動模式」位元）；false = 12B（V4.2 舊 Emm）
static bool gModeByte = true;

// ============================================================================
// 緩衝（靜態分配，專案禁止 new/malloc）
// ============================================================================
static uint8_t sTxBuf[32];
static uint8_t sRxBuf[64];
static uint8_t sTxLen = 0;

// ============================================================================
// 底層收發
// ============================================================================
static void printHex(const char *tag, const uint8_t *buf, uint8_t len) {
    Serial.print(tag);
    Serial.print(" (");
    Serial.print(len);
    Serial.print("B): ");
    for(uint8_t i = 0; i < len; i++) {
        Serial.printf("%02X ", buf[i]);
    }
    Serial.println();
}

static void sendFrame(const uint8_t *buf, uint8_t len) {
    Serial1.write(buf, len);
    Serial1.flush(); // 等 TX 真的送完，避免半框被 RX / 下一命令打斷
    // 把送出的幀印出來：跟手冊逐位元組對照用
    printHex("  TX", buf, len);
}

// 接收一幀，以「timeoutMs 無新資料」判定結束
static uint8_t receiveFrame(uint32_t timeoutMs) {
    uint8_t i = 0;
    uint32_t lastByteMs = millis();
    while(millis() - lastByteMs < timeoutMs) {
        while(Serial1.available() > 0) {
            if(i < sizeof(sRxBuf)) {
                sRxBuf[i++] = static_cast<uint8_t>(Serial1.read());
                lastByteMs = millis();
            } else {
                Serial1.read(); // 溢位丟棄
            }
        }
    }
    return i;
}

// 送命令前丟棄 RX 殘留（避免舊垃圾混進回覆）
static void flushInput() {
    while(Serial1.available() > 0) {
        Serial1.read();
    }
}

// 回覆碼 → 人話
static const char *replyCodeText(uint8_t code) {
    switch(code) {
    case X42S::RP_OK:
        return "02 = 命令正確（馬達收到）";
    case X42S::RP_PARAM:
        return "E2 = 參數錯誤或堵轉保護中（速度/加速度/脈衝數超出範圍，或馬達卡住）";
    case X42S::RP_FMT:
        return "EE = 格式錯誤（幀長度/功能碼/尾碼不對 → 固件版本不同，試 'F' 切 12B）";
    case X42S::RP_DONE:
        return "9F = 位置到位";
    default:
        return "未知回覆碼";
    }
}

// 印出回覆並嘗試解讀（位址 + 功能碼 + 回覆碼）
static void printRawReply(uint8_t n) {
    if(n == 0) {
        Serial.println("  reply (0B): 無回應！檢查 TX/RX 是否對調、共地、波特率、"
                       "馬達 P_Serial 是否設為 UART");
        return;
    }
    printHex("  reply", sRxBuf, n);
    for(uint8_t i = 0; i + 2 < n; i++) {
        if(sRxBuf[i] != X42S::PROTO_ADDR) {
            continue;
        }
        Serial.printf("  → addr=%u func=%02X : %s\n", sRxBuf[i], sRxBuf[i + 1],
                      replyCodeText(sRxBuf[i + 2]));
    }
}

// ============================================================================
// 命令
// ============================================================================
static void sendEnable(bool state) {
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_EN;
    sTxBuf[2] = X42S::AX_EN;
    sTxBuf[3] = state ? 0x01 : 0x00;
    sTxBuf[4] = 0; // 同步標誌
    sTxBuf[5] = X42S::TAIL;
    sendFrame(sTxBuf, 6);
}

// 讀使能狀態： [Addr][3A][6B] → [Addr][3A][01/00][6B]
static void sendReadStatus() {
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_STATUS;
    sTxBuf[2] = X42S::TAIL;
    sendFrame(sTxBuf, 3);
}

// Emm 位置模式（13B）：[Addr][FD][方向][速度 2B][加速度檔位][脈衝數 4B LE][運動模式][同步][6B]
//   運動模式：0=相對上次目標、1=絕對(相對零位)、2=相對當前即時位置
// V4.2 舊 Emm（12B）：[Addr][FD][方向][速度 2B][加速度檔位][脈衝數 4B LE][同步][6B]
static void sendPosition(int32_t pulses, uint8_t mode, uint8_t dir = 0) {
    uint32_t val = static_cast<uint32_t>(pulses);
    uint8_t i = 0;
    sTxBuf[i++] = X42S::PROTO_ADDR;
    sTxBuf[i++] = X42S::FD_POS;
    sTxBuf[i++] = dir;
    sTxBuf[i++] = static_cast<uint8_t>(gVelRpm >> 8);
    sTxBuf[i++] = static_cast<uint8_t>(gVelRpm >> 0);
    sTxBuf[i++] = gAccStep;
    sTxBuf[i++] = static_cast<uint8_t>(val >> 24);
    sTxBuf[i++] = static_cast<uint8_t>(val >> 16);
    sTxBuf[i++] = static_cast<uint8_t>(val >> 8);
    sTxBuf[i++] = static_cast<uint8_t>(val >> 0);
    if(gModeByte) {
        sTxBuf[i++] = mode;
    }
    sTxBuf[i++] = 0; // 同步標誌
    sTxBuf[i++] = X42S::TAIL;
    sendFrame(sTxBuf, i);
}

// 角度(°) → 脈衝數
static int32_t degToPulses(float degrees) {
    return static_cast<int32_t>(degrees * static_cast<float>(kPulsesPerRev) / 360.0f);
}

// 觸發回零 → 回絕對位置零位（9A，mode 04）
static void sendHome() {
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_HOME;
    sTxBuf[2] = 0x04; // 回零模式：回到絕對位置坐標零點
    sTxBuf[3] = 0x00; // 同步標誌
    sTxBuf[4] = X42S::TAIL;
    sendFrame(sTxBuf, 5);
}

// 把「當前位置」角度清零（0A 6D）→ 之後絕對位置 a 以這裡為 0°
static void sendClearPos() {
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_CLEARPOS;
    sTxBuf[2] = X42S::AX_CLEARPOS;
    sTxBuf[3] = X42S::TAIL;
    sendFrame(sTxBuf, 4);
}

static void sendStop() {
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_STOP;
    sTxBuf[2] = X42S::AX_STOP;
    sTxBuf[3] = 0x00; // 同步標誌
    sTxBuf[4] = X42S::TAIL;
    sendFrame(sTxBuf, 5);
}

static void sendClogReset() {
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_CLOG;
    sTxBuf[2] = X42S::AX_CLOG;
    sTxBuf[3] = X42S::TAIL;
    sendFrame(sTxBuf, 4);
}

// ============================================================================
// 重設 UART（掃描不同波特率時使用）
//   注意：arduino-pico 的 HardwareSerial 在「已 begin()」狀態下呼叫
//   setTX()/setRX() 會 panic("Attempting to set Serial1.TX while running")，
//   因此必須 end() → setTX/setRX → begin() 的順序。
// ============================================================================
static void configureUart(uint32_t baud) {
    Serial1.end();
    Serial1.setTX(Pin::MOTOR_UART_TX);
    Serial1.setRX(Pin::MOTOR_UART_RX);
    Serial1.begin(baud);
    flushInput();
}

// 送 enable 並檢查是否有 [addr][F3][02] 回覆
static bool probeOne(uint8_t addr, uint32_t ackWaitMs) {
    sTxBuf[0] = addr;
    sTxBuf[1] = X42S::FD_EN;
    sTxBuf[2] = X42S::AX_EN;
    sTxBuf[3] = 0x01;
    sTxBuf[4] = 0x00;
    sTxBuf[5] = X42S::TAIL;
    sendFrame(sTxBuf, 6);

    uint8_t n = receiveFrame(ackWaitMs);
    for(uint8_t i = 0; i + 2 < n; i++) {
        if(sRxBuf[i] == addr && sRxBuf[i + 1] == X42S::FD_EN &&
           sRxBuf[i + 2] == X42S::RP_OK) {
            return true;
        }
    }
    return false;
}

// ============================================================================
// 掃描馬達地址：對地址 1..255 各送一幀 enable，找出會回覆的馬達位址。
// ============================================================================
static void scanAddresses() {
    Serial.printf(">> 掃描地址 1..255 @ %lu bps（送 enable，找會回覆 'F3 02' 的馬達）...\n",
                  static_cast<unsigned long>(115200));
    uint8_t found = 0;
    for(uint16_t a = 1; a <= 255; a++) {
        if(probeOne(static_cast<uint8_t>(a), 12)) {
            Serial.printf("  addr %u : ✓ 有回覆\n", a);
            found++;
        }
    }
    if(found == 0) {
        Serial.println(">> 掃描完成：沒有馬達回覆。先確認 'e' 是否真的有回覆（看 TX/RX 幀）。");
    } else {
        Serial.printf(">> 掃描完成，共 %u 顆。\n", found);
    }
}

// ============================================================================
// 掃描「波特率 × 地址」：逐一重設 UART 波特率，並在每波特率下掃地址 1..64。
//   掃完自動恢復回目前設定（預設 115200）。
// ============================================================================
static void scanBaudAndAddr() {
    const uint32_t kBauds[] = {9600, 19200, 38400, 57600, 115200, 230400, 460800};
    const uint8_t kBaudCount = sizeof(kBauds) / sizeof(kBauds[0]);
    Serial.println(">> 掃描波特率×地址 1..64（送 enable 找馬達）...");

    for(uint8_t bi = 0; bi < kBaudCount; bi++) {
        uint32_t baud = kBauds[bi];
        configureUart(baud);
        bool found = false;
        for(uint16_t a = 1; a <= 64; a++) {
            if(probeOne(static_cast<uint8_t>(a), 15)) {
                Serial.printf("  ✓ FOUND baud=%lu addr=%u\n", static_cast<unsigned long>(baud), a);
                found = true;
            }
        }
        if(!found) {
            Serial.printf("  baud %lu : 無馬達回應\n", static_cast<unsigned long>(baud));
        }
    }

    configureUart(115200); // 恢復預設
    Serial.println(">> 掃描完成（已恢復 baud 115200）。");
}

// ============================================================================
// 讀即時位置 (Emm) — [Addr][36][回覆] [Addr][36][sign][4B LE][6B]
//   Emm：角度(°) = raw × 360 / 65536（0-65535 為一圈），sign 為正/負
// ============================================================================
static bool readPositionRaw(uint32_t &rawOut, bool &negOut, uint8_t &nOut) {
    flushInput();
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_READPOS;
    sTxBuf[2] = X42S::TAIL;
    sendFrame(sTxBuf, 3);

    uint8_t n = receiveFrame(50);
    nOut = n;
    // 跳過可能出現的 echo：找最後一幀 [addr][0x36]
    int idx = -1;
    for(int i = 0; i + 1 < n; i++) {
        if(sRxBuf[i] == X42S::PROTO_ADDR && sRxBuf[i + 1] == X42S::FD_READPOS) {
            idx = i;
        }
    }
    // 資料幀應為 [addr][36][sign][4B][6B] = 8 bytes
    if(idx < 0 || (n - idx) < 8 || sRxBuf[n - 1] != X42S::TAIL) {
        return false;
    }
    rawOut = (static_cast<uint32_t>(sRxBuf[idx + 3]) << 24) |
             (static_cast<uint32_t>(sRxBuf[idx + 4]) << 16) |
             (static_cast<uint32_t>(sRxBuf[idx + 5]) << 8) |
             (static_cast<uint32_t>(sRxBuf[idx + 6]) << 0);
    negOut = (sRxBuf[idx + 2] != 0);
    return true;
}

static bool readPosition(float &degOut) {
    uint32_t raw = 0;
    bool neg = false;
    uint8_t n = 0;
    if(!readPositionRaw(raw, neg, n)) {
        printRawReply(n);
        return false;
    }
    double deg = static_cast<double>(raw) * 360.0 / 65536.0;
    if(neg) {
        deg = -deg;
    }
    degOut = static_cast<float>(deg);
    return true;
}

// ============================================================================
// 印一位小數（避免依賴 %f / float printf）
// ============================================================================
static void printFix1(float v) {
    long v10 = static_cast<long>(v * 10.0f + (v < 0 ? -0.5f : 0.5f));
    bool neg = v10 < 0;
    unsigned long a = neg ? static_cast<unsigned long>(-v10) : static_cast<unsigned long>(v10);
    if(neg) {
        Serial.print('-');
    }
    Serial.print(static_cast<unsigned long>(a / 10));
    Serial.print('.');
    Serial.print(static_cast<unsigned long>(a % 10));
}

// ============================================================================
// 從 USB Serial 讀整數（可負）
// ============================================================================
static bool readIntInput(int32_t &val, const char *prompt, uint32_t waitMs = 10000) {
    // 先清掉 USB 殘留位元組（可能是前一條負號命令遺留的 '-'），避免污染本次輸入
    uint32_t fd0 = millis();
    while(millis() - fd0 < 40) {
        if(Serial.available()) {
            Serial.read();
            fd0 = millis();
        }
    }
    while(Serial.available()) {
        Serial.read();
    }

    Serial.print(prompt);
    long v = 0;
    bool neg = false;
    bool got = false;
    uint32_t t0 = millis();
    while(millis() - t0 < waitMs) {
        if(Serial.available()) {
            char ch = static_cast<char>(Serial.read());
            if(ch == '\n' || ch == '\r') {
                if(got) {
                    val = neg ? -v : v;
                    Serial.println();
                    return true;
                }
            } else if(ch == '-' && !got) { // 負號只在任何數字之前才有效
                neg = true;
            } else if(ch >= '0' && ch <= '9') {
                got = true;
                v = v * 10 + (ch - '0');
                Serial.write(ch);
            }
        }
    }
    Serial.println("  (逾時)");
    return false;
}

// ============================================================================
// 單步：相對移動 → 等到馬達停 → 印位置
// ============================================================================
static void doRelativeMove(float deg) {
    int32_t p = degToPulses(deg);
    Serial.print(">> 相對移動 ");
    printFix1(deg);
    Serial.print("° = ");
    Serial.print(p);
    Serial.printf(" 脈衝 @ %u RPM\n", gVelRpm);
    sendPosition(p, 2, 0); // mode=2 相對當前即時位置
    printRawReply(receiveFrame(100));

    // 位置變化偵測：最多觀察 4 秒
    float start = 0.0f;
    float now = 0.0f;
    bool moved = false;
    uint32_t t0 = millis();
    while(millis() - t0 < 4000) {
        delay(150);
        float pos = 0.0f;
        if(readPosition(pos)) {
            if(!moved) {
                start = pos;
                moved = true;
            }
            if(now != pos) {
                now = pos;
            }
        }
    }
    Serial.print(">> 移動後位置 = ");
    printFix1(now);
    Serial.print("°（起點 ");
    printFix1(start);
    Serial.println("°）");
    if(!moved) {
        Serial.println("   ⚠ 讀不到位置回覆，無法判斷是否真的轉動（看上面 reply 的原始位元組）");
    }
}

// ============================================================================
// 一鍵自我診斷：把「角度控制不動」的每一段都印清楚
// ============================================================================
static void selfTest() {
    Serial.println("\n========== 自我診斷開始 ==========");
    Serial.printf("現在設定：速度 %u RPM、加速度檔位 %u、FD 幀 %s、addr=%u\n", gVelRpm, gAccStep,
                  gModeByte ? "13B(第二代)" : "12B(V4.2)", X42S::PROTO_ADDR);

    Serial.println("[1] 使能");
    sendEnable(true);
    uint8_t n = receiveFrame(100);
    printRawReply(n);
    if(n == 0) {
        Serial.println("!! 使能就沒回覆 → 這是 UART/接線/波特率問題，不是角度命令問題。");
        return;
    }

    Serial.println("[2] 讀即時位置");
    float p0 = 0.0f;
    if(!readPosition(p0)) {
        Serial.println("!! 讀不到位置 → 固件回覆格式不同（位置讀取用 0x36），角度命令大概也是格式問題。");
    } else {
        Serial.print("    目前位置 = ");
        printFix1(p0);
        Serial.println("°");
    }

    Serial.println("[3] 相對 +90°（低速測試）");
    doRelativeMove(90.0f);

    Serial.println("[4] 相對 -90°（回原位）");
    doRelativeMove(-90.0f);

    Serial.println("[5] 清堵轉保護（若上一步回 E2）");
    sendClogReset();
    printRawReply(receiveFrame(100));

    Serial.println("========== 自我診斷結束 ==========");
    Serial.println("判讀：");
    Serial.println("  • 完全沒回覆        → '#' 掃波特率/地址，或查 P_Serial 選單");
    Serial.println("  • 回 EE             → 幀格式不符，按 'F' 切 12B 再試");
    Serial.println("  • 回 E2             → 速度/加速度/脈衝數超範圍或堵轉，'v' 降速或 'c' 清保護");
    Serial.println("  • 回 02 但馬達不動  → 馬達未真正使能（'i' 查狀態）或堵轉保護中");
    Serial.println("  • 有動但角度不對    → 細分 (P_Pul) 不是 16，改 kPulsesPerRev");
}

// ============================================================================
// 幫助
// ============================================================================
static void printHelp() {
    Serial.println();
    Serial.println("=== X42S (Emm 固件) 步進馬達 測試 ===");
    Serial.println("  't' 一鍵自我診斷（不知道哪裡壞時先按這個）");
    Serial.println("  'e' 使能         'd' 去使能        'i' 讀使能狀態");
    Serial.printf("  'v' 速度(RPM)    目前 %u    'k' 加速度檔位 目前 %u\n", gVelRpm, gAccStep);
    Serial.println("  'a' 絕對角度(°)  'g' 由目前位置再轉角度(°)");
    Serial.println("  'h' 歸零(絕對零位)  'z' 把目前位置清零(設為0°)");
    Serial.println("  'r' 讀即時位置    's' 立即停止    'c' 清除堵轉保護");
    Serial.println("  '*' 掃描地址 1..255   '#' 掃描波特率×地址");
    Serial.printf("  'F' 切換位置幀格式（目前 %s）\n", gModeByte ? "13B 第二代" : "12B V4.2");
    Serial.println("  '?' 此幫助");
    Serial.println();
}

// ============================================================================
// setup / loop
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n[emm42_test] Booting...");

    // setTX/setRX 必須在 begin() 之前（執行中呼叫會 panic）
    Serial1.setTX(Pin::MOTOR_UART_TX);
    Serial1.setRX(Pin::MOTOR_UART_RX);
    Serial1.begin(115200); // UART0 → 馬達
    flushInput();

    printHelp();
}

void loop() {
    if(!Serial.available()) {
        return;
    }
    char c = static_cast<char>(Serial.read());
    flushInput();

    switch(c) {
    case 't': {
        selfTest();
        break;
    }
    case 'e': {
        Serial.println(">> enable");
        sendEnable(true);
        printRawReply(receiveFrame(100));
        break;
    }
    case 'd': {
        Serial.println(">> disable");
        sendEnable(false);
        printRawReply(receiveFrame(100));
        break;
    }
    case 'i': {
        Serial.println(">> read enable status");
        sendReadStatus();
        printRawReply(receiveFrame(100));
        break;
    }
    case 'v': {
        int32_t rpm = 0;
        if(!readIntInput(rpm, "速度 (RPM, 1-3000)：")) {
            break;
        }
        if(rpm < kVelMinRpm || rpm > kVelMaxRpm) {
            Serial.printf("  ⚠ %ld 超出範圍，夾到 1-3000\n", static_cast<long>(rpm));
            rpm = (rpm < kVelMinRpm) ? kVelMinRpm : kVelMaxRpm;
        }
        gVelRpm = static_cast<uint16_t>(rpm);
        Serial.printf(">> 速度 = %u RPM\n", gVelRpm);
        break;
    }
    case 'k': {
        int32_t acc = 0;
        if(!readIntInput(acc, "加速度檔位 (0-255)：")) {
            break;
        }
        if(acc < 0 || acc > 255) {
            Serial.println("  ⚠ 超出 0-255，忽略");
            break;
        }
        gAccStep = static_cast<uint8_t>(acc);
        Serial.printf(">> 加速度檔位 = %u\n", gAccStep);
        break;
    }
    case 'a': { // 絕對角度 → 到位
        int32_t deg = 0;
        if(!readIntInput(deg, "目標絕對角度 (°)：")) {
            break;
        }
        int32_t p = degToPulses(static_cast<float>(deg));
        Serial.print(">> 絕對位置 → ");
        Serial.print(deg);
        Serial.print("° = ");
        Serial.print(p);
        Serial.println(" 脈衝");
        sendPosition(p, 1, 0); // mode=1 絕對
        printRawReply(receiveFrame(100));
        break;
    }
    case 'g': { // 由目前位置再轉
        int32_t deg = 0;
        if(!readIntInput(deg, "增加角度 (°) 由現在位置起 [負=反向]：")) {
            break;
        }
        doRelativeMove(static_cast<float>(deg));
        break;
    }
    case 'h': { // 歸零：回絕對零位
        Serial.println(">> home: 回絕對位置零位 (9A/04)");
        sendHome();
        printRawReply(receiveFrame(200));
        break;
    }
    case 'z': { // 當前位置角度清零
        Serial.println(">> 清空當前位置角度 → 設為 0° (0A/6D)");
        sendClearPos();
        printRawReply(receiveFrame(100));
        break;
    }
    case 'r': {
        float pos = 0.0f;
        if(readPosition(pos)) {
            Serial.print(">> position = ");
            printFix1(pos);
            Serial.println("°");
        } else {
            Serial.println(">> position read FAILED");
        }
        break;
    }
    case 's': {
        Serial.println(">> stop");
        sendStop();
        printRawReply(receiveFrame(100));
        break;
    }
    case 'c': {
        Serial.println(">> clear clog protection");
        sendClogReset();
        printRawReply(receiveFrame(100));
        break;
    }
    case '*': {
        scanAddresses();
        break;
    }
    case '#': {
        scanBaudAndAddr();
        break;
    }
    case 'F': {
        gModeByte = !gModeByte;
        Serial.printf(">> FD 位置幀格式 = %s (%d bytes)\n",
                      gModeByte ? "13B 第二代（含運動模式）" : "12B V4.2（無運動模式）",
                      gModeByte ? 13 : 12);
        break;
    }
    case '?':
    default: {
        printHelp();
        break;
    }
    }
}
