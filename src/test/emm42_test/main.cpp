// ============================================================================
// emm42_test — 張大頭 X42S 步進馬達（Emm 固件）簡化控制測試 (RP2350)
//
// 用途：
//   用 RP2350 (Waveshare RP2350 Plus, Arduino-Pico) 直接以 UART 控制一顆
//   X42S 步進馬達，支援使能/去使能、絕對角度到位、相對當前位置轉角、歸零。
//
// 協定（依 ZDT_X42S 第二代手册，Emm 固件）：
//   - 幀： [地址][功能碼][參數…][尾碼 0x6B]，地址預設 = 1
//   - 位置單位為「脈衝」：1.8°步進 ×16細分 = 3200 脈衝 = 360°
//   - 返回狀態：02=正確、E2=參數/堵轉保護、EE=格式錯誤、9F=動作完成
//
// 接線（RP2350 UART0）：GP0=TX→馬達 T/B/L，GP1=RX←馬達 R/A/H，115200，共地。
//
// 按鍵（115200 USB Serial）：
//   'e' 使能            'd' 去使能
//   'a' 設定絕對角度(°) → 快速到位（相對零位）
//   'g' 設定角度(°) → 由目前位置再轉（相對即時位置）
//   'h' 歸零：回絕對位置零位
//   'z' 把「目前位置」角度清零 → 設為 0°（之後 a 的 0° = 現在這裡）
//   'r' 讀即時位置(°)     's' 立即停止
//   'c' 清除堵轉保護      '?' 此幫助
// ============================================================================
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
// 輔助碼
constexpr uint8_t AX_EN = 0xAB;
constexpr uint8_t AX_STOP = 0x98;
constexpr uint8_t AX_CLEARPOS = 0x6D;
constexpr uint8_t AX_CLOG = 0x52;
} // namespace X42S

// Emm 位置換算：1.8°步進 ×16細分 = 3200 脈衝 / 圈 = 360°
constexpr uint16_t kPulsesPerRev = 3200;
// 角度定位（快旋）速度 / 加速度檔位
constexpr uint16_t kAngleVelRpm = 3600; // ≤3000 Rpm（已達上限）
constexpr uint8_t kAngleAccStep = 250;  // 0-255，越大加速越快（g/a 共用）

// ============================================================================
// 緩衝（靜態分配，專案禁止 new/malloc）
// ============================================================================
static uint8_t sTxBuf[32];
static uint8_t sRxBuf[64];

// ============================================================================
// 底層收發
// ============================================================================
static void sendFrame(const uint8_t *buf, uint8_t len) {
    Serial1.write(buf, len);
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

// 印出原始位元組（回覆診斷用）
static void printRawReply(uint8_t n) {
    Serial.print("  reply (");
    Serial.print(n);
    Serial.print("B): ");
    for(uint8_t i = 0; i < n; i++) {
        Serial.printf("%02X ", sRxBuf[i]);
    }
    Serial.println();
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

// Emm 位置模式：[Addr][FD][方向][速度 2B][加速度档位][脈衝數 4B LE][運動模式 1B][同步][6B] = 13B
//   運動模式：0=相對上次目標、1=絕對(相對零位)、2=相對當前即時位置
static void sendPosition(int32_t pulses, uint8_t mode, uint8_t dir = 0) {
    uint32_t val = static_cast<uint32_t>(pulses);
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_POS;
    sTxBuf[2] = dir;
    sTxBuf[3] = static_cast<uint8_t>(kAngleVelRpm >> 8);
    sTxBuf[4] = static_cast<uint8_t>(kAngleVelRpm >> 0);
    sTxBuf[5] = kAngleAccStep;
    sTxBuf[6] = static_cast<uint8_t>(val >> 24);
    sTxBuf[7] = static_cast<uint8_t>(val >> 16);
    sTxBuf[8] = static_cast<uint8_t>(val >> 8);
    sTxBuf[9] = static_cast<uint8_t>(val >> 0);
    sTxBuf[10] = mode;
    sTxBuf[11] = 0; // 同步標誌
    sTxBuf[12] = X42S::TAIL;
    sendFrame(sTxBuf, 13);
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
// 掃描馬達地址：對地址 1..255 各送一幀 enable，找出會回覆的馬達位址。
//   用在「換了一顆馬達結果 e 沒回覆」——可能是新馬達的「地址」不是 1。
//   若波特率/協定不同則掃不到（那就查螢幕的 波特率/固件）。
// ============================================================================
static void scanAddresses() {
    Serial.println(">> 掃描地址 1..255（送 enable，找會回覆 'F3 02' 的馬達，約幾秒）...");
    for(uint16_t a = 1; a <= 255; a++) {
        uint8_t addr = static_cast<uint8_t>(a);
        sTxBuf[0] = addr;
        sTxBuf[1] = X42S::FD_EN;
        sTxBuf[2] = X42S::AX_EN;
        sTxBuf[3] = 0x01; // 使能
        sTxBuf[4] = 0x00;
        sTxBuf[5] = X42S::TAIL;
        sendFrame(sTxBuf, 6);

        uint8_t n = receiveFrame(18);
        bool ack = false;
        for(int i = 0; i + 3 < n && !ack; i++) {
            if(sRxBuf[i] == addr && sRxBuf[i + 1] == X42S::FD_EN && sRxBuf[i + 2] == 0x02) {
                ack = true;
            }
        }
        if(ack) {
            Serial.printf("  addr %u : ✓ 回覆（這顆馬達的位址）\n", a);
        }
    }
    Serial.println(">> 掃描完成（若一顆都沒印出 ✓，多半是波特率或固件不同，查螢幕）。");
}

// ============================================================================
// 掃描「波特率 × 地址」：逐一重設 UART 波特率，並在每波特率下掃地址 1..16。
//   用在地址掃描找不到馬達時 —— 新馬達可能被設成不同波特率。
//   掃完自動把 UART 恢復回 115200。
// ============================================================================
static void scanBaudAndAddr() {
    const uint32_t kBauds[] = {9600, 19200, 38400, 57600, 115200, 230400, 460800};
    const uint8_t kBaudCount = sizeof(kBauds) / sizeof(kBauds[0]);
    Serial.println(">> 掃描波特率×地址 1..16（送 enable 找馬達）...");

    for(uint8_t bi = 0; bi < kBaudCount; bi++) {
        uint32_t baud = kBauds[bi];
        Serial1.end();
        Serial1.begin(baud);
        Serial1.setTX(Pin::MOTOR_UART_TX);
        Serial1.setRX(Pin::MOTOR_UART_RX);

        bool found = false;
        for(uint16_t a = 1; a <= 64; a++) {
            uint8_t addr = static_cast<uint8_t>(a);
            sTxBuf[0] = addr;
            sTxBuf[1] = X42S::FD_EN;
            sTxBuf[2] = X42S::AX_EN;
            sTxBuf[3] = 0x01;
            sTxBuf[4] = 0x00;
            sTxBuf[5] = X42S::TAIL;
            sendFrame(sTxBuf, 6);

            uint8_t n = receiveFrame(18);
            for(int i = 0; i + 3 < n; i++) {
                if(sRxBuf[i] == addr && sRxBuf[i + 1] == X42S::FD_EN && sRxBuf[i + 2] == 0x02) {
                    Serial.printf("  ✓ FOUND baud=%lu addr=%u\n", (unsigned long)baud, a);
                    found = true;
                    break;
                }
            }
        }
        if(!found) {
            Serial.printf("  baud %lu : 無馬達回應\n", (unsigned long)baud);
        }
    }

    // 恢復 115200（預設）
    Serial1.end();
    Serial1.begin(115200);
    Serial1.setTX(Pin::MOTOR_UART_TX);
    Serial1.setRX(Pin::MOTOR_UART_RX);
    Serial.println(">> 掃描完成（已恢復 baud 115200）。");
}

// ============================================================================
// 讀即時位置 (Emm) — [Addr][36][回覆] [Addr][36][sign][4B LE][6B]
//   Emm：角度(°) = raw × 360 / 65536（0-65535 為一圈），sign 為正/負
// ============================================================================
static bool readPosition(float &degOut) {
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_READPOS;
    sTxBuf[2] = X42S::TAIL;
    sendFrame(sTxBuf, 3);

    uint8_t n = receiveFrame(100);
    // 跳過 echo：找最後一幀 [addr][0x36]
    int idx = -1;
    for(int i = 0; i + 1 < n; i++) {
        if(sRxBuf[i] == X42S::PROTO_ADDR && sRxBuf[i + 1] == X42S::FD_READPOS) {
            idx = i;
        }
    }
    // 資料幀應為 [addr][36][sign][4B][6B] = 8 bytes
    if(idx < 0 || (n - idx) != 8 || sRxBuf[n - 1] != X42S::TAIL) {
        printRawReply(n);
        return false;
    }
    uint32_t raw = (static_cast<uint32_t>(sRxBuf[idx + 3]) << 24) |
                   (static_cast<uint32_t>(sRxBuf[idx + 4]) << 16) |
                   (static_cast<uint32_t>(sRxBuf[idx + 5]) << 8) |
                   (static_cast<uint32_t>(sRxBuf[idx + 6]) << 0);
    double deg = static_cast<double>(raw) * 360.0 / 65536.0;
    if(sRxBuf[idx + 2] != 0) {
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
// 幫助
// ============================================================================
static void printHelp() {
    Serial.println();
    Serial.println("=== X42S (Emm 固件) 步進馬達 測試 ===");
    Serial.println("  'e' 使能         'd' 去使能");
    Serial.println("  'a' 設定絕對角度(°) 快速到位    'g' 由目前位置再轉角度(°)");
    Serial.println("  'h' 歸零(回絕對零位)  'z' 把目前位置角度清零(設為0°)");
    Serial.println("  'r' 讀即時位置    's' 立即停止    'c' 清除堵轉保護");
    Serial.println("  '*' 掃描馬達地址 1..16（換馬達沒回覆時用）");
    Serial.println("  '#' 掃描波特率×地址（地址掃不到時用，掃完恢復 115200）");
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

    Serial1.begin(115200); // UART0 → 馬達
    Serial1.setTX(Pin::MOTOR_UART_TX);
    Serial1.setRX(Pin::MOTOR_UART_RX);

    printHelp();
}

void loop() {
    if(!Serial.available()) {
        return;
    }
    char c = static_cast<char>(Serial.read());
    flushInput();

    switch(c) {
    case 'e': {
        Serial.println(">> enable");
        sendEnable(true);
        uint8_t n = receiveFrame(100);
        printRawReply(n);
        break;
    }
    case 'd': {
        Serial.println(">> disable");
        sendEnable(false);
        uint8_t n = receiveFrame(100);
        printRawReply(n);
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
        uint8_t n = receiveFrame(100);
        printRawReply(n);
        break;
    }
    case 'g': { // 由目前位置再轉
        int32_t deg = 0;
        if(!readIntInput(deg, "增加角度 (°) 由現在位置起 [負=反向]：")) {
            break;
        }
        int32_t p = degToPulses(static_cast<float>(deg));
        Serial.print(">> 相對即時位置 → ");
        Serial.print(deg);
        Serial.print("° = ");
        Serial.print(p);
        Serial.println(" 脈衝");
        sendPosition(p, 2, 0); // mode=2 相對即時位置
        uint8_t n = receiveFrame(100);
        printRawReply(n);
        break;
    }
    case 'h': { // 歸零：回絕對零位
        Serial.println(">> home: 回絕對位置零位 (9A/04)");
        sendHome();
        uint8_t n = receiveFrame(200);
        printRawReply(n);
        break;
    }
    case 'z': { // 當前位置角度清零
        Serial.println(">> 清空當前位置角度 → 設為 0° (0A/6D)");
        sendClearPos();
        uint8_t n = receiveFrame(100);
        printRawReply(n);
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
        uint8_t n = receiveFrame(100);
        printRawReply(n);
        break;
    }
    case 'c': {
        Serial.println(">> clear clog protection");
        sendClogReset();
        uint8_t n = receiveFrame(100);
        printRawReply(n);
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
    case '?':
    default: {
        printHelp();
        break;
    }
    }
}
