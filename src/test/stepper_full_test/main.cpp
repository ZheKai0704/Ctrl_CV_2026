// ============================================================================
// stepper_full_test — 步進馬達（張大頭 X42S / Emm 韌體）全指令測試 (RP2350)
//
// 腳位：PIO 模擬 UART（與 src/shoot/main.cpp 仰角步進同腳位）
//   GP14 = TX → 馬達 R/A/H（或 T/B/L）
//   GP15 = RX ← 馬達 T/B/L（或 R/A/H）
//   115200 8N1，務必共地
//
// 此馬達可用的所有指令（Emm 韌體，幀 = [Addr][功能碼][參數…][尾碼 0x6B]，Addr=1）：
//   0xF3 AB 01/00 : 使能 / 去使能（回 02）
//   0xF6          : 速度模式（持續轉 / 停）
//   0xFD          : 位置模式（mode 0=相對上次 1=絕對 2=相對當前 3=絕對即時修正）
//   0xFE 98       : 立即停止
//   0x9A 04       : 觸發回零（回絕對零位）
//   0x0A 6D       : 當前位置角度清零（設為 0°）
//   0x0E 52       : 解除堵轉保護
//   0x36          : 讀即時位置（65536 = 360°）
//   0x3A          : 讀狀態旗標（bit0 使能 bit1 到位 bit2/3 堵轉 bit4/5 限位 bit7 掉電）
//   0x37          : 讀位置誤差（65536 = 360°）
//   0x27 / 0x26   : 讀相電流 / 總線電流（mA）
//   0x42 6C       : 讀驅動配置（28B，含堵轉保護參數）
//   0x48 D1 STORE : 寫驅動配置（STORE=1 掉電保存，本測試只改堵轉電流）
//   回覆碼：02=正確  E2=參數錯誤/堵轉  EE=格式錯誤  9F=到位
//   脈衝換算：1.8° × 16 細分 = 3200 脈衝/圈（1 脈衝 = 0.1125°，實測確認）
//
// 按鍵（USB Serial, 115200）：
//   't' 一鍵自我診斷（使能→讀位置→+90°→-90°）
//   'e' 使能    'd' 去使能   'i' 讀使能狀態(0x3A)
//   'v' 速度(RPM)  'k' 加速度檔位(0-255)
//   'a' 絕對角度(°)  'g' 相對角度(°)  'r' 即時修正角度(°，運動中可更新)
//   's' 立即停止  'h' 歸零  'z' 位置清零  'c' 清堵轉保護
//   'm' 速度模式持續轉  'n' 速度模式停
//   'p' 讀位置  'q' 讀狀態旗標  'x' 讀位置誤差  'y' 讀相/總線電流
//   'o' 讀驅動配置  'C' 改堵轉保護電流(mA)
//   'F' 切 FD 位置幀格式 13B/12B
//   '*' 掃描地址  '#' 掃描波特率×地址
//   '?' 幫助
// ============================================================================

#include <Arduino.h>

// ============================================================================
// 腳位（PIO 模擬 UART）
// ============================================================================
namespace Pin {
constexpr uint ELEV_UART_TX = 14; // GP14 → 馬達 R/A/H 或 T/B/L
constexpr uint ELEV_UART_RX = 15; // GP15 ← 馬達 T/B/L 或 R/A/H
constexpr uint HW_UART_TX = 0;    // GP0 → 備用：硬體 UART0（emm42_test 舊接法）
constexpr uint HW_UART_RX = 1;    // GP1 ← 備用
} // namespace Pin

static SerialPIO motorSerial(Pin::ELEV_UART_TX, Pin::ELEV_UART_RX);
// 硬體 UART0 用既有的全域 Serial1 實例（SerialUART 無預設建構子）

// ============================================================================
// 匯流排選擇：開機自動探測馬達接在哪組腳位
//   false = PIO 模擬 UART（GP14 TX / GP15 RX）
//   true  = 硬體 UART0（GP0 TX / GP1 RX，emm42_test 舊接法）
// ============================================================================
static bool gUseHwUart = false;
static Stream *gBus = nullptr;

static void busSelect(bool hw) {
    if(gUseHwUart && !hw) { // 要從硬體 UART 切回 PIO
        Serial1.end();
    }
    if(!gUseHwUart && hw) { // 要從 PIO 切到硬體 UART
        motorSerial.end();
    }
    gUseHwUart = hw;
    if(hw) {
        // SerialUART 已 begin() 時呼叫 setTX/setRX 會 panic，先 end()
        Serial1.end();
        Serial1.setTX(Pin::HW_UART_TX);
        Serial1.setRX(Pin::HW_UART_RX);
        Serial1.begin(115200);
        gBus = &Serial1;
    } else {
        motorSerial.begin(115200);
        gBus = &motorSerial;
    }
    while(gBus->available() > 0) {
        gBus->read();
    }
}

// ============================================================================
// 協定常數（Emm 韌體）
// ============================================================================
namespace X42S {
constexpr uint8_t PROTO_ADDR = 0x01;
constexpr uint8_t TAIL = 0x6B;
// 功能碼
constexpr uint8_t FD_EN = 0xF3;       // 使能/去使能（輔助碼 AB）
constexpr uint8_t FD_VEL = 0xF6;      // 速度模式
constexpr uint8_t FD_POS = 0xFD;      // 位置模式
constexpr uint8_t FD_STOP = 0xFE;     // 立即停止（輔助碼 98）
constexpr uint8_t FD_HOME = 0x9A;     // 觸發回零
constexpr uint8_t FD_CLEARPOS = 0x0A; // 當前位置角度清零（輔助碼 6D）
constexpr uint8_t FD_CLOG = 0x0E;     // 解除堵轉（輔助碼 52）
constexpr uint8_t FD_STATUS = 0x3A;   // 讀狀態旗標
constexpr uint8_t FD_READPOS = 0x36;  // 讀即時位置
constexpr uint8_t FD_POSERR = 0x37;   // 讀位置誤差
constexpr uint8_t FD_PHASECUR = 0x27; // 讀相電流
constexpr uint8_t FD_BUSCUR = 0x26;   // 讀總線電流
constexpr uint8_t FD_READCFG = 0x42;  // 讀驅動配置（校驗 0x6C）
constexpr uint8_t FD_WRITECFG = 0x48; // 寫驅動配置（校驗 0xD1）
// 輔助碼
constexpr uint8_t AX_EN = 0xAB;
constexpr uint8_t AX_STOP = 0x98;
constexpr uint8_t AX_CLEARPOS = 0x6D;
constexpr uint8_t AX_CLOG = 0x52;
constexpr uint8_t CFG_READ_CHK = 0x6C;
constexpr uint8_t CFG_WRITE_CHK = 0xD1;
// 回覆碼
constexpr uint8_t RP_OK = 0x02;
constexpr uint8_t RP_PARAM = 0xE2;
constexpr uint8_t RP_FMT = 0xEE;
constexpr uint8_t RP_DONE = 0x9F;
} // namespace X42S

// 脈衝換算（實測確認）：3200 脈衝/圈，1 脈衝 = 0.1125°
constexpr uint16_t kPulsesPerRev = 3200;

// 方向位元組：脈衝數只吃絕對值，方向由 dir 決定（實測若顛倒對調兩值）
constexpr uint8_t kDirDown = 0x01;
constexpr uint8_t kDirUp = 0x00;

// 可調參數
constexpr uint16_t kVelMinRpm = 1;
constexpr uint16_t kVelMaxRpm = 3000;
static uint16_t gVelRpm = 300;
static uint8_t gAccStep = 200; // 0-255（0 = 無加減速）
// FD 位置幀格式：true = 13B（第二代，含運動模式）；false = 12B（V4.2 舊 Emm）
static bool gModeByte = true;

// ============================================================================
// 緩衝（靜態分配，專案禁止 new/malloc）
// ============================================================================
static uint8_t sTxBuf[40];
static uint8_t sRxBuf[40];
static uint8_t sLastTx[40];
static uint8_t sLastTxLen = 0;
static bool sEchoGuideShown = false;

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
    memcpy(sLastTx, buf, len);
    sLastTxLen = len;
    gBus->write(buf, len);
    gBus->flush();
    printHex(gUseHwUart ? "  TX[GP0/1]" : "  TX[GP14/15]", buf, len);
}

static uint8_t receiveFrame(uint32_t timeoutMs) {
    uint8_t i = 0;
    uint32_t lastByteMs = millis();
    while(millis() - lastByteMs < timeoutMs) {
        while(gBus->available() > 0) {
            if(i < sizeof(sRxBuf)) {
                sRxBuf[i++] = static_cast<uint8_t>(gBus->read());
                lastByteMs = millis();
            } else {
                gBus->read();
            }
        }
    }
    return i;
}

static void flushInput() {
    while(gBus->available() > 0) {
        gBus->read();
    }
}

static const char *replyCodeText(uint8_t code) {
    switch(code) {
    case X42S::RP_OK:
        return "02 = 命令正確";
    case X42S::RP_PARAM:
        return "E2 = 參數錯誤或堵轉保護中";
    case X42S::RP_FMT:
        return "EE = 格式錯誤（試 'F' 切 12B）";
    case X42S::RP_DONE:
        return "9F = 位置到位";
    default:
        return "未知回覆碼";
    }
}

static uint8_t echoPrefixLen(uint8_t n) {
    if(n == 0 || sLastTxLen == 0) {
        return 0;
    }
    const uint8_t cmp = (n < sLastTxLen) ? n : sLastTxLen;
    if(cmp < 3) {
        return 0;
    }
    return (memcmp(sRxBuf, sLastTx, cmp) == 0) ? cmp : 0;
}

static void printEchoGuide() {
    Serial.println("  ⚠ 收到的位元組與 TX 完全相同 → 回音（loopback），不是馬達回覆");
    if(sEchoGuideShown) {
        return;
    }
    sEchoGuideShown = true;
    Serial.println("    常見原因：GP14/GP15 短路、馬達端 R/T 並接、RS485 半雙工回灌、");
    Serial.println("    或馬達 P_Serial 不是 UART 模式。驗證：拔掉馬達三條線再送一次命令。");
}

static void printRawReply(uint8_t n) {
    if(n == 0) {
        Serial.println("  reply (0B): 無回應！檢查 TX/RX 對調、共地、波特率、P_Serial=UART");
        return;
    }
    printHex("  reply", sRxBuf, n);
    const uint8_t echoLen = echoPrefixLen(n);
    if(echoLen > 0) {
        printEchoGuide();
    }
    bool decoded = false;
    for(uint8_t i = echoLen; i + 2 < n; i++) {
        if(sRxBuf[i] != X42S::PROTO_ADDR) {
            continue;
        }
        const uint8_t func = sRxBuf[i + 1];
        const uint8_t code = sRxBuf[i + 2];
        if(func == 0x00 || code == X42S::TAIL) {
            continue;
        }
        Serial.printf("  → addr=%u func=%02X 回覆碼=%02X : %s\n", sRxBuf[i], func, code,
                      replyCodeText(code));
        decoded = true;
    }
    if(!decoded && echoLen == 0) {
        Serial.println("  → 無法辨識的回覆格式");
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
    sTxBuf[4] = 0x00;
    sTxBuf[5] = X42S::TAIL;
    sendFrame(sTxBuf, 6);
}

// 速度模式：[Addr][F6][dir][vel 2B][acc][sync][6B]，vel=0 停止
static void sendVelocity(uint16_t rpm, uint8_t dir) {
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_VEL;
    sTxBuf[2] = dir;
    sTxBuf[3] = static_cast<uint8_t>(rpm >> 8);
    sTxBuf[4] = static_cast<uint8_t>(rpm);
    sTxBuf[5] = gAccStep;
    sTxBuf[6] = 0x00;
    sTxBuf[7] = X42S::TAIL;
    sendFrame(sTxBuf, 8);
}

// 位置模式：[Addr][FD][dir][vel 2B][acc][pulses 4B LE][mode][sync][6B]
//   mode：0=相對上次目標、1=絕對(相對零位)、2=相對當前即時位置、3=絕對即時修正
static void sendPosition(uint32_t pulses, uint8_t dir, uint8_t mode) {
    uint8_t i = 0;
    sTxBuf[i++] = X42S::PROTO_ADDR;
    sTxBuf[i++] = X42S::FD_POS;
    sTxBuf[i++] = dir;
    sTxBuf[i++] = static_cast<uint8_t>(gVelRpm >> 8);
    sTxBuf[i++] = static_cast<uint8_t>(gVelRpm);
    sTxBuf[i++] = gAccStep;
    sTxBuf[i++] = static_cast<uint8_t>(pulses >> 24);
    sTxBuf[i++] = static_cast<uint8_t>(pulses >> 16);
    sTxBuf[i++] = static_cast<uint8_t>(pulses >> 8);
    sTxBuf[i++] = static_cast<uint8_t>(pulses);
    if(gModeByte) {
        sTxBuf[i++] = mode;
    }
    sTxBuf[i++] = 0x00;
    sTxBuf[i++] = X42S::TAIL;
    sendFrame(sTxBuf, i);
}

static void sendStop() {
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_STOP;
    sTxBuf[2] = X42S::AX_STOP;
    sTxBuf[3] = 0x00;
    sTxBuf[4] = X42S::TAIL;
    sendFrame(sTxBuf, 5);
}

static void sendHome() {
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_HOME;
    sTxBuf[2] = 0x04; // 回零模式：回到絕對位置坐標零點
    sTxBuf[3] = 0x00;
    sTxBuf[4] = X42S::TAIL;
    sendFrame(sTxBuf, 5);
}

static void sendClearPos() {
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_CLEARPOS;
    sTxBuf[2] = X42S::AX_CLEARPOS;
    sTxBuf[3] = X42S::TAIL;
    sendFrame(sTxBuf, 4);
}

static void sendClogReset() {
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_CLOG;
    sTxBuf[2] = X42S::AX_CLOG;
    sTxBuf[3] = X42S::TAIL;
    sendFrame(sTxBuf, 4);
}

static void sendReadCmd(uint8_t func) {
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = func;
    sTxBuf[2] = X42S::TAIL;
    sendFrame(sTxBuf, 3);
}

// ============================================================================
// 讀取解析
// ============================================================================
// 狀態旗標：回傳原始 byte，失敗 -1
static int readStatusFlags(bool verbose) {
    flushInput();
    sendReadCmd(X42S::FD_STATUS);
    uint8_t n = receiveFrame(100);
    if(n < 4 || sRxBuf[0] != X42S::PROTO_ADDR || sRxBuf[1] != X42S::FD_STATUS) {
        if(verbose) {
            printHex("  reply", sRxBuf, n);
            Serial.println("  → 無回應或格式錯");
        }
        return -1;
    }
    const uint8_t f = sRxBuf[2];
    if(verbose) {
        Serial.printf(">> 狀態旗標 = 0x%02X：", f);
        Serial.printf("%s / 到位=%d / ", (f & 0x01) ? "已使能" : "未使能", (f >> 1) & 1);
        Serial.printf("堵轉標誌=%d / 堵轉保護中=%d", (f >> 2) & 1, (f >> 3) & 1);
        if(f & 0x30) {
            Serial.printf(" / 限位 L=%d R=%d", (f >> 4) & 1, (f >> 5) & 1);
        }
        if(f & 0x80) {
            Serial.print(" / 掉電標誌");
        }
        Serial.println();
    }
    return f;
}

// 讀 32-bit 數值（[Addr][func][sign][4B] / 或不帶 sign 的電流 2B）
static bool readValue32(uint8_t func, int32_t &valOut, bool hasSign) {
    flushInput();
    sendReadCmd(func);
    uint8_t n = receiveFrame(100);
    // 找最後一幀 [addr][func]
    int idx = -1;
    for(uint8_t i = 0; i + 1 < n; i++) {
        if(sRxBuf[i] == X42S::PROTO_ADDR && sRxBuf[i + 1] == func) {
            idx = i;
        }
    }
    if(idx < 0 || (n - idx) < (hasSign ? 7 : 5)) {
        printHex("  reply", sRxBuf, n);
        return false;
    }
    const uint8_t base = idx + 2;
    if(hasSign) {
        const uint32_t raw = (static_cast<uint32_t>(sRxBuf[base + 1]) << 24) |
                             (static_cast<uint32_t>(sRxBuf[base + 2]) << 16) |
                             (static_cast<uint32_t>(sRxBuf[base + 3]) << 8) |
                             static_cast<uint32_t>(sRxBuf[base + 4]);
        valOut = sRxBuf[base] ? -static_cast<int32_t>(raw) : static_cast<int32_t>(raw);
    } else {
        valOut = static_cast<int32_t>((static_cast<uint32_t>(sRxBuf[base]) << 8) |
                                      sRxBuf[base + 1]);
    }
    return true;
}

// 讀即時位置（°）：raw × 360 / 65536
static bool readPositionDeg(float &degOut) {
    int32_t v = 0;
    if(!readValue32(X42S::FD_READPOS, v, true)) {
        return false;
    }
    degOut = static_cast<float>(v) * 360.0f / 65536.0f;
    return true;
}

// 讀相電流 / 總線電流（mA）
static void readCurrents() {
    int32_t phase = 0;
    int32_t bus = 0;
    bool okP = readValue32(X42S::FD_PHASECUR, phase, false);
    bool okB = readValue32(X42S::FD_BUSCUR, bus, false);
    if(okP || okB) {
        Serial.printf(">> 相電流 = %ld mA，總線電流 = %ld mA\n", static_cast<long>(phase),
                      static_cast<long>(bus));
        Serial.println("   （相電流 ≈ 堵轉電流設定值且位置不動 → 堵轉）");
    }
}

// 讀驅動配置（28B）：offset 19 堵轉模式、20:22 速度、22:24 電流 mA、24:26 時間 ms
static bool readConfig(uint8_t cfgOut[28]) {
    flushInput();
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_READCFG;
    sTxBuf[2] = X42S::CFG_READ_CHK;
    sTxBuf[3] = X42S::TAIL;
    sendFrame(sTxBuf, 4);
    uint8_t n = receiveFrame(150);
    if(n < 31 || sRxBuf[0] != X42S::PROTO_ADDR || sRxBuf[1] != X42S::FD_READCFG) {
        printHex("  reply", sRxBuf, n);
        return false;
    }
    memcpy(cfgOut, &sRxBuf[2], 28);
    return true;
}

static void printConfig() {
    uint8_t cfg[28];
    if(!readConfig(cfg)) {
        return;
    }
    printHex("  cfg", cfg, 28);
    Serial.printf(">> 堵轉保護：模式=%d，速度=%u RPM，電流=%u mA，時間=%u ms\n", cfg[19],
                  (static_cast<uint16_t>(cfg[20]) << 8) | cfg[21],
                  (static_cast<uint16_t>(cfg[22]) << 8) | cfg[23],
                  (static_cast<uint16_t>(cfg[24]) << 8) | cfg[25]);
}

// 寫堵轉保護電流（0x48，STORE=1 掉電保存）
static bool writeStallCurrent(uint8_t cfg[28], uint16_t mA) {
    cfg[22] = static_cast<uint8_t>(mA >> 8);
    cfg[23] = static_cast<uint8_t>(mA & 0xFF);
    uint8_t i = 0;
    sTxBuf[i++] = X42S::PROTO_ADDR;
    sTxBuf[i++] = X42S::FD_WRITECFG;
    sTxBuf[i++] = X42S::CFG_WRITE_CHK;
    sTxBuf[i++] = 0x01; // STORE：掉電保存
    for(uint8_t k = 0; k < 28; k++) {
        sTxBuf[i++] = cfg[k];
    }
    sTxBuf[i++] = X42S::TAIL;
    sendFrame(sTxBuf, i);
    uint8_t n = receiveFrame(150);
    if(n >= 4 && sRxBuf[0] == X42S::PROTO_ADDR && sRxBuf[2] == X42S::RP_OK) {
        Serial.printf(">> 堵轉電流已寫入 %u mA（掉電保存）\n", mA);
        return true;
    }
    printHex("  reply", sRxBuf, n);
    return false;
}

// ============================================================================
// 工具
// ============================================================================
static int32_t degToPulses(float deg) {
    return static_cast<int32_t>(lroundf(deg * static_cast<float>(kPulsesPerRev) / 360.0f));
}

static void printFix1(float v) {
    long v10 = static_cast<long>(v * 10.0f + (v < 0 ? -0.5f : 0.5f));
    bool neg = v10 < 0;
    unsigned long a = neg ? static_cast<unsigned long>(-v10) : static_cast<unsigned long>(v10);
    if(neg) {
        Serial.print('-');
    }
    Serial.print(a / 10);
    Serial.print('.');
    Serial.print(a % 10);
}

static bool readIntInput(int32_t &val, const char *prompt, uint32_t waitMs = 10000) {
    uint32_t t0 = millis();
    while(millis() - t0 < 40) { // 清 USB 殘留
        if(Serial.available()) {
            Serial.read();
            t0 = millis();
        }
    }
    while(Serial.available()) {
        Serial.read();
    }
    Serial.print(prompt);
    long v = 0;
    bool neg = false;
    bool got = false;
    t0 = millis();
    while(millis() - t0 < waitMs) {
        if(Serial.available()) {
            char ch = static_cast<char>(Serial.read());
            if(ch == '\n' || ch == '\r') {
                if(got) {
                    val = neg ? -v : v;
                    Serial.println();
                    return true;
                }
            } else if(ch == '-' && !got) {
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

// 相對移動：mode=2 → 等停 → 印位置
static void doRelativeMove(float deg) {
    const int32_t p = degToPulses(deg);
    Serial.print(">> 相對移動 ");
    printFix1(deg);
    Serial.print("° = ");
    Serial.print(p);
    Serial.printf(" 脈衝 @ %u RPM\n", gVelRpm);
    sendPosition(p < 0 ? static_cast<uint32_t>(-p) : static_cast<uint32_t>(p),
                 p < 0 ? kDirUp : kDirDown, 2);
    printRawReply(receiveFrame(100));

    float now = 0.0f;
    bool moved = false;
    uint32_t t0 = millis();
    while(millis() - t0 < 4000) {
        delay(150);
        float pos = 0.0f;
        if(readPositionDeg(pos)) {
            moved = true;
            now = pos;
        }
    }
    Serial.print(">> 移動後位置 = ");
    printFix1(now);
    Serial.println("°");
    if(!moved) {
        Serial.println("   ⚠ 讀不到位置回覆，無法判斷是否轉動");
    }
}

// ============================================================================
// 掃描
// ============================================================================
static bool probeOne(uint8_t addr, uint32_t ackWaitMs) {
    sTxBuf[0] = addr;
    sTxBuf[1] = X42S::FD_EN;
    sTxBuf[2] = X42S::AX_EN;
    sTxBuf[3] = 0x01;
    sTxBuf[4] = 0x00;
    sTxBuf[5] = X42S::TAIL;
    gBus->write(sTxBuf, 6);
    gBus->flush();
    uint8_t n = receiveFrame(ackWaitMs);
    for(uint8_t i = 0; i + 2 < n; i++) {
        if(sRxBuf[i] == addr && sRxBuf[i + 1] == X42S::FD_EN && sRxBuf[i + 2] == X42S::RP_OK) {
            return true;
        }
    }
    return false;
}

static void scanAddresses() {
    Serial.println(">> 掃描地址 1..255 @ 115200（送 enable 找回覆）...");
    uint8_t found = 0;
    for(uint16_t a = 1; a <= 255; a++) {
        if(probeOne(static_cast<uint8_t>(a), 12)) {
            Serial.printf("  addr %u : ✓ 有回覆\n", a);
            found++;
        }
    }
    if(found == 0) {
        Serial.println(">> 沒有馬達回覆。先確認 'e' 是否有回覆。");
    } else {
        Serial.printf(">> 掃描完成，共 %u 顆。\n", found);
    }
}

static void scanBaudAndAddr() {
    // SerialPIO 不支援執行中改波特率，此命令僅在改程式碼波特率後使用
    Serial.println(">> ⚠ SerialPIO 建立後無法改波特率；本命令列出常見波特率供手動測試：");
    Serial.println("   9600 / 19200 / 38400 / 57600 / 115200 / 230400 / 460800");
    Serial.println("   （Emm 出廠預設 115200）");
}

// ============================================================================
// 開機自動探測：在兩組腳位各送一次 enable，看哪邊有馬達回覆
// ============================================================================
static void detectMotorPort() {
    Serial.println(">> 探測馬達腳位...");
    busSelect(false);
    Serial.println("   [1] PIO UART  GP14=TX / GP15=RX");
    if(probeOne(X42S::PROTO_ADDR, 120)) {
        Serial.println("   ✓ 馬達回覆 → 使用 GP14/15");
        return;
    }
    Serial.println("   無回應");
    busSelect(true);
    Serial.println("   [2] 硬體 UART0 GP0=TX / GP1=RX（emm42_test 舊接法）");
    if(probeOne(X42S::PROTO_ADDR, 120)) {
        Serial.println("   ✓ 馬達回覆 → 使用 GP0/1");
        return;
    }
    Serial.println("   無回應");
    // 兩邊都沒回覆：預設回 GP14/15，靠使用者接線修正
    Serial.println(">> ⚠ 兩組腳位都沒回應 → 查：TX/RX 對調、共地、馬達電源、P_Serial=UART。");
    Serial.println("   提示：兩組 TX/RX 互換再試（馬達端 R/A/H 與 T/B/L 接反很常見）。");
    busSelect(false);
}

// ============================================================================
// 原始 RX 監視器：印出任何收到的位元組（查接線 / 回音用）
// ============================================================================
static bool sRxMonitor = false;
static void rxMonitorUpdate() {
    if(!sRxMonitor) {
        return;
    }
    while(gBus->available() > 0) {
        Serial.printf("%02X ", static_cast<uint8_t>(gBus->read()));
    }
}

// 手動切換腳位組
static void switchPort(bool hw) {
    busSelect(hw);
    Serial.printf(">> 匯流排 = %s\n", hw ? "硬體 UART0 (GP0=TX, GP1=RX)"
                                       : "PIO UART (GP14=TX, GP15=RX)");
}

// ============================================================================
// 自我診斷
// ============================================================================
static void selfTest() {
    Serial.println("\n========== 自我診斷開始 ==========");
    Serial.printf("設定：速度 %u RPM、加速度 %u、FD 幀 %s、addr=%u\n", gVelRpm, gAccStep,
                  gModeByte ? "13B" : "12B", X42S::PROTO_ADDR);

    Serial.println("[1] 使能");
    sendEnable(true);
    uint8_t n = receiveFrame(100);
    printRawReply(n);
    if(n == 0) {
        Serial.println("!! 使能沒回覆 → UART/接線/波特率問題，先修硬體");
        return;
    }
    if(echoPrefixLen(n) > 0) {
        Serial.println("!! 只收到回音 → 先解決 loopback，後面測試沒意義");
        return;
    }

    Serial.println("[2] 讀狀態旗標");
    readStatusFlags(true);

    Serial.println("[3] 讀即時位置");
    float p0 = 0.0f;
    if(!readPositionDeg(p0)) {
        Serial.println("!! 讀不到位置 → 回覆格式問題");
    } else {
        Serial.print("    目前位置 = ");
        printFix1(p0);
        Serial.println("°");
    }

    Serial.println("[4] 相對 +90°");
    doRelativeMove(90.0f);
    Serial.println("[5] 相對 -90°（回原位）");
    doRelativeMove(-90.0f);

    Serial.println("[6] 清堵轉保護");
    sendClogReset();
    printRawReply(receiveFrame(100));

    Serial.println("========== 自我診斷結束 ==========");
    Serial.println("判讀：無回覆→掃地址/查接線；EE→'F'切12B；E2→降速或清堵轉；"
                   "02但不動→查使能/堵轉；動但角度不對→細分非16，改 kPulsesPerRev");
}

// ============================================================================
// 幫助 / setup / loop
// ============================================================================
static void printHelp() {
    Serial.println();
    Serial.println("=== 步進馬達全指令測試（X42S / Emm，GP14=TX GP15=RX 115200）===");
    Serial.println("  't' 一鍵自我診斷");
    Serial.println("  'e' 使能   'd' 去使能   'i' 讀狀態旗標(0x3A)");
    Serial.printf("  'v' 速度(RPM,目前 %u)   'k' 加速度檔位(目前 %u)\n", gVelRpm, gAccStep);
    Serial.println("  'a' 絕對角度(°)   'g' 相對角度(°)   'r' 即時修正絕對角度(°,mode=3)");
    Serial.println("  's' 立即停止   'h' 歸零   'z' 位置清零   'c' 清堵轉保護");
    Serial.printf("  'm' 速度模式正轉(%u RPM)   'n' 速度模式停\n", gVelRpm);
    Serial.println("  'p' 讀位置(0x36)   'x' 讀位置誤差(0x37)   'y' 讀電流(0x27/26)");
    Serial.println("  'o' 讀驅動配置(0x42)   'C' 改堵轉電流(mA, 0x48 STORE=1)");
    Serial.printf("  'F' 切位置幀格式（目前 %s）\n", gModeByte ? "13B 第二代" : "12B V4.2");
    Serial.println("  '*' 掃描地址   '?' 幫助");
    Serial.printf("  'u' 手動切 GP14/15 ↔ GP0/1（目前 %s）\n",
                  gUseHwUart ? "GP0/1 硬體 UART" : "GP14/15 PIO UART");
    Serial.printf("  'M' 原始 RX 監視器（目前 %s）\n", sRxMonitor ? "ON" : "OFF");
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n[stepper_full_test] Booting...");
    motorSerial.begin(115200); // 先建立預設匯流排（busSelect 會管理）
    gBus = &motorSerial;
    flushInput();
    detectMotorPort();
    printHelp();
}

void loop() {
    rxMonitorUpdate();

    if(!Serial.available()) {
        return;
    }
    char c = static_cast<char>(Serial.read());
    flushInput();

    switch(c) {
    case 't':
        selfTest();
        break;
    case 'e':
        Serial.println(">> enable");
        sendEnable(true);
        printRawReply(receiveFrame(100));
        break;
    case 'd':
        Serial.println(">> disable");
        sendEnable(false);
        printRawReply(receiveFrame(100));
        break;
    case 'i': {
        Serial.println(">> 讀狀態旗標 (0x3A)");
        readStatusFlags(true);
        break;
    }
    case 'v': {
        int32_t rpm = 0;
        if(!readIntInput(rpm, "速度 (RPM)：")) {
            break;
        }
        if(rpm < kVelMinRpm) {
            rpm = kVelMinRpm;
        }
        if(rpm > kVelMaxRpm) {
            rpm = kVelMaxRpm;
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
    case 'a': {
        int32_t deg = 0;
        if(!readIntInput(deg, "目標絕對角度 (°)：")) {
            break;
        }
        const int32_t p = degToPulses(static_cast<float>(deg));
        Serial.printf(">> 絕對位置 → %ld° = %ld 脈衝\n", static_cast<long>(deg),
                      static_cast<long>(p));
        sendPosition(p < 0 ? static_cast<uint32_t>(-p) : static_cast<uint32_t>(p),
                     p < 0 ? kDirUp : kDirDown, 1);
        printRawReply(receiveFrame(100));
        break;
    }
    case 'g': {
        int32_t deg = 0;
        if(!readIntInput(deg, "相對角度 (°) [負=反向]：")) {
            break;
        }
        doRelativeMove(static_cast<float>(deg));
        break;
    }
    case 'r': {
        int32_t deg = 0;
        if(!readIntInput(deg, "即時修正絕對角度 (°)：")) {
            break;
        }
        const int32_t p = degToPulses(static_cast<float>(deg));
        Serial.printf(">> 即時修正 → %ld° = %ld 脈衝（mode=3，運動中可更新）\n",
                      static_cast<long>(deg), static_cast<long>(p));
        sendPosition(p < 0 ? static_cast<uint32_t>(-p) : static_cast<uint32_t>(p),
                     p < 0 ? kDirUp : kDirDown, 3);
        printRawReply(receiveFrame(100));
        break;
    }
    case 'm':
        Serial.printf(">> 速度模式正轉 %u RPM\n", gVelRpm);
        sendVelocity(gVelRpm, kDirDown);
        printRawReply(receiveFrame(100));
        break;
    case 'n':
        Serial.println(">> 速度模式停（0 RPM）");
        sendVelocity(0, kDirDown);
        printRawReply(receiveFrame(100));
        break;
    case 's':
        Serial.println(">> 立即停止");
        sendStop();
        printRawReply(receiveFrame(100));
        break;
    case 'h':
        Serial.println(">> 歸零：回絕對零位 (9A/04)");
        sendHome();
        printRawReply(receiveFrame(200));
        break;
    case 'z':
        Serial.println(">> 當前位置角度清零 → 設為 0° (0A/6D)");
        sendClearPos();
        printRawReply(receiveFrame(100));
        break;
    case 'c':
        Serial.println(">> 清堵轉保護 (0E/52)");
        sendClogReset();
        printRawReply(receiveFrame(100));
        break;
    case 'p': {
        Serial.println(">> 讀即時位置 (0x36)");
        float deg = 0.0f;
        if(readPositionDeg(deg)) {
            Serial.print(">> position = ");
            printFix1(deg);
            Serial.println("°");
        } else {
            Serial.println(">> 位置讀取失敗");
        }
        break;
    }
    case 'x': {
        Serial.println(">> 讀位置誤差 (0x37)");
        int32_t raw = 0;
        if(readValue32(X42S::FD_POSERR, raw, true)) {
            const float deg = static_cast<float>(raw) * 360.0f / 65536.0f;
            Serial.print(">> 位置誤差 = ");
            Serial.print(deg, 2);
            Serial.println("°（|誤差|大且不縮小 → 堵轉/負載過大）");
        } else {
            Serial.println(">> 位置誤差讀取失敗");
        }
        break;
    }
    case 'y':
        Serial.println(">> 讀電流 (0x27 相 / 0x26 總線)");
        readCurrents();
        break;
    case 'o':
        Serial.println(">> 讀驅動配置 (0x42)");
        printConfig();
        break;
    case 'C': {
        int32_t ma = 0;
        if(!readIntInput(ma, "堵轉保護電流 (mA)：")) {
            break;
        }
        if(ma <= 0 || ma > 9999) {
            Serial.println("  ⚠ 範圍 1-9999 mA");
            break;
        }
        uint8_t cfg[28];
        if(readConfig(cfg)) {
            writeStallCurrent(cfg, static_cast<uint16_t>(ma));
        }
        break;
    }
    case 'F':
        gModeByte = !gModeByte;
        Serial.printf(">> FD 位置幀格式 = %s (%d bytes)\n",
                      gModeByte ? "13B 第二代（含運動模式）" : "12B V4.2（無運動模式）",
                      gModeByte ? 13 : 12);
        break;
    case '*':
        scanAddresses();
        break;
    case '#':
        scanBaudAndAddr();
        break;
    case 'u':
        switchPort(!gUseHwUart);
        break;
    case 'M':
        sRxMonitor = !sRxMonitor;
        Serial.printf(">> 原始 RX 監視器 = %s（收到的任何位元組都會印出來）\n",
                      sRxMonitor ? "ON" : "OFF");
        Serial.println("   開啟後碰一下馬達 TX 線或送命令，看 RX 是否有資料進來");
        break;
    case '?':
    default:
        printHelp();
        break;
    }
}
