// ============================================================================
// elev_uart_test — 仰角步進馬達精簡測試（X42S / Emm 韌體，PIO 模擬 UART）
//
// 用途：驗證「遙控連續隨機跟隨」與「堵轉偵測」，作為 src/shoot/main.cpp
//       ElevStepper 的移植依據。
//
// 接線（PIO 模擬 UART，與 src/shoot/main.cpp 同腳位）：
//   GP14 = TX → 馬達 R/A/H（或 T/B/L）
//   GP15 = RX ← 馬達 T/B/L（或 R/A/H）
//   115200 8N1，務必共地
//
// 協定（ZDT_X42S 第二代 / Emm 固件）：
//   - 幀： [地址][功能碼][參數…][尾碼 0x6B]，地址預設 = 1
//   - FD 位置幀（13B）：[Addr][FD][dir][速度 2B][加速度檔位][脈衝數 4B][mode][sync][6B]
//     脈衝單位 = 0.1125°/脈衝（1.8° ×16 細分 = 3200 脈衝/圈，實測確認）
//     方向由 dir 位元組控制（脈衝數只吃絕對值，實測確認）
//     運動模式：1=絕對（須等到位）、3=絕對即時修正（運動中可更新目標，實測確認）
//
// 按鍵（USB Serial, 115200）：
//   'f' 隨機跟隨：絕對角度即時修正（mode=3，運動中可更新）
//   'e' 使能            'd' 去使能
//   'q' 讀狀態旗標 0x3A（含堵轉標誌 bit2 / 堵轉保護 bit3）
//   'x' 讀位置誤差 0x37（堵轉時持續大）
//   'y' 讀相電流 0x27 / 總線電流 0x26（堵轉時相電流 ≈ 堵轉電流設定值）
//   'w' 切換堵轉自動輪詢（每 250ms 讀 0x3A，偵測到堵轉 → 印警報 + 自動 stop）
//   '?' 幫助
// ============================================================================

#include <Arduino.h>

namespace Pin {
constexpr uint ELEV_UART_TX = 14; // GP14 → 馬達 R/A/H 或 T/B/L
constexpr uint ELEV_UART_RX = 15; // GP15 ← 馬達 T/B/L 或 R/A/H
} // namespace Pin

static SerialPIO motorSerial(Pin::ELEV_UART_TX, Pin::ELEV_UART_RX);

// ============================================================================
// 協定常數（Emm 固件）
// ============================================================================
namespace X42S {
constexpr uint8_t PROTO_ADDR = 0x01;
constexpr uint8_t TAIL = 0x6B;
constexpr uint8_t FD_EN = 0xF3;          // 使能/去使能（輔助碼 AB）
constexpr uint8_t FD_POS = 0xFD;         // 位置模式控制
constexpr uint8_t FD_STOP = 0xFE;        // 立即停止（輔助碼 98）
constexpr uint8_t FD_STATUS = 0x3A;      // 讀電機狀態旗標
// 輔助碼
constexpr uint8_t AX_EN = 0xAB;
constexpr uint8_t AX_STOP = 0x98;
// 運動模式（13B FD 幀）
constexpr uint8_t POS_MODE_ABS_RT = 0x03; // 絕對位置即時修正（跟隨用，實測確認）
} // namespace X42S

// 脈衝換算（實測確認）：3200 脈衝/圈，1 脈衝 = 0.1125°
constexpr float PULSES_PER_DEG = 3200.0f / 360.0f;

// 方向位元組：脈衝數只吃絕對值，方向由 dir 決定。
// 實測若「上/下」顛倒，把這兩個值對調即可。
constexpr uint8_t kDirDown = 0x01; // 正角度 = 從最上面往下轉
constexpr uint8_t kDirUp   = 0x00;

// 可調參數
constexpr uint16_t kVelMaxRpm = 3000;
static uint16_t gVelRpm = 300;  // 保守速度
static uint8_t gAccStep = 200;  // 加速度檔位 0-255

// 堵轉自動輪詢
constexpr uint32_t STALL_POLL_MS = 250;

// ============================================================================
// 緩衝（靜態分配，專案禁止 new/malloc）
// ============================================================================
static uint8_t sTxBuf[40];
static uint8_t sRxBuf[40];

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
    motorSerial.write(buf, len);
    motorSerial.flush();
    printHex("  TX", buf, len);
}

// 接收一幀，以「timeoutMs 無新資料」判定結束
static uint8_t receiveFrame(uint32_t timeoutMs) {
    uint8_t i = 0;
    uint32_t lastByteMs = millis();
    while(millis() - lastByteMs < timeoutMs) {
        while(motorSerial.available() > 0) {
            if(i < sizeof(sRxBuf)) {
                sRxBuf[i++] = static_cast<uint8_t>(motorSerial.read());
                lastByteMs = millis();
            } else {
                motorSerial.read(); // 溢位丟棄
            }
        }
    }
    return i;
}

static void flushInput() {
    while(motorSerial.available() > 0) {
        motorSerial.read();
    }
}

// ============================================================================
// 命令
// ============================================================================
static void sendEnable(bool en) {
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_EN;
    sTxBuf[2] = X42S::AX_EN;
    sTxBuf[3] = en ? 0x01 : 0x00;
    sTxBuf[4] = 0x00; // 同步標誌
    sTxBuf[5] = X42S::TAIL;
    sendFrame(sTxBuf, 6);
}

static void sendStop() {
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_STOP;
    sTxBuf[2] = X42S::AX_STOP;
    sTxBuf[3] = 0x00;
    sTxBuf[4] = X42S::TAIL;
    sendFrame(sTxBuf, 5);
}

static void sendReadStatus() {
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = X42S::FD_STATUS;
    sTxBuf[2] = X42S::TAIL;
    sendFrame(sTxBuf, 3);
}

static void sendReadPosErr() { // 0x37 位置誤差
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = 0x37;
    sTxBuf[2] = X42S::TAIL;
    sendFrame(sTxBuf, 3);
}

static void sendReadCurrent(bool phase) { // 0x27 相電流 / 0x26 總線電流
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = phase ? 0x27 : 0x26;
    sTxBuf[2] = X42S::TAIL;
    sendFrame(sTxBuf, 3);
}

/// 絕對位置命令（mode=3 即時修正）：[Addr][FD][dir][vel 2B][acc][pulses 4B][mode][sync][6B]
static void sendPosition(int32_t pulses) {
    const uint8_t dir = (pulses < 0) ? kDirUp : kDirDown;
    const uint32_t mag = (pulses < 0) ? static_cast<uint32_t>(-pulses)
                                      : static_cast<uint32_t>(pulses);
    uint8_t i = 0;
    sTxBuf[i++] = X42S::PROTO_ADDR;
    sTxBuf[i++] = X42S::FD_POS;
    sTxBuf[i++] = dir;
    sTxBuf[i++] = static_cast<uint8_t>(gVelRpm >> 8);
    sTxBuf[i++] = static_cast<uint8_t>(gVelRpm & 0xFF);
    sTxBuf[i++] = gAccStep;
    sTxBuf[i++] = static_cast<uint8_t>(mag >> 24);
    sTxBuf[i++] = static_cast<uint8_t>(mag >> 16);
    sTxBuf[i++] = static_cast<uint8_t>(mag >> 8);
    sTxBuf[i++] = static_cast<uint8_t>(mag);
    sTxBuf[i++] = X42S::POS_MODE_ABS_RT;
    sTxBuf[i++] = 0x00; // 同步標誌
    sTxBuf[i++] = X42S::TAIL;
    sendFrame(sTxBuf, i);
}

// ============================================================================
// 狀態查詢 / 堵轉檢查
// ============================================================================
// 讀狀態旗標並解讀（含堵轉），回傳原始 flags byte；失敗回 -1
static int readStatusFlags(bool verbose) {
    flushInput();
    sendReadStatus();
    uint8_t n = receiveFrame(100);
    if(n < 4 || sRxBuf[0] != X42S::PROTO_ADDR || sRxBuf[1] != X42S::FD_STATUS) {
        if(verbose) {
            printHex("  reply", sRxBuf, n);
            Serial.println("  → 無回應或格式錯（檢查接線/波特率）");
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

static bool stalledFlags(int f) {
    return f >= 0 && ((f & 0x04) || (f & 0x08)); // bit2 堵轉標誌 / bit3 堵轉保護
}

// 讀位置誤差（0x37，int32，65536/圈），印出；失敗回 false
static bool readPosErr() {
    flushInput();
    sendReadPosErr();
    uint8_t n = receiveFrame(100);
    if(n < 8 || sRxBuf[0] != X42S::PROTO_ADDR || sRxBuf[1] != 0x37) {
        printHex("  reply", sRxBuf, n);
        return false;
    }
    const int32_t raw = (static_cast<int32_t>(sRxBuf[2]) << 24) |
                        (static_cast<int32_t>(sRxBuf[3]) << 16) |
                        (static_cast<int32_t>(sRxBuf[4]) << 8) |
                        static_cast<int32_t>(sRxBuf[5]);
    const float deg = static_cast<float>(raw) * 360.0f / 65536.0f;
    Serial.print(">> 位置誤差 = ");
    Serial.print(deg, 2);
    Serial.println("°（|誤差|大且不縮小 → 堵轉/負載過大）");
    return true;
}

// 讀驅動配置（0x42，Emm）：回傳 [Addr][42][28B][6B]
static bool readConfig(uint8_t cfgOut[28]) {
    flushInput();
    sTxBuf[0] = X42S::PROTO_ADDR;
    sTxBuf[1] = 0x42;
    sTxBuf[2] = 0x6C; // 協議校驗字節（Emm 讀配置）
    sTxBuf[3] = X42S::TAIL;
    sendFrame(sTxBuf, 4);
    uint8_t n = receiveFrame(150);
    if(n < 31 || sRxBuf[0] != X42S::PROTO_ADDR || sRxBuf[1] != 0x42) {
        printHex("  reply", sRxBuf, n);
        return false;
    }
    memcpy(cfgOut, &sRxBuf[2], 28);
    return true;
}

// 寫堵轉保護電流（0x48 修改配置，STORE=1 掉電保存）
// 幀：[Addr][48][D1][store][28B 配置區][6B]，其中 stall_current 在 offset 22:24
static bool writeStallCurrent(uint8_t cfg[28], uint16_t mA) {
    cfg[22] = static_cast<uint8_t>(mA >> 8);
    cfg[23] = static_cast<uint8_t>(mA & 0xFF);
    uint8_t i = 0;
    sTxBuf[i++] = X42S::PROTO_ADDR;
    sTxBuf[i++] = 0x48;
    sTxBuf[i++] = 0xD1; // 協議校驗字節（Emm 寫配置）
    sTxBuf[i++] = 0x01; // STORE：掉電保存
    for(uint8_t k = 0; k < 28; k++) {
        sTxBuf[i++] = cfg[k];
    }
    sTxBuf[i++] = X42S::TAIL;
    sendFrame(sTxBuf, i);
    uint8_t n = receiveFrame(150);
    if(n >= 4 && sRxBuf[0] == X42S::PROTO_ADDR && sRxBuf[2] == 0x02) {
        Serial.printf(">> 堵轉電流設定已寫入 %u mA（掉電保存）\n", mA);
        return true;
    }
    printHex("  reply", sRxBuf, n);
    return false;
}

constexpr uint16_t kStallCurrentMa = 2200; // 目標堵轉電流（400mA 太低會常保護，誤教訓）

// 讀相電流 / 總線電流（mA）
static void readCurrents() {
    flushInput();
    sendReadCurrent(true);
    uint8_t n = receiveFrame(100);
    long phase = -1;
    if(n >= 5 && sRxBuf[0] == X42S::PROTO_ADDR && sRxBuf[1] == 0x27) {
        phase = (static_cast<long>(sRxBuf[2]) << 8) | sRxBuf[3];
    } else {
        printHex("  reply", sRxBuf, n);
    }
    flushInput();
    sendReadCurrent(false);
    n = receiveFrame(100);
    long bus = -1;
    if(n >= 5 && sRxBuf[0] == X42S::PROTO_ADDR && sRxBuf[1] == 0x26) {
        bus = (static_cast<long>(sRxBuf[2]) << 8) | sRxBuf[3];
    }
    Serial.printf(">> 相電流 = %ld mA，總線電流 = %ld mA\n", phase, bus);
    Serial.println("   （相電流 ≈ 堵轉電流設定值且位置不動 → 堵轉）");

    // 讀堵轉保護參數，若電流 ≠ 400mA 則自動寫入
    uint8_t cfg[28];
    if(readConfig(cfg)) {
        const uint16_t cur = (static_cast<uint16_t>(cfg[22]) << 8) | cfg[23];
        const uint16_t spd = (static_cast<uint16_t>(cfg[20]) << 8) | cfg[21];
        const uint16_t tm = (static_cast<uint16_t>(cfg[24]) << 8) | cfg[25];
        Serial.printf(">> 堵轉保護：模式=%d，速度=%u RPM，電流=%u mA，時間=%u ms\n",
                      cfg[19], spd, cur, tm);
        if(cur != kStallCurrentMa) {
            Serial.printf(">> 將堵轉電流設定改為 %u mA…\n", kStallCurrentMa);
            writeStallCurrent(cfg, kStallCurrentMa);
        }
    }
}

// ============================================================================
// 堵轉自動輪詢（'w' 切換）
// ============================================================================
static bool sStallWatch = false;
static uint32_t sLastStallPollMs = 0;
static bool sStallAlerted = false; // 同一次堵轉只警報一次

static void stallWatchUpdate() {
    if(!sStallWatch) {
        return;
    }
    uint32_t now = millis();
    if(now - sLastStallPollMs < STALL_POLL_MS) {
        return;
    }
    sLastStallPollMs = now;
    int f = readStatusFlags(false);
    if(f < 0) {
        return; // 讀不到不誤判
    }
    if(stalledFlags(f)) {
        if(!sStallAlerted) {
            sStallAlerted = true;
            Serial.println("========================================");
            Serial.println("  ⚠⚠ 韌體回報堵轉！已自動送出立即停止");
            Serial.println("  檢查機構卡阻後重新上電，或用原完整版 'c' 清除保護");
            Serial.println("========================================");
            sendStop();
        }
        return;
    }
    sStallAlerted = false; // 解堵後重設
}

// ============================================================================
// 幫助 / setup / loop
// ============================================================================
static void printHelp() {
    Serial.println();
    Serial.println("=== 仰角步進 精簡測試（跟隨 + 堵轉檢查）===");
    Serial.println("  'f' 隨機跟隨：絕對角度即時修正(mode=3，運動中可更新)");
    Serial.println("  'e' 使能   'd' 去使能");
    Serial.println("  'q' 狀態旗標(含堵轉)  'x' 位置誤差  'y' 相/總線電流");
    Serial.printf("  'w' 堵轉自動輪詢（目前 %s）\n", sStallWatch ? "ON" : "OFF");
    Serial.println("  '?' 此幫助");
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n[elev_uart_test] Booting (精簡版)...");
    motorSerial.begin(115200); // PIO 模擬 UART（GP14→馬達, GP15←馬達）
    flushInput();
    printHelp();
}

void loop() {
    stallWatchUpdate();

    if(!Serial.available()) {
        return;
    }
    char c = static_cast<char>(Serial.read());
    flushInput();

    switch(c) {
    case 'f': { // 遙控隨機跟隨：絕對位置即時修正（mode=3）
        Serial.print("輸入絕對角度 (°) [負=向上]：");
        long v = 0;
        bool neg = false;
        bool got = false;
        uint32_t t0 = millis();
        while(millis() - t0 < 10000) {
            if(Serial.available()) {
                char ch = static_cast<char>(Serial.read());
                if(ch == '\n' || ch == '\r') {
                    if(got) {
                        break;
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
        if(!got) {
            Serial.println("  (逾時)");
            break;
        }
        int32_t deg = neg ? -static_cast<int32_t>(v) : static_cast<int32_t>(v);
        Serial.println();
        const int32_t p = static_cast<int32_t>(lroundf(deg * PULSES_PER_DEG));
        Serial.printf(">> 即時修正 → %ld° = %ld 脈衝（mode=3）\n",
                      static_cast<long>(deg), static_cast<long>(p));
        sendPosition(p);
        uint8_t n = receiveFrame(100);
        printHex("  reply", sRxBuf, n);
        break;
    }
    case 'e': {
        Serial.println(">> enable");
        sendEnable(true);
        uint8_t n = receiveFrame(100);
        printHex("  reply", sRxBuf, n);
        break;
    }
    case 'd': {
        Serial.println(">> disable");
        sendEnable(false);
        uint8_t n = receiveFrame(100);
        printHex("  reply", sRxBuf, n);
        break;
    }
    case 'C': { // 復原：清堵轉保護 + 把堵轉電流寫回 kStallCurrentMa
        Serial.println(">> 復原：清堵轉保護 (0x0E/52)");
        flushInput();
        sTxBuf[0] = X42S::PROTO_ADDR;
        sTxBuf[1] = 0x0E;
        sTxBuf[2] = 0x52;
        sTxBuf[3] = X42S::TAIL;
        sendFrame(sTxBuf, 4);
        uint8_t n = receiveFrame(150);
        printHex("  reply", sRxBuf, n);
        Serial.println(">> 讀配置並把堵轉電流寫回設定值…");
        uint8_t cfg[28];
        if(readConfig(cfg)) {
            const uint16_t cur = (static_cast<uint16_t>(cfg[22]) << 8) | cfg[23];
            Serial.printf(">> 目前堵轉電流 = %u mA\n", cur);
            if(cur != kStallCurrentMa) {
                writeStallCurrent(cfg, kStallCurrentMa);
            }
        }
        break;
    }
    case 'q': {
        Serial.println(">> 讀狀態旗標 (0x3A)");
        readStatusFlags(true);
        break;
    }
    case 'x': {
        Serial.println(">> 讀位置誤差 (0x37)");
        readPosErr();
        break;
    }
    case 'y': {
        Serial.println(">> 讀電流 (0x27/0x26)");
        readCurrents();
        break;
    }
    case 'w': {
        sStallWatch = !sStallWatch;
        sStallAlerted = false;
        Serial.printf(">> 堵轉自動輪詢 = %s（每 %lums 讀 0x3A，堵轉 → stop）\n",
                      sStallWatch ? "ON" : "OFF",
                      static_cast<unsigned long>(STALL_POLL_MS));
        break;
    }
    case '?':
    default:
        printHelp();
        break;
    }
}
