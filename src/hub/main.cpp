// ============================================================================
// Hub — 中央轉發節點 (RP2350) — 雙核心閉環設計
//
// 根據 CLAUDE.md 拓撲：Jetson ←USB→ Hub ←PIO UART→ Slaves
//
// 兩顆核心各自形成獨立的閉環系統：
//
//   Core 0 (loop):  自動駕駛與路由大腦
//     - USB CDC (Serial) 收發 Jetson COBS 控制幀
//     - PIO UART TX 轉發控制命令至 Slaves
//     - PIO UART RX 接收 Slaves 回傳的 COBS 遙測幀
//     - Jetson 斷線時 (逾時 500ms)，讀取 Core 1 的 RC 數據作為備援
//     - 所有 I/O（USB + PIO TX/RX）集中在 Core 0，簡化同步模型
//
//   Core 1 (loop1): 安全守門員與手動介入
//     - 專職 crsf.update() — 高頻 CRSF 中斷解包 + Failsafe 計時器
//     - 只管遙控器，絕不漏字，Failsafe 判定極度精準
//     - 解完 -100~100 資料後，寫入全域變數供 Core 0 讀取
//
//  核心之間如何協同：
//    - Core 1 持續寫入 rc_channels[] / rc_failsafe (volatile 共用變數)
//    - Core 0 在其 loop() 中以 memcpy 低代價讀取
//    - 無鎖、無 IRQ、無跨核 handshake — 各自獨立閉環
//
// 控制邏輯：自動為主、遙控為輔
//    - Jetson 正常發送 → Core 0 直接轉發至 slaves (自動模式)
//    - Jetson 逾時 (>500ms) → Core 0 改用 Core 1 的遙控數據 (備援)
//    - 遙控也斷線 (Failsafe) → 發送 E-Stop 廣播至所有 slaves
//
// 腳位分配：
//   USB    ←→ Jetson (CDC Serial)       — ChassisComms 綁定 &Serial
//   UART0  ←→ ELRS 接收機 (GP1 RX)       — CRSF @ 420000 baud
//   PIO SM0 → Move slave (GP2 TX)        — ChassisComms 綁定 moveUart
//   PIO SM0 ← Move slave (GP3 RX)        — 接收 Move 遙測 COBS 幀
//   PIO SM1 → Shoot slave (GP4 TX)       — ChassisComms 綁定 shootUart
//   PIO SM1 ← Shoot slave (GP5 RX)       — 接收 Shoot 遙測 COBS 幀
//   PIO SM2 → Storage slave (GP6 TX)     — ChassisComms 綁定 storageUart
//   PIO SM2 ← Storage slave (GP7 RX)     — 接收 Storage 遙測 COBS 幀
// 注意：GP4/GP5 在 UART1 除錯與 PIO SM1 (Shoot) 之間衝突，須在硬體層面確保
//       不會同時啟用。當前預設 PIO SM1 優先於 UART1。
// ============================================================================

#include "ChassisComms.h"
#include <Arduino.h>
#include <CrsfReceiver.h>
#include <SerialPIO.h>

// ============================================================================
// 跨核心共用變數 — Core 1 寫入，Core 0 讀取
// RP2350 SCU 保證 cache coherency，volatile 防止編譯器過度優化
// ============================================================================
static volatile int16_t rc_channels[10]; // -100 ~ 100，由 Core 1 更新
static volatile bool rc_failsafe;        // true = 遙控器斷線

// ============================================================================
// PIO UART 硬體分配 (Hub 端)
// 根據 CLAUDE.md 規格，GP2~GP7 分配如下：
//
//   Slave     TX (Hub→Slave)  RX (Hub←Slave)
//   Move      GP2             GP3
//   Shoot     GP4             GP5
//   Storage   GP6             GP7
//
// 交叉連接：Hub PIO TX → Slave UART0 RX(GP1)，Slave UART0 TX(GP0) → Hub PIO RX
//   Move：    Hub SM0 TX(GP2) → Move RX(GP1)， Move TX(GP0) → Hub SM0 RX(GP3)
//   Shoot：   Hub SM1 TX(GP4) → Shoot RX(GP1)， Shoot TX(GP0) → Hub SM1 RX(GP5)
//   Storage： Hub SM2 TX(GP6) → Storage RX(GP1)， Storage TX(GP0) → Hub SM2
//   RX(GP7)
// ============================================================================
static constexpr uint8_t PIN_MOVE_TX = 2;
static constexpr uint8_t PIN_MOVE_RX = 3;
static constexpr uint8_t PIN_SHOOT_TX = 4;
static constexpr uint8_t PIN_SHOOT_RX = 5;
static constexpr uint8_t PIN_STORAGE_TX = 6;
static constexpr uint8_t PIN_STORAGE_RX = 7;

// earlephilhower SerialPIO：每個 slave 一個實例，各自使用獨立的 PIO SM。
// SerialPIO(txPin, rxPin, fifoSize)
static SerialPIO moveUart(PIN_MOVE_TX, PIN_MOVE_RX);          // TX=GP2, RX=GP3
static SerialPIO shootUart(PIN_SHOOT_TX, PIN_SHOOT_RX);       // TX=GP4, RX=GP5
static SerialPIO storageUart(PIN_STORAGE_TX, PIN_STORAGE_RX); // TX=GP6, RX=GP7

// ============================================================================
// PIO UART RX 接收緩衝區 — 每個 slave 獨立靜態緩衝區
// 用於暫存 Slave 回傳的 COBS 遙測幀，直到遇到 0x00 終止符後解碼。
// ============================================================================
static constexpr uint16_t PIO_RX_BUF_SIZE = 512;

struct PioRxBuffer {
    uint8_t buf[PIO_RX_BUF_SIZE];
    uint16_t len;
};

static PioRxBuffer moveRxBuf;
static PioRxBuffer shootRxBuf;
static PioRxBuffer storageRxBuf;

// ============================================================================
// 遙測資料儲存 — Hub 解析 Slave 回傳的 MoveTelemetry 後存入此處
// 供 Core 0 主迴圈或其他模組讀取。
// ============================================================================
static MoveTelemetry latestMoveTm = {};
static MoveTelemetry latestShootTm = {};
static MoveTelemetry latestStorageTm = {};
static uint64_t lastMoveTmUs = 0; // 0 = 從未收到
static uint64_t lastShootTmUs = 0;
static uint64_t lastStorageTmUs = 0;

// ============================================================================
// Slave 當機偵測逾時時間
// 收到首次遙測後，若超過此時間無新遙測 → 發送 REBOOT 訊息。
// ============================================================================
static constexpr uint64_t SLAVE_TELEMETRY_TIMEOUT_US = 500000ULL; // 500ms

// ============================================================================
// 輔助函式：COBS 解碼（inline，從原始 COBS 位元組流解碼）
// 回傳解碼後的位元組數，失敗回傳 0。
// ============================================================================
static size_t cobsDecodeInline(const uint8_t *src, size_t len, uint8_t *dst,
                               size_t dstSize) {
    size_t readIdx = 0;
    size_t writeIdx = 0;

    while(readIdx < len) {
        uint8_t code = src[readIdx];
        size_t blockLen = static_cast<size_t>(code);

        // 來源位元組不足
        if(readIdx + blockLen > len)
            return 0;
        readIdx++;

        // 目的溢位
        if(writeIdx + (blockLen - 1) > dstSize)
            return 0;

        for(uint8_t i = 1; i < code; i++)
            dst[writeIdx++] = src[readIdx++];

        // 插入隱含的零值（非最後區塊）
        if(code < 0xFF && readIdx < len) {
            if(writeIdx >= dstSize)
                return 0;
            dst[writeIdx++] = 0x00;
        }
    }
    return writeIdx;
}

// ============================================================================
// 輔助函式：處理 PIO UART RX 收到的位元組
//
// 從指定 SerialPIO 讀取可用位元組，暫存至對應的 PioRxBuffer。
// 遇到 0x00 終止符時進行 COBS 解碼 + CRC 驗證：
//   - 72 bytes + TELEMETRY → 存入 telemetryOut，更新 lastTmUs
//   - 20 bytes + CONTROL/ESTOP/HEARTBEAT → Serial.printf log（保留除錯用）
// ============================================================================
static void processPioRx(SerialPIO &uart, PioRxBuffer &rxBuf, const char *name,
                         MoveTelemetry &telemetryOut, uint64_t &lastTmUs) {
    while(uart.available() > 0) {
        uint8_t byte = static_cast<uint8_t>(uart.read());

        if(byte == 0x00) {
            if(rxBuf.len == 0) {
                // 空幀或遺漏前一個終止符，跳過
                continue;
            }

            // COBS 解碼
            uint8_t decoded[512];
            size_t decodedLen = cobsDecodeInline(rxBuf.buf, rxBuf.len, decoded,
                                                 sizeof(decoded));

            if(decodedLen == 0) {
                Serial.printf("[PIO RX][%s] COBS decode fail: bufLen=%u\n",
                              name, rxBuf.len);
                rxBuf.len = 0;
                continue;
            }

            // CRC 驗證
            if(decodedLen <
               8) { // 最小:
                // sync(1)+msgType(1)+frameSeq(1)+targetID(1)+payloadLen(2)+crc16(2)
                Serial.printf("[PIO RX][%s] too short: %zu bytes\n", name,
                              decodedLen);
                rxBuf.len = 0;
                continue;
            }

            uint16_t storedCrc =
                static_cast<uint16_t>(decoded[decodedLen - 2]) |
                (static_cast<uint16_t>(decoded[decodedLen - 1]) << 8);
            uint16_t calcCrc =
                SignalUtils::calculateCrc(decoded, decodedLen - 2);

            if(calcCrc != storedCrc) {
                Serial.printf(
                    "[PIO RX][%s] CRC mismatch (calc=0x%04X, stored=0x%04X)\n",
                    name, calcCrc, storedCrc);
                rxBuf.len = 0;
                continue;
            }

            // CRC 通過 — 依大小分發
            if(decodedLen == sizeof(MoveTelemetry)) {
                // 遙測封包 (72 bytes)
                memcpy(&telemetryOut, decoded, sizeof(MoveTelemetry));
                lastTmUs = time_us_64();
                Serial.printf(
                    "[PIO RX][%s] telemetry OK: frameSeq=%d, timestamp=%lu\n",
                    name, telemetryOut.frameSeq,
                    static_cast<unsigned long>(telemetryOut.timestamp));
            } else if(decodedLen >= sizeof(ControlPacket)) {
                // 控制/心跳/E-Stop 封包 (20 bytes)
                const ControlPacket &pkt =
                    *reinterpret_cast<const ControlPacket *>(decoded);
                Serial.printf(
                    "[PIO RX][%s] pkt: msgType=%d, frameSeq=%d, targetID=%d\n",
                    name, pkt.msgType, pkt.frameSeq, pkt.targetID);
            } else {
                Serial.printf("[PIO RX][%s] unknown frame: %zu bytes\n", name,
                              decodedLen);
            }

            rxBuf.len = 0; // 重置緩衝區
        } else {
            // 一般位元組：寫入緩衝區
            if(rxBuf.len < PIO_RX_BUF_SIZE) {
                rxBuf.buf[rxBuf.len++] = byte;
            } else {
                // 緩衝區溢出：丟棄並重置
                rxBuf.len = 0;
                Serial.printf("[PIO RX][%s] buffer overflow, resetting\n",
                              name);
            }
        }
    }
}

// ============================================================================
// ChassisComms 通用協議實例
//
// ChassisComms 是綁定到任意 Stream 的 COBS 協議庫：
//   - jetsonComms ←→ &Serial (USB CDC, Hub→Jetson 雙向，有心跳)
//   - moveComms / shootComms / storageComms → PIO UART TX (僅發送，無心跳)
//
// hbIntervalUs=0 停用心跳 — 因為 Hub→Slaves 方向為主機發送控制幀，
// 不需要定期心跳（Slave 端已有指令逾時 100ms E-Stop 保護）。
// ============================================================================
static ChassisComms jetsonComms(&Serial, TargetID::HUB,
                                0); // 無心跳（測試未接 Jetson）
static ChassisComms moveComms(&moveUart, TargetID::MOVE, 0);
static ChassisComms shootComms(&shootUart, TargetID::SHOOT, 0);
static ChassisComms storageComms(&storageUart, TargetID::STORAGE, 0);

// ============================================================================
// Slave 當機偵測 + 發送 REBOOT 訊息
//
// 若某 Slave 會回傳遙測（lastTmUs > 0）但超過500ms 無新資料，
// 則透過對應的 ChassisComms 發送 REBOOT 訊息觸發 slave 重開機。
// 發送後將 lastTmUs 歸零，避免重複 reboot。
// ============================================================================
static void sendReboot(ChassisComms &comms, uint8_t targetID) {
    ControlPacket reboot = {};
    reboot.sync = 0xA5;
    reboot.msgType = MsgType::REBOOT;
    reboot.frameSeq = 0;
    reboot.targetID = targetID;
    reboot.payloadLen = sizeof(ControlPacket) - 6 - 2;
    reboot.crc16 = 0;
    reboot.crc16 = SignalUtils::calculateCrc(
        reinterpret_cast<const uint8_t *>(&reboot), sizeof(ControlPacket) - 2);
    comms.sendPacket(reboot);
    Serial.printf("[HUB] Sent REBOOT to targetID=%d\n", targetID);
}

static void checkSlaveTimeouts() {
    uint64_t now = time_us_64();

    // Move Slave
    if(lastMoveTmUs > 0 && (now - lastMoveTmUs > SLAVE_TELEMETRY_TIMEOUT_US)) {
        sendReboot(moveComms, TargetID::MOVE);
        lastMoveTmUs = 0;
    }

    // Shoot Slave
    if(lastShootTmUs > 0 &&
       (now - lastShootTmUs > SLAVE_TELEMETRY_TIMEOUT_US)) {
        sendReboot(shootComms, TargetID::SHOOT);
        lastShootTmUs = 0;
    }

    // Storage Slave
    if(lastStorageTmUs > 0 &&
       (now - lastStorageTmUs > SLAVE_TELEMETRY_TIMEOUT_US)) {
        sendReboot(storageComms, TargetID::STORAGE);
        lastStorageTmUs = 0;
    }
}

// ============================================================================
// 遙控接收機 — Core 1 專用，使用 UART0 (GP1 RX) @ 420000 baud
// 因 Jetson 改用 USB CDC 通訊，UART0 已空出給 CRSF 使用。
//
// 接線：ELRS 接收機 TX → Hub GP1 (UART0 RX)
//       需在 ELRS 接收機設定 serial_inverted = off
// ============================================================================
static CrsfReceiver crsf;

// ============================================================================
// Core 0 內部狀態 — Jetson 連線追蹤 & RC 備援序號
// ============================================================================
static uint64_t lastJetsonFrameUs = 0; // 最後收到 Jetson 有效封包的時間
static uint8_t rcFallbackSeq = 0;      // RC 備援轉發用的 frameSeq
static uint64_t lastRcForwardUs = 0;   // RC 備援最後轉發時間

// ============================================================================
// 轉發輔助函式：對所有 slaves 廣播一個 ControlPacket
// 主要用於 E-Stop 廣播（targetID=BROADCAST + msgType=ESTOP）
// 一般控制不使用此函式（各 Slave 有各自的 targetID 和 payload）
// ============================================================================
static void forwardAllSlaves(const ControlPacket &pkt) {
    moveComms.sendPacket(pkt);
    shootComms.sendPacket(pkt);
    storageComms.sendPacket(pkt);
}

// ============================================================================
// RC 通道分配（ELRS CRSF 協定，CH1~CH10，值域 -100~100）
//
//   通道編號 (1-indexed) → rc_channels[index] (0-indexed)
//
//   Move:
//     CH1 (Roll)    → rc_channels[0] → vx       左右平移
//     CH2 (Pitch)   → rc_channels[1] → vy       前後移動
//     CH4 (Yaw)     → rc_channels[3] → omega    旋轉
//
//   Shoot:
//     CH3 (Throttle) → rc_channels[2] → upSpeed   上射速度
//     CH5            → rc_channels[4] → downSpeed 下射速度
//     CH7            → rc_channels[6] → pitch     俯仰
//     CH8            → rc_channels[7] → row       旋轉
//
//   Storage:
//     CH9            → rc_channels[8]  → motorChannel  PCA9685 通道 0~15
//     CH10           → rc_channels[9]  → trigger       0/1（> 50 視為 1）
// ============================================================================

// 死區濾波：-10~10 → 0
static int16_t applyDeadband(int16_t val) {
    if(val > -10 && val < 10)
        return 0;
    return val;
}

// 將遙控通道值映射為 int8（含死區 + 限幅）
static int8_t rcToInt8(int16_t ch) {
    return static_cast<int8_t>(constrain(applyDeadband(ch), -100, 100));
}

// ============================================================================
// RC → Move ControlPacket
// ============================================================================
static ControlPacket rcToMovePacket() {
    ControlPacket pkt = {};
    pkt.sync = 0xA5;
    pkt.msgType = MsgType::CONTROL;
    pkt.frameSeq = rcFallbackSeq++;
    pkt.targetID = TargetID::MOVE;
    pkt.payloadLen = sizeof(ControlPacket) - 6 - 2;

    pkt.move.vx = rcToInt8(rc_channels[1]);    // CH1 → vx
    pkt.move.vy = rcToInt8(rc_channels[0]);    // CH0 → vy
    pkt.move.omega = rcToInt8(rc_channels[7]); // CH7 → omega

    pkt.crc16 = 0;
    pkt.crc16 = SignalUtils::calculateCrc(reinterpret_cast<uint8_t *>(&pkt),
                                          sizeof(ControlPacket) - 2);
    return pkt;
}

// ============================================================================
// RC → Shoot ControlPacket（測試：CH2 同時驅動上/下射兩顆無刷）
//
// CH2 (rc_channels[1]) 原始值 -100~100 直接填入 shoot.upSpeed / downSpeed，
// 由 Shoot 端測試程式自行映射為 0~100% 出力（-97→0, +97→100）。
// pitch / row 固定 0 — 本階段不驅動步進馬達。
// ============================================================================
static constexpr bool RC_FORWARD_SHOOT = true; // false = 回到只轉發 Move

static ControlPacket rcToShootPacket() {
    ControlPacket pkt = {};
    pkt.sync = 0xA5;
    pkt.msgType = MsgType::CONTROL;
    pkt.frameSeq = rcFallbackSeq++;
    pkt.targetID = TargetID::SHOOT;
    pkt.payloadLen = sizeof(ControlPacket) - 6 - 2;

    const int8_t ch2 = rcToInt8(rc_channels[1]); // CH2 → -100~100
    pkt.shoot.upSpeed = ch2;
    pkt.shoot.downSpeed = ch2;
    pkt.shoot.pitch = 0;
    pkt.shoot.row = 0;

    pkt.crc16 = 0;
    pkt.crc16 = SignalUtils::calculateCrc(reinterpret_cast<uint8_t *>(&pkt),
                                          sizeof(ControlPacket) - 2);
    return pkt;
}

// ============================================================================
// Core 0：Jetson 幀回呼
//
// jetsonComms 從 USB CDC 解出有效 ControlPacket 後觸發。
// Core 0 當場根據 targetID 呼叫對應 slave 的 ChassisComms.sendPacket()，
// 一條龍處理，完全不需跨核心傳遞。
// ============================================================================
static void onJetsonFrame(const ControlPacket &pkt) {
    lastJetsonFrameUs = time_us_64();

    switch(pkt.targetID) {
    case TargetID::MOVE:
        moveComms.sendPacket(pkt);
        break;
    case TargetID::SHOOT:
        shootComms.sendPacket(pkt);
        break;
    case TargetID::STORAGE:
        storageComms.sendPacket(pkt);
        break;
    case TargetID::BROADCAST:
        forwardAllSlaves(pkt);
        break;
    case TargetID::HUB:
    default:
        break;
    }
}

// ============================================================================
// Core 0：RC 備援轉發 — 當 Jetson 逾時時由 loop() 呼叫
// 以 ~100Hz 頻率將遙控數據轉發至所有 slave
// ============================================================================
static void tryRcFallback() {
    uint64_t now = time_us_64();

    // 節流至 ~100Hz 避免 flooding
    if(now - lastRcForwardUs < 10000ULL) // 10ms
        return;
    lastRcForwardUs = now;

    // 如果遙控也斷線 → E-Stop 廣播
    if(rc_failsafe) {
        ControlPacket estop = {};
        estop.sync = 0xA5;
        estop.msgType = MsgType::ESTOP;
        estop.frameSeq = rcFallbackSeq++;
        estop.targetID = TargetID::BROADCAST;
        estop.payloadLen = sizeof(ControlPacket) - 6 - 2;
        estop.crc16 = 0;
        estop.crc16 = SignalUtils::calculateCrc(
            reinterpret_cast<uint8_t *>(&estop), sizeof(ControlPacket) - 2);
        forwardAllSlaves(estop);
        return;
    }

    // 遙控正常 → 轉發控制幀給 Move
    // （實際控制測試僅接 Move；Shoot/Storage 未接線故不轉發）
    ControlPacket mv = rcToMovePacket();
    bool sent = moveComms.sendPacket(mv);
    static uint32_t sendOk = 0, sendFail = 0;
    if(sent)
        sendOk++;
    else
        sendFail++;

    // 遙控正常 → 轉發控制幀給 Shoot（PIO SM1, GP4 TX / GP5 RX）
    // CH2 同時寫入 upSpeed / downSpeed → Shoot 端兩顆無刷同轉速
    if(RC_FORWARD_SHOOT) {
        ControlPacket sh = rcToShootPacket();
        shootComms.sendPacket(sh);

        static uint64_t lastShootDbgUs = 0;
        uint64_t nowShoot = time_us_64();
        if(nowShoot - lastShootDbgUs > 1000000ULL) {
            lastShootDbgUs = nowShoot;
            Serial.printf("[HUB] → Shoot up=%d down=%d (CH2=%d)\n",
                          sh.shoot.upSpeed, sh.shoot.downSpeed, rc_channels[1]);
        }
    }

    // 除錯：直接對照「讀到的 rc_channels」與「送出的 mv.move.vx」
    // 確認 rcToMovePacket 是否真的把 rc_channels[1] 塞進 vx
    static uint64_t lastPktDbgUs = 0;
    uint64_t nowPkt = time_us_64();
    if(nowPkt - lastPktDbgUs > 1000000ULL) {
        lastPktDbgUs = nowPkt;
        const uint8_t *pr = reinterpret_cast<const uint8_t *>(&mv);
        Serial.printf("[HUB] chan1=%d chan0=%d chan7=%d | mv.vx=%d mv.vy=%d "
                      "mv.omega=%d | byte6,7,8=%02X %02X %02X\n",
                      rc_channels[1], rc_channels[0], rc_channels[7],
                      mv.move.vx, mv.move.vy, mv.move.omega, pr[6], pr[7],
                      pr[8]);
    }

    // 每 1 秒印一次 send 統計 + PIO 狀態（判斷是否真的寫入 GP2）
    static uint64_t lastSendDbgUs = 0;
    uint64_t nowDbg = time_us_64();
    if(nowDbg - lastSendDbgUs > 1000000ULL) {
        lastSendDbgUs = nowDbg;
        Serial.printf("[HUB] PIO send ok=%u fail=%u rxAvail=%d\n", sendOk,
                      sendFail, moveUart.available());
    }
}

// ============================================================================
// Core 0：setup()
// ============================================================================
void setup() {
    Serial.begin(921600); // Jetson USB CDC

    // Jetson USB CDC — ChassisCombs 綁定 &Serial, 30ms 心跳往 Jetson
    jetsonComms.begin();
    jetsonComms.setFrameCallback(onJetsonFrame);

    // Slave PIO UART — ChassisComms 綁定 SerialPIO，無心跳 (hbIntervalUs=0)
    moveUart.begin(RobotConfig::PIO_BAUDRATE);
    moveComms.begin();
    shootUart.begin(RobotConfig::PIO_BAUDRATE);
    shootComms.begin();
    storageUart.begin(RobotConfig::PIO_BAUDRATE);
    storageComms.begin();

    // 初始化接收緩衝區長度
    moveRxBuf.len = 0;
    shootRxBuf.len = 0;
    storageRxBuf.len = 0;

    // 初始化 Jetson 逾時追蹤
    lastJetsonFrameUs = time_us_64();
}

// ============================================================================
// Core 0: loop() — 自動駕駛與路由大腦 + PIO RX 接收 + 遙測監控
//
// 1. jetsonComms.update() → USB CDC 讀取 + COBS 解包 + 回呼轉發 + 30ms 心跳
// 2. PIO UART RX 輪詢 — 接收各 Slave 回傳的 COBS 遙測幀
// 3. Slave 當機偵測 — 超過500ms 無遙測 → 發送 REBOOT
// 4. 如果 Jetson 逾時 >500ms → 切換至 RC 備援
// ============================================================================
void loop() {
    jetsonComms.update();

    // 輪詢各 PIO UART RX，接收 Slave 回傳的 COBS 遙測幀
    processPioRx(moveUart, moveRxBuf, "Move", latestMoveTm, lastMoveTmUs);
    processPioRx(shootUart, shootRxBuf, "Shoot", latestShootTm, lastShootTmUs);
    processPioRx(storageUart, storageRxBuf, "Storage", latestStorageTm,
                 lastStorageTmUs);

    // Slave 當機偵測：逾時500ms 無遙測 → 發送 REBOOT
    checkSlaveTimeouts();

    uint64_t now = time_us_64();
    if(now - lastJetsonFrameUs > RobotConfig::HEARTBEAT_TIMEOUT_US) {
        // Jetson 斷線 → 啟用遙控備援
        tryRcFallback();
    }

    // ---- 除錯：每 500ms 印 RC 狀態與送了什麼到 Move ----
    static uint64_t lastDbgUs = 0;
    if(now - lastDbgUs > 500000ULL) {
        lastDbgUs = now;
        if(rc_failsafe) {
            Serial.println("[HUB] RC FAILSAFE → 送 E-STOP 到 Move");
        } else {
            int16_t ch[10];
            // 讀取 Core 1 的 RC 資料
            for(int i = 0; i < 10; i++)
                ch[i] = rc_channels[i];
            Serial.printf(
                "[HUB] ch0=%d ch1=%d ch2=%d ch3=%d ch4=%d ch5=%d ch6=%d "
                "ch7=%d ch8=%d ch9=%d\n",
                ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7], ch[8],
                ch[9]);
            Serial.printf("[HUB] → vx=%d vy=%d omega=%d\n", rcToInt8(ch[1]),
                          rcToInt8(ch[0]), rcToInt8(ch[7]));
        }
    }

    delayMicroseconds(20);
}

// ============================================================================
// Core 1: setup1() — 初始化 CRSF 遙控接收機
//
// CRSF 使用 UART0 (GP1 RX, 420000 baud)。
// 因 Jetson 改用 USB CDC，UART0 已空出給 CRSF 使用。
// ELRS 接收機需設為非反向輸出 (serial_inverted = off)。
// ============================================================================
void setup1() { crsf.begin(0, 1, 420000); }

// ============================================================================
// Core 1: loop1() — 安全守門員與手動介入
//
// 專職 crsf.update() 高頻解包，什麼都不管，只專心抓遙控器。
// 解完後寫入全域 rc_channels / rc_failsafe，供 Core 0 低代價讀取。
// ============================================================================
void loop1() {
    while(true) {
        crsf.update();

        // 逐元素寫入全域變數 (避免 volatile pointer 傳入 memcpy 的編譯錯誤)
        // RP2350 SCU 保證 cache coherency，Core 0 讀取時可見最新值
        int16_t ch[10];
        crsf.getChannels(ch);
        for(int i = 0; i < 10; i++) {
            rc_channels[i] = ch[i];
        }
        rc_failsafe = crsf.isFailsafe();
    }
}
