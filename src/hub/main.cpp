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
// 控制邏輯（Jetson 主控、遙控器只當備援/手動介入）
//    - JETSON 模式：Jetson 持有「全部」控制權
//        * Move / Shoot / Storage 所有命令、所有通道值都只來自 Jetson
//        * 遙控器只准讀 rc[5]（控制來源切換開關），其他 rc 通道一律不理
//        * 即使 RC Failsafe 也不會因此送 E-Stop（不干擾 Jetson）
//    - RC_SWITCH：rc[5] < -10 → 遙控器接管全部 Slave（Jetson 命令丟棄）
//    - RC_FALLBACK：Jetson 逾時 (>500ms) → 自動改用遙控數據備援
//    - 遙控也斷線 (Failsafe) 且 Jetson 也逾時 → 發送 E-Stop 廣播至所有 slaves
//    - 遙控 Failsafe 時不看 rc[5]（讀不到），自動退回 Jetson，不會鎖死
//
// 控制來源每圈 loop() 只評估一次（ControlSource），onJetsonFrame() 與 RC
// 轉發都只看同一個值 → 不會出現「兩邊同時送命令」的競態。
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
#include <MathUtils.h>
#include <SerialPIO.h>
#include <StatusLed.h>
#include <cmath>         // lroundf / fabsf
#include <cstring>       // memcpy
#include <pico/stdlib.h> // time_us_64()

// ============================================================================
// 跨核心共用變數 — Core 1 寫入，Core 0 讀取
// RP2350 SCU 保證 cache coherency，volatile 防止編譯器過度優化
// ============================================================================
static volatile int16_t rc_channels[10]; // -100 ~ 100，由 Core 1 更新
static volatile bool rc_failsafe;        // true = 遙控器斷線

// ============================================================================
// 序列輸出路徑
//   false（預設）→ 只印 RC 通道數據（rc_channels[0]~[9]，每 500ms 一行）
//   true         → 額外印出 PIO RX / 轉發 / LED 等除錯訊息
// ============================================================================
static constexpr bool HUB_LOG_VERBOSE = false;

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
//   fifoSize 預設只有 32 bytes → 一旦 loop() 被 Serial.printf / USB CDC 阻塞
//   ~350us 就會掉位元組，造成遙測 COBS 解碼失敗、CRC mismatch。
//   放大到 256 bytes（約 2.8ms @921600）給予足夠餘裕。
static SerialPIO moveUart(PIN_MOVE_TX, PIN_MOVE_RX, 256);          // TX=GP2, RX=GP3
static SerialPIO shootUart(PIN_SHOOT_TX, PIN_SHOOT_RX, 256);       // TX=GP4, RX=GP5
static SerialPIO storageUart(PIN_STORAGE_TX, PIN_STORAGE_RX, 256); // TX=GP6, RX=GP7

// ============================================================================
// PIO UART RX 接收緩衝區 — 每個 slave 獨立靜態緩衝區
// 用於暫存 Slave 回傳的 COBS 遙測幀，直到遇到 0x00 終止符後解碼。
// ============================================================================
static constexpr uint16_t PIO_RX_BUF_SIZE = 512;

struct PioRxBuffer {
    uint8_t buf[PIO_RX_BUF_SIZE];
    uint16_t len;
    uint64_t lastLogUs; // 日誌節流（避免 Slave 心跳/遙測把監控洗版）
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
// 輔助函式：處理 PIO UART RX 收到的位元組
//
// 從指定 SerialPIO 讀取可用位元組，暫存至對應的 PioRxBuffer。
// 遇到 0x00 終止符時進行 COBS 解碼 + CRC 驗證：
//   - 72 bytes + TELEMETRY → 存入 telemetryOut，更新 lastTmUs
//   - 20 bytes + CONTROL/REBOOT/ESTOP → 更新狀態（日誌節流）
//   - HEARTBEAT（Slave 每 30ms 一次）→ 只更新狀態，不洗版
//
// 日誌策略：正常封包每秒最多印一次摘要；錯誤（CRC/解碼失敗）節流 500ms，
// 避免把序列監控塞滿而看不到真正重要的訊息。
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

            const uint64_t nowUs = time_us_64();

            // COBS 解碼（共用版上限 COBS_DST_MAX = 512，與本緩衝區同尺寸）
            uint8_t decoded[SignalUtils::COBS_DST_MAX];
            size_t decodedLen =
                SignalUtils::cobsDecode(rxBuf.buf, rxBuf.len, decoded);

            if(decodedLen == 0) {
                if(HUB_LOG_VERBOSE && nowUs - rxBuf.lastLogUs > 500000ULL) {
                    rxBuf.lastLogUs = nowUs;
                    Serial.printf("[PIO RX][%s] COBS decode fail: bufLen=%u\n",
                                  name, rxBuf.len);
                }
                rxBuf.len = 0;
                continue;
            }

            // 最小長度: sync+msgType+frameSeq+targetID+payloadLen+crc16 = 8
            if(decodedLen < 8) {
                if(HUB_LOG_VERBOSE && nowUs - rxBuf.lastLogUs > 500000ULL) {
                    rxBuf.lastLogUs = nowUs;
                    Serial.printf("[PIO RX][%s] too short: %zu bytes\n", name,
                                  decodedLen);
                }
                rxBuf.len = 0;
                continue;
            }

            const uint16_t storedCrc =
                static_cast<uint16_t>(decoded[decodedLen - 2]) |
                (static_cast<uint16_t>(decoded[decodedLen - 1]) << 8);
            const uint16_t calcCrc =
                SignalUtils::calculateCrc(decoded, decodedLen - 2);

            if(calcCrc != storedCrc) {
                // 常見原因：Slave 端送出的封包沒有戳 CRC（如 Move 遙測漏填）
                if(HUB_LOG_VERBOSE && nowUs - rxBuf.lastLogUs > 500000ULL) {
                    rxBuf.lastLogUs = nowUs;
                    Serial.printf("[PIO RX][%s] CRC mismatch (calc=0x%04X, "
                                  "stored=0x%04X, len=%zu)\n",
                                  name, calcCrc, storedCrc, decodedLen);
                }
                rxBuf.len = 0;
                continue;
            }

            // 標頭檢查：sync 必須為 0xA5，否則視為雜訊丟棄
            if(decoded[0] != 0xA5) {
                if(HUB_LOG_VERBOSE && nowUs - rxBuf.lastLogUs > 500000ULL) {
                    rxBuf.lastLogUs = nowUs;
                    Serial.printf("[PIO RX][%s] bad sync 0x%02X\n", name,
                                  decoded[0]);
                }
                rxBuf.len = 0;
                continue;
            }

            const uint8_t msgType = decoded[1];

            // CRC 通過 — 依 msgType / 大小分發
            if(msgType == MsgType::TELEMETRY &&
               decodedLen == sizeof(MoveTelemetry)) {
                // 遙測封包 (72 bytes)
                memcpy(&telemetryOut, decoded, sizeof(MoveTelemetry));
                lastTmUs = nowUs;
                if(HUB_LOG_VERBOSE && nowUs - rxBuf.lastLogUs > 1000000ULL) {
                    rxBuf.lastLogUs = nowUs;
                    Serial.printf("[PIO RX][%s] telemetry OK: seq=%d t=%lu\n",
                                  name, telemetryOut.frameSeq,
                                  static_cast<unsigned long>(
                                      telemetryOut.timestamp));
                }
            } else if(decodedLen >= sizeof(ControlPacket)) {
                // 控制/心跳/Reboot/E-Stop 封包 (20 bytes)
                const ControlPacket &pkt =
                    *reinterpret_cast<const ControlPacket *>(decoded);
                // 心跳每 30ms 一次 → 只節流記錄，不洗版
                if(HUB_LOG_VERBOSE && pkt.msgType != MsgType::HEARTBEAT &&
                   nowUs - rxBuf.lastLogUs > 1000000ULL) {
                    rxBuf.lastLogUs = nowUs;
                    Serial.printf(
                        "[PIO RX][%s] pkt: msgType=%d seq=%d targetID=%d\n",
                        name, pkt.msgType, pkt.frameSeq, pkt.targetID);
                }
            } else if(HUB_LOG_VERBOSE &&
                      nowUs - rxBuf.lastLogUs > 500000ULL) {
                rxBuf.lastLogUs = nowUs;
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
                if(HUB_LOG_VERBOSE)
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
    if(HUB_LOG_VERBOSE)
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
static uint32_t jetsonFrameCount = 0;  // Jetson 有效封包累計（LED 判斷用）
static uint32_t jetsonDroppedCount = 0; // 因「遙控接管」而被丟棄的 Jetson 幀數
static uint8_t rcFallbackSeq = 0;      // RC 備援轉發用的 frameSeq
static uint64_t lastRcForwardUs = 0;   // RC 備援最後轉發時間

// ============================================================================
// 控制來源（本版規則）
//
//   JETSON      — Jetson 持有「全部」控制權：
//                 Move / Shoot / Storage 的所有命令、所有通道值都只來自 Jetson。
//                 遙控器只准讀 rc[5]（切換開關），其他 rc[0..4]、rc[6..9]
//                 一律不參與，連 RC Failsafe 也不會送 E-Stop。
//   RC_SWITCH   — rc[5] 切到遙控邊（< -10）：遙控器接管全部 Slave，
//                 Jetson 命令全部丟棄（仍保留心跳/時間戳）。
//   RC_FALLBACK — Jetson 逾時 >500ms：自動備援，遙控器接管。
//
// 判斷結果每圈 loop() 只算一次，onJetsonFrame() 與 RC 轉發都只看這個值，
// 避免「同一個 loop 內兩邊各判一次」造成兩邊同時送命令。
enum class ControlSource : uint8_t { JETSON = 0,
                                     RC_SWITCH,
                                     RC_FALLBACK };
static ControlSource gSource = ControlSource::JETSON;

// ============================================================================
// 狀態 LED（板載 LED_BUILTIN）— 與 Move Slave 相同的顯示邏輯
//
//   * E-Stop（Jetson 逾時且 RC Failsafe）→ 長亮
//   * 剛開機、從未收到 Jetson 幀 → 長亮
//   * 資料通訊中（< ACTIVE_TIMEOUT_MS）→ 1Hz 短脈衝
//   * 已連線但 Idle（Jetson 暫停送資料、RC 仍正常）→ 0.5Hz 慢閃
// ============================================================================
namespace LedCfg {
// 視為「資料通訊中」的視窗（略大於典型控制週期，避免閃斷）
constexpr uint32_t ACTIVE_TIMEOUT_MS = 150;
} // namespace LedCfg

enum class LedState : uint8_t { BOOT = 0,
                                ESTOP = 1,
                                IDLE = 2,
                                ACTIVE = 3 };

// 亮滅圖樣表：索引 = LedState 的數值（相位推進交給 StatusLed）
//   BOOT   → 長亮（剛開機、從未收到 Jetson 幀）
//   ESTOP  → 長亮（Jetson 逾時且 RC Failsafe）
//   IDLE   → 0.5Hz 慢閃，鎮定不擾人
//   ACTIVE → 1Hz 短脈衝心跳（像路由器資料燈）
static const StatusLed::Pattern kLedPatterns[] = {
    {0, 0, true},       // BOOT：長亮
    {0, 0, true},       // ESTOP：長亮
    {500, 1500, false}, // IDLE：亮 500ms / 熄 1500ms
    {80, 920, false},   // ACTIVE：亮 80ms / 熄 920ms
};
static constexpr uint8_t LED_PATTERN_COUNT =
    static_cast<uint8_t>(sizeof(kLedPatterns) / sizeof(kLedPatterns[0]));
static StatusLed statusLed(LED_BUILTIN, kLedPatterns, LED_PATTERN_COUNT);

static LedState currentLedState() {
    if(jetsonFrameCount == 0)
        return LedState::BOOT; // 開機至今從未收到 Jetson 有效幀
    if(time_us_64() - lastJetsonFrameUs <
       static_cast<uint64_t>(LedCfg::ACTIVE_TIMEOUT_MS) * 1000ULL)
        return LedState::ACTIVE; // Jetson 控制流正常
    if(rc_failsafe)
        return LedState::ESTOP; // Jetson 斷線且遙控也斷線
    return LedState::IDLE;      // Jetson 無資料，但遙控備援仍連線
}

static void updateStatusLed() {
    statusLed.update(static_cast<uint8_t>(currentLedState()), millis());
}

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
// RC 通道分配（ELRS CRSF 協定，值域 -100~100）
//
//   統一用語：RC 通道一律以 rc[N] 表示，N = 0~9（0-based）；
//   程式碼中的陣列名稱是 rc_channels[N]，N 就是這裡的同一組 0~9。
//   不要再使用「遙控器 CHx」這種 1-based 說法，避免差一位。
//
//   Move:
//     rc[0] → vx       左右平移
//     rc[1] → vy       前後移動
//     rc[6] → omega    cw/ccw 旋轉
//     （方向實測：前後正常；左右與 cw/ccw 需反相 VX_SIGN/OMEGA_SIGN = -1）
//
//   控制來源：
//     rc[5] → 控制來源切換：< -10 → 遙控接管全部 Slave
//                            其餘 → JETSON 模式（遙控器只讀這個開關，
//                                    rc[0..4]、rc[6..9] 完全不參與）
//
//   Shoot:
//     rc[2] → pitch          上下仰角
//     rc[3] → row            左右水平
//     rc[4] → 投射輪啟停：< 0 → 開，≥ 0 → 0（停轉）
//              開的時候轉速固定 50%（不隨通道值變動）
//     rc[6] → 上下射轉速微調（上下兩顆同時套用）：
//               < -10 → 每觸發一次 +5%
//               > +10 → 每觸發一次 -5%
//               範圍 0~100%，0% = 1500µs 停轉
//
//   Storage:
//     rc[8] → trigger        0/1（< 0 視為 1）
//     （motorChannel / sw 本版保留未使用）
//
//   ⚠ rc[6] 目前同時被 Move 的 omega 與 Shoot 的轉速微調使用（見 RcMap::OMEGA）。
//     若只拿來微調上下射轉速，請把 OMEGA 改成另一個空通道。
// ============================================================================

// 死區門檻（單一來源）：rc_channels[] 是 -100~100 的整數刻度。
// RcMoveCfg::DEADBAND（float 刻度）由這個值推導，兩者永遠一致
// （rcAxisToWire 同時用到 int 與 float 兩種型別）。
// 邊界語意：MathUtils::deadband 用嚴格不等式（|v| < db → 0），
// 所以 -10 / +10 仍是有效值，-9 / +9 才是 0。
static constexpr int16_t RC_DEADBAND = 10;

// 將 rc_channels[] 值映射為 int8（含死區 + 限幅）
static int8_t rcToInt8(int16_t rcVal) {
    return static_cast<int8_t>(
        constrain(MathUtils::deadband(rcVal, RC_DEADBAND), -100, 100));
}

// ============================================================================
// RC → Move 速度刻度
//
// ControlPacket.move 是 int8，SlaveComms 解碼時會乘上
// RobotConfig::MAX_RPS/127 (≈0.0551) 變成車身 RPS。
// 若把遙控 -100~100 直接塞進去，滿舵 = 5.5 RPS（約 1.7 m/s）太快，
// 且四輪只要有些微摩擦差就會偏。這裡先把「想要的車身 RPS」換成線路刻度。
// ============================================================================
// RcMap：rc_channels[] 索引（就是 rc[0]~rc[9] 的 0-based 索引）
//   VX              = rc[0] → 左右平移
//   VY              = rc[1] → 前後
//   OMEGA           = rc[6] → cw/ccw 旋轉
//   STORAGE_TRIGGER = rc[8] → < 0 觸發機構
//         若你的旋轉改插別的通道，把 OMEGA 換成對應的 0-based 索引即可。
namespace RcMap {
constexpr uint8_t VX = 0;              // rc[0]
constexpr uint8_t VY = 1;              // rc[1]
constexpr uint8_t OMEGA = 6;           // rc[6]
// Shoot 投射輪啟停：< 0 → 開（轉速固定 50%），≥ 0 → 0%（停轉）
constexpr uint8_t SHOOT_ENABLE = 4;    // rc[4]
// 控制來源切換：< -10 → 遙控優先（強制走 RC），其餘 → Jetson 優先
constexpr uint8_t SOURCE_SELECT = 5;   // rc[5]
// 上下射轉速微調：< -10 → +5%，> +10 → -5%（上下兩顆同時）
constexpr uint8_t SHOOT_SPEED = 6;     // rc[6]
// Storage 機構觸發：< 0 就開始「全部通道跑一次」（Storage 端做上緣觸發）
constexpr uint8_t STORAGE_TRIGGER = 8; // rc[8]
// 方向修正（實機實測）：前後正常，左右與 cw/ccw 相反 → 這兩個反相。
//   只影響「遙控備援」；Jetson/自駕送進來的 vx/omega 不受影響。
constexpr int8_t VX_SIGN = -1;
constexpr int8_t VY_SIGN = +1;
constexpr int8_t OMEGA_SIGN = -1;
// 來源切換門檻：rc_channels[SOURCE_SELECT] < SOURCE_RC_THRESHOLD → 遙控優先
constexpr int16_t SOURCE_RC_THRESHOLD = -10;
} // namespace RcMap

namespace RcMoveCfg {
constexpr float FULL_TRANSLATE_RPS = 1.20f; // 前後 / 左右平移滿舵 (RPS)
constexpr float FULL_OMEGA = 7.00f;         // 旋轉滿舵（firmware omega 刻度）
// 死區寬度：直接由 RC_DEADBAND 推導（單一來源，不要在此另寫一個數字，
// 否則 deadband 的邊界判定與這裡的重映射零點會不一致）
constexpr float DEADBAND = static_cast<float>(RC_DEADBAND);
} // namespace RcMoveCfg

// 車身 RPS → int8 線路刻度（Slave 端再乘 MAX_RPS/127）
static int8_t rpsToWire(float rps) {
    const float wire =
        MathUtils::clampf(rps * 127.0f / RobotConfig::MAX_RPS, -127.0f, 127.0f);
    return static_cast<int8_t>(lroundf(wire));
}

// rc_channels[] 值 -100~100 → 線路刻度（死區後 10..100 重新映射 0..100，過死區不跳動）
//   sign = ±1：實機方向修正（不影響 Jetson 路徑）
static int8_t rcAxisToWire(int16_t rcVal, float fullRange, int8_t sign = 1) {
    const float v = static_cast<float>(MathUtils::deadband(rcVal, RC_DEADBAND));
    if(v == 0.0f)
        return 0;
    const float span = 100.0f - RcMoveCfg::DEADBAND;
    const float mag = MathUtils::clampf(
        (fabsf(v) - RcMoveCfg::DEADBAND) / span, 0.0f, 1.0f);
    return rpsToWire((v > 0.0f ? mag : -mag) * fullRange *
                     static_cast<float>(sign));
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

    // ⚠ 舊版把 vx/vy 對調（推前進變橫移）。omega 依 rc[6]
    //    （cw/ccw 旋轉），可用上面 RcMap::OMEGA 一行改通道。
    pkt.move.vx = rcAxisToWire(rc_channels[RcMap::VX],
                               RcMoveCfg::FULL_TRANSLATE_RPS,
                               RcMap::VX_SIGN);
    pkt.move.vy = rcAxisToWire(rc_channels[RcMap::VY],
                               RcMoveCfg::FULL_TRANSLATE_RPS,
                               RcMap::VY_SIGN);
    pkt.move.omega = rcAxisToWire(rc_channels[RcMap::OMEGA],
                                  RcMoveCfg::FULL_OMEGA,
                                  RcMap::OMEGA_SIGN);

    pkt.crc16 = 0;
    pkt.crc16 = SignalUtils::calculateCrc(reinterpret_cast<uint8_t *>(&pkt),
                                          sizeof(ControlPacket) - 2);
    return pkt;
}

// ============================================================================
// RC → Shoot ControlPacket
//
// 通道對照（rc[N]，N = 0~9）：
//   rc[2] → pitch  上下仰角
//   rc[3] → row    左右水平
//   rc[4] → 投射輪啟停：< 0 → 開，≥ 0 → 0（停轉）
//   rc[6] → 上下射轉速微調（上下兩顆同時，範圍 0~100%）
//
//   ⚠ 舊版把 rc[1]（前後移動）同時灌進 up/down 兩顆無刷：
//     推前進時發射馬達也跟著轉，瞬間大電流造成電壓下陷 → 四輪出力不一致
//     → 直走會偏。現在投射輪只由 rc[4] 開關控制。
// ============================================================================
// 遙控備援時是否同時轉發 Shoot（步進兩軸）。現在 True：
//   rc_channels[2] → 仰角、rc_channels[3] → 水平，rc_channels[4] → 投射輪啟停。
static constexpr bool RC_FORWARD_SHOOT = true;

// ============================================================================
// 上下射轉速＝固定基準 50% + rc[6] 微調
//
//   rc[6] < -10 → 每「觸發一次」+5%（增轉速）
//   rc[6] > +10 → 每「觸發一次」-5%（減轉速）
//   觸發一次 = 方向邊緣（從中立/反向進入 <-10 或 >10 的那一次）
//   若希望「按住就連續微調」，把 HOLD_REPEAT_MS 設成 > 0（例如 250ms）
//
//   微調量與基準分開存（TRIM），所以拉到 100% 後不會「吃掉」步數：
//   回推同樣次數就回到 50%。上下兩顆無刷永遠同一個值。
// ============================================================================
namespace RcShootCfg {
constexpr int16_t BASE_PCT = 50;   // 固定基準轉速 (%)
constexpr int16_t STEP_PCT = 5;    // 每次微調量 (%)
constexpr int16_t TRIM_MIN = -50;  // 微調下限 → 50-50 = 0%
constexpr int16_t TRIM_MAX = +50;  // 微調上限 → 50+50 = 100%
constexpr int16_t DEADBAND = 10;   // |rc| <= 10 視為中立（不動作）
constexpr uint32_t HOLD_REPEAT_MS = 0; // 0 = 只做方向邊緣觸發
} // namespace RcShootCfg

static int16_t sShootTrim = 0;    // 微調量 (%)，Core 0 專用
static int16_t sShootStepDir = 0; // 上次方向：-1 增 / +1 減 / 0 中立
static uint32_t sShootStepMs = 0; // 上次步進時間（HOLD_REPEAT_MS 用）

// 清掉「方向邊緣」狀態，讓下次進入任一方向都算新的一次觸發
// （控制來源切換時呼叫，避免殘留的半次步進跨模式生效）
static void resetRcShootStepState() {
    sShootStepDir = 0;
    sShootStepMs = 0;
}

// 目前上下射轉速 (%)：固定 50% + 微調，限幅 0~100
static int16_t rcShootSpeedPct() {
    int16_t pct = RcShootCfg::BASE_PCT + sShootTrim;
    if(pct < 0)
        pct = 0;
    if(pct > 100)
        pct = 100;
    return pct;
}

// 讀 rc_channels[6] 並依邊緣（或按住）步進微調量
static void updateRcShootSpeed() {
    const int16_t rc = rc_channels[RcMap::SHOOT_SPEED];
    int16_t dir = 0;
    if(rc < -RcShootCfg::DEADBAND)
        dir = -1; // 增轉速
    else if(rc > RcShootCfg::DEADBAND)
        dir = +1; // 減轉速

    if(dir == 0) { // 回中立：下次進入任一方向都算新的一次觸發
        sShootStepDir = 0;
        return;
    }

    const uint32_t nowMs = millis();
    bool doStep = (dir != sShootStepDir); // 方向邊緣 → 立即一步
    if(!doStep && RcShootCfg::HOLD_REPEAT_MS > 0 &&
       (nowMs - sShootStepMs) >= RcShootCfg::HOLD_REPEAT_MS)
        doStep = true; // 按住連續微調
    sShootStepDir = dir;
    if(!doStep)
        return;
    sShootStepMs = nowMs;

    sShootTrim += (dir < 0) ? RcShootCfg::STEP_PCT : -RcShootCfg::STEP_PCT;
    if(sShootTrim < RcShootCfg::TRIM_MIN)
        sShootTrim = RcShootCfg::TRIM_MIN;
    if(sShootTrim > RcShootCfg::TRIM_MAX)
        sShootTrim = RcShootCfg::TRIM_MAX;

    Serial.printf("[HUB] 上下射轉速 → %d%%（微調 %d%%，rc[%u]=%d）\r\n",
                  static_cast<int>(rcShootSpeedPct()),
                  static_cast<int>(sShootTrim),
                  static_cast<unsigned>(RcMap::SHOOT_SPEED),
                  static_cast<int>(rc));
}

static ControlPacket rcToShootPacket() {
    // 先依 rc_channels[6] 更新微調（邊緣觸發，100Hz 呼叫也只會走一次）
    updateRcShootSpeed();

    ControlPacket pkt = {};
    pkt.sync = 0xA5;
    pkt.msgType = MsgType::CONTROL;
    pkt.frameSeq = rcFallbackSeq++;
    pkt.targetID = TargetID::SHOOT;
    pkt.payloadLen = sizeof(ControlPacket) - 6 - 2;

    // rc_channels[4] < 0 → 開啟（固定 50% + 微調）；≥ 0 → 0
    // （Shoot 端 0 = 1500µs 中立、停轉；速度會寫入 EEPROM 並即時套用）
    const int8_t shootSpeed =
        (rc_channels[RcMap::SHOOT_ENABLE] < 0)
            ? static_cast<int8_t>(rcShootSpeedPct())
            : static_cast<int8_t>(0);
    pkt.shoot.upSpeed = shootSpeed;
    pkt.shoot.downSpeed = shootSpeed;
    pkt.shoot.pitch = rcToInt8(rc_channels[2]); // rc_channels[2] 上下仰角
    pkt.shoot.row = rcToInt8(rc_channels[3]);   // rc_channels[3] 左右水平

    pkt.crc16 = 0;
    pkt.crc16 = SignalUtils::calculateCrc(reinterpret_cast<uint8_t *>(&pkt),
                                          sizeof(ControlPacket) - 2);
    return pkt;
}

// ============================================================================
// RC → Storage ControlPacket
//
//   rc[8] < 0 → trigger=1，Storage 在「上緣」觸發
//                 全部已啟用伺服通道跑一次循環
//   rc[8] ≥ 0 → trigger=0（Storage 停止並全部回 HOME）
//
// motorChannel / sw 本版 Storage 不使用，固定填 0。
// ============================================================================
static constexpr bool RC_FORWARD_STORAGE = true;

static ControlPacket rcToStoragePacket() {
    ControlPacket pkt = {};
    pkt.sync = 0xA5;
    pkt.msgType = MsgType::CONTROL;
    pkt.frameSeq = rcFallbackSeq++;
    pkt.targetID = TargetID::STORAGE;
    pkt.payloadLen = sizeof(ControlPacket) - 6 - 2;

    pkt.storage.motorChannel = 0; // 本版不使用
    pkt.storage.sw = 0;           // 本版不使用
    pkt.storage.trigger = (rc_channels[RcMap::STORAGE_TRIGGER] < 0) ? 1 : 0;

    pkt.crc16 = 0;
    pkt.crc16 = SignalUtils::calculateCrc(reinterpret_cast<uint8_t *>(&pkt),
                                          sizeof(ControlPacket) - 2);
    return pkt;
}

// ============================================================================
// 控制來源切換（rc[5]）
//
//   值 < -10 → 遙控接管：即使 Jetson 還在送幀，也一律走 RC 路徑
//   其餘      → Jetson 持有全部控制權（Jetson 逾時 >500ms 才自動切 RC 備援）
//
//   安全：遙控本身 Failsafe 時 rcSourceOverrideRequested() 回 false，
//         避免遙控斷線後鎖死在 RC；此時若 Jetson 也在線就交回 Jetson，
//         若 Jetson 也逾時則由 tryRcFallback() 送 E-Stop 廣播。
// ============================================================================
static bool rcSourceOverrideRequested() {
    if(rc_failsafe)
        return false;
    return rc_channels[RcMap::SOURCE_SELECT] < RcMap::SOURCE_RC_THRESHOLD;
}

// 每圈 loop() 只呼叫一次：算出本輪唯一的控制來源
static ControlSource evaluateControlSource(uint64_t nowUs) {
    if(rcSourceOverrideRequested())
        return ControlSource::RC_SWITCH;
    if(nowUs - lastJetsonFrameUs > RobotConfig::HEARTBEAT_TIMEOUT_US)
        return ControlSource::RC_FALLBACK;
    return ControlSource::JETSON;
}

static const char *controlSourceName(ControlSource s) {
    switch(s) {
    case ControlSource::RC_SWITCH:
        return "RC(ch5-switch)";
    case ControlSource::RC_FALLBACK:
        return "RC(jetson-timeout)";
    case ControlSource::JETSON:
    default:
        return "JETSON";
    }
}

// 來源切換時清掉「只屬於 RC 模式」的暫態，避免殘留進 Jetson 模式。
// （Shoot 轉速微調量 sShootTrim 刻意保留：下次切回遙控時接續上次的值）
static void onControlSourceChanged(ControlSource from, ControlSource to) {
    resetRcShootStepState();
    Serial.printf("[HUB] 控制來源 %s → %s（Jetson 模式下遙控器只讀 rc[%u] 開關）\n",
                  controlSourceName(from), controlSourceName(to),
                  static_cast<unsigned>(RcMap::SOURCE_SELECT));
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
    if(jetsonFrameCount < UINT32_MAX)
        jetsonFrameCount++;

    // 非 Jetson 模式（遙控開關接管 / Jetson 逾時備援）→ 丟棄 Jetson 命令。
    // 仍更新時間戳（上面），這樣切回 Jetson 模式時可立即判定 Jetson 是否還在線。
    if(gSource != ControlSource::JETSON) {
        if(jetsonDroppedCount < UINT32_MAX)
            jetsonDroppedCount++;
        return;
    }

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
    // rc_channels[2] → 上下仰角、rc_channels[3] → 左右水平；
    // rc_channels[4] < 0 → 投射輪開啟（固定 50% + rc_channels[6] 微調），否則停轉
    if(RC_FORWARD_SHOOT) {
        ControlPacket sh = rcToShootPacket();
        shootComms.sendPacket(sh);

        static uint64_t lastShootDbgUs = 0;
        uint64_t nowShoot = time_us_64();
        if(HUB_LOG_VERBOSE && nowShoot - lastShootDbgUs > 1000000ULL) {
            lastShootDbgUs = nowShoot;
            Serial.printf("[HUB] → Shoot pitch=%d row=%d "
                          "(rc[2]=%d rc[3]=%d) "
                          "up=%d down=%d (rc[4]=%d rc[6]=%d)\n",
                          sh.shoot.pitch, sh.shoot.row, rc_channels[2],
                          rc_channels[3], sh.shoot.upSpeed,
                          sh.shoot.downSpeed,
                          static_cast<int>(rc_channels[RcMap::SHOOT_ENABLE]),
                          static_cast<int>(rc_channels[RcMap::SHOOT_SPEED]));
        }
    }

    // 遙控正常 → 轉發控制幀給 Storage（PIO SM2, GP6 TX / GP7 RX）
    // rc[8] < 0 → trigger=1，
    // Storage 端以上緣觸發「全部通道跑一次」
    if(RC_FORWARD_STORAGE) {
        ControlPacket st = rcToStoragePacket();
        storageComms.sendPacket(st);

        static uint64_t lastStorageDbgUs = 0;
        static uint8_t lastTrigger = 0;
        uint64_t nowStorage = time_us_64();
        if(HUB_LOG_VERBOSE &&
           (nowStorage - lastStorageDbgUs > 1000000ULL ||
            st.storage.trigger != lastTrigger)) {
            lastStorageDbgUs = nowStorage;
            lastTrigger = st.storage.trigger;
            Serial.printf("[HUB] → Storage trigger=%u (rc_channels[%u]=%d)\n",
                          static_cast<unsigned>(st.storage.trigger),
                          static_cast<unsigned>(RcMap::STORAGE_TRIGGER),
                          rc_channels[RcMap::STORAGE_TRIGGER]);
        }
    }

    // 除錯：直接對照「讀到的 rc_channels」與「送出的 mv.move.vx」
    // 確認 rcToMovePacket 是否真的把 rc_channels[RcMap::VX] 塞進 vx
    static uint64_t lastPktDbgUs = 0;
    uint64_t nowPkt = time_us_64();
    if(HUB_LOG_VERBOSE && nowPkt - lastPktDbgUs > 1000000ULL) {
        lastPktDbgUs = nowPkt;
        const uint8_t *pr = reinterpret_cast<const uint8_t *>(&mv);
        Serial.printf("[HUB] rc_channels[%u]=%d rc_channels[%u]=%d "
                      "rc_channels[%u]=%d | mv.vx=%d mv.vy=%d "
                      "mv.omega=%d | byte6,7,8=%02X %02X %02X\n",
                      static_cast<unsigned>(RcMap::VX),
                      rc_channels[RcMap::VX],
                      static_cast<unsigned>(RcMap::VY),
                      rc_channels[RcMap::VY],
                      static_cast<unsigned>(RcMap::OMEGA),
                      rc_channels[RcMap::OMEGA],
                      mv.move.vx, mv.move.vy, mv.move.omega, pr[6], pr[7],
                      pr[8]);
    }

    // 每 1 秒印一次 send 統計 + PIO 狀態（判斷是否真的寫入 GP2）
    static uint64_t lastSendDbgUs = 0;
    uint64_t nowDbg = time_us_64();
    if(HUB_LOG_VERBOSE && nowDbg - lastSendDbgUs > 1000000ULL) {
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

    // 狀態 LED
    statusLed.begin();
}

// ============================================================================
// Core 0: loop() — 自動駕駛與路由大腦 + PIO RX 接收 + 遙測監控
//
// 1. jetsonComms.update() → USB CDC 讀取 + COBS 解包 + 回呼轉發 + 30ms 心跳
// 2. PIO UART RX 輪詢 — 接收各 Slave 回傳的 COBS 遙測幀
// 3. Slave 當機偵測 — 超過500ms 無遙測 → 發送 REBOOT
// 4. 控制來源每圈只算一次：Jetson 優先，rc[5] 切換或 Jetson 逾時才走 RC
// ============================================================================
void loop() {
    jetsonComms.update();

    // 狀態 LED（與 Move Slave 相同邏輯：E-Stop 長亮、通訊中快閃、Idle 慢閃）
    updateStatusLed();

    // 輪詢各 PIO UART RX，接收 Slave 回傳的 COBS 遙測幀
    processPioRx(moveUart, moveRxBuf, "Move", latestMoveTm, lastMoveTmUs);
    processPioRx(shootUart, shootRxBuf, "Shoot", latestShootTm, lastShootTmUs);
    processPioRx(storageUart, storageRxBuf, "Storage", latestStorageTm,
                 lastStorageTmUs);

    // Slave 當機偵測：逾時500ms 無遙測 → 發送 REBOOT
    checkSlaveTimeouts();

    uint64_t now = time_us_64();

    // ---- 控制來源：每圈只算一次，全域單一真相 ----
    //   JETSON      → 所有 Slave 全部聽 Jetson；遙控器只讀 rc[5] 開關
    //   RC_SWITCH   → rc[5] 切到遙控邊，遙控接管全部 Slave
    //   RC_FALLBACK → Jetson 逾時 >500ms 自動備援
    const ControlSource src = evaluateControlSource(now);
    if(src != gSource) {
        onControlSourceChanged(gSource, src);
        gSource = src;
    }

    // 只有非 Jetson 模式才會產生任何 RC 命令（Jetson 模式完全碰不到 RC 值）
    if(gSource != ControlSource::JETSON)
        tryRcFallback();

    // ---- 序列輸出：每 500ms 只印一行 RC 通道數據 ----
    static uint64_t lastDbgUs = 0;
    if(now - lastDbgUs > 500000ULL) {
        lastDbgUs = now;
        int16_t rcSnapshot[10];
        // 讀取 Core 1 的 RC 資料
        for(int i = 0; i < 10; i++)
            rcSnapshot[i] = rc_channels[i];
        Serial.printf("[HUB] rc[0..9]: %4d %4d %4d %4d %4d %4d %4d %4d %4d "
                      "%4d%s%s\n",
                      rcSnapshot[0], rcSnapshot[1], rcSnapshot[2],
                      rcSnapshot[3], rcSnapshot[4], rcSnapshot[5],
                      rcSnapshot[6], rcSnapshot[7], rcSnapshot[8],
                      rcSnapshot[9],
                      (gSource == ControlSource::JETSON) ? "" : "  [RC 接管]",
                      rc_failsafe ? "  (FAILSAFE)" : "");
        Serial.printf("[HUB] src=%s | shoot: enable(rc[4])=%d speed(rc[6])=%d%% "
                      "| jetsonFrames=%lu dropped=%lu\n",
                      controlSourceName(gSource),
                      static_cast<int>(rcSnapshot[RcMap::SHOOT_ENABLE]),
                      static_cast<int>(rcShootSpeedPct()),
                      static_cast<unsigned long>(jetsonFrameCount),
                      static_cast<unsigned long>(jetsonDroppedCount));
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
        int16_t rcSnapshot[10];
        crsf.getChannels(rcSnapshot);
        for(int i = 0; i < 10; i++) {
            rc_channels[i] = rcSnapshot[i];
        }
        rc_failsafe = crsf.isFailsafe();
    }
}
