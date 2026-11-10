#pragma once
#include <cstdint>
#include <cstddef>
#include <Arduino.h>

// ============================================================================
// RobotConfig — 系統層級常數（鮑率、時序、馬達限制）
// 根據 CLAUDE.md 拓撲：
//   Hub ↔ Jetson：  USB CDC (Serial)
//   Hub ↔ Slaves：  PIO UART (GP2/GP3, GP4/GP5, GP6/GP7)
//   Slave UART0：   預設腳位 TX=GP0, RX=GP1
//   除錯調試：      硬體 UART1 (GP4/GP5)
// ============================================================================
namespace RobotConfig {

// 鮑率，對應 CLAUDE.md UART 表格
constexpr uint32_t JETSON_BAUDRATE = 921600U; // Hub ↔ Jetson：USB CDC (Serial)
constexpr uint32_t PIO_BAUDRATE = 921600U;    // Hub ↔ Slaves：PIO UART
constexpr uint32_t DEBUG_BAUDRATE = 115200U;  // 除錯調試 UART1 (GP4 TX, GP5 RX)

// 時序 — CLAUDE.md §超時保護 & §心跳
constexpr uint32_t CONTROL_DT_MS = 10U;              // 100Hz 控制週期
constexpr uint64_t COMMAND_TIMEOUT_US = 100000ULL;   // 100ms → 緊急停止
constexpr uint64_t HEARTBEAT_TIMEOUT_US = 500000ULL; // 500ms 逾時
constexpr uint64_t HEARTBEAT_INTERVAL_US = 30000ULL; // 30ms 發送間隔

// 馬達 / PID 極限值
constexpr float MAX_RPS = 7.0f;
constexpr float KF_GAIN = 21.5f;
constexpr float PID_OUT_LIMIT = 30.0f;
constexpr float SLEW_RATE_LIMIT = 50.0f;
constexpr uint16_t MOTOR_MAX_DUTY = 40000U;

// COBS 協定限制
constexpr size_t COBS_MAX_PAYLOAD = 254U;
constexpr size_t COBS_ENCODED_MAX = 256U;

} // namespace RobotConfig

// ============================================================================
// 協定常數 — 根據 CLAUDE.md §二進位結構體
// ============================================================================

namespace MsgType {
constexpr uint8_t HEARTBEAT = 0x01;
constexpr uint8_t CONTROL = 0x02;
constexpr uint8_t TELEMETRY = 0x03;
constexpr uint8_t REBOOT = 0x04; // Hub → Slave：觸發 watchdog 重開機
constexpr uint8_t ESTOP = 0xFF;
} // namespace MsgType

namespace TargetID {
constexpr uint8_t HUB = 0x00;
constexpr uint8_t MOVE = 0x01;
constexpr uint8_t SHOOT = 0x02;
constexpr uint8_t STORAGE = 0x03;
constexpr uint8_t BROADCAST = 0xFF;
} // namespace TargetID

namespace RobotMode {
constexpr uint8_t MANUAL = 0;
constexpr uint8_t AUTO = 1;
constexpr uint8_t FAILSAFE = 2;
} // namespace RobotMode

// ============================================================================
// ControlPacket — 線路格式封包，根據 CLAUDE.md 協定規範
//
// 標頭：sync(0xA5)、msgType、frameSeq、targetID、payloadLen
// CRC16 在封包尾部（payload 之後），計算範圍不含 crc16 欄位自身。
// 使用小端序位元組順序（RP2350 原生）。
//
// 注意：payload 部份是 union，依 targetID 不同有不同的欄位。
//       所有變體共用同一塊記憶體，總大小固定 20 bytes（含標頭 + CRC）。
// ============================================================================

// Move 控制資料 (3 bytes)
struct MovePayload {
    int8_t vx;
    int8_t vy;
    int8_t omega;
};

// Shoot 控制資料 (4 bytes)
struct ShootPayload {
    int8_t upSpeed;   // 上射速度 -100~100
    int8_t downSpeed; // 下射速度 -100~100
    int8_t pitch;     // 俯仰 -100~100
    int8_t row;       // 旋轉 -100~100
};

// Storage 控制資料 (3 bytes)
struct StoragePayload {
    uint8_t motorChannel; // PCA9685 通道編號 0~15
    uint8_t sw;           // switch 0/1
    uint8_t trigger;      // trigger 0/1
};

// 完整的 ControlPacket（20 bytes packed）
struct ControlPacket {
    uint8_t sync;        // 0xA5 幀同步位元組
    uint8_t msgType;     // MsgType 常數
    uint8_t frameSeq;    // 遞增序號（繞回至 0）— 抗重複
    uint8_t targetID;    // TargetID 常數
    uint16_t payloadLen; // 固定為 sizeof(ControlPacket) - 6 - 2 = 12

    union {
        MovePayload move;       // targetID == MOVE (3 bytes)
        ShootPayload shoot;     // targetID == SHOOT (4 bytes)
        StoragePayload storage; // targetID == STORAGE (3 bytes)
        uint8_t raw[12];        // 12 bytes payload 空間
    };

    uint16_t crc16; // 封包尾部 CRC-CCITT 校驗（不包含自身）
} __attribute__((packed));

static_assert(sizeof(ControlPacket) == 20, "ControlPacket 存在填充位元組！");
static_assert(sizeof(MovePayload) == 3, "MovePayload 不為 3 bytes！");
static_assert(sizeof(ShootPayload) == 4, "ShootPayload 不為 4 bytes！");
static_assert(sizeof(StoragePayload) == 3, "StoragePayload 不為 3 bytes！");

// ============================================================================
// MoveTelemetry — Slave → Hub 遙測封包（線路格式）
// Slave 定期（預設 50ms）透過 COBS 回傳至 Hub，Hub 再轉發至 Jetson。
// CRC16 在封包尾部，計算範圍不含 crc16 欄位自身。
// ============================================================================
struct MoveTelemetry {
    uint8_t sync;     // 0xA5
    uint8_t msgType;  // MsgType::TELEMETRY
    uint8_t frameSeq; // 遞增序號
    uint8_t targetID; // TargetID::HUB

    float currentRps[4]; // 四輪當前轉速 (RPS)
    float targetRps[4];  // 四輪期望轉速 (RPS)
    float pwmOutput[4];  // 四輪 PID+FF 輸出（歸一化）
    float cmdVx;         // 當前車身速度命令 (RPS)
    float cmdVy;         // 當前車身速度命令 (RPS)
    float cmdOmega;      // 當前車身角速度命令 (rad/s)
    uint8_t mode;        // RobotMode
    uint8_t estopFlag;   // 0=正常, 1=E-Stop
    uint32_t timestamp;  // 取樣時間戳 (us)

    uint16_t crc16; // 封包尾部 CRC-CCITT（不包含自身）
} __attribute__((packed));

// 預期大小 = 2 (sync+msgType) + 2 (frameSeq+targetID) + 4*4*3 (currentRps/targetRps/pwmOutput) + 4*3 (cmdVx/vy/omega) + 2 (mode+estopFlag) + 4 (timestamp) + 2 (crc16)
// = 2+2+48+12+2+4+2 = 72 bytes
static_assert(sizeof(MoveTelemetry) == 72, "MoveTelemetry 大小不正確！");

// ============================================================================
// ChassisSnapshot — 記憶體內解碼後的命令（非線路格式）
// 內部用於核心間命令傳遞，根據 CLAUDE.md §核心間通訊
//
// 所有 Slave 共用同一個結構，但依 targetID 使用不同欄位：
//   MOVE:    vx, vy, omega
//   SHOOT:   shootUp, shootDown, pitch, row
//   STORAGE: motorChannel, sw, trigger
// ============================================================================
struct ChassisSnapshot {
    // 共用控制值
    float vx;    // MOVE: 線速度 X (RPS)
    float vy;    // MOVE: 線速度 Y (RPS)
    float omega; // MOVE: 角速度 (rad/s)

    // SHOOT 專用
    float shootUpSpeed;   // 上射速度 (-100~100)
    float shootDownSpeed; // 下射速度 (-100~100)
    float pitch;          // 俯仰 (-100~100)
    float row;            // 旋轉 (-100~100)

    // STORAGE 專用
    uint8_t motorChannel; // PCA9685 通道編號 0~15
    uint8_t sw;           // switch 0/1
    uint8_t trigger;      // trigger 0/1

    // 通用
    uint8_t frameSeq;
    uint64_t lastArrivalTick;
};

// ============================================================================
// SignalUtils — CRC-CCITT（多項式 0x1021）及 COBS 編解碼
//
// CRC 表為編譯期常數，儲存在 Flash 中 — 所有編譯單元共用同一份。
// 無需呼叫 initCrcTable()，不浪費 .bss 空間。
// ============================================================================
namespace SignalUtils {

// CRC-CCITT 查找表（多項式 0x1021），可常駐 Flash。
// constexpr 保證編譯期計算 — 零執行時期初始化，
// 使編譯器能積極內聯與預取。
static constexpr uint16_t crcTable[256] = {
    0x0000,
    0x1021,
    0x2042,
    0x3063,
    0x4084,
    0x50A5,
    0x60C6,
    0x70E7,
    0x8108,
    0x9129,
    0xA14A,
    0xB16B,
    0xC18C,
    0xD1AD,
    0xE1CE,
    0xF1EF,
    0x1231,
    0x0210,
    0x3273,
    0x2252,
    0x52B5,
    0x4294,
    0x72F7,
    0x62D6,
    0x9339,
    0x8318,
    0xB37B,
    0xA35A,
    0xD3BD,
    0xC39C,
    0xF3FF,
    0xE3DE,
    0x2462,
    0x3443,
    0x0420,
    0x1401,
    0x64E6,
    0x74C7,
    0x44A4,
    0x5485,
    0xA56A,
    0xB54B,
    0x8528,
    0x9509,
    0xE5EE,
    0xF5CF,
    0xC5AC,
    0xD58D,
    0x3653,
    0x2672,
    0x1611,
    0x0630,
    0x76D7,
    0x66F6,
    0x5695,
    0x46B4,
    0xB75B,
    0xA77A,
    0x9719,
    0x8738,
    0xF7DF,
    0xE7FE,
    0xD79D,
    0xC7BC,
    0x48C4,
    0x58E5,
    0x6886,
    0x78A7,
    0x0840,
    0x1861,
    0x2802,
    0x3823,
    0xC9CC,
    0xD9ED,
    0xE98E,
    0xF9AF,
    0x8948,
    0x9969,
    0xA90A,
    0xB92B,
    0x5AF5,
    0x4AD4,
    0x7AB7,
    0x6A96,
    0x1A71,
    0x0A50,
    0x3A33,
    0x2A12,
    0xDBFD,
    0xCBDC,
    0xFBBF,
    0xEB9E,
    0x9B79,
    0x8B58,
    0xBB3B,
    0xAB1A,
    0x6CA6,
    0x7C87,
    0x4CE4,
    0x5CC5,
    0x2C22,
    0x3C03,
    0x0C60,
    0x1C41,
    0xEDAE,
    0xFD8F,
    0xCDEC,
    0xDDCD,
    0xAD2A,
    0xBD0B,
    0x8D68,
    0x9D49,
    0x7E97,
    0x6EB6,
    0x5ED7,
    0x4EF4,
    0x3E13,
    0x2E32,
    0x1E51,
    0x0E70,
    0xFF9F,
    0xEFBE,
    0xDFDD,
    0xCFFC,
    0xBF1B,
    0xAF3A,
    0x9F59,
    0x8F78,
    0x9188,
    0x81A9,
    0xB1CA,
    0xA1EB,
    0xD10C,
    0xC12D,
    0xF14E,
    0xE16F,
    0x1080,
    0x00A1,
    0x30C2,
    0x20E3,
    0x5004,
    0x4025,
    0x7046,
    0x6067,
    0x83B9,
    0x9398,
    0xA3FB,
    0xB3DA,
    0xC33D,
    0xD31C,
    0xE37F,
    0xF35E,
    0x02B1,
    0x1290,
    0x22F3,
    0x32D2,
    0x4235,
    0x5214,
    0x6277,
    0x7256,
    0xB5EA,
    0xA5CB,
    0x95A8,
    0x8589,
    0xF56E,
    0xE54F,
    0xD52C,
    0xC50D,
    0x34E2,
    0x24C3,
    0x14A0,
    0x0481,
    0x7466,
    0x6447,
    0x5424,
    0x4405,
    0xA7DB,
    0xB7FA,
    0x8799,
    0x97B8,
    0xE75F,
    0xF77E,
    0xC71D,
    0xD73C,
    0x26D3,
    0x36F2,
    0x0691,
    0x16B0,
    0x6657,
    0x7676,
    0x4615,
    0x5634,
    0xD94C,
    0xC96D,
    0xF90E,
    0xE92F,
    0x99C8,
    0x89E9,
    0xB98A,
    0xA9AB,
    0x5844,
    0x4865,
    0x7806,
    0x6827,
    0x18C0,
    0x08E1,
    0x3882,
    0x28A3,
    0xCB7D,
    0xDB5C,
    0xEB3F,
    0xFB1E,
    0x8BF9,
    0x9BD8,
    0xABBB,
    0xBB9A,
    0x4A75,
    0x5A54,
    0x6A37,
    0x7A16,
    0x0AF1,
    0x1AD0,
    0x2AB3,
    0x3A92,
    0xFD2E,
    0xED0F,
    0xDD6C,
    0xCD4D,
    0xBDAA,
    0xAD8B,
    0x9DE8,
    0x8DC9,
    0x7C26,
    0x6C07,
    0x5C64,
    0x4C45,
    0x3CA2,
    0x2C83,
    0x1CE0,
    0x0CC1,
    0xEF1F,
    0xFF3E,
    0xCF5D,
    0xDF7C,
    0xAF9B,
    0xBFBA,
    0x8FD9,
    0x9FF8,
    0x6E17,
    0x7E36,
    0x4E55,
    0x5E74,
    0x2E93,
    0x3EB2,
    0x0ED1,
    0x1EF0,
};

// 計算 CRC-CCITT（初始值 0xFFFF）。
// 注意：len 應為 sizeof(封包) - 2 以排除 crc16 欄位。
inline uint16_t calculateCrc(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFFU;
    for(size_t i = 0; i < len; i++) {
        crc = static_cast<uint16_t>(
            (crc << 8) ^ crcTable[((crc >> 8) ^ data[i]) & 0xFF]);
    }
    return crc;
}

// 遙測封包收尾：一次填好標頭（sync / msgType / frameSeq / targetID）＋時間戳
// ＋CRC16（計算範圍不含 crc16 自身）。
//
// 為什麼需要：Move / Shoot / Storage 的 buildTelemetry() 都手寫同樣 6 行標頭與
// CRC，只要漏填 sync 或忘記算 CRC，Hub 端就會整包丟棄（序列監控只看到
// "CRC mismatch"，該節點完全隱形）。集中成一個 inline 函式避免漏填。
//
// ⚠ timestamp 由呼叫端傳入（多數節點是 time_us_64() & 0xFFFFFFFF）：本檔不依賴
//   pico/Arduino 的時間函式，超音波等獨立節點也能共用此標頭。
//
// ⚠ frameSeq 注意：本函式填的 frameSeq 不會出現在實際上線的封包裡。
//   ChassisComms::sendTelemetry() 送出前會整組重新戳記
//   sync / msgType / frameSeq / targetID / crc16（見 lib/ChassisComms/ChassisComms.cpp），
//   線路上的序號由 ChassisComms 內部的 telemetrySeq 遞增。也就是說呼叫端
//   傳進來的 frameSeq 只是「填滿欄位」用，不影響去重/排序；真正生效的是
//   ChassisComms 的序號。若日後要自己控制序號，必須改 sendTelemetry()，
//   改這裡沒用。
inline void finalizeTelemetry(MoveTelemetry &tm, uint8_t frameSeq,
                              uint32_t timestampUs) {
    tm.sync = 0xA5;
    tm.msgType = MsgType::TELEMETRY;
    tm.frameSeq = frameSeq;
    tm.targetID = TargetID::HUB;
    tm.timestamp = timestampUs;
    tm.crc16 = 0; // 先歸零，CRC 才不會把自己的殘值算進去
    tm.crc16 = calculateCrc(reinterpret_cast<const uint8_t *>(&tm),
                            sizeof(MoveTelemetry) - 2);
}

// COBS 編碼：回傳編碼後的位元組數。
// dst 至少需 len + len / 254 + 2 個位元組。
inline size_t cobsEncode(const uint8_t *src, size_t len, uint8_t *dst) {
    size_t codeIdx = 0;
    size_t readIdx = 0;
    size_t writeIdx = 1;
    uint8_t code = 1;

    while(readIdx < len) {
        if(src[readIdx] == 0x00) {
            dst[codeIdx] = code;
            code = 1;
            codeIdx = writeIdx++;
            readIdx++;
        } else {
            dst[writeIdx++] = src[readIdx++];
            code++;
            if(code == 0xFF) {
                dst[codeIdx] = code;
                code = 1;
                codeIdx = writeIdx++;
            }
        }
    }
    dst[codeIdx] = code;
    return writeIdx;
}

// COBS 解碼：回傳解碼後的位元組數（錯誤時回傳 0）。
//
// ** 深度防禦 **
//   1. 進入內部複製迴圈前，驗證剩餘來源位元組是否足夠：readIdx + (code-1) <= len
//   2. 追蹤 writeIdx 不超過 512 位元組上限（所有呼叫者均使用 512 位元組 dst 緩衝區）。
//      這能捕捉 EMI 位元翻轉造成的損毀 code 值，避免溢寫 dst 而觸發 HardFault。
//
// 兩種防護都在失敗時回傳 0（解碼失敗）— 由呼叫者丟棄該幀。
static constexpr size_t COBS_DST_MAX = 512;

inline size_t cobsDecode(const uint8_t *src, size_t len, uint8_t *dst) {
    size_t readIdx = 0;
    size_t writeIdx = 0;

    while(readIdx < len) {
        uint8_t code = src[readIdx];
        size_t blockLen = static_cast<size_t>(code);

        // 防護 1：來源位元組數不足，無法滿足宣告的區塊長度
        if(readIdx + blockLen > len) {
            return 0;
        }
        readIdx++;

        // 防護 2：目的地將溢位（EMI 損毀的 code 值）
        // blockLen == 1  → 無資料位元組需複製（僅為零值標記）
        // blockLen >= 2  → 將寫入 (blockLen-1) 個資料位元組
        if(writeIdx + (blockLen - 1) > COBS_DST_MAX) {
            return 0;
        }

        for(uint8_t i = 1; i < code; i++)
            dst[writeIdx++] = src[readIdx++];

        // 插入隱含的零值（除非這是最後一個區塊標記）
        if(code < 0xFF && readIdx < len) {
            if(writeIdx >= COBS_DST_MAX)
                return 0;
            dst[writeIdx++] = 0x00;
        }
    }
    return writeIdx;
}

} // namespace SignalUtils
