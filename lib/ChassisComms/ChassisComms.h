#pragma once
#include "signal_shared.h"
#include <Arduino.h>

// ============================================================================
// ChassisComms — 通用 COBS 協議庫（支援控制與遙測）
//
// 根據 CLAUDE.md 規範，綁定到任意 Stream 介面，提供完整的 COBS 封幀
// 收發、CRC-CCITT 驗證、幀序號去重、與 30ms 心跳。
//
// 遙測支援：
//   - sendTelemetry()     向 Hub 發送 MoveTelemetry 封包（Slave 端呼叫）
//   - setTelemetryCallback() 註冊回呼以接收解碼後的 MoveTelemetry 幀（Hub 端呼叫）
//   - processFrame() 會自動辨識 msgType::TELEMETRY 並分發至遙測回呼
//
// 支援的 Stream 類型（任意實作 Stream 介面的物件）：
//   - Serial       (USB CDC — Hub 與 Jetson 通訊)
//   - SerialPIO    (PIO UART — Hub 與 Slaves 通訊, Slave 與 Hub 通訊)
//   - HardwareSerial (UART 週邊)
//
// 使用方式（Hub 端）：
//   static ChassisComms jetsonComms(&Serial, TargetID::HUB);
//   jetsonComms.begin();
//   jetsonComms.setFrameCallback(onJetsonFrame);
//   // 在 loop() 中呼叫：
//   jetsonComms.update();
//
// 使用方式（Slave 端）：
//   static SerialPIO uart(0, 0, PIN_TX);
//   static ChassisComms hubComms(&uart, TargetID::HUB);
//   hubComms.begin();
//   hubComms.setFrameCallback(onHubFrame);
//   // 在 loop() 中呼叫：
//   hubComms.update();
// ============================================================================
class ChassisComms {
  public:
    /// @param stream      綁定的 Stream（Serial / SerialPIO / HardwareSerial）
    /// @param hbTarget    心跳的 targetID（預設 TargetID::HUB）
    /// @param hbIntervalUs 心跳間隔微秒（預設 30000us = 30ms，設 0 停用心跳）
    ChassisComms(Stream *stream,
                 uint8_t hbTarget = TargetID::HUB,
                 uint64_t hbIntervalUs = RobotConfig::HEARTBEAT_INTERVAL_US);

    /// 開始運作（重設內部 COBS 掃描狀態、幀序號、心跳計時器）
    void begin();

    /// 每次主迴圈呼叫：
    ///   1. 從綁定的 Stream 讀取所有可用位元組
    ///   2. 掃描 COBS 分隔符 (0x00)
    ///   3. 遇到分隔符 → COBS 解碼 → CRC 驗證 → 回呼分發
    ///   4. 若達心跳間隔 → 產生心跳 ControlPacket
    void update();

    /// 收到有效解碼遙測幀時觸發的回呼
    using TelemetryCallback = void (*)(const MoveTelemetry &telemetry);
    void setTelemetryCallback(TelemetryCallback cb) { telemetryCb = cb; }

    /// 發送 MoveTelemetry 遙測封包（COBS 編碼 + 寫入 Stream）
    /// 由 Slave 端在 Core 1 中呼叫，將遙測資料回傳給 Hub
    bool sendTelemetry(const MoveTelemetry &telemetry);

    /// 收到有效解碼幀時觸發的回呼
    using FrameCallback = void (*)(const ControlPacket &pkt);
    void setFrameCallback(FrameCallback cb) { frameCb = cb; }

    // ---- RX 診斷（給 Slave 端序列監控用）----
    /// 通過 CRC 並送入 processFrame 的幀數（含重複/非本機 targetID）
    uint32_t rxFrameCount() const { return rxFrameCount_; }
    /// CRC 驗證失敗的幀數（接線/鮑率/對端未戳 CRC 時會持續增加）
    uint32_t rxCrcErrorCount() const { return rxCrcErrorCount_; }
    /// 最後一個通過 CRC 的幀的 msgType / targetID / frameSeq
    uint8_t lastRxMsgType() const { return lastMsgType_; }
    uint8_t lastRxTargetID() const { return lastTargetID_; }
    uint8_t lastRxFrameSeq() const { return lastFrameSeq_; }

    /// 發送 ControlPacket（COBS 編碼 + CRC 戳記 + 寫入 Stream）
    /// 非阻塞：若 Stream 有剩餘 TX 空間則寫入，否則靜默丟棄
    /// @return true = 已寫入, false = 已丟棄
    bool sendPacket(const ControlPacket &pkt);

  private:
    ChassisComms(const ChassisComms &) = delete;
    ChassisComms &operator=(const ChassisComms &) = delete;

    void processFrame(const uint8_t *data, size_t len);

    Stream *stream;
    uint8_t hbTargetID;
    uint64_t hbIntervalUs;

    // COBS 掃描狀態
    uint8_t parseWorkBuf[512];
    size_t workBufIdx = 0;

    // 幀追蹤
    FrameCallback frameCb = nullptr;
    TelemetryCallback telemetryCb = nullptr;
    uint8_t lastFrameSeq = 0;
    bool firstFrame = true;

    // 心跳
    uint64_t lastHeartbeatTime = 0;
    uint8_t txFrameSeq = 0;

    // 遙測幀序號（sendTelemetry 內部戳記）
    uint8_t telemetrySeq = 0;

    // RX 診斷計數
    uint32_t rxFrameCount_ = 0;
    uint32_t rxCrcErrorCount_ = 0;
    uint8_t lastMsgType_ = 0;
    uint8_t lastTargetID_ = 0;
    uint8_t lastFrameSeq_ = 0;
};
