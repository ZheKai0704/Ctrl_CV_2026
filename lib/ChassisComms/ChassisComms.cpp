#include "ChassisComms.h"
#include <cstring>

ChassisComms::ChassisComms(Stream *stream,
                           uint8_t hbTarget,
                           uint64_t hbIntervalUs)
    : stream(stream),
      hbTargetID(hbTarget),
      hbIntervalUs(hbIntervalUs) {}

void ChassisComms::begin() {
    // 重設 COBS 掃描狀態
    workBufIdx = 0;
    firstFrame = true;
    lastFrameSeq = 0;

    // 重設心跳計時器
    lastHeartbeatTime = 0;
    txFrameSeq = 0;
}

void ChassisComms::processFrame(const uint8_t *data, size_t len) {
    // Check for telemetry frame (larger than ControlPacket)
    if(len == sizeof(MoveTelemetry)) {
        const MoveTelemetry &tm = *reinterpret_cast<const MoveTelemetry *>(data);
        if(tm.sync == 0xA5 && tm.msgType == MsgType::TELEMETRY) {
            if(telemetryCb != nullptr)
                telemetryCb(tm);
            return;
        }
    }

    if(len != sizeof(ControlPacket))
        return;

    const ControlPacket &pkt = *reinterpret_cast<const ControlPacket *>(data);
    if(pkt.sync != 0xA5)
        return;

    // RX 診斷：記錄最後一個通過 CRC 的幀（Core 0 寫入，Core 0 讀取）
    rxFrameCount_++;
    lastMsgType_ = pkt.msgType;
    lastTargetID_ = pkt.targetID;
    lastFrameSeq_ = pkt.frameSeq;

    // E-Stop 為中斷事件：跳過幀序號去重，每次都必須傳遞（CLAUDE.md §E-Stop）
    if(pkt.msgType == MsgType::ESTOP) {
        if(frameCb != nullptr)
            frameCb(pkt);
        return;
    }

    // 幀序號抗重複 (CLAUDE.md §幀序號抗干擾)
    if(!firstFrame && pkt.frameSeq == lastFrameSeq)
        return;
    firstFrame = false;
    lastFrameSeq = pkt.frameSeq;

    if(frameCb != nullptr)
        frameCb(pkt);
}

bool ChassisComms::sendTelemetry(const MoveTelemetry &telemetry) {
    if(stream == nullptr)
        return false;

    // 集中戳記標頭 + CRC：呼叫端就算忘了填 sync/msgType/targetID/crc16，
    // 送出的遙測仍能通過 Hub 的 CRC 驗證（Move 端曾漏填 → Hub 全部丟棄）。
    MoveTelemetry tm = telemetry;
    tm.sync = 0xA5;
    tm.msgType = MsgType::TELEMETRY;
    tm.targetID = TargetID::HUB;
    tm.frameSeq = telemetrySeq++;
    tm.crc16 = 0;
    tm.crc16 = SignalUtils::calculateCrc(
        reinterpret_cast<const uint8_t *>(&tm), sizeof(MoveTelemetry) - 2);

    uint8_t cobsBuf[sizeof(MoveTelemetry) + 2];
    size_t cobsLen = SignalUtils::cobsEncode(
        reinterpret_cast<const uint8_t *>(&tm),
        sizeof(MoveTelemetry),
        cobsBuf);
    cobsBuf[cobsLen] = 0x00;

    size_t totalBytes = cobsLen + 1;

    // 同 sendPacket：檢查至少 1 byte 空間即可嘗試送整包（SerialPIO FIFO 僅 8）
    if(stream == nullptr || stream->availableForWrite() <= 0) {
        return false;
    }
    size_t written = stream->write(cobsBuf, totalBytes);
    return written == totalBytes;
}

bool ChassisComms::sendPacket(const ControlPacket &source) {
    if(stream == nullptr)
        return false;

    // 集中戳記 CRC：確保送出的每一幀都帶正確校驗值。
    // （呼叫端可自行先填，重算結果相同；漏填時也不會送出壞幀）
    ControlPacket pkt = source;
    pkt.crc16 = 0;
    pkt.crc16 = SignalUtils::calculateCrc(
        reinterpret_cast<const uint8_t *>(&pkt), sizeof(ControlPacket) - 2);

    uint8_t cobsBuf[sizeof(ControlPacket) + 2];
    size_t cobsLen = SignalUtils::cobsEncode(
        reinterpret_cast<const uint8_t *>(&pkt),
        sizeof(ControlPacket),
        cobsBuf);
    cobsBuf[cobsLen] = 0x00;

    size_t totalBytes = cobsLen + 1;

    // 非阻塞寫入：只要有餘裕寫入至少 1 byte 即嘗試送整包。
    // (註) 不能檢查 availableForWrite() >= totalBytes：
    //   SerialPIO 的 TX FIFO 僅 8 bytes，遠小於一包 (~22 bytes)，
    //   若要求整包空間會永遠失敗而靜默丟棄 → PIO 鏈路完全不通。
    //   SerialPIO::write() 內部為 pio_sm_put_blocking，會自行等 FIFO 空間。
    if(stream == nullptr || stream->availableForWrite() <= 0) {
        return false; // 完全無緩衝空間 → 丟棄
    }
    size_t written = stream->write(cobsBuf, totalBytes);
    return written == totalBytes;
}

void ChassisComms::update() {
    // ---- 1. 從綁定的 Stream 讀取 ----
    if(stream == nullptr)
        return;

    int avail = stream->available();
    if(avail > 0) {
        // 每次最多處理 64 bytes，避免佔用 loop() 太久
        int toRead = (avail > 64) ? 64 : avail;
        for(int i = 0; i < toRead; i++) {
            int byte = stream->read();
            if(byte < 0)
                break;

            uint8_t b = static_cast<uint8_t>(byte);

            if(b == 0x00) {
                // COBS 幀結束 — 嘗試解碼
                if(workBufIdx >= sizeof(ControlPacket)) {
                    uint8_t decoded[512];
                    size_t decLen = SignalUtils::cobsDecode(parseWorkBuf,
                                                            workBufIdx,
                                                            decoded);
                    if(decLen >= sizeof(ControlPacket)) {
                        // CRC 計算範圍：整個解碼後的資料除去尾部2 bytes (crc16)
                        // 用 memcpy 取值，避免 unaligned 讀取告警。
                        uint16_t storedCrc = 0;
                        memcpy(&storedCrc, decoded + decLen - 2, sizeof(uint16_t));
                        uint16_t calcCrc = SignalUtils::calculateCrc(
                            decoded,
                            decLen - 2);
                        if(storedCrc == calcCrc) {
                            processFrame(decoded, decLen);
                        } else {
                            // CRC 失敗 → 靜默丟棄 (CLAUDE.md 規則)，僅計數供診斷
                            rxCrcErrorCount_++;
                        }
                    }
                }
                workBufIdx = 0;
            } else {
                if(workBufIdx < sizeof(parseWorkBuf) - 1)
                    parseWorkBuf[workBufIdx++] = b;
            }
        }
    }

    // ---- 2. 心跳 (CLAUDE.md §心跳) ----
    if(hbIntervalUs == 0)
        return; // 停用心跳

    uint64_t now = time_us_64();
    if(now - lastHeartbeatTime >= hbIntervalUs) {
        lastHeartbeatTime = now;

        // 注意：必須值初始化，否則 payload 後 9 bytes 是未定義的堆疊殘值，
        // 會帶著垃圾資料上線（CRC 仍正確，但內容不該外洩）。
        ControlPacket pkt = {};
        pkt.sync = 0xA5;
        pkt.msgType = MsgType::HEARTBEAT;
        pkt.frameSeq = txFrameSeq++;
        pkt.targetID = hbTargetID;
        pkt.payloadLen = sizeof(ControlPacket) - 6 - 2;

        sendPacket(pkt);
    }
}
