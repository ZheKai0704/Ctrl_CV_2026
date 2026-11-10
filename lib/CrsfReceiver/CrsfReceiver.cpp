#include "CrsfReceiver.h"
#include <hardware/irq.h>
#include <hardware/uart.h>

// 常數定義
static constexpr uint8_t CRSF_SYNC_BYTE = 0xC8;
static constexpr uint8_t CRSF_TYPE_RC = 0x16;
static constexpr int16_t RC_CENTER = 992;
static constexpr int16_t RC_RANGE = 820;
static constexpr unsigned long FAILSAFE_TIMEOUT_MS = 500;
static constexpr size_t RING_SIZE = 256;

// 因為 RP2350 中斷需要靜態或全域的快取，我們在這裡建立全域環形緩衝區
static uint8_t ringBuf0[RING_SIZE];
static volatile size_t ringHead0 = 0, ringTail0 = 0;

static uint8_t ringBuf1[RING_SIZE];
static volatile size_t ringHead1 = 0, ringTail1 = 0;

// CRSF CRC 查找表
static constexpr uint8_t crsfCrcTable[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83,
    0xD7, 0x02, 0xA8, 0x7D, 0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F, 0xA4, 0x71, 0xDB, 0x0E,
    0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75,
    0x21, 0xF4, 0x5E, 0x8B, 0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0, 0xCF, 0x1A, 0xB0, 0x65,
    0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA,
    0xEE, 0x3B, 0x91, 0x44, 0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16, 0xEF, 0x3A, 0x90, 0x45,
    0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E,
    0x6A, 0xBF, 0x15, 0xC0, 0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36, 0x19, 0xCC, 0x66, 0xB3,
    0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1,
    0xA5, 0x70, 0xDA, 0x0F, 0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D, 0xD6, 0x03, 0xA9, 0x7C,
    0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07,
    0x53, 0x86, 0x2C, 0xF9};

// 中斷服務程式實作
void CrsfReceiver::onUartRx0() {
    while(uart_is_readable(uart0)) {
        uint8_t byte = uart_getc(uart0);
        size_t next = (ringHead0 + 1) % RING_SIZE;
        if(next != ringTail0) {
            ringBuf0[ringHead0] = byte;
            ringHead0 = next;
        } else {
            ringTail0 = (ringTail0 + 1) % RING_SIZE;
            ringBuf0[ringHead0] = byte;
            ringHead0 = next;
        }
    }
}

void CrsfReceiver::onUartRx1() {
    while(uart_is_readable(uart1)) {
        uint8_t byte = uart_getc(uart1);
        size_t next = (ringHead1 + 1) % RING_SIZE;
        if(next != ringTail1) {
            ringBuf1[ringHead1] = byte;
            ringHead1 = next;
        } else {
            ringTail1 = (ringTail1 + 1) % RING_SIZE;
            ringBuf1[ringHead1] = byte;
            ringHead1 = next;
        }
    }
}

CrsfReceiver::CrsfReceiver()
    : _uart_id(0), _lastValidFrameTime(0), _isFailsafeActive(true),
      _crcErrorCount(0), _lastLedToggleTime(0), _frameLen(0), _frameNeed(1),
      _state(SYNC) {
    memset(_finalChannels, 0, sizeof(_finalChannels));
    _finalChannels[4] = -100; // 預設安全開關為斷線狀態
}

void CrsfReceiver::begin(uint8_t uart_id, uint8_t rx_pin, uint32_t baudrate) {
    _uart_id = uart_id;
    uart_inst_t *uart = (_uart_id == 0) ? uart0 : uart1;
    uint irq_num = (_uart_id == 0) ? UART0_IRQ : UART1_IRQ;

    uart_init(uart, baudrate);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    uart_set_fifo_enabled(uart, true);
    uart_set_format(uart, 8, 1, UART_PARITY_NONE);

    if(_uart_id == 0) {
        irq_set_exclusive_handler(irq_num, onUartRx0);
    } else {
        irq_set_exclusive_handler(irq_num, onUartRx1);
    }
    irq_set_enabled(irq_num, true);
    uart_set_irq_enables(uart, true, false);

    // LED 預設交給呼叫端的 StatusLed 管理；只有 setLedEnabled(true) 才由本類別驅動
    if(_ledEnabled)
        pinMode(LED_BUILTIN, OUTPUT);
    _lastValidFrameTime = millis();
}

bool CrsfReceiver::feedByte(uint8_t byte) {
    switch(_state) {
    case SYNC:
        if(byte == CRSF_SYNC_BYTE) {
            _frameBuf[0] = byte;
            _frameLen = 1;
            _state = LEN;
        }
        break;
    case LEN:
        if(byte >= 3 && byte <= 62) {
            _frameBuf[1] = byte;
            _frameLen = 2;
            _frameNeed = byte;
            _state = TYPE_PAYLOAD;
        } else {
            _state = SYNC;
        }
        break;
    case TYPE_PAYLOAD:
        _frameBuf[_frameLen++] = byte;
        _frameNeed--;
        if(_frameNeed == 1) {
            _state = CRC;
        }
        break;
    case CRC: {
        _frameBuf[_frameLen++] = byte;
        uint8_t crc = 0;
        for(size_t i = 2; i < _frameLen - 1; i++) {
            crc = crsfCrcTable[crc ^ _frameBuf[i]];
        }
        bool crcOk = (byte == crc);
        _state = SYNC;
        if(crcOk && _frameBuf[2] == CRSF_TYPE_RC)
            return true;
        if(!crcOk)
            _crcErrorCount++;
        break;
    }
    }
    return false;
}

void CrsfReceiver::decodeRcChannels(const uint8_t *payload,
                                    uint16_t *channels) {
    // CRSF RC 封包：16 通道以 11-bit 連續位元流排列（LSB-first，無間隙）。
    // 逐一取 bit 是最穩健且經實測驗證的寫法（等同 user 端 unpack11）：
    //   每通道累積 11 bits，bitpos/bytepos 持續遞增，永不回退。
    // CRSF payload 共 22 bytes = 176 bits = 16 通道 × 11，剛好不越界。
    uint8_t bytepos = 0, bitpos = 0;
    for(uint8_t i = 0; i < 16; i++) {
        uint16_t v = 0;
        for(uint8_t b = 0; b < 11; b++) {
            uint8_t bit = static_cast<uint8_t>((payload[bytepos] >> bitpos) & 1U);
            v |= static_cast<uint16_t>(bit) << b;
            bitpos++;
            if(bitpos >= 8) {
                bitpos = 0;
                bytepos++;
            }
        }
        channels[i] = v;
    }
}

int16_t CrsfReceiver::normalizeChannel(uint16_t raw_val, uint8_t idx) {
    int32_t val = raw_val;
    if(val < (RC_CENTER - RC_RANGE))
        val = RC_CENTER - RC_RANGE;
    if(val > (RC_CENTER + RC_RANGE))
        val = RC_CENTER + RC_RANGE;

    // 統一雙極映射：中間點 (RC_CENTER=992) → 0，全範圍 (992±820) → -100~+100
    // 適用所有通道（含 Throttle），讓每個遙桿在中立點都回傳 0。
    int16_t out = static_cast<int16_t>((val - RC_CENTER) * 100 / RC_RANGE);

    // rc[6] 方向校正（遙控器特定設置，不影響中立點）
    if(idx == 6)
        out = -out;
    return out;
}

void CrsfReceiver::update() {
    unsigned long currentTime = millis();
    bool newFrameReceived = false;

    // 從對應的環形緩衝區撈資料
    if(_uart_id == 0) {
        while(ringTail0 != ringHead0) {
            uint8_t byte = ringBuf0[ringTail0];
            ringTail0 = (ringTail0 + 1) % RING_SIZE;
            _rxBytes++;
            if(feedByte(byte))
                newFrameReceived = true;
        }
    } else {
        while(ringTail1 != ringHead1) {
            uint8_t byte = ringBuf1[ringTail1];
            ringTail1 = (ringTail1 + 1) % RING_SIZE;
            _rxBytes++;
            if(feedByte(byte))
                newFrameReceived = true;
        }
    }

    if(newFrameReceived) {
        _lastValidFrameTime = currentTime;
        _isFailsafeActive = false;
        _frameCount++;

        uint16_t raw_channels[16];
        decodeRcChannels(_frameBuf + 3, raw_channels);

        for(uint8_t idx = 0; idx < 10; idx++) {
            _finalChannels[idx] = normalizeChannel(raw_channels[idx], idx);
        }
        if(_ledEnabled)
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // 正常閃爍
    }

    // 檢查超時
    if(currentTime - _lastValidFrameTime > FAILSAFE_TIMEOUT_MS) {
        _isFailsafeActive = true;
        for(int idx = 0; idx < 10; idx++)
            _finalChannels[idx] = 0;
        _finalChannels[4] = 0; // rc[4]：Failsafe 保護 → < 0 是「投射輪開」，所以強制歸 0（停轉）

        if(_ledEnabled && currentTime - _lastLedToggleTime >= 100) {
            _lastLedToggleTime = currentTime;
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // 狂暴快閃
        }
    }
}

void CrsfReceiver::getChannels(int16_t output[10]) {
    memcpy(output, _finalChannels, sizeof(_finalChannels));
}