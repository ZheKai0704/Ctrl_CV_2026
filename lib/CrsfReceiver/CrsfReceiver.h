#ifndef CRSF_RECEIVER_H
#define CRSF_RECEIVER_H

#include <Arduino.h>

class CrsfReceiver {
  public:
    // 預設構造函數
    CrsfReceiver();

    // 初始化 UART 與中斷（傳入你想使用的 UART 編號與 RX 腳位）
    void begin(uint8_t uart_id = 0, uint8_t rx_pin = 1,
               uint32_t baudrate = 420000U);

    // 主更新程式：必須在 loop() 中不斷呼叫，負責讀取並解析
    void update();

    // 獲取最後常態化後的前 10 通道數據（rc[0]~rc[9]，返回 -100 ~ 100）
    // 如果目前處於 Failsafe 狀態，會自動返回安全保護值（全 0 = 停轉）
    void getChannels(int16_t output[10]);

    // 檢查目前是否處於斷線保護狀態
    bool isFailsafe() const { return _isFailsafeActive; }

    // 獲取 CRC 錯誤計數
    uint32_t getCrcErrors() const { return _crcErrorCount; }

  private:
    // 內部 CRSF 核心解析函式
    bool feedByte(uint8_t byte);
    void decodeRcChannels(const uint8_t *payload, uint16_t *channels);
    int16_t normalizeChannel(uint16_t raw_val, uint8_t idx);

    // 靜態中斷服務程式 (ISR)
    static void onUartRx0();
    static void onUartRx1();

    // 內部變數
    uint8_t _uart_id;
    unsigned long _lastValidFrameTime;
    bool _isFailsafeActive;
    uint32_t _crcErrorCount;
    unsigned long _lastLedToggleTime;

    int16_t _finalChannels[10]; // 快取最終常態化後的通道數據（rc_channels[0]~[9]）

    // 解析狀態機變數
    uint8_t _frameBuf[64];
    size_t _frameLen;
    size_t _frameNeed;
    enum { SYNC,
           LEN,
           TYPE_PAYLOAD,
           CRC } _state;
};

#endif // CRSF_RECEIVER_H