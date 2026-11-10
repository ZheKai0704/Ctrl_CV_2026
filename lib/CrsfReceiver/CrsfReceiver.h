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

    // 是否由本類別直接驅動 LED_BUILTIN（預設關閉）
    //   Hub 的 LED 由 Core 0 的 StatusLed 統一管理，Core 1 絕不可碰同一個腳位，
    //   否則會與狀態機互搶、出現無法解釋的快閃。
    //   只有純 CRSF 測試程式（沒有自己的狀態 LED）才需要開啟。
    void setLedEnabled(bool enabled) { _ledEnabled = enabled; }

    // 獲取最後常態化後的前 10 通道數據（rc[0]~rc[9]，返回 -100 ~ 100）
    // 如果目前處於 Failsafe 狀態，會自動返回安全保護值（全 0 = 停轉）
    void getChannels(int16_t output[10]);

    // 檢查目前是否處於斷線保護狀態
    bool isFailsafe() const { return _isFailsafeActive; }

    // 獲取 CRC 錯誤計數
    uint32_t getCrcErrors() const { return _crcErrorCount; }

    // 診斷用：UART0 收到的位元組數 / 解出的有效 RC 幀數
    //   兩者都是 Core 1 寫、Core 0 可讀（volatile，單一寫入者）
    //   rxBytes=0          → 完全沒收到任何位元組（接線 / 接收機沒輸出）
    //   rxBytes 增加、frames=0 → 有訊號但解不出（飽率 / 反向 / CRC）
    uint32_t getRxBytes() const { return _rxBytes; }
    uint32_t getFrameCount() const { return _frameCount; }

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
    bool _ledEnabled = false; // 預設不驅動 LED（交給各 main 的 StatusLed）

    volatile uint32_t _rxBytes = 0;   // 累計收到的位元組（診斷用）
    volatile uint32_t _frameCount = 0; // 累計解出的有效 RC 幀（診斷用）

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