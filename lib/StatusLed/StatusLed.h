#pragma once
#include <Arduino.h>
#include <cstdint>

// ============================================================================
// StatusLed — 非阻塞「相位狀態機」狀態 LED
//
// 為什麼抽出來：
//   Move / Shoot / Hub 三個 main.cpp 的 updateStatusLed() 是同一段程式：
//   狀態切換 → 重置相位並立刻點亮；長亮狀態持續 HIGH；其餘狀態亮滿 onMs 熄、
//   熄滿 offMs 亮。差別只在「狀態集合」與「各狀態的亮/滅時間」。
//   因此把相位推進與 Pattern 表交給 lib，各 main 保留自己的 LedState enum、
//   ledStateName()、currentLedState() 判定（狀態集合不同，不該硬塞進 lib）。
//
// 來源：src/move/main.cpp、src/shoot/main.cpp、src/hub/main.cpp 的
//       updateStatusLed()（三份內容一致）與各自的 LedCfg 亮滅時間。
//
// 使用限制：
//   - update() 必須由同一個核心週期性呼叫（本專案都在 Core 0 的 loop()），
//     內部狀態不是 atomic，不可跨核心共享。
//   - 不阻塞（只做兩次 digitalWrite 比較的整數運算），可在 loop() 每圈呼叫。
//   - 完全不碰 Serial，符合 AGENTS.md「Serial 只給 Core 0」的規定。
//
// 使用方式：
//   enum class LedState : uint8_t { BOOT = 0, IDLE = 1, ACTIVE = 2 };
//   static const StatusLed::Pattern kPatterns[] = {
//       {0, 0, true},        // BOOT：長亮
//       {500, 1500, false},  // IDLE：0.5Hz 慢閃
//       {80, 920, false},    // ACTIVE：1Hz 短脈衝
//   };
//   static StatusLed statusLed(LED_BUILTIN, kPatterns, 3);
//   // setup(): statusLed.begin();
//   // loop():  statusLed.update(static_cast<uint8_t>(currentLedState()), millis());
// ============================================================================
class StatusLed {
  public:
    /// 單一狀態的閃爍圖樣（索引 = 傳給 update() 的 state 值）
    struct Pattern {
        uint32_t onMs;  // 亮多久（solidOn 時忽略）
        uint32_t offMs; // 熄多久（solidOn 時忽略）
        bool solidOn;   // true = 長亮
    };

    /// @param pin    LED 腳位（通常是 LED_BUILTIN）
    /// @param table  Pattern 表（必須存活超過 StatusLed 生命期，通常是 static const）
    /// @param count  表格格數（state 超出範圍 → 視為長亮）
    StatusLed(uint8_t pin, const Pattern *table, uint8_t count)
        : pin_(pin), table_(table), count_(count) {}

    /// 初始化腳位（等效原本 setup() 的 pinMode(OUTPUT) + digitalWrite(LOW)）
    void begin() {
        pinMode(pin_, OUTPUT);
        digitalWrite(pin_, LOW);
    }

    /// 每圈呼叫：state 改變 → 重置相位並立刻點亮；solidOn → 長亮；
    /// 其餘 → 亮滿 onMs 熄、熄滿 offMs 亮（非阻塞）。
    /// @param state  狀態編號（= Pattern 表的索引）
    /// @param nowMs  millis()
    void update(uint8_t state, uint32_t nowMs) {
        // 狀態切換 → 重置相位並立刻點亮（先給一個脈衝，新狀態立即可見）
        if(state != lastState_) {
            lastState_ = state;
            phaseStartMs_ = nowMs;
            ledOn_ = true;
            digitalWrite(pin_, HIGH);
        }

        const Pattern &pattern = patternOf(state);

        // 長亮（如：剛開機從未連線、E-Stop）
        if(pattern.solidOn) {
            digitalWrite(pin_, HIGH);
            return;
        }

        // 非阻塞相位推進：亮滿 onMs → 熄，熄滿 offMs → 亮
        const uint32_t phaseMs = ledOn_ ? pattern.onMs : pattern.offMs;
        if(nowMs - phaseStartMs_ >= phaseMs) {
            phaseStartMs_ = nowMs;
            ledOn_ = !ledOn_;
            digitalWrite(pin_, ledOn_ ? HIGH : LOW);
        }
    }

    /// 目前套用的狀態值（供除錯輸出比對；未呼叫過 update() 時為 0xFF）
    uint8_t state() const { return lastState_; }

  private:
    /// 狀態 → Pattern；超出表格範圍時退回長亮（安全側：至少看得見有電）
    const Pattern &patternOf(uint8_t state) const {
        static const Pattern kFallback = {0, 0, true};
        if(table_ == nullptr || state >= count_)
            return kFallback;
        return table_[state];
    }

    uint8_t pin_;
    const Pattern *table_;
    uint8_t count_;

    // 相位狀態（僅由 update() 的呼叫核心存取）
    // 初值 0xFF：保證第一次 update() 一定走「狀態切換」分支（重置相位並點亮）。
    uint8_t lastState_ = 0xFF;
    bool ledOn_ = false;
    uint32_t phaseStartMs_ = 0;
};
