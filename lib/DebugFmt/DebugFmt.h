#pragma once
#include <Arduino.h>
#include <cmath>
#include <cstdint>

// ============================================================================
// DebugFmt — 序列監控用的數值格式化（純整數手法，不用 %f）
//
// 為什麼抽出來：
//   newlib-nano 沒有開 _printf_float，`printf("%f")` / `"%.1f"` 會直接印不出
//   東西（印成空白），所以各 main.cpp 各自用「×100 整數」或「整數 + 小數點」
//   的土法把 float 印出來。同樣的手法在 Move / Shoot / Hub 重複出現，
//   抽成一份避免三份各自漂移。
//
// 來源（原封搬出，輸出字串逐字相同）：
//   - scaled100()   ← src/move/main.cpp（vx/vy/omega/RPS 一律以 x0.01 輸出）
//   - printFixed1() ← src/shoot/main.cpp（仰角一位小數）
//
// 使用限制：
//   - printFixed() 的 decimals 只支援 0~3（超過視為 3）。
//   - 只透過 Print 介面輸出（Serial / Serial1 / SerialPIO…）。Print 在 TX FIFO
//     滿時會阻塞（USB CDC 實測最長 1 秒），因此 AGENTS.md 規定只能在 Core 0
//     呼叫；Core 1 要印東西請走 SlaveLog。
//   - 呼叫端須確保送進來的是有限值（與原程式一致，本模組不加額外防護）。
// ============================================================================
namespace DebugFmt {

/// 把小數轉成 ×100 整數（序列監控的 vx/vy/omega/RPS 都是 x0.01）。
/// 非有限值（NaN / Inf）→ 0，避免印出巨大的垃圾整數。
inline int scaled100(float v) {
    if(!std::isfinite(v))
        return 0;
    return static_cast<int>(v * 100.0f);
}

/// 以「整數 + 小數」輸出 float（decimals = 0~3）。
/// 例：printFixed(Serial, 12.34f, 1) → "12.3"、printFixed(Serial, 12.34f, 2) → "12.34"
inline void printFixed(Print &out, float v, uint8_t decimals) {
    if(decimals > 3)
        decimals = 3;

    // 10^decimals 對照表（靜態常數，無動態配置）
    static const int32_t kPow10[4] = {1, 10, 100, 1000};
    const int32_t scale = kPow10[decimals];

    int32_t fixedVal =
        static_cast<int32_t>(lroundf(v * static_cast<float>(scale)));
    const char *sign = (fixedVal < 0) ? "-" : "";
    if(fixedVal < 0) {
        fixedVal = -fixedVal;
    }

    if(decimals == 0) {
        out.printf("%s%ld", sign, static_cast<long>(fixedVal));
        return;
    }

    out.printf("%s%ld.", sign, static_cast<long>(fixedVal / scale));

    // 小數部位逐位輸出並補零：不依賴 printf 的動態寬度 / 精度支援，
    // 確保任何 core 版本印出來的位數都一樣。
    const int32_t frac = fixedVal % scale;
    for(int32_t digit = scale / 10; digit > 0; digit /= 10)
        out.print(static_cast<char>('0' + (frac / digit) % 10));
}

/// 一位小數的常用縮寫（原 shoot 的 printFixed1）。
inline void printFixed1(Print &out, float v) {
    printFixed(out, v, 1);
}

} // namespace DebugFmt
