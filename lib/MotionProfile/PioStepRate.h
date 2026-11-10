#pragma once

// ============================================================================
// PioStepRate — PIO 步進脈衝「每步速率 → clkdiv」換算（原 Shoot 的 divForSps）
//
// 為什麼抽出來：
//   脈衝由 PIO 產生時，速率與 clkdiv 的關係是純算式
//   （f_step = clk_sys / (cyclesPerStep × clkdiv)），與哪一顆步進馬達、
//   STEP/DIR 接哪支腳都無關。抽成 lib 後仰角/水平兩軸（甚至其他節點）
//   共用同一份，避免各自寫一次上下限夾擠而出現差異。
//
// 來源：src/shoot/main.cpp 的 RowStepper::divForSps()
//       （kCyclesPerStep = 7：set[3] 4 + set[1] 2 + jmp 1）。
//
// 使用限制：
//   - RP2040/RP2350 PIO clkdiv 有效範圍 1.0~65536.0，超出會被夾擠。
//   - sps <= 0 時不做防護（除零 → inf → 被 divMax 夾擠），呼叫端應避免。
// ============================================================================
namespace PioStepRate {

/// 步進速率 (steps/s) → PIO clkdiv
/// @param sysHz         PIO 時鐘（RP2350 預設 150MHz）
/// @param cyclesPerStep 每個 STEP 脈衝的 PIO 指令週期數（含脈衝本身）
/// @param sps           目標步進速率 (steps/s)
/// @param divMin        最小 clkdiv（1.0 → 最高速）
/// @param divMax        最大 clkdiv（65536 → 最低速）
inline float clkdivForSps(float sysHz, float cyclesPerStep, float sps,
                          float divMin = 1.0f, float divMax = 65536.0f) {
    float div = sysHz / (cyclesPerStep * sps);
    if(div < divMin)
        div = divMin;
    if(div > divMax)
        div = divMax;
    return div;
}

} // namespace PioStepRate
