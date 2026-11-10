#pragma once
#include <cmath>
#include <cstdint>

// ============================================================================
// MathUtils — 各節點共用的純量小工具（header-only，無狀態、無動態配置）
//
// 為什麼抽出來：
//   clampf / 死區 / 百分比防護這幾個函式在 Move（clampf）、Hub（applyDeadband）、
//   Shoot（sanitizeSpeed）各自有一份，內容幾乎相同但寫法不一致。集中在這裡，
//   lib 內的演算法（MotionProfile / AdaptiveFeedforward）也能共用同一份 clampf，
//   不必各自再複製一次。
//
// 來源：
//   - clampf()          ← src/move/main.cpp
//   - deadband()        ← src/hub/main.cpp applyDeadband()（通用化成可傳死區寬度）
//   - sanitizePercent() ← src/shoot/main.cpp sanitizeSpeed()
//   - isFiniteF()       ← 對 std::isfinite 的包裝（原本散在各處直接用 std::isfinite）
//
// 使用限制：
//   - 全部 inline / 無狀態，可在任意核心、任意 context 呼叫（不會阻塞）。
//   - 邊界行為與原程式完全一致（deadband 用嚴格不等式、sanitizePercent 只擋 NaN）。
// ============================================================================
namespace MathUtils {

/// 浮點有限值檢查（NaN / ±Inf → false）。
inline bool isFiniteF(float v) {
    return std::isfinite(v);
}

/// 上下限夾擠（靜態 inline，無動態分配）。
/// 注意：v 為 NaN 時回傳 NaN（與原 move 的 clampf 相同，呼叫端自行防護）。
inline float clampf(float v, float lo, float hi) {
    if(v < lo)
        return lo;
    if(v > hi)
        return hi;
    return v;
}

/// 死區濾波：|v| < db → 0，其餘原值輸出（用嚴格不等式，邊界值保留）。
/// 原 Hub 的 applyDeadband() 等於 deadband(v, 10)。
inline int16_t deadband(int16_t v, int16_t db) {
    if(v > -db && v < db)
        return 0;
    return v;
}

/// 百分比命令正規化（單向電調 / 機構限幅用）：
///   NaN → 0、負值取絕對值、上限 100。
/// 原 Shoot 的 sanitizeSpeed()：電調單向，負值視為同速反轉方向。
inline float sanitizePercent(float v) {
    if(std::isnan(v))
        return 0.0f;
    if(v < 0.0f)
        v = -v;
    if(v > 100.0f)
        v = 100.0f;
    return v;
}

} // namespace MathUtils
