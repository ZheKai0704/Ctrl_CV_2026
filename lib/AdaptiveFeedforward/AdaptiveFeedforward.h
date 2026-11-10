#pragma once
#include "MathUtils.h"
#include <cmath>

// ============================================================================
// AdaptiveFeedforward — 線上自適應前饋（單參數 RLS）
//
// 為什麼抽出來：
//   這是純數學（遞迴最小平方法 + 前饋合成），與麥克納姆輪的極性、編碼器
//   PIO、PID 都無關；Move 只是使用者。抽出來後調參值仍留在 Move 的 RlsCfg，
//   透過 RlsConfig 傳入，lib 本身不含任何車體常數。
//
// 來源：src/move/main.cpp 的 RlsState / rlsUpdate() / 前饋合成
//       （`rls[i].a * targetRps[i] + rls[i].b`），數學與註解原封搬出。
//
// 模型（每輪）：duty ≈ a*rps（rps 為輪轉速，單一參數；b 恆為 0）
//   安全網由呼叫端負責：合成後仍須經 ±PID_MAX_DUTY clamp、NaN 防護、
//   積分風扇餘裕，模型輸出必須落在既有物理界限內。
//
// 使用限制：
//   - 每輪各自一個 RlsState（Move 是 4 輪 = 4 個，靜態分配）。
//   - 只在「追蹤穩定 + 實際移動 + 未飽和」時呼叫 update()（門檻由呼叫端判斷），
//     否則暫態樣本會污染模型。
//   - 無阻塞、不配置記憶體，可在 Core 1 控制迴圈中每圈呼叫。
// ============================================================================

// ============================================================================
// RLS 參數（Move 以 RlsCfg 的常數填入）
// ============================================================================
struct RlsConfig {
    float lambda;       // 遺忘因子 (0,1)：愈小追得愈快但對雜訊愈敏感
    float pInit;        // 初始協方差（大 → 開機前幾秒快速適應目前電壓/地板）
    float pMin;         // 協方差下限（防退化，失去適應力）
    float pMax;         // 協方差上限（防爆炸，數值炸裂）
    float minSpeedRps;  // 低於此轉速不採樣（避開零速/靜摩擦雜訊）
    float settleTolRps; // 追蹤穩定判定（供呼叫端決定是否採樣）
    float aMin;         // 增益 a 物理下限（duty%，防模型越界）
    float aMax;         // 增益 a 物理上限
    float bMin;         // 截距 b 界限（保留欄位，單一參數模型下不使用）
    float bMax;
};

// ============================================================================
// 每輪 RLS 狀態（靜態分配；Move 為 [FL, FR, BL, BR]）
//   a / P00 / P11 的初值沿用原 Move 預設（a = 33.0f、P00 = P11 = RlsCfg::P_INIT
//   = 1000.0f）；開機時 Move 的 setup() 會再以 kPidCfg.ff / RlsCfg::P_INIT 覆寫。
// ============================================================================
struct RlsState {
    float a = 33.0f;    // 增益 duty%/RPS（開機由 setup 以 kPidCfg.ff 覆寫）
    float b = 0.0f;     // 保留欄位，單一參數模型下恆為 0
    float P00 = 1000.0f; // 純量 RLS 協方差（= RlsCfg::P_INIT）
    float P01 = 0.0f;   // 保留欄位（不再使用）
    float P11 = 1000.0f;
};

namespace AdaptiveFeedforward {

/// RLS 更新：餵入一筆 (rps, duty)，更新單一增益 a（b 固定 0）。
///   原本用 2 參數 (a, b) 模型 duty ≈ a*rps + b：等速時 φ=[rps,1] 幾乎共線，
///   (a,b) 無法辨識，會沿 null 方向亂飄（實測 a 在 0~30、b 在 ±15 間跳動），
///   前饋因此失準，閉迴路長期追不到目標。改成單一增益模型後可辨識且穩定。
inline void update(RlsState &s, const RlsConfig &cfg, float rps, float duty) {
    if(fabsf(rps) < cfg.minSpeedRps)
        return;
    s.b = 0.0f;
    const float pred = s.a * rps;
    const float err = duty - pred;

    // 純量 RLS：K = P*rps / (λ + rps²*P)
    const float Pphi = s.P00 * rps;
    const float denom = cfg.lambda + rps * Pphi;
    if(fabsf(denom) < 1e-6f) // 數值退化防護
        return;
    const float K = Pphi / denom;

    s.a += K * err;
    s.P00 = MathUtils::clampf((s.P00 - K * Pphi) / cfg.lambda, cfg.pMin,
                              cfg.pMax);
    s.P01 = 0.0f; // 2×2 結構保留但不再使用
    s.P11 = cfg.pMin;
    s.a = MathUtils::clampf(s.a, cfg.aMin, cfg.aMax);
}

/// 前饋合成（duty%）：FF = a*targetRps + b
///   呼叫端仍須自行 clamp 到 ±PID_MAX_DUTY 並做 NaN 防護。
inline float feedforward(const RlsState &s, float targetRps) {
    return s.a * targetRps + s.b;
}

} // namespace AdaptiveFeedforward
