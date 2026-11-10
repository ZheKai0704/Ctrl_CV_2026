#pragma once
#include <cmath>

// ============================================================================
// MotionProfile — jerk-limited 非對稱加減速 S 曲線（原 Move 的 SCurveRamp）
//
// 為什麼抽出來：
//   S 曲線平滑器是純數學，與麥克納姆輪的腳位、馬達極性、PID 參數無關；
//   Move 只是「使用者」。抽出來後調參值仍留在 Move 的 MoveCfg，透過
//   configure() 傳入，lib 本身不含任何車體常數。
//
// 來源：src/move/main.cpp 的 struct SCurveRamp（含註解原封搬出）。
//
// 使用限制：
//   - configure() 必須先呼叫一次；未 configure 時 cfg_ 全為 0，輸出會卡住不動。
//   - update() 必須以「實際經過時間」dt 呼叫（Move 用 Core 1 控制週期），
//     dt 不可為 0 或負值。
//   - 無狀態共享問題：每個軸各自一個實例（Move 有 vx / vy / omega 三套）。
//   - 不阻塞、不配置記憶體，可在 Core 1 控制迴圈中每圈呼叫。
// ============================================================================

// ============================================================================
// S 曲線參數（單位：速度 RPS、加速度 RPS/s、Jerk RPS/s²）
//   Move 以 MoveCfg 的 SCURVE_* 常數填入；換節點只要換這組數字。
// ============================================================================
struct SCurveConfig {
    float accel;        // 加速上限
    float decel;        // 減速上限（通常 > accel：往 0 收要快）
    float jerk;         // Jerk 上限（加速度的變化率，S 形圓角）
    float startSlowVel; // 起步慢段速度窗（|y| 低於此值時改用 startAccel）
    float startAccel;   // 起步加速上限（慢提克服靜摩擦）
    float settleTol;    // 到位判定（|err| 小於此值直接鎖定 target）
};

// ============================================================================
// S 曲線速度剖面（jerk-limited，非對稱加/減速）
//   每車身軸（vx / vy / omega）一套，單位皆 RPS（加速度 RPS/s）。
//   作用於原始指令 → 平滑後指令，再餵給運動學/PID。
//   目標：
//     起步（從 0）→ 慢段小坡克服靜摩擦 → 中段加速 → 平滑到位
//     減速（往 0）→ 快速但 Jerk 限幅、不瞬停
//   穩態追蹤時 y==target（無誤差、無積分風扇）；僅在目標改變時作用。
// ============================================================================
class SCurveRamp {
  public:
    /// 填入調參值（通常在 setup() 呼叫一次）
    void configure(const SCurveConfig &cfg) { cfg_ = cfg; }

    /// 更新並回傳平滑後輸出（一行呼叫）
    float updatePw(float target, float dt) {
        update(target, dt);
        return y_;
    }

    /// 每控制週期呼叫，把 y 往 target 平滑推進
    void update(float target, float dt) {
        float err = target - y_;
        if(fabsf(err) < cfg_.settleTol) {
            y_ = target;
            accel_ = 0.f;
            return;
        }
        // 判斷加/減速：目標比目前更靠近 0 → 減速（快速，且維持方向）
        const bool decel = (y_ != 0.f) && ((err > 0.f) != (y_ > 0.f));
        float cap = decel ? cfg_.decel : cfg_.accel;
        // 起步慢段：從 0 出發且速度尚小 → 限加速度以小斜率起步克服靜摩擦
        if(!decel && fabsf(y_) < cfg_.startSlowVel)
            cap = (cap < cfg_.startAccel) ? cap : cfg_.startAccel;

        // Jerk 限幅：加速度以 jerk 斜率往目標 cap 逼近 → S 形圓角
        const float aim = (err > 0.f) ? cap : -cap;
        const float maxDa = cfg_.jerk * dt;
        float da = aim - accel_;
        if(da > maxDa)
            da = maxDa;
        if(da < -maxDa)
            da = -maxDa;
        accel_ += da;

        y_ += accel_ * dt;

        // 防止越過目標（overshoot）：越過即鎖定在 target
        if((y_ - target) * err < 0.f)
            y_ = target;
    }

    /// 歸零（E-Stop 用）：若停留在舊速度，解除後的第一個命令會直接以
    /// 舊斜率／舊速度餵 PID → 瞬間大出力（暴衝）。
    void reset() {
        y_ = 0.f;
        accel_ = 0.f;
    }

    float value() const { return y_; }    // 平滑後指令
    float accel() const { return accel_; } // 目前加速度

  private:
    SCurveConfig cfg_ = {};
    float y_ = 0.f;     // 平滑後指令輸出（此軸）
    float accel_ = 0.f; // 目前加速度（RPS/s）
};
