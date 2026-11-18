// ============================================================================
// stepdir_gp14_test — 步進馬達 EN / STEP / DIR 脈衝控制測試（PIO 版本, RP2350）
// 通用接法版：STEP=GP14, DIR=GP15, EN=GP13
//
// 用途：
//   給「只能用 EN / STEP / DIR / COM 四線」的步進驅動器（TB6600、DM542、
//   A4988、DRV8825…）做功能驗證。STEP 脈衝由 PIO 硬體產生，CPU 只負責
//   設定 clock divider（調速 / 加減速）與偵測到位旗標，因此可跑到
//   driver 的極限而不受主迴圈負擔影響。
//
// 架構：
//   PIO0 SM0 ── step_pulse.pio：固定 7 週期/步，f = clk_sys / (7 × clkdiv)
//   CPU ── pio_sm_put(脈衝數) 下令；pio_interrupt_get() 偵測 irq 0 到位
//   DIR / EN ── 一般 GPIO（CPU 直接控制，DIR 切換後等 10µs 建立時間）
//
// ⚠ 轉速下限（硬體限制）：
//   divider 最大 65536 ⇒ 最低速 = clk_sys / (7 × 65536) ≈ 327 steps/s
//   （150 MHz）。3200 步/圈時約 6.1 RPM，也就是這支 PIO 程式做不出比
//   6 RPM 更慢的連續脈衝；要更慢必須在 .pio 內加 delay 或降低 clk_sys。
//   startMotion() 低於下限時會印警告，不會偷偷用「看起來比較慢」的假數值。
//
// ⚠ STEP 電位：PIO 程式第一行就是 `set pins, 0`，每次下令都先確保 STEP
//   為低電位（中止時若停在脈衝高的 delay 中，latch 可能殘留高電位）。
//
// 接線（預設，可改 namespace Pin）：
//   RP2350 GP14 → 驅動器 PUL / STEP  （PIO0 SM0）
//   RP2350 GP15 → 驅動器 DIR
//   RP2350 GP13 → 驅動器 ENA / EN
//   RP2350 GND → 驅動器 COM
//
// ⚠ COM 接法（光耦輸入）：
//   1) 共陰（預設）：COM 接 GND，訊號吃 3.3V
//   2) 共陽：COM 接 +5V，訊號改低電位有效 → 用 'n' 切 EN 極性
//
// 按鍵（USB Serial, 115200）：
//   't' 一鍵自我測試（最低速 1 圈 → +1 圈 → −1 圈 → 低速 36°）
//   'e' 使能      'd' 去使能     'i' 印目前狀態
//   'v' 設轉速(RPM)              'k' 設加速度(steps/s²)
//   'a' 轉指定角度(°)  'f' 正轉 N 步   'b' 反轉 N 步
//   '1' 連續正轉   '2' 連續反轉   '0' 停止
//   'c' 每圈脈衝數(細分)  'n' EN 極性  'r' DIR 反向
//   '?' 說明
// ============================================================================
#include <Arduino.h>
#include <hardware/clocks.h>
#include <hardware/pio.h>
#include "step_pulse.pio.h"

// ============================================================================
// 腳位
// ============================================================================
namespace Pin {
constexpr uint STEP = 14; // → 驅動器 PUL（PIO0 SM0 產生，固定 7 週期/步）
constexpr uint DIR = 15;  // → 驅動器 DIR
constexpr uint EN = 13;   // → 驅動器 ENA
} // namespace Pin

// ============================================================================
// PIO 組態
// ============================================================================
static PIO gPio = pio0;
static uint gSm = 0;
static uint gOffset = 0;
constexpr uint gIrqFlag = 0;           // step_pulse.pio 到位時設定的 IRQ 旗標
constexpr float kCyclesPerStep = 7.0f; // set[3] = 4 + set[1] = 2 + jmp = 1
constexpr float kDivMax = 65536.0f;    // PIO clock divider 上限（INT=0 視為 65536）
static float gClkSysHz = 150000000.0f;

// ============================================================================
// 參數（可調）
// ============================================================================
static bool gEnActiveLow = true;            // EN 低電位致能（多數 TB6600/DM542）
static bool gDirInvert = false;             // DIR 方向反向
static float gStepsPerRev = 200.0f * 16.0f; // 1.8° 步進 × 16 細分 = 3200（必須對上驅動器 DIP 細分）
static float gSpeedRpm = 15.0f;             // 目標轉速（預設低速，驗證用；用 'v' 提速）
static float gAccelSps2 = 20000.0f;         // 加速度 steps/s²（0 = 不加速）
// 脈衝速率上下限由 PIO clock divider 決定：clk_sys / (7 × 1) ~ clk_sys / (7 × 65536)
// 下限用「實際 clk_sys」在 setup() 內重算，避免寫死 330 造成低速測試失真
static float gMinSps = 150000000.0f / (kCyclesPerStep * kDivMax); // ≈ 327 steps/s
constexpr float kMaxSps = 150000.0f; // 軟體上限；再上去多數驅動器跟不上

// ============================================================================
// 運動狀態（靜態配置，專案禁止 new/malloc）
// ============================================================================
struct StepperState {
    bool running = false;
    bool continuous = false;
    bool enabled = false;
    int32_t total = 0;      // 本次命令總步數（連續模式為 0）
    float speedSps = 0.0f;  // 目前命令速率 steps/s
    float targetSps = 0.0f; // 目標速率 steps/s
    float estSteps = 0.0f;  // 已發出步數估算（積分速率）
    int8_t dir = 1;         // +1 正轉，-1 反轉
    uint32_t lastRampUs = 0;
};
static StepperState gSt;

static int32_t sArgValue = 0;

// ============================================================================
// PIO 底層
// ============================================================================
static void pioSetup() {
    // 交棒給 PIO 之前，先用 SIO 把 STEP 定成輸出的低電位（避免上電浮動）
    gpio_init(Pin::STEP);
    gpio_set_dir(Pin::STEP, GPIO_OUT);
    gpio_put(Pin::STEP, 0);

    gOffset = pio_add_program(gPio, &step_pulse_program);
    pio_sm_config c = step_pulse_program_get_default_config(gOffset);

    // STEP 交給 PIO 的 "set pins" 群組（1 bit）
    sm_config_set_set_pins(&c, Pin::STEP, 1);
    pio_gpio_init(gPio, Pin::STEP);
    pio_sm_init(gPio, gSm, gOffset, &c);
    // pindirs 必須在 pio_sm_init() 之後設定（init 會覆寫 PINCTRL 組態）
    pio_sm_set_consecutive_pindirs(gPio, gSm, Pin::STEP, 1, true);
    pio_sm_set_pins(gPio, gSm, 0); // STEP 先拉低
    pio_sm_set_enabled(gPio, gSm, true);
}

// 速率 → clock divider（含小數，SDK 內部為 16.8 定點）
static float divForSps(float sps) {
    float div = (sps > 0.0f) ? gClkSysHz / (kCyclesPerStep * sps) : kDivMax;
    if(div < 1.0f) {
        div = 1.0f;
    }
    if(div > kDivMax) {
        div = kDivMax;
    }
    return div;
}

// 目前 divider 對應的實際速率（量化後）
static float actualSps(float spsRequest) {
    return gClkSysHz / (kCyclesPerStep * divForSps(spsRequest));
}

static void applySpeed(float sps) {
    pio_sm_set_clkdiv(gPio, gSm, divForSps(sps));
}

// 停止脈衝：關 SM、清 FIFO、清到位旗標
// 註：SM 關閉時 pio_sm_set_pins() 只是把 `set pins` 排進 SMx_INSTR，並不保證
//     立刻生效（且 startPulses() 的 pio_sm_restart() 會清掉這條指令）。真正保證
//     STEP 回到低電位的是 PIO 程式第一行 `set pins, 0`，每次下令都會先執行一次。
static void haltPulses() {
    pio_sm_set_enabled(gPio, gSm, false);
    pio_sm_clear_fifos(gPio, gSm);
    pio_sm_set_pins(gPio, gSm, 0);
    pio_interrupt_clear(gPio, gIrqFlag);
}

// 送脈衝數並啟動；count = 0xFFFFFFFF 表示連續
static void startPulses(uint32_t count) {
    pio_sm_set_enabled(gPio, gSm, false);
    pio_sm_clear_fifos(gPio, gSm);
    pio_sm_restart(gPio, gSm);
    pio_sm_exec(gPio, gSm, pio_encode_jmp(gOffset)); // PC 指回程式開頭
    pio_interrupt_clear(gPio, gIrqFlag);
    applySpeed(gSt.speedSps);
    pio_sm_put(gPio, gSm, count); // pull block 會取走
    pio_sm_set_enabled(gPio, gSm, true);
}

// ============================================================================
// GPIO（DIR / EN 由 CPU 控制）
// ============================================================================
static void applyEnablePin() {
    // enabled + 低有效 → 0；enabled + 高有效 → 1；未使能時反之
    const bool level = (gSt.enabled != gEnActiveLow);
    gpio_put(Pin::EN, level ? 1 : 0);
}

static void applyDir(int8_t dir) {
    bool level = (dir > 0);
    if(gDirInvert) {
        level = !level;
    }
    gpio_put(Pin::DIR, level ? 1 : 0);
    delayMicroseconds(10); // DIR 建立時間（TB6600 ≥5µs）
}

static void setEnabled(bool en) {
    gSt.enabled = en;
    applyEnablePin();
    Serial.printf(">> %s（EN 準位=%d, 低電位有效=%d）\n", en ? "使能" : "去使能",
                  gpio_get(Pin::EN), gEnActiveLow ? 1 : 0);
}

// ============================================================================
// 換算
// ============================================================================
static float rpmToSps(float rpm) {
    return rpm * gStepsPerRev / 60.0f;
}

static float spsToRpm(float sps) {
    return sps * 60.0f / gStepsPerRev;
}

static float clampSps(float sps) {
    if(sps < gMinSps) {
        return gMinSps;
    }
    if(sps > kMaxSps) {
        return kMaxSps;
    }
    return sps;
}

// ============================================================================
// 運動控制
// ============================================================================
static void stopMotion(const char *why) {
    bool wasRunning = gSt.running;
    haltPulses();
    gSt.running = false;
    gSt.continuous = false;
    gSt.speedSps = 0.0f;
    if(wasRunning) {
        Serial.printf(">> 停止（%s），已發約 %ld 步\n", why, static_cast<long>(gSt.estSteps));
    }
}

// steps <= 0 + continuous = 連續運轉
static void startMotion(int32_t steps, int8_t dir, bool continuous) {
    if(!gSt.enabled) {
        Serial.println(">> ⚠ 尚未使能，先按 'e'");
        return;
    }
    if(!continuous && steps <= 0) {
        Serial.println(">> ⚠ 步數必須 > 0");
        return;
    }
    haltPulses();

    gSt.dir = (dir >= 0) ? 1 : -1;
    applyDir(gSt.dir);

    float wantSps = rpmToSps(gSpeedRpm);
    if(wantSps < gMinSps * 0.999f) {
        Serial.printf(">> ⚠ 要求 %.1f steps/s 低於硬體下限 %.1f steps/s（divider 上限），改以 %.1f steps/s 執行\n",
                      static_cast<double>(wantSps), static_cast<double>(gMinSps),
                      static_cast<double>(gMinSps));
    }
    gSt.targetSps = clampSps(wantSps);
    // 短行程：把峰值速度壓到「加速段 + 減速段剛好走完」的值（三角曲線 v_peak = √(a·S)）
    if(!continuous && gAccelSps2 > 0.0f && steps > 0) {
        float peak = sqrtf(gAccelSps2 * static_cast<float>(steps));
        if(peak < gSt.targetSps) {
            gSt.targetSps = (peak < gMinSps) ? gMinSps : peak;
        }
    }
    gSt.speedSps = (gAccelSps2 > 0.0f) ? gMinSps : gSt.targetSps;
    gSt.continuous = continuous;
    gSt.total = continuous ? 0 : steps;
    gSt.estSteps = 0.0f;
    gSt.lastRampUs = micros();
    gSt.running = true;

    startPulses(continuous ? 0xFFFFFFFFu : static_cast<uint32_t>(steps));

    if(continuous) {
        Serial.printf(">> 連續%s：目標 %u RPM（%.0f steps/s）\n", gSt.dir > 0 ? "正轉" : "反轉",
                      static_cast<unsigned>(gSpeedRpm), static_cast<double>(gSt.targetSps));
    } else {
        Serial.printf(">> %s %ld 步（%.1f°）：峰值 %.0f steps/s（設定 %u RPM）\n",
                      gSt.dir > 0 ? "正轉" : "反轉", static_cast<long>(steps),
                      static_cast<double>(steps) * 360.0 / gStepsPerRev,
                      static_cast<double>(gSt.targetSps), static_cast<unsigned>(gSpeedRpm));
    }
}

// 每 loop() 呼叫：只做「調速 + 到位偵測」，脈衝由 PIO 產生
static void serviceMotion() {
    if(!gSt.running) {
        return;
    }

    // 到位：PIO 發完設了 irq 0
    if(pio_interrupt_get(gPio, gIrqFlag)) {
        pio_interrupt_clear(gPio, gIrqFlag);
        gSt.running = false;
        if(gSt.continuous) {
            Serial.printf(">> 連續運轉結束（已發約 %ld 步）\n", static_cast<long>(gSt.estSteps));
        } else {
            Serial.printf(">> 到位（%ld 步，結束轉速 %.1f RPM）\n", static_cast<long>(gSt.total),
                          static_cast<double>(spsToRpm(gSt.speedSps)));
        }
        return;
    }

    uint32_t now = micros();
    uint32_t dtUs = now - gSt.lastRampUs;
    if(dtUs < 200) { // 500Hz 更新加減速即可
        return;
    }
    gSt.lastRampUs = now;
    gSt.estSteps += gSt.speedSps * static_cast<float>(dtUs) * 1e-6f;

    // --- 梯形加減速 ---
    float targetSps = gSt.targetSps;
    if(!gSt.continuous && gAccelSps2 > 0.0f && gSt.speedSps > gMinSps) {
        float left = static_cast<float>(gSt.total) - gSt.estSteps;
        float decelDist = (gSt.speedSps * gSt.speedSps) / (2.0f * gAccelSps2);
        if(left <= decelDist + 1.0f) {
            targetSps = gMinSps;
        }
    }
    if(gAccelSps2 > 0.0f) {
        float dv = gAccelSps2 * static_cast<float>(dtUs) * 1e-6f;
        if(gSt.speedSps < targetSps) {
            gSt.speedSps += dv;
            if(gSt.speedSps > targetSps) {
                gSt.speedSps = targetSps;
            }
        } else if(gSt.speedSps > targetSps) {
            gSt.speedSps -= dv;
            if(gSt.speedSps < targetSps) {
                gSt.speedSps = targetSps;
            }
        }
    } else {
        gSt.speedSps = targetSps;
    }
    applySpeed(clampSps(gSt.speedSps));
}

// 阻塞等到到位（自我測試用）
static void waitMotionDone(bool printProgress) {
    uint32_t t0 = millis();
    uint32_t lastPrint = t0;
    while(gSt.running) {
        serviceMotion();
        if(printProgress && millis() - lastPrint >= 500) {
            lastPrint = millis();
            if(gSt.continuous) {
                Serial.printf("   … 運轉中 %.1f RPM, 已發約 %ld 步\n",
                              static_cast<double>(spsToRpm(gSt.speedSps)),
                              static_cast<long>(gSt.estSteps));
            } else {
                Serial.printf("   … 剩約 %ld 步, %.1f RPM\n",
                              static_cast<long>(static_cast<float>(gSt.total) - gSt.estSteps),
                              static_cast<double>(spsToRpm(gSt.speedSps)));
            }
        }
        if(millis() - t0 > 120000) { // 保險上限 2 分鐘
            stopMotion("逾時保護");
            break;
        }
    }
}

// ============================================================================
// 輸入（USB Serial 讀整數，可負）
// 註：等待輸入期間仍會呼叫 serviceMotion()，否則連續運轉時按 'v'/'k' 會讓
//     加減速與到位偵測停擺（最多 15 秒）。
// ============================================================================
static bool readIntInput(int32_t &val, const char *prompt, uint32_t waitMs = 15000) {
    uint32_t fd0 = millis();
    while(millis() - fd0 < 40) { // 清殘留
        serviceMotion();
        if(Serial.available()) {
            Serial.read();
            fd0 = millis();
        }
    }

    Serial.print(prompt);
    long v = 0;
    bool neg = false;
    bool got = false;
    uint32_t t0 = millis();
    while(millis() - t0 < waitMs) {
        serviceMotion(); // 等待輸入時仍要維持 PIO 加減速 / 到位偵測
        if(millis() - t0 > 2000 && !got && !Serial.available()) {
            break;
        }
        if(Serial.available()) {
            char ch = static_cast<char>(Serial.read());
            if(ch == '\n' || ch == '\r') {
                if(got) {
                    val = static_cast<int32_t>(neg ? -v : v);
                    Serial.println();
                    return true;
                }
            } else if(ch == '-' && !got) {
                neg = true;
            } else if(ch >= '0' && ch <= '9') {
                got = true;
                v = v * 10 + (ch - '0');
                Serial.write(ch);
            }
        }
    }
    Serial.println("  (逾時/取消)");
    return false;
}

// ============================================================================
// 一鍵自我測試
// ============================================================================
static void selfTest() {
    Serial.println("\n========== 自我測試開始（PIO 脈衝）==========");
    Serial.printf("設定：%.0f 步/圈, %u RPM, 加速度 %.0f steps/s², EN低有效=%d, DIR反向=%d\n",
                  static_cast<double>(gStepsPerRev), static_cast<unsigned>(gSpeedRpm),
                  static_cast<double>(gAccelSps2), gEnActiveLow ? 1 : 0, gDirInvert ? 1 : 0);
    Serial.printf("PIO：clk_sys=%.0f MHz, 7 cycles/step, divider=%.3f\n",
                  static_cast<double>(gClkSysHz / 1e6), static_cast<double>(divForSps(gSt.targetSps)));

    if(!gSt.enabled) {
        setEnabled(true);
        delay(200); // 等驅動器建立力矩
    }

    int32_t revSteps = static_cast<int32_t>(gStepsPerRev);

    Serial.printf("[0] 硬體最低速 1 圈 @200 步/圈（%.0f steps/s ≈ %.0f RPM，divider 上限）\n",
                  static_cast<double>(gMinSps), static_cast<double>(spsToRpm(gMinSps)));
    {
        float keepRev = gStepsPerRev;
        float keepRpm = gSpeedRpm;
        gStepsPerRev = 200.0f; // 假設 1 細分；即使 DIP 不是 16 細分也只影響轉速
        gSpeedRpm = spsToRpm(gMinSps); // 這支 PIO 能達到的最低轉速，不是 10 RPM
        startMotion(200, +1, false);   // 200 步 = 1 圈（約 0.6 秒，肉眼可見一整圈）
        waitMotionDone(false);
        gStepsPerRev = keepRev;
        gSpeedRpm = keepRpm;
        Serial.println("    → 完整轉一圈就代表 STEP / DIR / EN 與細分基本正確");
        Serial.println("    → 完全不動？查 EN 極性('n')、共地、驅動器 DIP 電流、線圈接線");
        Serial.println("    → 比預期慢很多？divider 上限使這支 PIO 無法低於約 6 RPM @3200 步");
        delay(500);
    }

    Serial.println("[1] 正轉 1 圈");
    uint32_t t0 = millis();
    startMotion(revSteps, +1, false);
    waitMotionDone(false);
    uint32_t t1 = millis();
    Serial.printf("    完成，耗時 %lu ms（目標 %lu ms）\n", static_cast<unsigned long>(t1 - t0),
                  static_cast<unsigned long>(revSteps * 1000.0f / gSt.targetSps));
    delay(500);

    Serial.println("[2] 反轉 1 圈（應回到原點）");
    startMotion(revSteps, -1, false);
    waitMotionDone(false);
    uint32_t t2 = millis();
    Serial.printf("    完成，耗時 %lu ms\n", static_cast<unsigned long>(t2 - t1));
    delay(500);

    Serial.println("[3] 36°（細分/極性目視檢查）");
    float keepRpm = gSpeedRpm;
    gSpeedRpm = 20.0f;
    startMotion(revSteps / 10, +1, false);
    waitMotionDone(false);
    gSpeedRpm = keepRpm;

    Serial.println("========== 自我測試結束 ==========");
    Serial.println("判讀：");
    Serial.println("  • 完全不動 / 有嗡嗡聲但不轉 → EN 極性或 COM 接法錯，按 'n' 再測");
    Serial.println("  • 只往一方向轉            → 馬達一組線圈接反（對調 A+ / A−）");
    Serial.println("  • 方向與預期相反          → 按 'r' 或對調一組線圈");
    Serial.println("  • 步數對但角度不對        → 'c' 改每圈脈衝數（對上驅動器細分 DIP）");
    Serial.println("  • 高速失步（轉不到整圈）  → 'k' 加加速度、'v' 降轉速");
    Serial.println("  • 耗時遠大於目標          → 加速度太低，ramp 佔掉大部分時間");
}

// ============================================================================
// 說明
// ============================================================================
static void printHelp() {
    Serial.println();
    Serial.println("=== 步進馬達 EN/STEP/DIR 測試（PIO 脈衝）===");
    Serial.printf("  腳位：STEP=GP%u(PIO0 SM0)  DIR=GP%u  EN=GP%u\n", Pin::STEP, Pin::DIR, Pin::EN);
    Serial.printf("  目前：%.0f 步/圈, %u RPM, 加速度 %.0f steps/s²\n",
                  static_cast<double>(gStepsPerRev), static_cast<unsigned>(gSpeedRpm),
                  static_cast<double>(gAccelSps2));
    Serial.printf("  EN 低電位有效=%d  DIR 反向=%d  使能=%d\n", gEnActiveLow ? 1 : 0,
                  gDirInvert ? 1 : 0, gSt.enabled ? 1 : 0);
    Serial.printf("  速率範圍：%.0f ~ %.0f steps/s（≈ %.0f ~ %.0f RPM @ 目前細分）\n",
                  static_cast<double>(gMinSps), static_cast<double>(kMaxSps),
                  static_cast<double>(spsToRpm(gMinSps)), static_cast<double>(spsToRpm(kMaxSps)));
    Serial.printf("  ⚠ 低於 %.0f steps/s 做不出來（divider 上限），要更慢須改 step_pulse.pio 加 delay\n",
                  static_cast<double>(gMinSps));
    Serial.println("  't' 自我測試            'e'/'d' 使能/去使能      'i' 狀態");
    Serial.println("  'v' 轉速(RPM)           'k' 加速度(steps/s²)");
    Serial.println("  'a' 轉角度(°)           'f' 正轉 N 步   'b' 反轉 N 步");
    Serial.println("  '1' 連續正轉  '2' 連續反轉  '0' 停止");
    Serial.println("  'c' 每圈脈衝數          'n' EN 極性    'r' DIR 反向");
    Serial.println("  '?' 說明");
    Serial.println();
}

// ============================================================================
// setup / loop
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n[stepdir_gp14_test] Booting... (PIO STEP generator, STEP=GP14 DIR=GP15 EN=GP13)");

    gClkSysHz = static_cast<float>(clock_get_hz(clk_sys));
    // 依實際 clk_sys 重算硬體最低速（divider 上限 65536）
    gMinSps = gClkSysHz / (kCyclesPerStep * kDivMax);

    // DIR / EN 由 CPU 控制
    gpio_init(Pin::DIR);
    gpio_set_dir(Pin::DIR, GPIO_OUT);
    gpio_put(Pin::DIR, 0);

    gpio_init(Pin::EN);
    gpio_set_dir(Pin::EN, GPIO_OUT);
    gSt.enabled = false;
    applyEnablePin(); // 上電先保持去使能

    // STEP 交給 PIO（pioSetup 內會先用 SIO 拉低，再 pio_gpio_init 接管）
    pioSetup();

    printHelp();
}

void loop() {
    serviceMotion();

    static uint32_t lastReport = 0;
    if(gSt.running && gSt.continuous && millis() - lastReport >= 500) {
        lastReport = millis();
        Serial.printf("   → 運轉中 %.1f RPM (%s)\n", static_cast<double>(spsToRpm(gSt.speedSps)),
                      gSt.dir > 0 ? "正轉" : "反轉");
    }

    if(!Serial.available()) {
        return; // 脈衝由 PIO 產生，主迴圈不需要忙等/延遲
    }
    char c = static_cast<char>(Serial.read());

    switch(c) {
    case '\r':
    case '\n':
    case ' ':
        break;
    case 't':
        selfTest();
        break;
    case 'e':
        setEnabled(true);
        break;
    case 'd':
        stopMotion("去使能");
        setEnabled(false);
        break;
    case 'i':
        Serial.printf(">> 使能=%d 運轉=%d 連續=%d 方向=%s 命令 %.1f RPM / 實際 %.1f RPM "
                      "divider=%.3f 已發約 %ld 步\n",
                      gSt.enabled ? 1 : 0, gSt.running ? 1 : 0, gSt.continuous ? 1 : 0,
                      gSt.dir > 0 ? "正轉" : "反轉",
                      static_cast<double>(spsToRpm(gSt.speedSps)),
                      static_cast<double>(spsToRpm(actualSps(clampSps(gSt.speedSps)))),
                      static_cast<double>(divForSps(clampSps(gSt.speedSps))),
                      static_cast<long>(gSt.estSteps));
        break;
    case 'v': {
        Serial.printf("  目前每圈 %.0f 步，可設定 %.0f ~ %.0f RPM\n", static_cast<double>(gStepsPerRev),
                      static_cast<double>(spsToRpm(gMinSps)), static_cast<double>(spsToRpm(kMaxSps)));
        if(!readIntInput(sArgValue, "轉速 RPM：")) {
            break;
        }
        if(sArgValue < 1) {
            sArgValue = 1;
        }
        float sps = clampSps(rpmToSps(static_cast<float>(sArgValue)));
        gSpeedRpm = spsToRpm(sps);
        float rampS = (gAccelSps2 > 0.0f) ? (sps - gMinSps) / gAccelSps2 : 0.0f;
        Serial.printf(">> 轉速 = %.0f RPM（%.0f steps/s，divider %.3f，加速約 %.2f 秒）\n",
                      static_cast<double>(gSpeedRpm), static_cast<double>(sps),
                      static_cast<double>(divForSps(sps)), static_cast<double>(rampS));
        break;
    }
    case 'k': {
        if(!readIntInput(sArgValue, "加速度 steps/s² (0=等速, 建議 50000-500000)：")) {
            break;
        }
        if(sArgValue < 0) {
            sArgValue = 0;
        }
        gAccelSps2 = static_cast<float>(sArgValue);
        Serial.printf(">> 加速度 = %ld steps/s²\n", static_cast<long>(sArgValue));
        break;
    }
    case 'a': {
        if(!readIntInput(sArgValue, "角度 (°) [負=反轉]：")) {
            break;
        }
        float stepsF = static_cast<float>(sArgValue) * gStepsPerRev / 360.0f;
        int32_t steps = static_cast<int32_t>(stepsF < 0 ? -stepsF : stepsF);
        if(steps <= 0) {
            Serial.println("  忽略：步數為 0");
            break;
        }
        startMotion(steps, sArgValue >= 0 ? 1 : -1, false);
        break;
    }
    case 'f':
    case 'b': {
        if(!readIntInput(sArgValue, "步數：")) {
            break;
        }
        if(sArgValue <= 0) {
            break;
        }
        startMotion(sArgValue, c == 'f' ? 1 : -1, false);
        break;
    }
    case '1':
        startMotion(0, +1, true);
        break;
    case '2':
        startMotion(0, -1, true);
        break;
    case '0':
        stopMotion("使用者停止");
        break;
    case 'c': {
        if(!readIntInput(sArgValue, "每圈脈衝數 (200×細分，例如 16細分=3200)：")) {
            break;
        }
        if(sArgValue < 200) {
            Serial.println("  ⚠ 至少 200");
            break;
        }
        gStepsPerRev = static_cast<float>(sArgValue);
        // 細分改變後可用轉速範圍也變了，重算一次避免顯示/行為不一致
        gSpeedRpm = spsToRpm(clampSps(rpmToSps(gSpeedRpm)));
        Serial.printf(">> 每圈脈衝數 = %ld（%.4f 步/度，目前轉速 %.1f RPM）\n", static_cast<long>(sArgValue),
                      static_cast<double>(gStepsPerRev / 360.0f),
                      static_cast<double>(gSpeedRpm));
        break;
    }
    case 'n': {
        gEnActiveLow = !gEnActiveLow;
        applyEnablePin();
        Serial.printf(">> EN 極性 = %s\n", gEnActiveLow ? "低電位致能" : "高電位致能");
        break;
    }
    case 'r': {
        gDirInvert = !gDirInvert;
        applyDir(gSt.dir);
        Serial.printf(">> DIR 反向 = %d\n", gDirInvert ? 1 : 0);
        break;
    }
    case '?':
    default:
        printHelp();
        break;
    }
}
