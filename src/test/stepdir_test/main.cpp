// ============================================================================
// stepdir_test — 步進馬達 EN / STEP / DIR 脈衝控制測試（PIO 版本, RP2350）
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
// 接線（預設，可改 namespace Pin）：
//   RP2350 GP2 → 驅動器 PUL / STEP   （PIO0 SM0）
//   RP2350 GP3 → 驅動器 DIR
//   RP2350 GP4 → 驅動器 ENA / EN
//   RP2350 GND → 驅動器 COM
//
// ⚠ COM 接法（光耦輸入）：
//   1) 共陰（預設）：COM 接 GND，訊號吃 3.3V
//   2) 共陽：COM 接 +5V，訊號改低電位有效 → 用 'n' 切 EN 極性
//
// 按鍵（USB Serial, 115200）：
//   't' 一鍵自我測試（+1圈 → 停 → −1圈 → 停 → 低速 36°）
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
constexpr uint STEP = 2; // → 驅動器 PUL（PIO0 SM0 產生，固定 7 週期/步）
constexpr uint DIR = 3;  // → 驅動器 DIR
constexpr uint EN = 4;   // → 驅動器 ENA
} // namespace Pin

// ============================================================================
// PIO 組態
// ============================================================================
static PIO gPio = pio0;
static uint gSm = 0;
static uint gOffset = 0;
constexpr uint gIrqFlag = 0;        // step_pulse.pio 到位時設定的 IRQ 旗標
constexpr float kCyclesPerStep = 7.0f; // set[3] = 4 + set[1] = 2 + jmp = 1
static float gClkSysHz = 150000000.0f;

// ============================================================================
// 參數（可調）
// ============================================================================
static bool gEnActiveLow = true;            // EN 低電位致能（多數 TB6600/DM542）
static bool gDirInvert = false;             // DIR 方向反向
static float gStepsPerRev = 200.0f * 16.0f; // 1.8° 步進 × 16 細分 = 3200
static float gSpeedRpm = 120.0f;            // 目標轉速
static float gAccelSps2 = 50000.0f;         // 加速度 steps/s²（0 = 不加速）
// 脈衝速率上下限由 PIO clock divider 決定：clk_sys / (7 × 1) ~ clk_sys / (7 × 65536)
constexpr float kMinSps = 330.0f;   // ≈ 6 RPM @ 3200 步/圈（divider 上限）
constexpr float kMaxSps = 150000.0f; // 軟體上限；再上去多數驅動器跟不上

// ============================================================================
// 運動狀態（靜態配置，專案禁止 new/malloc）
// ============================================================================
struct StepperState {
    bool running = false;
    bool continuous = false;
    bool enabled = false;
    int32_t total = 0;       // 本次命令總步數（連續模式為 0）
    float speedSps = 0.0f;   // 目前命令速率 steps/s
    float targetSps = 0.0f;  // 目標速率 steps/s
    float estSteps = 0.0f;   // 已發出步數估算（積分速率）
    int8_t dir = 1;          // +1 正轉，-1 反轉
    uint32_t lastRampUs = 0;
};
static StepperState gSt;

static int32_t sArgValue = 0;

// ============================================================================
// PIO 底層
// ============================================================================
static void pioSetup() {
    gOffset = pio_add_program(gPio, &step_pulse_program);
    pio_sm_config c = step_pulse_program_get_default_config(gOffset);

    // STEP 交給 PIO 的 "set pins" 群組（1 bit）
    sm_config_set_set_pins(&c, Pin::STEP, 1);
    pio_gpio_init(gPio, Pin::STEP);
    pio_sm_set_consecutive_pindirs(gPio, gSm, Pin::STEP, 1, true);

    pio_sm_init(gPio, gSm, gOffset, &c);
    pio_sm_set_pins(gPio, gSm, 0); // STEP 先拉低
    pio_sm_set_enabled(gPio, gSm, true);
}

// 速率 → clock divider（含小數，SDK 內部為 16.8 定點）
static float divForSps(float sps) {
    float div = gClkSysHz / (kCyclesPerStep * sps);
    if(div < 1.0f) {
        div = 1.0f;
    }
    if(div > 65536.0f) {
        div = 65536.0f;
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

// 停止脈衝：關 SM、清 FIFO、STEP 拉低、清到位旗標
static void haltPulses() {
    pio_sm_set_enabled(gPio, gSm, false);
    pio_sm_clear_fifos(gPio, gSm);
    pio_sm_set_pins(gPio, gSm, 0); // 若停在 STEP 高電位，強制拉低
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
    bool active = gSt.enabled ? gEnActiveLow : !gEnActiveLow;
    gpio_put(Pin::EN, active ? 0 : 1);
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
    if(sps < kMinSps) {
        return kMinSps;
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
    haltPulses();

    gSt.dir = (dir >= 0) ? 1 : -1;
    applyDir(gSt.dir);

    gSt.targetSps = clampSps(rpmToSps(gSpeedRpm));
    // 短行程：把峰值速度壓到「加速段 + 減速段剛好走完」的值，避免衝上去馬上煞停的抖動
    if(!continuous && gAccelSps2 > 0.0f && steps > 0) {
        float peak = sqrtf(2.0f * gAccelSps2 * static_cast<float>(steps));
        if(peak < gSt.targetSps) {
            gSt.targetSps = (peak < kMinSps) ? kMinSps : peak;
        }
    }
    gSt.speedSps = (gAccelSps2 > 0.0f) ? kMinSps : gSt.targetSps;
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
        Serial.printf(">> 到位（%ld 步，結束轉速 %.1f RPM）\n", static_cast<long>(gSt.total),
                      static_cast<double>(spsToRpm(gSt.speedSps)));
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
    if(!gSt.continuous && gAccelSps2 > 0.0f && gSt.speedSps > kMinSps) {
        float left = static_cast<float>(gSt.total) - gSt.estSteps;
        float decelDist = (gSt.speedSps * gSt.speedSps) / (2.0f * gAccelSps2);
        if(left <= decelDist + 1.0f) {
            targetSps = kMinSps;
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
            Serial.printf("   … 剩約 %ld 步, %.1f RPM\n",
                          static_cast<long>(static_cast<float>(gSt.total) - gSt.estSteps),
                          static_cast<double>(spsToRpm(gSt.speedSps)));
        }
        if(millis() - t0 > 120000) { // 保險上限 2 分鐘
            stopMotion("逾時保護");
            break;
        }
    }
}

// ============================================================================
// 輸入（USB Serial 讀整數，可負）
// ============================================================================
static bool readIntInput(int32_t &val, const char *prompt, uint32_t waitMs = 15000) {
    uint32_t fd0 = millis();
    while(millis() - fd0 < 40) { // 清殘留
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

    Serial.println("[3] 低速 36°（細分/極性目視檢查）");
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
                  static_cast<double>(kMinSps), static_cast<double>(kMaxSps),
                  static_cast<double>(spsToRpm(kMinSps)), static_cast<double>(spsToRpm(kMaxSps)));
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
    Serial.println("\n[stepdir_test] Booting... (PIO STEP generator)");

    gClkSysHz = static_cast<float>(clock_get_hz(clk_sys));

    // DIR / EN 由 CPU 控制
    gpio_init(Pin::DIR);
    gpio_set_dir(Pin::DIR, GPIO_OUT);
    gpio_put(Pin::DIR, 0);

    gpio_init(Pin::EN);
    gpio_set_dir(Pin::EN, GPIO_OUT);
    gSt.enabled = false;
    applyEnablePin(); // 上電先保持去使能

    // STEP 交給 PIO（pio_gpio_init 會接管腳位功能）
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
                      static_cast<double>(spsToRpm(kMinSps)), static_cast<double>(spsToRpm(kMaxSps)));
        if(!readIntInput(sArgValue, "轉速 RPM：")) {
            break;
        }
        if(sArgValue < 1) {
            sArgValue = 1;
        }
        float sps = clampSps(rpmToSps(static_cast<float>(sArgValue)));
        gSpeedRpm = spsToRpm(sps);
        float rampS = (gAccelSps2 > 0.0f) ? (sps - kMinSps) / gAccelSps2 : 0.0f;
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
        Serial.printf(">> 每圈脈衝數 = %ld（%.4f 步/度）\n", static_cast<long>(sArgValue),
                      static_cast<double>(gStepsPerRev / 360.0f));
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
