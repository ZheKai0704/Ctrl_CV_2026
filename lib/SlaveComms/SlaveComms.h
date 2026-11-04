#pragma once
#include "ChassisComms.h"
#include "signal_shared.h"
#include <Arduino.h>
#include <atomic>
#include <cstring>
#include <hardware/watchdog.h>

// ============================================================================
// SlaveComms — 通用 Slave 雙核心通訊框架
//
// 把所有 Slave 共用的通訊邏輯（UART0 收發、ChassisComms、COBS 解包、
// CRC 驗證、E-Stop 處理、逾時保護、Core 0↔Core 1 命令傳遞）封裝成
// 一個可重複使用的類別。三個 Slave 只需填入各自的馬達控制 callback，
// 不再需要各自維護雙緩衝區與 spinloop 邏輯。
//
// 核心設計：
//   - Core 0 (loop)：  收 ChassisComms → 解命令 → 寫入 sharedCmd
//   - Core 1 (loop1)： 自旋讀取 sharedCmd → 執行馬達控制
//   - 同步機制：單緩衝區 + atomic<bool> flag（memory_order release/acquire）
//   - E-Stop：atomic<bool> 旗標，Core 0 觸發，Core 1 讀取後歸零 PWM
//
// 使用方式：
//
//   1. 實作你的 SlaveApp：
//
//      static void onMoveCmd(const ChassisSnapshot& cmd) {
//          // Mecanum → PID → PWM
//      }
//      static SlaveApp moveApp = {
//          .targetID  = TargetID::MOVE,
//          .onCommand = onMoveCmd,
//      };
//
//   2. 建立實例：
//
//      static SlaveComms slave(&moveApp);
//
//   3. 填入標準的 setup/loop/loop1：
//
//      void setup() {
//          Serial1.begin(921600);
//          slave.begin();
//      }
//      void loop() {
//          slave.update();
//      }
//      void loop1() {
//          slave.runCore1();
//      }
//
// 注意：
//   所有 memory barrier 使用 memory_order_release（寫入端）和
//   memory_order_acquire（讀取端），確保 RP2350 SCU cache coherency。
//   無需額外的 volatile 修飾。
// ============================================================================

// ============================================================================
// 應用層回呼介面 — 由使用者在自己的 main.cpp 中提供
// ============================================================================
struct SlaveApp {
    /// 此 Slave 的 TargetID（TargetID::MOVE / SHOOT / STORAGE）
    uint8_t targetID;

    /// 收到有效控制幀時呼叫（Core 1 context，不可阻塞）
    /// @param cmd  解碼後的命令（vx/vy/omega 已轉為 float，單位 RPS）
    void (*onCommand)(const ChassisSnapshot &cmd);

    /// E-Stop 觸發時呼叫（Core 0 或 Core 1 context）
    /// 應立即歸零所有 PWM / 馬達輸出
    void (*onEstop)();
};

// ============================================================================
// SlaveComms — 核心類別
// ============================================================================
class SlaveComms {
  public:
    /// @param app  應用層回呼介面（必須存活超過 SlaveComms 生命期）
    explicit SlaveComms(SlaveApp *app)
        : app_(app),
          hubComms_(&Serial1, TargetID::HUB, RobotConfig::HEARTBEAT_INTERVAL_US) {}

    /// 初始化（在 setup() 中呼叫）
    /// 1. 啟動 Serial1（硬體 UART0，預設 TX=GP0, RX=GP1）
    /// 2. 初始化 ChassisComms 綁定 Serial1
    /// 3. 註冊幀回呼
    /// 4. 重設內部狀態
    void begin() {
        Serial1.begin(RobotConfig::PIO_BAUDRATE);
        hubComms_.begin();
        hubComms_.setFrameCallback(onFrameThunk);

        instance_ = this;

        cmdValid_.store(false, std::memory_order_relaxed);
        estopActive_.store(false, std::memory_order_relaxed);
        lastValidCmdTime_ = time_us_64();
    }

    /// Core 0 主迴圈（在 loop() 中呼叫）
    ///
    /// 1. 檢查 Reboot 旗標 → watchdog 重開機
    /// 2. ChassisComms::update() → 收 COBS 幀 → CRC 驗證 → 回呼分發
    /// 3. 100ms 逾時檢查 → E-Stop
    /// 4. 發送遙測（每50ms 一次）
    void update() {
        // Reboot 優先：收到 Hub 的 REBOOT 訊息後立即重開機
        if(rebootRequested_) {
            watchdog_reboot(0, 0, 0); // RP2350 watchdog 立即標準重開機
        }

        hubComms_.update();

        uint64_t now = time_us_64();
        if(!estopActive_.load(std::memory_order_acquire) &&
           (now - lastValidCmdTime_ > RobotConfig::COMMAND_TIMEOUT_US)) {
            triggerEstop();
        }

        // 遙測定時發送（50ms 間隔 = 20Hz）
        sendTelemetryIfReady();

        delayMicroseconds(20);
    }

    /// 設定 Core 1 空閒回呼（選用）
    ///
    /// 在 runCore1() 的 spinloop 中，每次迭代沒有新命令且非 E-Stop 時呼叫。
    /// 用於需要持續執行的維護工作（如步進馬達增量移動）。
    /// 注意：在 Core 1 context 中執行，不可阻塞。
    void setIdleHook(void (*hook)()) {
        idleHook_ = hook;
    }

    /// Core 1 控制迴圈（在 loop1() 中呼叫）
    ///
    /// 自旋等待，執行以下邏輯：
    ///   1. E-Stop 旗標檢查
    ///   2. 有命令時讀取 sharedCmd → 執行 app.onCommand
    ///   3. 沒命令時繼續 spin
    ///
    /// 不回傳，永遠執行直到系統重啟。
    void runCore1() {
        while(true) {
            // E-Stop 優先檢查
            if(estopActive_.load(std::memory_order_acquire)) {
                if(app_->onEstop != nullptr)
                    app_->onEstop();
                continue;
            }

            // 讀取命令
            if(cmdValid_.load(std::memory_order_acquire)) {
                ChassisSnapshot localCmd;
                memcpy(&localCmd, &sharedCmd_, sizeof(ChassisSnapshot));
                cmdValid_.store(false, std::memory_order_release);

                if(app_->onCommand != nullptr)
                    app_->onCommand(localCmd);
                continue; // 執行完命令後直接下一輪，不經 idleHook
            }

            // 空閒回呼（如步進馬達增量移動）
            if(idleHook_ != nullptr)
                idleHook_();
        }
    }

    /// 手動觸發 E-Stop（可在外部呼叫）
    void triggerEstop() {
        if(estopActive_.load(std::memory_order_acquire))
            return;
        estopActive_.store(true, std::memory_order_release);
        if(app_->onEstop != nullptr)
            app_->onEstop();
    }

    /// 清除 E-Stop
    void clearEstop() {
        estopActive_.store(false, std::memory_order_release);
    }

    /// 檢查是否處於 E-Stop 狀態
    bool isEstopActive() const {
        return estopActive_.load(std::memory_order_acquire);
    }

    /// 距上次收到有效控制幀的時間 (ms)。逾時保護用之除錯：若持續增大，
    /// 表示 UART0 沒收到 Hub 有效幀（接線/波特率/COBS 問題）。
    uint32_t msSinceLastValidCmd() const {
        return static_cast<uint32_t>((time_us_64() - lastValidCmdTime_) / 1000ULL);
    }

    /// 收到有效控制幀的總次數（除錯用）
    uint32_t validCmdCount() const { return validCmdCount_; }

    // ---- RX 診斷（轉發自 ChassisComms，供序列監控確認「到底有沒有收到」）----
    uint32_t rxFrameCount() const { return hubComms_.rxFrameCount(); }
    uint32_t rxCrcErrorCount() const { return hubComms_.rxCrcErrorCount(); }
    uint8_t lastRxMsgType() const { return hubComms_.lastRxMsgType(); }
    uint8_t lastRxTargetID() const { return hubComms_.lastRxTargetID(); }
    uint8_t lastRxFrameSeq() const { return hubComms_.lastRxFrameSeq(); }

    /// 取得 ChassisComms 參考（用於自訂收發）
    ChassisComms &comms() { return hubComms_; }

    /// Core 1 呼叫：準備遙測資料，由 Core 0 在下次 update() 中發送。
    /// 在 onCommand() callback 中構建 MoveTelemetry 後呼叫此方法。
    /// 使用 memcpy + atomic flag 進行跨核心傳遞。
    void prepareTelemetry(const MoveTelemetry &tm) {
        memcpy(&pendingTelemetry_, &tm, sizeof(MoveTelemetry));
        telemetryReady_.store(true, std::memory_order_release);
    }

  private:
    SlaveComms(const SlaveComms &) = delete;
    SlaveComms &operator=(const SlaveComms &) = delete;

    // ========================================================================
    // ChassisComms 幀回呼（static thunk → 成員函式）
    // ========================================================================
    static void onFrameThunk(const ControlPacket &pkt) {
        if(instance_ != nullptr)
            instance_->onFrame(pkt);
    }

    void onFrame(const ControlPacket &pkt) {
        // E-Stop 廣播 — 優先處理（CLAUDE.md §E-Stop 安全機制）
        if(pkt.msgType == MsgType::ESTOP) {
            triggerEstop();
            return;
        }

        // Reboot 指令 — Hub 偵測到本 Slave 遙測逾時時發送。
        // 只接受「指定本機」或「廣播」的 REBOOT，避免誤重開別台。
        if(pkt.msgType == MsgType::REBOOT) {
            if(pkt.targetID == app_->targetID ||
               pkt.targetID == TargetID::BROADCAST) {
                instance_->rebootRequested_ = true;
            }
            return;
        }

        // BROADCAST 型別 → E-Stop 檢查：只有 msgType=ESTOP 時才觸發
        // 一般 CONTROL 廣播（如 RC 備援）只送 MOVE 的 targetID

        // 只處理控制幀
        if(pkt.msgType != MsgType::CONTROL)
            return;

        // 過濾非自己的幀
        if(pkt.targetID != app_->targetID && pkt.targetID != TargetID::BROADCAST)
            return;

        // 更新最後有效幀時間（逾時保護用）+ 計數
        lastValidCmdTime_ = time_us_64();
        validCmdCount_++;

        // 根據 targetID 解碼不同的 payload
        constexpr float SCALE = RobotConfig::MAX_RPS / 127.0f;
        ChassisSnapshot snap = {};

        if(pkt.targetID == TargetID::MOVE) {
            snap.vx = static_cast<float>(pkt.move.vx) * SCALE;
            snap.vy = static_cast<float>(pkt.move.vy) * SCALE;
            snap.omega = static_cast<float>(pkt.move.omega) * SCALE;
        } else if(pkt.targetID == TargetID::SHOOT) {
            snap.shootUpSpeed = static_cast<float>(pkt.shoot.upSpeed);
            snap.shootDownSpeed = static_cast<float>(pkt.shoot.downSpeed);
            snap.pitch = static_cast<float>(pkt.shoot.pitch);
            snap.row = static_cast<float>(pkt.shoot.row);
        } else if(pkt.targetID == TargetID::STORAGE) {
            snap.motorChannel = pkt.storage.motorChannel;
            snap.sw = pkt.storage.sw;
            snap.trigger = pkt.storage.trigger;
        }

        snap.frameSeq = pkt.frameSeq;
        snap.lastArrivalTick = lastValidCmdTime_;

        // 寫入共享命令緩衝區（單緩衝 + atomic flag）
        // 使用 memcpy 確保 struct 的原子性複製
        memcpy(&sharedCmd_, &snap, sizeof(ChassisSnapshot));

        // release barrier：確保 sharedCmd_ 寫入完成後才讓 Core 1 看到 flag
        cmdValid_.store(true, std::memory_order_release);

        // 如果 E-Stop 曾觸發，現在收到有效幀 → 解除
        if(estopActive_.load(std::memory_order_acquire)) {
            estopActive_.store(false, std::memory_order_release);
        }
    }

    // ========================================================================
    // 成員變數
    // ========================================================================
    SlaveApp *app_;
    ChassisComms hubComms_;

    // Core 0 ↔ Core 1 共享命令（單緩衝 + atomic flag）
    static inline ChassisSnapshot sharedCmd_{};
    static inline std::atomic<bool> cmdValid_{false};
    static inline std::atomic<bool> estopActive_{false};
    static inline uint64_t lastValidCmdTime_ = 0;
    static inline uint32_t validCmdCount_ = 0;

    // 遙測：Core 1 寫入 → Core 0 發送（單緩衝 + atomic flag）
    static constexpr uint64_t TELEMETRY_INTERVAL_US = 50000ULL; // 50ms = 20Hz
    MoveTelemetry pendingTelemetry_{};
    std::atomic<bool> telemetryReady_{false};
    uint64_t lastTelemetrySendUs_ = 0;

    // Reboot：Hub 發送 REBOOT → Core 0 執行 watchdog 重開機
    volatile bool rebootRequested_ = false;

    // static this 指標供 thunk 使用
    static inline SlaveComms *instance_ = nullptr;

    // Core 1 空閒回呼（不為 null 時每次 spinloop 迭代非命令/非 E-Stop 時呼叫）
    void (*idleHook_)() = nullptr;

    // ========================================================================
    // 遙測定時發送（Core 0 context）
    // ========================================================================
    void sendTelemetryIfReady() {
        if(!telemetryReady_.load(std::memory_order_acquire))
            return;

        uint64_t now = time_us_64();
        if(now - lastTelemetrySendUs_ < TELEMETRY_INTERVAL_US)
            return;

        // 準備快照並發送
        MoveTelemetry snapshot;
        memcpy(&snapshot, &pendingTelemetry_, sizeof(MoveTelemetry));
        telemetryReady_.store(false, std::memory_order_release);

        hubComms_.sendTelemetry(snapshot);
        lastTelemetrySendUs_ = now;
    }
};
