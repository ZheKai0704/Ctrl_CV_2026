#pragma once
#include <Arduino.h>
#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <cstdio>

// ============================================================================
// SlaveLog — Core 1 → Core 0 的環形文字日誌（SPSC，單一實例）
//
// 為什麼抽出來：
//   ⚠ AGENTS.md 規定 Serial 只給 Core 0。Core 1 直接 Serial.printf() 在 USB CDC
//     TX FIFO 滿時會阻塞（實測最長 1 秒），會把機構序列卡在半途、看起來像
//     「seq=RUN 但馬達不動」。因此 Core 1 只把短字串丟進環形佇列，
//     由 Core 0 的 loop() 統一把它印出來（drain()）。
//
//   Storage 原本有一份可用的實作（LogQ，用 volatile head/tail）。抽成 lib 並把
//   head/tail 改成 std::atomic<uint8_t> + release/acquire。
//
//   記憶體屏障語意（為什麼不能只靠 relaxed）：
//     - buf_[t] 的內容是「非原子」寫入（逐 char 複製），relaxed 只保證
//       head_/tail_ 本身是原子存取，不保證 payload 對另一顆核心可見。若 tail
//       用 relaxed store，Core 0 可能看到新 tail 卻讀到只寫一半的字串。
//     - push()：先寫完 payload，再 tail_.store(release)。release 保證「payload
//       的所有寫入」都排在 tail 更新之前被看見 → Core 0 一旦觀察到新 tail，
//       就保證能讀到完整的那一行。
//     - drain()：tail_.load(acquire) 與上面的 release 配對，取得 payload 的可見性，
//       之後才 println(buf_[h])，不會印出截斷/混雜的日誌行。
//     - head_ 對稱處理：Core 0 用 head_.store(release) 宣告「這一格我讀完了」，
//       Core 1 用 head_.load(acquire) 判斷佇列是否已滿，確保 Core 1 覆寫該格時
//       Core 0 的讀取已完成（不會邊印邊被覆寫）。
//     - 兩邊都只讀「自己的」索引時（tail 在 push、head 在 drain）可用 relaxed：
//       那是同核心自己的值，本來就有順序。
//
//   目前只有 storage 使用（Core 1 狀態機 → Core 0 印出）；shoot/move 的 Serial
//   呼叫都在 loop()（Core 0），暫不需要。
//
// 來源：src/storage/main.cpp 的 namespace LogQ。
//
// 使用限制：
//   - SPSC：只能「一個」Core 1 寫者 + 「一個」Core 0 讀者（本專案即如此）。
//   - push() / printf() 不阻塞、不配置記憶體；佇列滿就丟棄並計數（dropped()）。
//   - printf() 只支援整數格式（newlib-nano 沒開浮點 printf，%f 印不出東西）。
//   - 單行上限 LINE_LEN（含 '\0'），超過會被截斷（不會溢寫）。
//   - 實例請宣告成 static（緩衝區 32×96 = 3072 bytes 落在 .bss）。
// ============================================================================
class SlaveLog {
  public:
    static constexpr uint8_t SLOTS = 32;    // 佇列格數
    static constexpr uint8_t LINE_LEN = 96; // 單行長度

    /// Core 1 呼叫：寫入一行（不阻塞；佇列滿就丟棄並計數）
    void push(const char *text) {
        // tail_ 是「本核心（寫者）自己的」索引 → relaxed 即可
        const uint8_t t = tail_.load(std::memory_order_relaxed);
        // head_ 由 Core 0 用 release 寫 → 這裡用 acquire 讀，才看得到「已讀完」
        if(nextIdx(t) == head_.load(std::memory_order_acquire)) { // 滿了
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        char *dst = buf_[t];
        uint8_t i = 0;
        for(; text[i] != '\0' && i < LINE_LEN - 1; i++)
            dst[i] = text[i];
        dst[i] = '\0';
        // release：保證上面 buf_[t] 的 payload 寫入先於 tail 更新被 Core 0 看見
        tail_.store(nextIdx(t), std::memory_order_release);
    }

    /// Core 1 呼叫：格式化後寫入一行（只支援整數格式，newlib-nano 無浮點 printf）
    void printf(const char *fmt, ...) {
        char tmp[LINE_LEN];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(tmp, sizeof(tmp), fmt, ap);
        va_end(ap);
        push(tmp);
    }

    /// Core 0 呼叫：把佇列裡累積的訊息印出來
    void drain(Print &out) {
        // tail_.load(acquire) 與 push() 的 release store 配對：觀察到新 tail
        // 就保證 buf_[h] 的 payload 已寫完（不會印到只寫一半的行）。
        // head_ 是 Core 0 自己的索引 → relaxed 即可。
        while(head_.load(std::memory_order_relaxed) !=
              tail_.load(std::memory_order_acquire)) {
            const uint8_t h = head_.load(std::memory_order_relaxed);
            out.println(buf_[h]);
            // release：宣告「這一格讀完了」，Core 1 才能安全覆寫它
            head_.store(nextIdx(h), std::memory_order_release);
        }
    }

    /// 因佇列滿而丟棄的行數（Core 0 讀，用於診斷日誌是否塞爆）
    uint32_t dropped() const {
        return dropped_.load(std::memory_order_relaxed);
    }

  private:
    static uint8_t nextIdx(uint8_t i) {
        return static_cast<uint8_t>((i + 1) % SLOTS);
    }

    char buf_[SLOTS][LINE_LEN] = {};
    std::atomic<uint8_t> head_{0};       // Core 0 讀取位置
    std::atomic<uint8_t> tail_{0};       // Core 1 寫入位置
    std::atomic<uint32_t> dropped_{0};   // Core 1 累加、Core 0 讀取
};
