#pragma once
#include <EEPROM.h>
#include <cmath>
#include <cstddef>
#include <cstdint>

// ============================================================================
// EepromStore — EEPROM（flash 模擬）上的「magic + checksum」版面通用存取
//
// 為什麼抽出來：
//   Storage 的機構校正（Persist / MAGIC "PCA4"）與 Shoot 的轉速設定都想避免
//   讀到「抹除態 = 全 0xFF」或半寫入的資料。原本只有 Storage 自己做 magic +
//   逐 byte 加總 checksum 驗證；抽成 lib 後第二個使用者不必重寫一份驗證邏輯。
//
// 來源：
//   - checksum32()     ← src/storage/main.cpp Limits::checksumOf()
//   - load() / save()  ← src/storage/main.cpp Limits::load() 的驗證流程
//   - sanitizeFloat()  ← src/shoot/main.cpp SpeedStorage::init() 的 NaN → 0 處理
//
// ⚠ EEPROM 版面（偏移、欄位順序、magic 值）一律由呼叫端的 struct 決定，
//   本模組只負責「驗證」與「寫入」，不得更動任何既有版面，否則現車上的
//   機構校正 / 轉速設定會被清掉。
//
// 使用限制：
//   - T 必須有 uint32_t magic 欄位，且「尾端」必須是 uint32_t checksum
//     （checksum 計算範圍 = 前面所有 bytes，與原 Storage 相同）。
//   - save() 會呼叫 EEPROM.commit()（flash 抹寫，耗時數十 ms，期間 XIP 停頓）
//     → 只在值真的改變時呼叫（呼叫端負責比對）。
//   - EEPROM.begin(size) 由呼叫端負責，本模組不決定大小。
// ============================================================================
namespace EepromStore {

/// 逐 byte 加總（與原 Storage 的 checksumOf 完全相同，不要改成 CRC）
/// ⚠ 目前沒有 lib 外的呼叫者（只有 load() / save() 內部使用）；保留作為
///   本 lib 的公開介面，header-only 用不到就不佔空間。
inline uint32_t checksum32(const void *data, size_t len) {
    const uint8_t *raw = static_cast<const uint8_t *>(data);
    uint32_t sum = 0;
    for(size_t i = 0; i < len; i++)
        sum += raw[i];
    return sum;
}

/// 從 EEPROM 讀出 T 並驗證 magic + checksum。
/// @param magic 期望的 magic 值（如 Storage 的 0x50434134 "PCA4"）
/// @param addr  EEPROM 偏移
/// @param out   讀出的原始內容（驗證失敗時為「讀到的原樣」，可能是 0xFF 抹除態
///              → 呼叫端負責套用預設值）
/// @return true = magic 與 checksum 都相符
template <typename T>
bool load(uint32_t magic, int addr, T &out) {
    static_assert(sizeof(T) > sizeof(uint32_t),
                  "T 尾端必須有 uint32_t checksum 欄位");
    EEPROM.get(addr, out);
    if(out.magic != magic)
        return false;
    return out.checksum == checksum32(&out, sizeof(T) - sizeof(uint32_t));
}

/// 寫入 T（自動填上 magic 與尾端 checksum）。
/// ⚠ 目前尚無呼叫者（storage 只讀不寫、shoot 只借用 sanitizeFloat）；
///   保留作為本 lib 的完整介面，給後續需要持久化的節點。
/// @return EEPROM.commit() 的結果
/// @note 呼叫前請確保 EEPROM.begin(size) 已完成，且只在值真的變更時呼叫。
template <typename T>
bool save(uint32_t magic, int addr, const T &in) {
    static_assert(sizeof(T) > sizeof(uint32_t),
                  "T 尾端必須有 uint32_t checksum 欄位");
    T rec = in; // 拷貝一份再填 magic/checksum，不改動呼叫端資料
    rec.magic = magic;
    rec.checksum = checksum32(&rec, sizeof(T) - sizeof(uint32_t));
    EEPROM.put(addr, rec);
    return EEPROM.commit();
}

/// 抹除態（0xFF）讀成 float 會是 NaN → 歸零。
/// 原 Shoot 的 SpeedStorage::init()：flash 尚未初始化時兩顆 BLDC 轉速會是 NaN。
inline float sanitizeFloat(float v) {
    if(std::isnan(v))
        return 0.0f;
    return v;
}

} // namespace EepromStore
