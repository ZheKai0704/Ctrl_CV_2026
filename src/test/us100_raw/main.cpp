// ============================================================================
// US-100 最小裸測 — 逐一順序觸發、雙極性測試、無 PIO
//
// 目的：隔離 ultrasonic 節點的問題來源。本程式刻意使用最單純的手段：
//   - 不用 PIO、不用分組、不用濾波
//   - 一次只觸發「一顆」感測器，輪流 8 顆
//   - 每顆同時測「標準 active-HIGH」與「反相 active-LOW」兩種觸發極性
//   - echo 用 Arduino pulseIn() 直接量 HIGH 脈寬
//
// 可一次性回答：
//   1. 這批 US-100 的觸發極性到底要 std 還是 inv
//   2. 順序單發、120ms 間隔下，能否穩定量到 5m
//   3. 不穩是韌體管線問題，還是感測器/接線問題
//
// 輸出（115200）：
//   "R0:FL std=2855mm(16900us) inv=---"
//   std = 標準（Trig 閒置 LOW，HIGH 12us 脈衝）
//   inv = 反相（Trig 閒置 HIGH，LOW 12us 脈衝）
//   --- = 該極性 30ms 無回音
//
// 距離換算：mm = echo_us * 338 / 2000（338 m/s 往返）
// 預期 raw µs 參考：30cm≈1770、1m≈5917、2.559m≈15142、5m≈29586
// ============================================================================

#include <Arduino.h>

struct SensorDef {
    const char *label;
    uint8_t trig;
    uint8_t echo;
};

// 與 src/ultrasonic/main.cpp 相同的腳位對應
static const SensorDef kSensors[8] = {
    {"FL", 20, 21}, // front left
    {"FR", 2, 3},   // front right
    {"L1", 16, 17}, // left front
    {"L2", 18, 19}, // left rear
    {"R1", 6, 7},   // right front
    {"R2", 10, 11}, // right rear
    {"BL", 12, 13}, // back left
    {"BR", 14, 15}, // back right
};

static constexpr uint32_t TRIG_PULSE_US = 12;
static constexpr uint32_t ECHO_TIMEOUT_US = 35000; // ≈5.9m
static constexpr uint32_t INTER_SENSOR_GAP_MS = 120; // > US-100 建議 60ms

static int32_t measureOne(uint8_t trig, uint8_t echo, bool inverted,
                          uint32_t *rawUs) {
    // 閒置電位：std = LOW，inv = HIGH
    digitalWrite(trig, inverted ? HIGH : LOW);
    delayMicroseconds(5000); // 讓感測器看清閒置狀態

    // 觸發脈衝：std = HIGH 12us，inv = LOW 12us
    digitalWrite(trig, inverted ? LOW : HIGH);
    delayMicroseconds(TRIG_PULSE_US);
    digitalWrite(trig, inverted ? HIGH : LOW);

    // echo HIGH 脈寬（US-100 echo 為 HIGH 脈寬 = 飛行時間）
    const uint32_t us = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
    *rawUs = us;
    if(us == 0)
        return -1;
    const int32_t mm = static_cast<int32_t>(
        (static_cast<uint64_t>(us) * 338ULL) / 2000ULL);
    return mm;
}

void setup() {
    Serial.begin(115200);
    while(!Serial) {
        tight_loop_contents();
    }

    for(const auto &s : kSensors) {
        pinMode(s.trig, OUTPUT);
        digitalWrite(s.trig, LOW); // 先以標準閒置 LOW 起始
        pinMode(s.echo, INPUT);    // 純輸入，不上拉不下拉，看真實電位
    }
    delay(200);

    // 開機即報告各 echo 腳的靜態電位（極性診斷線索）
    Serial.println("BOOT: echo idle levels (LOW/HIGH):");
    for(const auto &s : kSensors) {
        Serial.printf("  %s echo(GP%u) = %s\n", s.label, s.echo,
                      digitalRead(s.echo) ? "HIGH" : "LOW");
    }
}

void loop() {
    for(const auto &s : kSensors) {
        uint32_t usStd = 0, usInv = 0;
        const int32_t mmStd = measureOne(s.trig, s.echo, false, &usStd);
        delay(10);
        const int32_t mmInv = measureOne(s.trig, s.echo, true, &usInv);

        Serial.printf("R0:%s std=", s.label);
        if(mmStd > 0)
            Serial.printf("%dmm(%luus)", mmStd,
                          static_cast<unsigned long>(usStd));
        else
            Serial.print("---");
        Serial.print(" inv=");
        if(mmInv > 0)
            Serial.printf("%dmm(%luus)", mmInv,
                          static_cast<unsigned long>(usInv));
        else
            Serial.print("---");
        Serial.println();

        delay(INTER_SENSOR_GAP_MS);
    }
}
