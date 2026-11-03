#!/usr/bin/env python3
"""PCA9685 硬體自動化測試 (RP2350 + pca9685_test 固件)

流程：
  1. (可選) 燒錄 pca9685_test 固件
  2. 開序列埠，依序送出 s / i / t / v
  3. 解析輸出並判定 PASS / FAIL

用法：
  python3 tools/pca9685_hwtest.py                # 自動找埠、不燒錄
  python3 tools/pca9685_hwtest.py --upload       # 先燒錄再測試
  python3 tools/pca9685_hwtest.py -p /dev/cu.usbmodem101 -v
  python3 tools/pca9685_hwtest.py --addr 0x41    # 非預設位址

建議用 PlatformIO 內建 Python 執行（已含 pyserial）：
  ~/.platformio/penv/bin/python tools/pca9685_hwtest.py --upload

離開碼：0 = 全部 PASS，1 = 有 FAIL，2 = 環境/連線問題
"""

import argparse
import glob
import re
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial：請用 ~/.platformio/penv/bin/python 執行本腳本")

ENV = "pca9685_test"
BAUD = 115200


# ----------------------------------------------------------------------------
# 序列埠
# ----------------------------------------------------------------------------
def find_port() -> str | None:
    """找 usbmodem / usbserial 類型的埠。"""
    for pattern in ("/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/ttyACM*",
                    "/dev/ttyUSB*"):
        ports = sorted(glob.glob(pattern))
        if ports:
            return ports[0]
    return None


def upload(port: str | None) -> None:
    cmd = ["pio", "run", "-e", ENV, "-t", "upload"]
    if port:
        cmd += ["--upload-port", port]
    print(f"$ {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True)
    tail = (result.stdout + result.stderr).strip().splitlines()[-3:]
    for line in tail:
        print("  " + line)
    if result.returncode != 0:
        sys.exit("燒錄失敗，中止測試")
    time.sleep(1.5)  # 等 USB CDC 重新枚舉


class Device:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, BAUD, timeout=0.2)
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def send(self, cmd: str, wait: float = 1.2) -> str:
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode())
        self.ser.flush()
        buf = bytearray()
        deadline = time.time() + wait
        while time.time() < deadline:
            chunk = self.ser.read(4096)
            if chunk:
                buf.extend(chunk)
                deadline = time.time() + 0.25  # 收到資料後延長等待
        return buf.decode("utf-8", "replace")

    def send_retry(self, cmd: str, wait: float = 1.2, attempts: int = 3,
                   expect: str | None = None) -> str:
        """送命令；若沒回應（或回應不含 expect）就重送。"""
        out = ""
        for _ in range(attempts):
            out = self.send(cmd, wait)
            if out and (expect is None or expect in out):
                return out
            time.sleep(0.2)
        return out

    def close(self):
        self.ser.close()


# ----------------------------------------------------------------------------
# 測試項目
# ----------------------------------------------------------------------------
class Report:
    def __init__(self):
        self.rows: list[tuple[str, bool, str]] = []

    def add(self, name: str, ok: bool, detail: str = ""):
        self.rows.append((name, ok, detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" +
              (f"  — {detail}" if detail else ""))

    def summary(self) -> int:
        print("\n===== 測試結果 =====")
        width = max(len(r[0]) for r in self.rows)
        for name, ok, detail in self.rows:
            line = f"  {name.ljust(width)}  {'PASS' if ok else 'FAIL'}"
            if detail:
                line += f"  {detail}"
            print(line)
        failed = [r for r in self.rows if not r[1]]
        print(f"\n{len(self.rows) - len(failed)}/{len(self.rows)} 通過")
        if failed:
            print("未通過：")
            for name, _, detail in failed:
                print(f"  - {name} {detail}")
            return 1
        return 0


def run_tests(dev: Device, addr: int, report: Report) -> None:
    # --- 1. I2C 掃描 ---
    out = dev.send("s")
    found = re.findall(r"^\s*0x([0-9A-Fa-f]{2})", out, re.M)
    found_addrs = [int(a, 16) for a in found]
    report.add("I2C 匯流排有裝置回應", len(found_addrs) > 0,
               f"找到 {len(found_addrs)} 個：{', '.join(f'0x{a:02X}' for a in found_addrs)}")
    report.add(f"PCA9685 位於 0x{addr:02X}", addr in found_addrs,
               "" if addr in found_addrs else "掃不到 → 接線/上拉/電源/位址跳線")

    # --- 2. 初始化 ---
    out = dev.send("i", wait=1.5)
    report.add("PCA9685 初始化 (50Hz)", "[INIT] OK" in out,
               out.strip().splitlines()[-1] if out.strip() else "無回應")

    # --- 3. 暫存器狀態 ---
    out = dev.send("r")
    mode1 = re.search(r"MODE1\s*=\s*0x([0-9A-Fa-f]{2})", out)
    prescale = re.search(r"PRE_SCALE\s*=\s*(\d+)", out)
    freq = re.search(r"約\s*(\d+)\s*Hz", out)

    if mode1:
        m1 = int(mode1.group(1), 16)
        report.add("MODE1 SLEEP=0（震盪器運作中）", (m1 & 0x10) == 0,
                   f"MODE1=0x{m1:02X}")
        report.add("MODE1 AI=1（auto-increment）", (m1 & 0x20) != 0,
                   f"MODE1=0x{m1:02X}")
    else:
        report.add("MODE1 SLEEP=0（震盪器運作中）", False, "讀不到 MODE1")
        report.add("MODE1 AI=1（auto-increment）", False, "讀不到 MODE1")

    if prescale and freq:
        report.add("PWM 頻率 ≈50Hz", abs(int(freq.group(1)) - 50) <= 2,
                   f"PRE_SCALE={prescale.group(1)}, 約 {freq.group(1)}Hz")
    else:
        report.add("PWM 頻率 ≈50Hz", False, "讀不到 PRE_SCALE")

    # --- 4. MODE2 讀寫回測 ---
    out = dev.send("t")
    report.add("MODE2 讀寫回測", "[TEST] MODE2 讀寫回測 PASS" in out,
               out.strip().splitlines()[-1] if out.strip() else "無回應")

    # --- 5. 16 通道寫入→讀回驗證（連跑 3 次確認穩定）---
    for run in range(3):
        out = dev.send_retry("v", wait=3.0, expect="[VERIFY] RESULT")
        ch_lines = re.findall(
            r"\[VERIFY\] CH(\d+).*?tries=(\d+) (PASS|FAIL)", out)
        bad = [ch for ch, _, res in ch_lines if res == "FAIL"]
        result = re.search(r"\[VERIFY\] RESULT (PASS|FAIL) (\d+)/(\d+)", out)
        extra = ""
        m = re.search(r"I2C 錯誤 \+(\d+), 重試 \+(\d+)", out)
        if m:
            extra = f", I2C 錯誤 {m.group(1)}, 重試 {m.group(2)}"
        report.add(f"16 通道寫入/讀回（第 {run + 1} 次）",
                   bool(result) and result.group(1) == "PASS",
                   (f"{result.group(2)}/{result.group(3)}{extra}" if result
                    else "無回應"))
        if bad:
            report.add(f"失敗通道（第 {run + 1} 次）", False, "CH" + ", CH".join(bad))
        time.sleep(0.3)

    # --- 6. 連續讀取穩定度（不寫入，只看匯流排）---
    out = dev.send_retry("G 0 200", wait=4.0, expect="[STRESS]")
    m = re.search(r"mismatch=(\d+)", out)
    report.add("連續 200 次讀取穩定度", bool(m) and m.group(1) == "0",
               f"mismatch={m.group(1)}" if m else "無回應")

    # --- 7. 收尾：全部關閉 ---
    out = dev.send_retry("x", expect="[ALL]")
    report.add("全部通道關閉（安全收尾）", "[ALL] 全部關閉" in out)


def main() -> int:
    ap = argparse.ArgumentParser(description="PCA9685 硬體自動化測試")
    ap.add_argument("-p", "--port", help="序列埠（預設自動偵測）")
    ap.add_argument("--upload", action="store_true", help="先燒錄 pca9685_test 固件")
    ap.add_argument("--addr", default="0x40", help="PCA9685 位址（預設 0x40）")
    ap.add_argument("--verbose", "-v", action="store_true", help="印出原始序列輸出")
    args = ap.parse_args()

    addr = int(args.addr, 0)
    port = args.port or find_port()
    if not port:
        print("找不到序列埠，請確認 USB 已接上或指定 -p")
        return 2
    print(f"序列埠：{port}  @ {BAUD}")

    if args.upload:
        upload(port)
        port = find_port() or port

    try:
        dev = Device(port)
    except Exception as exc:  # noqa: BLE001
        print(f"無法開啟序列埠：{exc}")
        return 2

    try:
        # 確保目標位址正確
        dev.send(f"n {args.addr}", wait=0.5)
        time.sleep(0.2)
        report = Report()
        print("\n===== 開始測試 =====")
        run_tests(dev, addr, report)
        if args.verbose:
            raw = dev.send("s")
            print("\n--- 原始輸出 ---\n" + raw)
        return report.summary()
    finally:
        dev.close()


if __name__ == "__main__":
    sys.exit(main())
