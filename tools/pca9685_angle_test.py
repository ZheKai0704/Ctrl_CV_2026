#!/usr/bin/env python3
"""PCA9685 角度對應自動化測試（0~180°）

搭配固件 src/test/pca9685_angle/main.cpp（PlatformIO 環境 pca9685_angle）。

測試項目：
  1. 角度對應表 (d)：驗證固件算出的 0~180° 脈寬/offCount 是否正確
  2. 寫入→讀回 (v)：每個角度實際寫進 PCA9685 再讀回比對（硬體驗證）
  3. 可視化 - 逐點定位 (--visual CH)：0/15/.../180° 每點停留，肉眼看伺服
  4. 可視化 - 連續掃描 (--sweep CH)：0↔180° 來回掃描
  5. 端點校正 (--calibrate CH)：輸入你量到/觀察到的安全脈寬，寫入端點

用法：
  ~/.platformio/penv/bin/python tools/pca9685_angle_test.py            # 自動測試
  ~/.platformio/penv/bin/python tools/pca9685_angle_test.py --upload   # 先燒錄
  ~/.platformio/penv/bin/python tools/pca9685_angle_test.py --visual 0
  ~/.platformio/penv/bin/python tools/pca9685_angle_test.py --sweep 0 --seconds 8
  ~/.platformio/penv/bin/python tools/pca9685_angle_test.py --calibrate 0
  ~/.platformio/penv/bin/python tools/pca9685_angle_test.py -p /dev/cu.usbmodem1101

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

ENV = "pca9685_angle"
BAUD = 115200

# 固件預設端點（可用 --min/--max 覆蓋，需與固件 'ma' 設定一致）
DEF_MIN_US = 600
DEF_MAX_US = 2400


# ----------------------------------------------------------------------------
# 序列埠
# ----------------------------------------------------------------------------
def find_port():
    for pattern in ("/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/ttyACM*",
                    "/dev/ttyUSB*"):
        ports = sorted(glob.glob(pattern))
        if ports:
            return ports[0]
    return None


def upload(port):
    cmd = ["pio", "run", "-e", ENV, "-t", "upload"]
    if port:
        cmd += ["--upload-port", port]
    print(f"$ {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True)
    for line in (result.stdout + result.stderr).strip().splitlines()[-3:]:
        print("  " + line)
    if result.returncode != 0:
        sys.exit("燒錄失敗")
    time.sleep(2.0)
    if port and not glob.glob(port):
        # 燒錄後埠號可能改變
        new_port = find_port()
        print(f"序列埠變更：{port} → {new_port}")
        return new_port
    return port


class Device:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.2)
        time.sleep(0.3)
        try:
            self.ser.dtr = True
            self.ser.rts = True
        except Exception:  # noqa: BLE001
            pass
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def send(self, cmd, wait=1.2):
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode())
        self.ser.flush()
        buf = bytearray()
        deadline = time.time() + wait
        while time.time() < deadline:
            chunk = self.ser.read(4096)
            if chunk:
                buf.extend(chunk)
                deadline = time.time() + 0.25
        return buf.decode("utf-8", "replace")


# ----------------------------------------------------------------------------
# 參考公式（與固件一致）
# ----------------------------------------------------------------------------
def angle_to_us(deg, min_us, max_us):
    return int((max_us - min_us) * deg / 180.0 + min_us + 0.5)


def us_to_count(us):
    return (us * 4096 + 10000) // 20000


def count_to_us(cnt):
    return cnt * 20000 // 4096


# ----------------------------------------------------------------------------
# 測試
# ----------------------------------------------------------------------------
class Report:
    def __init__(self):
        self.rows = []

    def add(self, name, ok, detail=""):
        self.rows.append((name, ok, detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" +
              (f"  — {detail}" if detail else ""))

    def summary(self):
        print("\n===== 結果 =====")
        width = max(len(r[0]) for r in self.rows)
        for name, ok, detail in self.rows:
            print(f"  {name.ljust(width)}  {'PASS' if ok else 'FAIL'}" +
                  (f"  {detail}" if detail else ""))
        failed = [r for r in self.rows if not r[1]]
        print(f"\n{len(self.rows) - len(failed)}/{len(self.rows)} 通過")
        return 1 if failed else 0


def fetch_config(dev, wait=1.5):
    """讀 l：得到各通道啟用/0°/180°/工作角度（權威來源）。"""
    out = dev.send("l", wait=wait)
    cfg = {}
    for m in re.finditer(
            r"\[LIMIT\]\s+(\d+)\s+(on|OFF)\s+(fwd|rev)\s+(\d+)\s+(\d+)"
            r"\s+(\d+)~(\d+)°", out):
        cfg[int(m.group(1))] = {
            "enabled": m.group(2) == "on",
            "invert": m.group(3) == "rev",
            "zero": int(m.group(4)),
            "max": int(m.group(5)),
            "minDeg": int(m.group(6)),
            "maxDeg": int(m.group(7)),
        }
    return cfg, out


def check_config(dev, report):
    cfg, out = fetch_config(dev)
    report.add("通道設定 (l) 可讀取", len(cfg) == 16, f"{len(cfg)}/16 通道")
    disabled = [ch for ch, c in cfg.items() if not c["enabled"]]
    if disabled:
        report.add("停用通道", True,
                   ", ".join(f"CH{ch}" for ch in sorted(disabled)))
    inverted = [ch for ch, c in cfg.items() if c["invert"]]
    if inverted:
        report.add("反向通道 (0°↔180°)", True,
                   ", ".join(f"CH{ch}" for ch in sorted(inverted)))
    return cfg


def check_table(dev, report, ch, cfg):
    out = dev.send(f"d {ch}", wait=2.0)
    rows = re.findall(r"\[TABLE\]\s+([\d.]+)\s+(\d+)\s+(\d+)", out)
    if not rows:
        report.add(f"角度對應表 (d {ch})", False, "無回應")
        return
    if ch not in cfg:
        report.add(f"角度對應表 (d {ch})", False, "l 沒有此通道")
        return

    c = cfg[ch]
    zero, span = c["zero"], c["max"] - c["zero"]
    # 工作角度是「實體」角度；反向時邏輯範圍要鏡射
    if c["invert"]:
        lo, hi = 180 - c["maxDeg"], 180 - c["minDeg"]
    else:
        lo, hi = c["minDeg"], c["maxDeg"]
    bad = []
    for deg_s, us_s, cnt_s in rows:
        deg, us, cnt = float(deg_s), int(us_s), int(cnt_s)
        phys = (180.0 - deg) if c["invert"] else deg
        phys = min(max(phys, c["minDeg"]), c["maxDeg"])
        exp_us = int(zero + span * phys / 180.0 + 0.5)
        exp_cnt = us_to_count(exp_us)
        if us != exp_us or cnt != exp_cnt:
            bad.append(f"{deg:g}° 期望 {exp_us}us/{exp_cnt}，實得 {us}us/{cnt}")
    degs = [float(r[0]) for r in rows]
    if degs[0] < lo - 0.01 or degs[-1] > hi + 0.01:
        bad.append(f"表格 {degs[0]:g}~{degs[-1]:g}° 超出邏輯範圍 {lo}~{hi}°")

    report.add(
        f"角度對應表 (d {ch}, 0°={zero}us, 180°={c['max']}us, "
        f"實體工作 {c['minDeg']}~{c['maxDeg']}°, "
        f"{'反向' if c['invert'] else '正向'})",
        not bad, f"{len(rows)} 點全部正確" if not bad else bad[0])


def check_verify(dev, report, ch, attempts=3):
    for run in range(attempts):
        out = dev.send(f"v {ch}", wait=3.0)
        rows = re.findall(
            r"\[VERIFY\]\s+([\d.]+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(PASS|FAIL)",
            out)
        if not rows:
            report.add(f"寫入→讀回 (v {ch})", False, "無回應")
            return
        result = re.search(
            r"\[VERIFY\] RESULT (PASS|FAIL) (\d+)/(\d+), 單調(遞增|遞減)=(YES|NO), "
            r"I2C 錯誤 \+(\d+)", out)
        bad = [r[0] for r in rows if r[5] == "FAIL"]
        detail = ""
        if result:
            detail = (f"{result.group(2)}/{result.group(3)}, "
                      f"單調{result.group(4)}={result.group(5)}, "
                      f"I2C 錯誤 {result.group(6)}")
        report.add(f"寫入→讀回 (v {ch}, 第 {run + 1} 次)",
                   bool(result) and result.group(1) == "PASS",
                   detail or "解析失敗")
        if bad:
            report.add(f"失敗角度 (第 {run + 1} 次)", False,
                       ", ".join(f"{b}°" for b in bad))
        time.sleep(0.3)


def visual(dev, ch, hold, step=15):
    cfg, _ = fetch_config(dev)
    if ch not in cfg:
        print("讀不到通道設定，先確認 PCA9685 連線")
        return
    c = cfg[ch]
    start, end = c["minDeg"], c["maxDeg"]
    print(f"\n=== 逐點定位 CH{ch}（0°={c['zero']}us, 180°={c['max']}us, "
          f"{start}~{end}°, {'反向' if c['invert'] else '正向'}，每點 {hold}s）===")
    print(f"{'角度':>6}  {'實測脈寬':>10}")
    for deg in range(start, end + 1, step):
        dev.send(f"a {ch} {deg}", wait=0.1)
        time.sleep(min(hold, 0.4))
        out = dev.send(f"p {ch}", wait=0.5)
        m = re.search(r"→\s*(\d+)\s*us", out)
        us = m.group(1) if m else "?"
        print(f"{deg:>5}°  {us:>7}us", end="\r" if deg + step <= end else "\n")
        time.sleep(max(0.0, hold - 0.4))
    print()
    dev.send(f"o {ch}", wait=0.3)


def sweep(dev, ch, seconds, step, interval_ms):
    print(f"\n=== 連續掃描 CH{ch}（{seconds}s，每步 {step}°，{interval_ms}ms）===")
    dev.send(f"w {ch} {step} {interval_ms}", wait=0.5)
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            time.sleep(1.0)
            print(f"  已掃描 {time.time() - t0:.0f}s…", end="\r")
    finally:
        dev.send("q", wait=0.5)
        print("\n  停止")


def calibrate(dev, ch, report):
    print(f"\n=== 端點校正 CH{ch} ===")
    print("提示：先下 c <ch> <us> 慢慢逼近，找到不會嗡嗡叫的安全值。")
    print("（固件命令，例如 c 0 650；本模式只負責把值寫進該通道）")
    try:
        min_us = int(input("最小脈寬 minUs [600]: ") or "600")
        max_us = int(input("最大脈寬 maxUs [2400]: ") or "2400")
    except (EOFError, ValueError):
        print("取消校正")
        return

    out = dev.send(f"ma {ch} {min_us} {max_us}", wait=0.8)
    print(out.strip())
    ok = "端點 =" in out
    report.add(f"設定 CH{ch} 端點 (ma)", ok, f"0°={min_us}us, 180°={max_us}us")

    # 到 0° 與 180° 讓使用者確認
    for deg in (0, 180):
        dev.send(f"a {ch} {deg}", wait=0.1)
        print(f"  CH{ch} → {deg}°（{angle_to_us(deg, min_us, max_us)}us），請確認")
        time.sleep(1.0)
    dev.send(f"o {ch}", wait=0.3)


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="PCA9685 角度對應測試 (0~180°)")
    ap.add_argument("-p", "--port", help="序列埠（預設自動偵測）")
    ap.add_argument("--upload", action="store_true", help="先燒錄 pca9685_angle")
    ap.add_argument("--ch", type=int, default=0, help="測試通道（預設 0）")
    ap.add_argument("--limits", action="store_true", help="列出各通道端點設定")
    ap.add_argument("--visual", type=int, metavar="CH", help="逐點定位可視化")
    ap.add_argument("--hold", type=float, default=1.0, help="可視化每點停留秒數")
    ap.add_argument("--sweep", type=int, metavar="CH", help="連續掃描可視化")
    ap.add_argument("--seconds", type=float, default=8.0, help="掃描秒數")
    ap.add_argument("--step", type=float, default=2.0, help="掃描每步角度")
    ap.add_argument("--interval", type=int, default=20, help="掃描間隔 ms")
    ap.add_argument("--calibrate", type=int, metavar="CH", help="端點校正")
    ap.add_argument("--runs", type=int, default=3, help="寫入→讀回重複次數")
    ap.add_argument("--ch-min", type=int, default=None, help="(保留) 未使用")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        print("找不到序列埠（-p 指定）")
        return 2
    print(f"序列埠：{port}")

    if args.upload:
        port = upload(port) or port

    try:
        dev = Device(port)
    except Exception as exc:  # noqa: BLE001
        print(f"無法開啟序列埠：{exc}")
        return 2

    try:
        # 先確認裝置在線
        out = dev.send("s", wait=2.0)
        if "0x" not in out:
            print("I2C 掃不到 PCA9685，先檢查接線：")
            print(out.strip())
            return 2
        print(dev.send("i", wait=1.5).strip())

        if args.limits:
            print(dev.send("l", wait=1.5).strip())
            return 0

        if args.calibrate is not None:
            report = Report()
            calibrate(dev, args.calibrate, report)
            return report.summary()

        if args.visual is not None:
            visual(dev, args.visual, args.hold)
            return 0

        if args.sweep is not None:
            sweep(dev, args.sweep, args.seconds, args.step, args.interval)
            return 0

        report = Report()
        print("\n===== 開始角度測試 =====")
        cfg = check_config(dev, report)
        check_table(dev, report, args.ch, cfg)
        check_verify(dev, report, args.ch, args.runs)
        dev.send("x", wait=0.5)
        return report.summary()
    finally:
        dev.close()


if __name__ == "__main__":
    sys.exit(main())
