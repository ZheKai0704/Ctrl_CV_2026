#!/usr/bin/env python3
"""機構動作測試自動驗證（搭配 mechanism_test 固件）

固件行為：HOME(30°) → 逐顆 CH0→CH15 → 每顆 OUT(120°) 停 1 秒 → 回 HOME(30°)

本腳本只做「驗證」，不改變機構邏輯：
  - 讀取 `l` / `p` 確認校正與啟用狀態
  - 執行 `s`（單次）或 `g <n>`（單組），收集 `[SEQ]` 輸出
  - 驗證順序、角度、停留時間是否符合預期

⚠️ 預設「不動作」；要真的驅動機構必須加 --run（確認機構安全後再用）。

用法：
  # 只看設定與狀態（不動）
  ~/.platformio/penv/bin/python tools/mechanism_test.py

  # 真正跑一次完整循環並驗證
  ~/.platformio/penv/bin/python tools/mechanism_test.py --run

  # 只測第 0 組（CH0/CH1）
  ~/.platformio/penv/bin/python tools/mechanism_test.py --run --group 0

  # 改角度與時間後再跑
  ~/.platformio/penv/bin/python tools/mechanism_test.py --run --home 30 --out 120 --dwell 1000 --settle 500

  # 先燒錄再測
  ~/.platformio/penv/bin/python tools/mechanism_test.py --upload --run

離開碼：0 = PASS，1 = FAIL，2 = 連線/環境問題
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
    sys.exit("缺少 pyserial：請用 ~/.platformio/penv/bin/python 執行")

ENV = "mechanism_test"
BAUD = 115200


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
    return find_port() or port


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

    def send(self, cmd, wait=1.0):
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode())
        self.ser.flush()
        buf = bytearray()
        deadline = time.time() + wait
        while time.time() < deadline:
            chunk = self.ser.read(4096)
            if chunk:
                buf.extend(chunk)
                deadline = time.time() + 0.2
        return buf.decode("utf-8", "replace")

    def collect(self, duration):
        """連續讀取，回傳 [(相對秒數, 行)]。"""
        out = []
        t0 = time.time()
        buf = bytearray()
        while time.time() - t0 < duration:
            chunk = self.ser.read(4096)
            if not chunk:
                continue
            buf.extend(chunk)
            while b"\n" in buf:
                raw, _, rest = buf.partition(b"\n")
                buf = bytearray(rest)
                line = raw.decode("utf-8", "replace").strip()
                if line:
                    out.append((time.time() - t0, line))
        return out


def parse_config(dev):
    out = dev.send("l", wait=1.5)
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


def parse_status(dev):
    out = dev.send("p", wait=1.0)
    home = re.search(r"home=([\d.]+)°", out)
    outd = re.search(r"out=([\d.]+)°", out)
    dwell = re.search(r"dwell=(\d+)ms", out)
    settle = re.search(r"settle=(\d+)ms", out)
    return {
        "home": float(home.group(1)) if home else None,
        "out": float(outd.group(1)) if outd else None,
        "dwell": int(dwell.group(1)) if dwell else None,
        "settle": int(settle.group(1)) if settle else None,
        "raw": out,
    }


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


def expected_steps(cfg, group):
    """回傳預期的通道順序（已啟用且符合群組）。"""
    if group is not None:
        cand = [group * 2, group * 2 + 1]
    else:
        cand = list(range(16))
    return [ch for ch in cand if cfg.get(ch, {}).get("enabled")]


def analyse(lines, steps, home, out, dwell_ms, settle_ms, tol_ms=400):
    """回傳 (events, problems)。events = [(t, ch, angle)]"""
    events = []
    problems = []
    pat = re.compile(r"\[SEQ\] CH(\d+) → ([\d.]+)°")
    for t, line in lines:
        m = pat.search(line)
        if m:
            events.append((t, int(m.group(1)), float(m.group(2))))

    expected = []
    for ch in steps:
        expected.append((ch, out))
        expected.append((ch, home))

    if len(events) != len(expected):
        problems.append(f"事件數 {len(events)}，預期 {len(expected)}")
    else:
        for i, ((t, ch, deg), (ech, edeg)) in enumerate(zip(events, expected)):
            if ch != ech or abs(deg - edeg) > 0.6:
                problems.append(
                    f"第 {i + 1} 個動作 CH{ch} {deg}°，預期 CH{ech} {edeg}°")

    # 時間驗證
    for i in range(0, len(events) - 1, 2):  # out -> home
        if i + 1 < len(events):
            dt = (events[i + 1][0] - events[i][0]) * 1000
            if abs(dt - dwell_ms) > tol_ms:
                problems.append(
                    f"CH{events[i][1]} 停留 {dt:.0f}ms，預期 {dwell_ms}ms")
    for i in range(1, len(events) - 1, 2):  # home -> next out
        dt = (events[i + 1][0] - events[i][0]) * 1000
        if abs(dt - settle_ms) > tol_ms:
            problems.append(
                f"CH{events[i][1]}→CH{events[i + 1][1]} 間隔 {dt:.0f}ms，"
                f"預期 {settle_ms}ms")
    return events, problems


def main():
    ap = argparse.ArgumentParser(description="機構動作測試驗證")
    ap.add_argument("-p", "--port", help="序列埠（預設自動偵測）")
    ap.add_argument("--upload", action="store_true", help="先燒錄 mechanism_test")
    ap.add_argument("--run", action="store_true", help="實際驅動機構（預設只讀狀態）")
    ap.add_argument("--group", type=int, default=None, help="只測第 n 組 (0~7)")
    ap.add_argument("--home", type=float, default=None, help="HOME 角度")
    ap.add_argument("--out", type=float, default=None, help="OUT 角度")
    ap.add_argument("--dwell", type=int, default=None, help="OUT 停留 ms")
    ap.add_argument("--settle", type=int, default=None, help="回位等待 ms")
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

    report = Report()
    try:
        if args.home is not None and args.out is not None:
            print(dev.send(f"m {args.home:g} {args.out:g}", wait=1.2).strip())
        if args.dwell is not None and args.settle is not None:
            print(dev.send(f"t {args.dwell} {args.settle}", wait=0.8).strip())

        cfg, raw_l = parse_config(dev)
        report.add("讀取通道設定 (l)", len(cfg) == 16, f"{len(cfg)}/16")
        if not cfg:
            print(raw_l.strip())
            return 2

        status = parse_status(dev)
        print("\n=== 目前設定 ===")
        print(status["raw"].strip())
        home = status["home"]
        out = status["out"]
        dwell = status["dwell"]
        settle = status["settle"]
        report.add("讀取狀態 (p)", None not in (home, out, dwell, settle),
                   f"home={home}° out={out}° dwell={dwell}ms settle={settle}ms")
        if None in (home, out, dwell, settle):
            return 2

        steps = expected_steps(cfg, args.group)
        report.add("預期通道順序", len(steps) > 0,
                   " → ".join(f"CH{ch}" for ch in steps) if steps else "沒有啟用通道")

        if not args.run:
            print("\n（未加 --run，只讀設定，不動機構）")
            print("確認機構安全後執行：")
            print("  ~/.platformio/penv/bin/python tools/mechanism_test.py --run")
            return report.summary()

        if not steps:
            return 2

        if args.group is not None:
            cmd = f"g {args.group}"
        else:
            cmd = "s"

        # 預估總時間：每顆 dwell + settle，加上通訊餘量
        total_s = len(steps) * (dwell + settle) / 1000.0 + 5.0
        print(f"\n=== 執行 {cmd}（預估 {total_s:.0f}s）===")
        dev.ser.reset_input_buffer()
        dev.ser.write((cmd + "\n").encode())
        dev.ser.flush()
        lines = dev.collect(total_s)

        events, problems = analyse(lines, steps, home, out, dwell, settle)
        report.add("動作事件收集", len(events) > 0, f"{len(events)} 個動作")
        report.add("順序與角度", not problems,
                   "全部正確" if not problems else problems[0])
        report.add("停留/間隔時間", not any("預期" in p and "ms" in p
                                          for p in problems),
                   f"dwell={dwell}ms settle={settle}ms")

        if events:
            print("\n=== 實際動作 ===")
            t0 = events[0][0]
            for t, ch, deg in events:
                print(f"  +{(t - t0) * 1000:7.0f} ms  CH{ch:<2} → {deg:5.1f}°")

        # 收尾：確認回到 HOME
        status2 = parse_status(dev)
        report.add("結束後回到閒置", "閒置" in status2["raw"],
                   status2["raw"].splitlines()[-1] if status2["raw"] else "")
        return report.summary()
    finally:
        dev.close()


if __name__ == "__main__":
    sys.exit(main())
