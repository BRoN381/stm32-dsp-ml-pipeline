#!/usr/bin/env python3
"""
Collect labeled gesture data from the STM32 over the UART/serial port.

The device must be built with STREAM_TRAINING_DATA = 1 (see Core/Src/main.c),
which streams filtered 6-axis samples as CSV at 50 Hz:

    ax,ay,az,gx,gy,gz

Usage (collect one label per session):

    python collect_data.py --port COM5 --label circle --out Model/dataset/data.csv

Workflow:
    - Press ENTER to START recording one gesture.
    - Perform the gesture (any duration).
    - Press ENTER to STOP. The samples in between are saved as one "take".
    - Repeat. Ctrl-C to quit.

Output CSV columns:  take_id,label,t,ax,ay,az,gx,gy,gz
(one take = one gesture instance; rows of a take share take_id, t = 0..n-1)
"""
import argparse
import csv
import os
import sys
import threading

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial not installed.  Run:  pip install pyserial")

EXPECTED_COLS = 6  # ax,ay,az,gx,gy,gz


class SerialReader(threading.Thread):
    """Continuously drains the serial port; buffers samples while recording."""

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.recording = threading.Event()
        self.buffer = []
        self._lock = threading.Lock()
        self._stop = threading.Event()

    def start_take(self):
        with self._lock:
            self.buffer = []
        self.ser.reset_input_buffer()   # drop stale bytes before the gesture
        self.recording.set()

    def stop_take(self):
        self.recording.clear()
        with self._lock:
            return list(self.buffer)

    def run(self):
        while not self._stop.is_set():
            try:
                line = self.ser.readline().decode("ascii", errors="ignore").strip()
            except serial.SerialException:
                break
            if not line or not self.recording.is_set():
                continue
            parts = line.split(",")
            if len(parts) != EXPECTED_COLS:
                continue                # skip banners / partial lines
            try:
                vals = [int(p) for p in parts]
            except ValueError:
                continue
            with self._lock:
                self.buffer.append(vals)

    def stop(self):
        self._stop.set()


def next_take_id(path):
    """Continue take_id numbering if the file already has data."""
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return 0
    last = -1
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for row in reader:
            if row:
                try:
                    last = max(last, int(row[0]))
                except (ValueError, IndexError):
                    pass
    return last + 1


def main():
    ap = argparse.ArgumentParser(description="Collect labeled gesture data over serial.")
    ap.add_argument("--port", required=True, help="serial port, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("--label", required=True, help="gesture label for this session")
    ap.add_argument("--out", default="Model/dataset/data.csv", help="output CSV path")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate", type=int, default=50, help="device stream rate (Hz), for duration display")
    ap.add_argument("--raw", action="store_true",
                    help="just echo raw lines from the device (diagnostic), then exit")
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    take_id = next_take_id(args.out)
    new_file = not os.path.exists(args.out) or os.path.getsize(args.out) == 0

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"Could not open {args.port}: {e}")

    if args.raw:
        print(f"Connected ({args.port}) @ {args.baud}.  Echoing raw lines (Ctrl-C to stop):\n")
        try:
            while True:
                line = ser.readline().decode("ascii", errors="ignore").strip()
                if line:
                    print(repr(line))
        except KeyboardInterrupt:
            print("\nDone.")
        finally:
            ser.close()
        return

    reader = SerialReader(ser)
    reader.start()

    out = open(args.out, "a", newline="")
    writer = csv.writer(out)
    if new_file:
        writer.writerow(["take_id", "label", "t", "ax", "ay", "az", "gx", "gy", "gz"])

    print(f"Connected ({args.port}).  Label = '{args.label}'.  Writing to {args.out}")
    print("Press ENTER to START a take, ENTER again to STOP.  Ctrl-C to quit.")
    try:
        while True:
            input(f"\n[take {take_id}] ENTER to START >>> ")
            reader.start_take()
            print("  RECORDING... perform the gesture, then press ENTER to STOP.")
            input()
            samples = reader.stop_take()
            n = len(samples)
            if n == 0:
                print("  (no samples captured — is STREAM_TRAINING_DATA=1 and the device running?)")
                continue
            for t, vals in enumerate(samples):
                writer.writerow([take_id, args.label, t] + vals)
            out.flush()
            print(f"  saved take #{take_id}: {n} samples ({n / args.rate:.2f}s)")
            take_id += 1
    except KeyboardInterrupt:
        print("\nDone.")
    finally:
        reader.stop()
        out.close()
        ser.close()


if __name__ == "__main__":
    main()
