#!/usr/bin/env python3
"""Capture the wing's real-time CSV logger (checklist I3) to a file.

The firmware prints the same CSV line to USB serial AND to UDP port 5005
(one subscriber at a time — it streams to whoever last sent it a datagram).

Serial (bench, cable attached):

    python3 capture.py --port /dev/tty.usbserial-0001 -o bench_T0.csv

UDP (WiFi AP of the aircraft, no cable — HELLO re-sent every 5 s):

    python3 capture.py --udp 192.168.4.1:5005 -o flight_T2.csv

Then plot it:

    python3 plot.py bench_T0.csv --out bench_T0.png

Schema reference: esp32-airplane/docs/test-campaign.md §1.
"""
import argparse
import socket
import sys
import time

HEADER_PREFIX = "t_ms,"


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port, e.g. /dev/tty.usbserial-0001")
    src.add_argument("--udp", metavar="HOST:PORT",
                     help="firmware UDP logger, e.g. 192.168.4.1:5005")
    p.add_argument("-o", "--out", required=True, help="output CSV file")
    p.add_argument("--baud", type=int, default=115200,
                   help="serial baud (firmware default 115200)")
    p.add_argument("--hello", type=float, default=5.0,
                   help="UDP re-subscribe interval, seconds")
    p.add_argument("--duration", type=float, default=0,
                   help="stop after N seconds (0 = until Ctrl-C)")
    return p.parse_args()


class Source:
    """Yields logger lines; owns stats about line pacing."""

    def __init__(self, args):
        self.deltas = []          # t_ms deltas seen since last report
        self.last_t = None
        if args.port:
            try:
                import serial            # pyserial
            except ImportError:
                sys.exit("pyserial missing: pip3 install pyserial")
            self.ser = serial.Serial(args.port, args.baud, timeout=0.5)
            self.sock = None
            self.mode = f"serial {args.port}@{args.baud}"
        else:
            host, port = args.udp.rsplit(":", 1)
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(("", 0))
            self.sock.settimeout(0.5)
            self.server = (host, int(port))
            self.hello_period = args.hello
            self.next_hello = 0.0
            self.ser = None
            self.mode = f"udp {args.udp}"

    def lines(self):
        if self.ser:
            raw = self.ser.readline()
            return raw.decode("ascii", "replace").splitlines() if raw else []
        now = time.time()
        if now >= self.next_hello:               # keep our subscription fresh
            self.sock.sendto(b"HELLO", self.server)
            self.next_hello = now + self.hello_period
        try:
            data, _ = self.sock.recvfrom(4096)
        except socket.timeout:
            return []
        return data.decode("ascii", "replace").splitlines()

    def note(self, t_ms):
        if self.last_t is not None:
            self.deltas.append(t_ms - self.last_t)
        self.last_t = t_ms


def main():
    args = parse_args()
    src = Source(args)
    print(f"[capture] source: {src.mode} -> {args.out}")
    print("[capture] waiting for the CSV header line (t_ms,...) — "
          "power-cycle the board if it never comes…")

    out = open(args.out, "w", newline="")
    n_lines = 0
    header_ok = False
    gaps = 0
    t0 = time.time()
    last_report = t0
    try:
        while True:
            if args.duration and time.time() - t0 >= args.duration:
                break
            for line in src.lines():
                line = line.strip()
                if not line:
                    continue
                if not header_ok:
                    if line.startswith(HEADER_PREFIX):
                        out.write(line + "\n")
                        header_ok = True
                        print(f"[capture] header locked: {len(line.split(','))} "
                              "columns")
                    continue          # skip boot log ([cfg], init:, …)
                if not line[0].isdigit():
                    continue          # stray non-CSV output
                src.note(int(line.split(",", 1)[0]))
                out.write(line + "\n")
                n_lines += 1

            if header_ok and time.time() - last_report >= 5.0:
                d = src.deltas
                if d:
                    d.sort()
                    med = d[len(d) // 2]
                    gaps += sum(1 for x in d if x > 3 * max(med, 1))
                    rate = 1000.0 / med if med else 0
                    print(f"[capture] {n_lines} lines, {rate:.1f} Hz, "
                          f"{gaps} gaps (>3x period)")
                    src.deltas.clear()
                last_report = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        out.close()
        dur = time.time() - t0
        print(f"[capture] done: {n_lines} lines in {dur:.1f} s "
              f"({n_lines / dur if dur else 0:.1f} lines/s) -> {args.out}")
        if not header_ok:
            print("[capture] WARNING: no CSV header captured — empty/incomplete file")


if __name__ == "__main__":
    main()
