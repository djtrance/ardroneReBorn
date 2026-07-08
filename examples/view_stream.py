#!/usr/bin/env python
"""
view_stream.py — View H.264 video stream from AR.Drone 2.0 on macOS.

Receives RTP/H.264 video from the drone's UDP port 5000 and
displays it using ffplay.

Usage:
  python view_stream.py [--host 192.168.1.1] [--port 5000]

Requirements:
  - ffmpeg/ffplay installed (brew install ffmpeg)
  - WiFi connected to drone network
"""

import argparse
import subprocess
import sys
import signal

def main():
    parser = argparse.ArgumentParser(description='View AR.Drone 2.0 video stream')
    parser.add_argument('--host', default='192.168.1.1',
                        help='Drone IP address')
    parser.add_argument('--port', type=int, default=5000,
                        help='Video UDP port')
    parser.add_argument('--output', '-o', default=None,
                        help='Save stream to file (optional)')
    args = parser.parse_args()

    source = "udp://@{}:{}".format(args.host, args.port)

    if args.output:
        cmd = [
            "ffmpeg",
            "-i", source,
            "-c", "copy",
            "-y", args.output
        ]
        print("Saving stream to {}...".format(args.output))
    else:
        cmd = [
            "ffplay",
            "-i", source,
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-framedrop",
            "-infbuf"
        ]
        print("Viewing stream from {}:{}".format(args.host, args.port))
        print("Press Ctrl+C to exit.")

    try:
        proc = subprocess.Popen(cmd)
        proc.wait()
    except KeyboardInterrupt:
        proc.terminate()
        print("\nExiting.")


if __name__ == '__main__':
    main()
