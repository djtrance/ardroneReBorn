#!/usr/bin/env python
"""
ardrone2_helper.py — Deploy and manage code on AR.Drone 2.0

Usage:
  python ardrone2_helper.py status
  python ardrone2_helper.py upload_binary ../build/bin/drone_encoder
  python ardrone2_helper.py upload_gst_module ../build/bin/libgstparrot_enc.so
  python ardrone2_helper.py startvision
  python ardrone2_helper.py reboot
"""

from __future__ import print_function
import re
import argparse
import socket
import telnetlib
from time import sleep
from ftplib import FTP

DRONE_IP = "192.168.1.1"
DRONE_PATH = "/data/video/"
GST_PLUGIN_PATH = "/data/video/opt/arm/gst/lib/gstreamer-0.10/"


def is_ip(address):
    try:
        socket.inet_aton(address)
        return True
    except socket.error:
        return False


def execute_command(tn, command):
    tn.write(command + '\n')
    return tn.read_until('# ')[len(command) + 2:-4]


def upload_file(host, local_path, remote_name=None):
    """Upload a file to the drone via FTP."""
    if remote_name is None:
        remote_name = local_path.split('/')[-1]

    try:
        ftp = FTP(host)
        ftp.login()
        with open(local_path, 'rb') as f:
            ftp.storbinary("STOR " + remote_name, f)
        ftp.quit()
        print("Uploaded {} as {}".format(local_path, remote_name))
        return True
    except Exception as e:
        print("FTP upload failed: {}".format(e))
        return False


def check_vision_installed(tn):
    result = execute_command(tn, 'ls /data/video/opt/arm/gst/bin 2>/dev/null | head -1')
    return bool(result.strip())


def check_vision_running(tn):
    result = execute_command(tn, 'ls /opt/arm/gst/bin 2>/dev/null | head -1')
    return bool(result.strip())


def start_vision(tn):
    """Start the DSP/GStreamer vision framework on the drone."""
    print("Starting vision framework...")

    execute_command(tn, "killall -9 program.elf 2>/dev/null")
    execute_command(tn, "killall -9 gst-launch-0.10 2>/dev/null")

    execute_command(tn, "mkdir -p /opt/arm")
    execute_command(tn, "mkdir -p /lib/dsp")
    execute_command(tn, "mount --bind /data/video/opt/arm /opt/arm")
    execute_command(tn, "mount --bind /data/video/opt/arm/lib/dsp /lib/dsp")

    execute_command(tn, "export PATH=/opt/arm/gst/bin:$PATH")
    execute_command(tn, "export DSP_PATH=/opt/arm/tidsp-binaries-23.i3.8/")

    execute_command(tn, "/bin/dspbridge/cexec.out -T /opt/arm/tidsp-binaries-23.i3.8/baseimage.dof -v 2>/dev/null")
    execute_command(tn, "/bin/dspbridge/dynreg.out -r /opt/arm/tidsp-binaries-23.i3.8/m4venc_sn.dll64P -v 2>/dev/null")

    print("Vision framework started.")


def drone_status(host):
    """Print status information from the drone."""
    try:
        tn = telnetlib.Telnet(host)
        tn.read_until('# ')
    except Exception as e:
        print("Could not connect to {}: {}".format(host, e))
        return

    version = execute_command(tn, 'cat /firmware/version.txt 2>/dev/null || echo "unknown"')
    running = execute_command(tn, 'ps 2>/dev/null')

    print("=== AR.Drone 2.0 Status ===")
    print("Host:               {}".format(host))
    print("Firmware:           {}".format(version.strip()))
    print()

    if 'program.elf' in running:
        print("Native program:     RUNNING")
    else:
        print("Native program:     STOPPED (ok to deploy)")

    if 'gst-launch' in running:
        print("GStreamer:          RUNNING")

    if check_vision_installed(tn):
        print("Vision framework:   INSTALLED")
    else:
        print("Vision framework:   NOT INSTALLED")

    if check_vision_running(tn):
        print("Vision framework:   RUNNING")
    else:
        print("Vision framework:   NOT RUNNING")

    print()
    filesystem = execute_command(tn, 'df -h /data 2>/dev/null')
    print("Filesystem (/data):")
    print(filesystem.strip())

    tn.close()


def upload_binary(host, local_path):
    """Upload and prepare a binary for execution."""
    filename = local_path.split('/')[-1]
    remote = DRONE_PATH + filename

    if not upload_file(host, local_path):
        return

    try:
        tn = telnetlib.Telnet(host)
        tn.read_until('# ')
        execute_command(tn, "killall -9 {} 2>/dev/null".format(filename))
        execute_command(tn, "chmod 777 {}".format(remote))
        execute_command(tn, "ls -la {}".format(remote))
        tn.close()
    except Exception as e:
        print("Telnet failed: {}".format(e))

    print()
    print("To run: telnet {} && {}".format(host, remote))


def upload_gst_module(host, local_path):
    """Upload a GStreamer plugin .so to the drone."""
    filename = local_path.split('/')[-1]
    if not filename.endswith('.so'):
        print("Warning: {} does not look like a .so file".format(filename))

    # Upload to temp location first
    if not upload_file(host, local_path, filename):
        return

    try:
        tn = telnetlib.Telnet(host)
        tn.read_until('# ')

        # Move to GStreamer plugin directory
        execute_command(tn, "mv /data/video/{} {} 2>/dev/null".format(filename, GST_PLUGIN_PATH))
        execute_command(tn, "chmod 777 {}{}".format(GST_PLUGIN_PATH, filename))

        # Check if vision framework is running
        if not check_vision_running(tn):
            print("Vision framework not running. Starting it...")
            start_vision(tn)

        execute_command(tn, "ls -la {}{}".format(GST_PLUGIN_PATH, filename))
        tn.close()
    except Exception as e:
        print("Telnet failed: {}".format(e))

    print()
    print("Plugin deployed. Use in pipeline:")
    print("  gst-launch-0.10 ... ! parrotenc ! ...")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='AR.Drone 2.0 deployment helper')
    parser.add_argument('--host', default=DRONE_IP, help='Drone IP address')
    parser.add_argument('command', nargs='+', help='Command or file to upload')

    args = parser.parse_args()
    cmd = args.command[0]

    if cmd == 'status':
        drone_status(args.host)

    elif cmd == 'reboot':
        tn = telnetlib.Telnet(args.host)
        tn.read_until('# ')
        execute_command(tn, 'reboot')
        tn.close()
        print("Drone rebooting...")

    elif cmd == 'startvision':
        try:
            tn = telnetlib.Telnet(args.host)
            tn.read_until('# ')
            start_vision(tn)
            tn.close()
        except Exception as e:
            print("Telnet failed: {}".format(e))

    elif cmd == 'upload_binary':
        if len(args.command) < 2:
            print("Usage: {} upload_binary <file>".format(__file__))
            exit(1)
        upload_binary(args.host, args.command[1])

    elif cmd == 'upload_gst_module':
        if len(args.command) < 2:
            print("Usage: {} upload_gst_module <file.so>".format(__file__))
            exit(1)
        upload_gst_module(args.host, args.command[1])

    else:
        print("Unknown command: {}".format(cmd))
        print("Available: status, reboot, startvision, upload_binary, upload_gst_module")
