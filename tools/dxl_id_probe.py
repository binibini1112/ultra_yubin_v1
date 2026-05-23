#!/usr/bin/env python3
"""Probe Dynamixel IDs through the Ultra96 UDP bridge."""

import argparse
import socket
import sys


def request(host, port, cmd, timeout):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        sock.sendto((cmd.rstrip() + "\n").encode("ascii"), (host, port))
        data, _ = sock.recvfrom(2048)
        return data.decode("ascii", errors="replace").strip()
    finally:
        sock.close()


def parse_fields(reply):
    fields = {}
    for item in reply.split(","):
        if "=" in item:
            key, value = item.split("=", 1)
            fields[key.strip()] = value.strip()
    return fields


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.3.1")
    parser.add_argument("--port", type=int, default=5016)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--start", type=int, default=1)
    parser.add_argument("--end", type=int, default=10)
    args = parser.parse_args()

    print("Checking bridge...")
    try:
        print(request(args.host, args.port, "PLPING", args.timeout))
    except Exception as exc:
        print(f"PLPING failed: {exc}", file=sys.stderr)
        return 1

    print("Probing IDs with DREL <id> 0. read=1 means the ID responded.")
    found = []
    for dxl_id in range(args.start, args.end + 1):
        try:
            reply = request(args.host, args.port, f"DREL {dxl_id} 0", args.timeout)
        except Exception as exc:
            print(f"id={dxl_id}: timeout/error {exc}")
            continue
        fields = parse_fields(reply)
        read_ok = fields.get("read") == "1"
        config_ok = fields.get("config") == "1"
        usb_ok = fields.get("usb") == "1"
        present = fields.get("present", "-")
        goal = fields.get("goal", "-")
        marker = "FOUND" if read_ok else "----"
        print(f"{marker} id={dxl_id:2d} read={int(read_ok)} config={int(config_ok)} usb={int(usb_ok)} present={present} goal={goal} reply={reply}")
        if read_ok:
            found.append(dxl_id)
    print(f"found_ids={found}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
