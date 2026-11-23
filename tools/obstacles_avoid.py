#!/usr/bin/env python3
"""Get or set the Go2's built-in obstacle avoidance switch.

Uses the official ObstaclesAvoidClient of unitree_sdk2_python (service "obstacles_avoid",
api 1001 SwitchSet {"enable": bool}, api 1002 SwitchGet), as in the SDK's
example/obstacles_avoid/obstacles_avoid_switch.py. It sends NO motion command.

Usage (inside the pilot container):
    python3 tools/obstacles_avoid.py --iface eth0 get
    python3 tools/obstacles_avoid.py --iface eth0 off    # before the pilot
    python3 tools/obstacles_avoid.py --iface eth0 on     # undo, after the pilot
"""
import argparse
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("action", choices=["get", "off", "on"])
    a = ap.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient

    ChannelFactoryInitialize(0, a.iface)
    c = ObstaclesAvoidClient()
    c.SetTimeout(3.0)
    c.Init()

    code, before = c.SwitchGet()
    print(f"before: SwitchGet code {code}, enable {before}")
    if code != 0:
        sys.exit("!! SwitchGet failed: nothing was changed")
    if a.action == "get":
        return
    want = a.action == "on"
    code = c.SwitchSet(want)
    print(f"SwitchSet({want}) code {code}")
    time.sleep(0.5)
    code2, after = c.SwitchGet()
    print(f"after:  SwitchGet code {code2}, enable {after}")
    if code != 0 or code2 != 0 or after is not want:
        sys.exit(f"!! the switch did not end at enable={want}")


if __name__ == "__main__":
    main()
