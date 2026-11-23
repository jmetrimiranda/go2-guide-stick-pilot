#!/usr/bin/env python3
"""Passive readout of the Go2 state for the pilot (T3): battery SOC, power, posture.

READ-ONLY: it only subscribes to rt/lowstate and rt/sportmodestate. It never creates a
SportClient and never publishes anything.

Usage (inside the pilot container, e.g. the same docker run as tools/pilot_in_container.sh):
    python3 tools/probe_state.py --iface eth0 [--seconds 5]
"""
import argparse
import statistics
import threading
import time

SOC_MIN = 30   # % - below this the session stops (Jorge, 2026-11-23)


def stats(xs):
    return f"mean {statistics.fmean(xs):.2f}  min {min(xs):.2f}  max {max(xs):.2f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--seconds", type=float, default=5.0)
    a = ap.parse_args()

    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_

    ChannelFactoryInitialize(0, a.iface)
    lock = threading.Lock()
    low, sport, last = [], [], {}

    def on_low(m):
        mech = sum(abs(ms.tau_est * ms.dq) for ms in m.motor_state[:12])
        with lock:
            low.append((m.power_v, m.power_a, m.power_v * m.power_a, m.bms_state.current / 1000.0, mech))
            last["low"] = m

    def on_sport(m):
        with lock:
            sport.append(m.yaw_speed)
            last["sport"] = m

    s1 = ChannelSubscriber("rt/lowstate", LowState_)
    s1.Init(on_low, 10)
    s2 = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    s2.Init(on_sport, 10)
    time.sleep(a.seconds)
    with lock:
        low_rows, sport_rows = list(low), list(sport)
        lm, sm = last.get("low"), last.get("sport")

    print(f"window {a.seconds:.0f} s   lowstate {len(low_rows) / a.seconds:.0f} Hz   "
          f"sportmodestate {len(sport_rows) / a.seconds:.0f} Hz")
    if lm is None:
        print("!! no lowstate: check --iface")
        return
    b = lm.bms_state
    cells = [v for v in b.cell_vol if v > 0]
    print(f"battery: SOC {b.soc} %   cells {len(cells)} x {min(cells)}-{max(cells)} mV "
          f"(sum {sum(cells) / 1000:.2f} V)   cycles {b.cycle}   bms status {b.status}")
    if b.soc < SOC_MIN:
        print(f"!! SOC below {SOC_MIN} %: stop the session and charge")
    pv, pa, pw, bi, mech = zip(*low_rows)
    print(f"power_v [V]         {stats(pv)}")
    print(f"power_a [A]         {stats(pa)}")
    print(f"power_v*power_a [W] {stats(pw)}")
    print(f"bms current [A]     {stats(bi)}   -> |I| x sum(cells) ~ "
          f"{abs(statistics.fmean(bi)) * sum(cells) / 1000:.1f} W")
    print(f"mech power sum|tau*dq| [W] {stats(mech)}")
    print(f"foot_force (low): {list(lm.foot_force)}   motor temps: "
          f"{[ms.temperature for ms in lm.motor_state[:12]]}")
    if sm is not None:
        print(f"sport: mode {sm.mode}  gait_type {sm.gait_type}  body_height {sm.body_height:.3f} m  "
              f"rpy {[round(x, 3) for x in sm.imu_state.rpy]}  yaw_speed {stats(sport_rows)}")


if __name__ == "__main__":
    main()
