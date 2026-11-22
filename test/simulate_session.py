# Synthetic session in the exact file formats of go2_turn_pilot.py + Sensor Logger, to test the analysis.
#
#   python3 test/simulate_session.py OUT                 (default session: rigid + flexible, 5 reps each)
#   python3 test/simulate_session.py OUT --help          (optional scenario flags for test/test_analysis.py)
#
# With no flags the output is the same as the original simulator (same seed, same files, same values).
import argparse, csv, math, os, sys, json
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import go2_turn_pilot as P
from scipy.signal import lfilter

ap = argparse.ArgumentParser(description="Write a synthetic pilot session (robot CSVs + Sensor Logger export).")
ap.add_argument("out")
ap.add_argument("--reps", type=int, default=5, help="trials per profile and stick (default 5)")
ap.add_argument("--sticks", default="rigid,flexible", help="comma-separated stick blocks (default rigid,flexible)")
ap.add_argument("--seed", type=int, default=1, help="noise seed (default 1)")
ap.add_argument("--balanced", action="store_true",
                help="the pilot's own trial order (go2_turn_pilot.block_order, seed 7): within every 4 trials "
                     "step goes to one L and one R (the default is the old free shuffle, unbalanced)")
ap.add_argument("--equal-dynamics", action="store_true",
                help="no profile effect at all: the turn costs the same joint power for both profiles, so "
                     "any step-smooth difference in the metrics is analysis bias")
ap.add_argument("--gyro-sign", type=int, choices=(1, -1), default=1,
                help="sign of the logged robot yaw rate (sport.csv gz/yaw_speed, low.csv gz) vs the command")
ap.add_argument("--lr-asym", type=float, default=0.0, help="right turns get (1 + G) robot yaw gain")
ap.add_argument("--no-gravity", action="store_true", help="export without Gravity.csv")
ap.add_argument("--total-accel", action="store_true",
                help="Android-style export: TotalAcceleration.csv instead of Gravity.csv")
ap.add_argument("--g-units", action="store_true", help="accelerometer and gravity values in g")
ap.add_argument("--ios", action="store_true",
                help="iOS sign convention of Sensor Logger: acceleration and gravity negated, gyroscope unchanged")
ap.add_argument("--no-sync-end", action="store_true", help="no sync_end/ (the phone keeps recording 10 s)")
ap.add_argument("--phone-cut-end", action="store_true",
                help="the phone record stops 5 s after the last trial, before sync_end")
ap.add_argument("--gap", action="append", default=[], metavar="TRIAL:SECONDS",
                help="remove phone samples for SECONDS starting 0.3 s after the turn command of TRIAL")
ap.add_argument("--spurious-knocks", action="store_true",
                help="gait impacts on the phone while walking, handling bursts after each sync, "
                     "and 2 extra peaks in each robot sync")
ap.add_argument("--drop-phone-knock", action="store_true", help="the phone misses the 3rd knock of sync_start")
ap.add_argument("--aborted-row", action="store_true",
                help="append an 'aborted' and a 'skipped' row (empty times) to trials.csv")
ap.add_argument("--hand-sway", type=float, default=0.12, metavar="RAD_S",
                help="amplitude of the 1.8 Hz walking sway of the hand yaw rate (default 0.12)")
ap.add_argument("--gyro-bias", type=float, default=0.0, metavar="RAD_S",
                help="constant bias added to the logged robot yaw rate")
ap.add_argument("--still-quiet", action="store_true",
                help="no trot yaw oscillation while the robot stands still (still phases)")
ap.add_argument("--clock-step", default="", metavar="TRIAL:SECONDS",
                help="the robot wall clock steps by SECONDS just before TRIAL (later robot times and sync_end "
                     "shift, the pilot's 'clock step' note goes on TRIAL); also writes sync_*/meta.json")
ap.add_argument("--repeat-sync-start", type=float, default=0.0, metavar="SECONDS",
                help="the phone record also holds an earlier knock pattern SECONDS before sync_start "
                     "(a repeated sync attempt)")
args = ap.parse_args()

out = args.out; os.makedirs(out, exist_ok=True)
rng = np.random.default_rng(args.seed)
xrng = np.random.default_rng(args.seed + 1000)   # extra scenario noise; keeps rng's stream unchanged
T0 = 1795000000.0
OFF, DRIFT = 37.42, 2e-5            # phone = robot + OFF + DRIFT*(t-T0)
ph_t, ph_acc, ph_gyr = [], [], []    # phone signals in robot time, world-ish (lateral, fwd, up)
gaps = [(int(a), float(b)) for a, b in (g.split(":") for g in args.gap)]
step_k, step_s = args.clock_step.split(":") if args.clock_step else (math.inf, 0.0)

def clock(n):
    """Robot wall-clock error (s) while logging trial n (sync_end: n = inf)."""
    return float(step_s) if n >= float(step_k) else 0.0

def first_order(x, tau, fs):
    a = math.exp(-1 / (tau * fs)); return lfilter([1 - a], [1, -a], x)

def delay(x, d, fs):
    n = int(d * fs); return np.concatenate([np.zeros(n), x[:-n]]) if n else x

def pulses(t, times, heights, width):
    return sum(h * np.exp(-((t - k) / width) ** 2) for k, h in zip(times, heights)) if len(times) else 0 * t

def write(path, cols, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f); w.writerow(cols); w.writerows(rows)

def sync(tag, ts):
    d = os.path.join(out, f"sync_{tag}"); os.makedirs(d, exist_ok=True)
    t = np.arange(ts, ts + 20, 1 / 500)
    ks = ts + np.array([3, 3.45, 3.9, 6.5, 6.9])
    spike = sum(15 * np.exp(-((t - k) / 0.004) ** 2) for k in ks)
    if args.spurious_knocks:                       # robot bumped while the phone is taped on / removed
        spike = spike + pulses(t, ts + np.array([1.2, 13.7]), [6, 5], 0.004)
    sh = clock(math.inf if tag == "end" else 0)
    rows = [(ti + sh, 0.05 * rng.normal(), 0.05 * rng.normal(), 9.81 + s + 0.05 * rng.normal(), 0, 28.5, 2.0, 30)
            for ti, s in zip(t, spike)]
    write(os.path.join(d, "low.csv"), P.LOW_COLS, rows)
    write(os.path.join(d, "sport.csv"), P.SPORT_COLS, [])
    if args.clock_step:                            # time.time() - time.monotonic(), as the pilot saves it
        with open(os.path.join(d, "meta.json"), "w") as f:
            json.dump({"tag": tag, "wall_minus_monotonic_s": 1000.0 + sh, "wall_minus_monotonic_end_s": 1000.0 + sh}, f)
    tp = np.arange(ts, ts + 20, 0.01)
    pks = [k for i, k in enumerate(ks) if not (args.drop_phone_knock and tag == "start" and i == 2)]
    sp = sum(8 * np.exp(-((tp - k) / 0.008) ** 2) for k in pks)
    if args.spurious_knocks:                       # handling: phone taped to / removed from the handle
        hk = ts + 9 + np.sort(xrng.uniform(0, 10, 8))
        sp = sp + pulses(tp, hk, xrng.uniform(3, 9, len(hk)), 0.008)
    ph_t.append(tp); ph_acc.append(np.column_stack([0 * tp, 0 * tp, sp])); ph_gyr.append(np.zeros((len(tp), 3)))

def trial(n, stick, prof, d, ts):
    fs = 500; rows_cmd = []; t = ts; ev = {}
    for phase, dur in P.trial_schedule():
        ev[phase + "_start"] = t; tp = 0.0
        while True:
            if phase == "turn":
                if P.turn_rate(prof, tp)[1]: break
            elif tp >= dur: break
            vx, wz = P.command_at(prof, d, phase, tp); rows_cmd.append((t, phase, vx, wz))
            tp += 0.02; t += 0.02
        ev[phase + "_end"] = t
    te = t
    tt = np.arange(ts, te, 1 / fs)
    c = np.array(rows_cmd); ct = c[:, 0].astype(float); cw = c[:, 3].astype(float)
    wz_cmd = np.interp(tt, ct, cw)
    gain = 1 + args.lr_asym if d == "R" else 1.0
    turn = gain * first_order(delay(wz_cmd, 0.08, fs), 0.12, fs)   # robot yaw response, same for both profiles
    osc = 0.05 * np.sin(2 * math.pi * 2.6 * tt)                       # trot yaw oscillation
    if args.still_quiet:
        osc = osc * ((tt >= ev["accel_start"]) & (tt < ev["decel_end"]))
    gz = turn + osc + 0.01 * rng.normal(size=len(tt))                # physical yaw rate
    gzw = args.gyro_sign * gz + args.gyro_bias                        # as logged
    sh = clock(n)                                                     # logged robot time = true time + sh
    td = os.path.join(out, f"trial_{n:03d}"); os.makedirs(td, exist_ok=True)
    write(os.path.join(td, "cmd.csv"), ["t", "phase", "vx", "wz"], [(r[0] + sh,) + r[1:] for r in rows_cmd])
    write(os.path.join(td, "sport.csv"), P.SPORT_COLS,
          [(a + sh, b, 0, 0, b, 0, 0, 0, 0.3, 0, 0, 0, 1, 1) for a, b in zip(tt, gzw)])
    if args.equal_dynamics:
        # walking power with peaky gait pulses (mean 20 W, median far lower, random gait phase) plus a turn
        # cost proportional to the turn angle: the true extra work of the turn is the same for both profiles
        gait = 20 * np.abs(np.sin(math.pi * 2.6 * (tt - ts) + xrng.uniform(0, math.pi))) ** 6 / 0.3125
        mech = 35 + gait + 25 * np.abs(turn) + rng.normal(size=len(tt))
    else:
        mech = 35 + 25 * np.abs(gz) + 40 * np.abs(np.gradient(first_order(gz, 0.05, fs), 1 / fs)) * (1.4 if prof == "step" else 1) + rng.normal(size=len(tt))
    write(os.path.join(td, "low.csv"), P.LOW_COLS,
          [(a + sh, 0, 0, 9.81, b, 28.5, 3 + m / 30, m) for a, b, m in zip(tt, gzw, mech)])
    # hand
    lag, tau = (0.10, 0.07) if stick == "rigid" else (0.26, 0.22)
    hy = first_order(first_order(delay(gz, lag, fs), tau, fs), tau / 2, fs) + args.hand_sway * np.sin(2 * math.pi * 1.8 * tt + n)
    dhy = np.gradient(first_order(hy, 0.03, fs), 1 / fs)
    a_lat = 0.3 * hy + 0.35 * dhy + 0.25 * rng.normal(size=len(tt))
    a_fwd = 0.2 * np.sin(2 * math.pi * 1.8 * tt) + 0.2 * rng.normal(size=len(tt))
    a_up = 0.8 * np.sin(2 * math.pi * 3.6 * tt) + 0.2 * rng.normal(size=len(tt))
    if args.spurious_knocks:                       # heel strikes reach the handle while walking
        steps = np.arange(ev["accel_start"] + 0.3, ev["decel_end"], 1 / 1.8)
        a_up = a_up + pulses(tt, steps, xrng.uniform(3, 5, len(steps)), 0.008)
    tp = np.arange(ts, te, 0.01)
    ph_t.append(tp)
    ph_acc.append(np.column_stack([np.interp(tp, tt, a_lat), np.interp(tp, tt, a_fwd), np.interp(tp, tt, a_up)]))
    ph_gyr.append(np.column_stack([0.05 * rng.normal(size=len(tp)), 0.05 * rng.normal(size=len(tp)), np.interp(tp, tt, hy)]))
    note = f"clock step {sh:+.3f} s" if n == float(step_k) else ""   # as go2_turn_pilot.ClockWatch.note()
    return [n, stick, prof, d, ev["still_start"] + sh, ev["turn_start"] + sh, ev["turn_end"] + sh,
            ev["still2_end"] + sh, "", "", note]

def profile_order(reps):
    if args.balanced:                              # what go2_turn_pilot.py run does (directions alternate L, R)
        return P.block_order(reps, 7)[0]
    order = ["step", "smooth"] * reps; np.random.default_rng(7).shuffle(order); return order

sync("start", T0)
if args.repeat_sync_start:                         # an earlier sync attempt in the same phone recording
    tq = np.arange(T0 - args.repeat_sync_start - 5, T0, 0.01)
    sq = pulses(tq, T0 - args.repeat_sync_start + np.array([3, 3.45, 3.9, 6.5, 6.9]), [8] * 5, 0.008)
    ph_t.append(tq); ph_acc.append(np.column_stack([0 * tq, 0 * tq, sq])); ph_gyr.append(np.zeros((len(tq), 3)))
t = T0 + 40; trows = []; n = 1
for stick in args.sticks.split(","):
    order = profile_order(args.reps)
    for k, prof in enumerate(order):
        trows.append(trial(n, stick, prof, "L" if k % 2 == 0 else "R", t)); t += 30; n += 1
if args.aborted_row:                               # rows as go2_turn_pilot.py writes them (empty times)
    for note in ("aborted: state stream stale", "skipped"):
        os.makedirs(os.path.join(out, f"trial_{n:03d}"), exist_ok=True)
        trows.append([n, trows[-1][1], "step", "L", "", "", "", "", "", "", note]); n += 1
last_end = max(r[7] - clock(r[0]) for r in trows if r[7] != "")   # true time of the last trial end
if args.no_sync_end:                               # phone keeps recording a little after the last trial
    tq = np.arange(last_end, last_end + 10, 0.01)
    ph_t.append(tq); ph_acc.append(np.zeros((len(tq), 3))); ph_gyr.append(np.zeros((len(tq), 3)))
else:
    sync("end", t + 10)
write(os.path.join(out, "trials.csv"), ["trial", "stick", "profile", "direction", "t_start", "t_turn_start",
      "t_turn_end", "t_end", "clarity_1to7", "comfort_1to7", "note"], trows)

# phone: fill gaps with walking noise, rotate into a tilted phone frame, write Sensor Logger CSVs
T = np.concatenate(ph_t); A = np.concatenate(ph_acc); G = np.concatenate(ph_gyr)
o = np.argsort(T); T, A, G = T[o], A[o], G[o]
full = np.arange(T[0], T[-1], 0.01)
if args.phone_cut_end:
    full = full[full <= last_end + 5]
A = np.column_stack([np.interp(full, T, A[:, i]) for i in range(3)]) + 0.05 * rng.normal(size=(len(full), 3))
G = np.column_stack([np.interp(full, T, G[:, i]) for i in range(3)])
th = math.radians(35); R = np.array([[1, 0, 0], [0, math.cos(th), -math.sin(th)], [0, math.sin(th), math.cos(th)]])
A, G = A @ R, G @ R; up = np.array([0, 0, 1.0]) @ R
tphone = full + OFF + DRIFT * (full - T0)
keep = np.ones(len(full), bool)                    # phone data gaps (battery optimisation, pauses)
for k, secs in gaps:
    t_turn = float(trows[k - 1][5]) - clock(k)
    keep &= ~((full >= t_turn + 0.3) & (full < t_turn + 0.3 + secs))
GR = np.tile(up * 9.81, (len(full), 1))
files = [("Accelerometer", A), ("Gyroscope", G)]
if args.total_accel:
    files.append(("TotalAcceleration", A + GR))
elif not args.no_gravity:
    files.append(("Gravity", GR))
os.makedirs(os.path.join(out, "phone"), exist_ok=True)
for name, M in files:
    if args.ios and name != "Gyroscope":
        M = -M
    if args.g_units and name != "Gyroscope":
        M = M / 9.81
    write(os.path.join(out, "phone", f"{name}.csv"), ["time", "seconds_elapsed", "z", "y", "x"],
          [(int(tp * 1e9), round(tp - tphone[0], 4), m[2], m[1], m[0]) for tp, m, kp in zip(tphone, M, keep) if kp])
print("simulated", sum(1 for r in trows if r[4] != ""), "trials; true offset", OFF)
