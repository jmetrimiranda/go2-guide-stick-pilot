#!/usr/bin/env python3
"""
Analyse a Go2 guide-stick pilot session.

  python3 analyze_pilot.py data/s1            (phone CSVs in data/s1/phone/)
  python3 analyze_pilot.py data/s1 --robot-knock-floor 0.8    (if T4 shows weak knocks on the robot IMU)

Phone: Sensor Logger export (Accelerometer.csv, Gyroscope.csv, Gravity.csv) or any CSV with a
time column and x/y/z columns. Without Gravity.csv the vertical comes from TotalAcceleration.csv
(Android) when present; otherwise the analysis stops. Clocks are aligned with the knock pattern
recorded in sync_start/ (and sync_end/ for drift), so the phone clock does not need to be correct.

If the robot clock stepped during the session (go2_turn_pilot.py writes 'clock step +X s' in the trial's
note), trials before the step use sync_start only, trials after it use sync_end only, and the noted trial is
skipped.

Outputs in <session>/results/: trials.csv, summary.csv, summary.md, example_trials.png, metrics.png
(plus summary_by_direction.csv and sync.json); outputs of an earlier run are removed first.
A trial whose note starts with 'aborted' or 'skipped' is left out even if its times are complete, so a
bad trial can be excluded by editing its note.
"""
import argparse, glob, json, os, sys
from collections import namedtuple
import numpy as np
trapz = getattr(np, "trapezoid", None) or np.trapz
import pandas as pd
from scipy.signal import butter, filtfilt, find_peaks

FS = 100.0
BLUE, ORANGE = "#2a78d6", "#eb6834"         # rigid, flexible (categorical slots 1-2)
INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#8a8984", "#e4e3df"

TAIL = 1.5             # s after the turn command (the straight2 phase) in the turn window
LAG_PRE = 0.5          # s before the turn command in the centroid-lag window
JERK_WIN = 4.12        # s from turn start for the turn jerk RMS, both profiles (step's native window)
MAX_GAP = 0.1          # s; a longer gap in the raw phone samples of a trial -> its hand metrics are NaN
ROBOT_MAX_GAP = 0.25   # s; a robot stream with a longer gap inside the analysis window is not used
SYNC_TOL = 0.06        # s, tolerance when matching robot and phone knocks
MAX_DRIFT = 1e-4       # s/s allowed between start and end syncs (real drift ~1e-5; knocks are 0.4 s apart)
MIN_SYNC_ROWS = 100    # a sync low.csv with fewer rows is ignored
CLOCK_STEP_TOL = 0.05  # s; a larger change of the robot's wall - monotonic time is a clock step (as in the pilot)
GYRO_STILL_STD = 0.02  # rad/s; the gyro bias is measured in the 1 s 'still' phase only if the yaw rate is this quiet
CUE_THR, CUE_HOLD = 0.1, 0.25   # rad/s, s: hand yaw rate held this long = the hand feels the turn (cue onset)
TURN_OK, HAND_LAG_OK = (80.0, 100.0), (0.0, 1.0)     # sanity ranges of the T6 checks
TRIAL_COLS = ["trial", "stick", "profile", "direction", "t_start", "t_turn_start", "t_turn_end", "t_end"]
HAND_METRICS = ["hand_lag_s", "hand_onset_lag_s", "hand_cue_onset_s", "hand_peak_yaw", "hand_peak_ah",
                "hand_jerk_turn", "hand_jerk_pre", "hand_jerk_excess", "hand_sway_pre"]
RESULT_COLS = ["trial", "stick", "profile", "direction", "robot_turn_deg", "robot_lag_s",
               "robot_peak_yaw_acc", "hand_lag_s", "hand_peak_yaw", "hand_peak_ah", "hand_jerk_turn",
               "hand_jerk_pre", "mech_work_J", "elec_energy_J", "clarity", "comfort", "hand_jerk_excess",
               # added columns
               "robot_onset_lag_s", "hand_onset_lag_s", "hand_cue_onset_s", "mech_excess_J", "elec_excess_J",
               "mech_walk_W", "elec_walk_W", "turn_win_s", "robot_yaw_src", "gyro_bias", "gyro_still_std",
               "robot_sway_pre", "hand_sway_pre", "phone_max_gap_s", "robot_sign_ok", "hand_sign_ok", "note"]
SUMMARY_COLS = ["robot_turn_deg", "robot_lag_s", "hand_lag_s", "hand_peak_yaw", "hand_peak_ah",
                "hand_jerk_excess", "mech_work_J", "elec_energy_J", "mech_excess_J", "elec_excess_J",
                "hand_cue_onset_s"]
OUTPUTS = ["trials.csv", "summary.csv", "summary_by_direction.csv", "summary.md", "sync.json",
           "example_trials.png", "metrics.png"]
DIFF_LABELS = [("hand_jerk_excess", "hand jerk added (m/s³)"), ("hand_peak_ah", "peak hand accel (m/s²)"),
               ("hand_lag_s", "hand lag (s)"), ("hand_cue_onset_s", "hand cue onset (s)"),
               ("robot_lag_s", "robot lag (s)"),
               ("mech_excess_J", "joint work above walking (J)"),
               ("elec_excess_J", "battery energy above walking (J)"), ("robot_turn_deg", "turn (deg)")]


def lp(x, fc, fs=FS, order=2):
    b, a = butter(order, fc / (fs / 2))
    return filtfilt(b, a, x, axis=0) if len(x) > 3 * max(len(a), len(b)) else x


def loud(msg):
    bar = "!" * 78
    print(f"\n{bar}\nWARNING: {msg}\n{bar}")


def max_gap(traw, a, b):
    """Largest spacing between consecutive raw samples around [a, b] (inf if [a, b] is not covered)."""
    if len(traw) < 2 or traw[0] > a or traw[-1] < b:
        return np.inf
    i0 = np.searchsorted(traw, a, side="right") - 1         # last sample <= a
    i1 = np.searchsorted(traw, b, side="left")              # first sample >= b
    return float(np.max(np.diff(traw[i0:i1 + 1]))) if i1 > i0 else 0.0


# ------------------------------------------------------------------ phone loading
def _read_xyz(path):
    df = pd.read_csv(path)
    def num(c):                                         # bad cells -> NaN
        return pd.to_numeric(df[c], errors="coerce").to_numpy(float)
    cols = {c.lower().strip(): c for c in df.columns}
    if "time" in cols and np.nanmedian(num(cols["time"])) > 1e15:          # Sensor Logger ns epoch
        t = num(cols["time"]) / 1e9
    else:
        tc = next(c for c in df.columns if "time" in c.lower() or c.lower().startswith("seconds"))
        t = num(tc)
    def pick(ax):
        for c in df.columns:
            cl = c.lower().strip()
            if cl == ax or cl.startswith(ax + " ") or cl.startswith(ax + "("):
                return num(c)
        raise KeyError(f"no {ax} column in {path}")
    x = np.column_stack([pick("x"), pick("y"), pick("z")])
    # one NaN (e.g. a truncated last line) would spread through the filters and disable the knock sync
    good = np.isfinite(t) & np.isfinite(x).all(axis=1)
    if not good.all():
        print(f"{os.path.basename(path)}: dropped {np.sum(~good)} rows with missing or non-numeric values")
        t, x = t[good], x[good]
    o = np.argsort(t, kind="stable")                    # np.interp needs increasing times
    t, x = t[o], x[o]
    keep = np.r_[True, np.diff(t) > 0]                  # drop duplicated timestamps
    return t[keep], x[keep]


def load_phone(folder):
    def find(*names):
        for n in names:
            m = sorted(p for p in glob.glob(os.path.join(folder, n))
                       if "uncalibrated" not in os.path.basename(p).lower())
            if m:
                return m[0]
        return None
    fa = find("Accelerometer.csv", "TotalAcceleration.csv", "*ccelerat*.csv")
    fg = find("Gyroscope.csv", "*yro*.csv")
    fgr = find("Gravity.csv", "*ravity*.csv")
    if not fa or not fg:
        sys.exit(f"Phone CSVs not found in {folder}")
    ta, a = _read_xyz(fa)
    tg, g = _read_xyz(fg)
    for name, tr in (("accelerometer", ta), ("gyroscope", tg)):
        dt = np.diff(tr)
        print(f"phone {name}: {len(tr)} samples, {1 / np.median(dt):.0f} Hz median, largest gap {dt.max():.3f} s")
    t0, t1 = max(ta[0], tg[0]), min(ta[-1], tg[-1])
    t = np.arange(t0, t1, 1 / FS)
    def on_grid(tf, v):
        return np.column_stack([np.interp(t, tf, v[:, i]) for i in range(3)])
    A, G = on_grid(ta, a), on_grid(tg, g)
    if fgr:
        GR = on_grid(*_read_xyz(fgr))
        if np.median(np.linalg.norm(GR, axis=1)) < 2.0:            # units of g
            A, GR = A * 9.81, GR * 9.81
    else:
        # Sensor Logger's Accelerometer.csv has gravity removed, so the vertical can only come from a
        # signal that still contains gravity (TotalAcceleration.csv on Android, or a generic CSV).
        src = find("TotalAcceleration.csv") or fa
        GR = lp(on_grid(*_read_xyz(src)), 0.5)
        if np.median(np.linalg.norm(GR, axis=1)) < 5.0:
            sys.exit(f"Gravity.csv not found in {folder}, and {os.path.basename(src)} has no gravity in it "
                     "(Sensor Logger's Accelerometer.csv is gravity-free), so the phone's vertical is unknown. "
                     "Enable Gravity in Sensor Logger (or TotalAcceleration on Android) and record again.")
        print(f"WARNING: no Gravity.csv; the vertical is estimated by low-passing {os.path.basename(src)}")
    up = lp(GR, 1.0)
    up /= np.linalg.norm(up, axis=1, keepdims=True) + 1e-9
    return {"t": t, "acc": A, "gyr": G, "up": up, "raw_t": (ta, tg)}


# ------------------------------------------------------------------ clock sync
def knocks(t, acc, fs, floor=1.5):
    """Knock times: sharp peaks of |acc| above max(median + 8 MAD, floor), at least 0.25 s apart.
    Also returns (times, heights) of all detector peaks and the threshold, to tune the floor."""
    mag = np.linalg.norm(acc, axis=1)
    hp = np.abs(mag - lp(mag, 2.0, fs))
    thr = max(np.median(hp) + 8 * (np.median(np.abs(hp - np.median(hp))) + 1e-6), floor)
    idx, _ = find_peaks(hp, distance=max(1, int(0.25 * fs)))
    return t[idx[hp[idx] >= thr]], (t[idx], hp[idx], thr)


def top5(heights):
    return [round(float(h), 2) for h in np.sort(heights)[::-1][:5]]


def nearest(sorted_a, x):
    """Index of the element of sorted_a nearest to each value of x."""
    i = np.searchsorted(sorted_a, x)
    lo, hi = np.clip(i - 1, 0, len(sorted_a) - 1), np.clip(i, 0, len(sorted_a) - 1)
    return np.where(np.abs(sorted_a[lo] - x) <= np.abs(sorted_a[hi] - x), lo, hi)


# n: matched knocks; offset: phone time = robot time + offset (s); extra: unmatched phone peaks in the
# mapped knock window (about 0 for a real sync, many for a chance match on gait or handling peaks)
Cand = namedtuple("Cand", "n offset extra")


def sync_candidates(rk, pk, tol=SYNC_TOL, n_min=3):
    """Distinct offsets that line up >= n_min robot knocks with phone knocks, as [Cand], best first."""
    if len(rk) < n_min or len(pk) == 0:
        return []
    offs = (pk[None, :] - rk[:, None]).ravel()             # every robot-phone knock pairing
    j = nearest(pk, rk[None, :] + offs[:, None])            # phone knock nearest each mapped robot knock
    d = pk[j] - rk[None, :]
    ok = np.abs(d - offs[:, None]) < tol
    n = ok.sum(axis=1)
    found = []
    for i in np.flatnonzero(n >= n_min):
        o = float(np.median(d[i][ok[i]]))
        w = (pk > rk.min() + o - 0.5) & (pk < rk.max() + o + 0.5)
        found.append(Cand(int(n[i]), o, int(w.sum()) - len(set(j[i][ok[i]]))))
    found.sort(key=lambda c: (-c.n, c.extra, c.offset))
    clusters = []
    for c in found:
        if all(abs(c.offset - k.offset) > 2 * tol for k in clusters):
            clusters.append(c)
    # A weaker candidate within the knock pattern's span of a stronger one is the same knocks matched one
    # knock interval off (3 of 5). Dropping it keeps it from pairing with the other sync after a clock step.
    span = rk.max() - rk.min()
    return [c for c in clusters if not any(k.n > c.n and abs(k.offset - c.offset) <= span for k in clusters)]


def clock_steps(session, trials):
    """Signs that the robot wall clock stepped (NTP, timedatectl) during the session, which breaks a single
    robot -> phone clock model. Returns (trial numbers whose trials.csv note has go2_turn_pilot.py's
    'clock step +X s', step between the syncs in s from sync_*/meta.json or None, warnings)."""
    flagged = sorted({int(k) for k, (_, r) in zip(pd.to_numeric(trials.trial, errors="coerce"), trials.iterrows())
                      if pd.notna(k) and "clock step" in row_note(r).lower()})
    wall_mono, warnings = {}, []            # tag -> time.time() - time.monotonic() at (sync start, sync end)
    for tag in ("start", "end"):
        try:
            with open(os.path.join(session, f"sync_{tag}", "meta.json")) as f:
                meta = json.load(f)
            a = float(meta["wall_minus_monotonic_s"])
            b = float(meta.get("wall_minus_monotonic_end_s", a))
        except (OSError, ValueError, KeyError, TypeError):
            continue                        # older sessions have no meta.json or no clock fields
        wall_mono[tag] = (a, b)
        if abs(b - a) > CLOCK_STEP_TOL:
            warnings.append(f"the robot clock stepped by {b - a:+.3f} s during sync_{tag} (sync_{tag}/meta.json): "
                            "its knocks may not line up")
    jump = wall_mono["end"][0] - wall_mono["start"][1] if len(wall_mono) == 2 else None
    return flagged, (jump if jump is not None and abs(jump) > CLOCK_STEP_TOL else None), warnings


def clock_model(session, phone, trials, robot_floor=1.5):
    """Robot time -> phone time offsets from the knock syncs. Returns (offset_for, info): offset_for(row)
    gives (offset_fn, None) for a trials.csv row, or (None, why that trial has no clock alignment).
    Models: a robot clock step noted in trials.csv splits the session (sync_start before the step, sync_end
    after it); otherwise a consistent (start, end) candidate pair gives a line with drift ('knock
    (start+end)'); otherwise one sync gives a constant offset ('knock (single)'); otherwise wall clocks."""
    pk, (ppt, pph, pthr) = knocks(phone["t"], phone["acc"], FS)
    print(f"phone: {len(pk)} knock-like peaks above {pthr:.2f} m/s² over {(phone['t'][-1] - phone['t'][0]) / 60:.1f} min")
    info = {"kind": None, "syncs": {}, "drift_ms": None, "warnings": []}

    def warn(msg):                       # console, sync.json and the T6 checks of summary.md
        loud(msg)
        info["warnings"].append(msg)

    cands, spans = {}, {}
    for tag in ("start", "end"):
        f = os.path.join(session, f"sync_{tag}", "low.csv")
        if not os.path.exists(f):
            continue
        low = read_table(f)
        s = info["syncs"][tag] = {"robot_knocks": 0, "matched": 0, "offset": None, "used": False}
        if low is None or len(low) < MIN_SYNC_ROWS:
            s["problem"] = f"low.csv has {0 if low is None else len(low)} rows"
            print(f"sync_{tag}: low.csv has {0 if low is None else len(low)} rows (< {MIN_SYNC_ROWS}); sync skipped")
            continue
        tr = low.t.to_numpy(float)
        fs = (len(tr) - 1) / (tr[-1] - tr[0])               # robust to bursty arrival stamps
        rk, (_, rph, rthr) = knocks(tr, low[["ax", "ay", "az"]].to_numpy(float), fs, robot_floor)
        c = sync_candidates(rk, pk)
        spans[tag] = (tr[0], tr[-1])
        s.update(robot_knocks=len(rk), robot_threshold=round(float(rthr), 2), robot_top_peaks=top5(rph),
                 candidates=[(k.n, round(k.offset, 4), k.extra) for k in c[:4]])
        print(f"sync_{tag}: {len(rk)} robot knocks (threshold {rthr:.2f} m/s², top peaks {top5(rph)}); "
              f"candidates (matched, offset s, extra phone peaks): {s['candidates']}")
        if c:
            cands[tag] = (float(np.median(rk)), c)
    ok_rows = trials[[row_problem(r) is None for _, r in trials.iterrows()]]

    def inside(o, rows):
        """Number of these trials that lie inside the phone record with offset o."""
        return int(sum((float(r.t_start) + o >= phone["t"][0]) and (float(r.t_end) + o <= phone["t"][-1])
                       for _, r in rows.iterrows()))

    def single_score(c, rows):
        """Rank of a one-sync candidate: puts >= 50% of the trials inside the phone record, then most matched
        knocks, fewest extra phone peaks, most trials inside."""
        k = inside(c.offset, rows)
        return (2 * k >= max(1, len(rows)), c.n, -c.extra, k)

    def best_single(tags, rows):
        """(tag, Cand) of the best one-sync candidate for these trials. On a full tie the later phone pattern
        wins: a repeated sync leaves both patterns in the phone record, and the pilot renames the earlier
        attempt to sync_<tag>_old_*, so the folder holds the last one."""
        ranked = sorted(((tag, c) for tag in tags for c in cands[tag][1]),
                        key=lambda tc: single_score(tc[1], rows) + (tc[1].offset,), reverse=True)
        tag, c = ranked[0]
        ties = [f"{k.offset:.3f}" for _, k in ranked[1:]
                if single_score(k, rows) == single_score(c, rows) and abs(k.offset - c.offset) > 2 * SYNC_TOL]
        if ties:
            warn(f"ambiguous sync_{tag}: its knocks match the phone record equally well at offsets {c.offset:.3f} "
                 f"and {', '.join(ties)} s (sync repeated in one phone recording?); the latest, {c.offset:.3f} s, "
                 "was used. Check that hand_lag_s is plausible.")
        return tag, c

    def use(tag, c):                     # t_robot: robot time where the offset was measured
        info["syncs"][tag].update(matched=c.n, offset=c.offset, used=True, t_robot=cands[tag][0])

    def constant(o):
        return lambda tr: np.full_like(np.asarray(tr, float), o)

    flagged, jump, step_warnings = clock_steps(session, trials)
    for msg in step_warnings:
        warn(msg)
    fn = offset_for = None
    if flagged and cands:
        # The clock moved at a noted trial: each sync only holds on its own side of the step. The noted
        # trial itself is skipped, since the step may have happened while it was logged.
        k0, k1 = flagged[0], flagged[-1]
        nums = pd.to_numeric(ok_rows.trial, errors="coerce")
        offsets = {}
        for tag, rows in (("start", ok_rows[nums < k0]), ("end", ok_rows[nums > k1])):
            if tag in cands and len(rows):
                _, c = best_single([tag], rows)
                use(tag, c)
                offsets[tag] = c.offset
        where = f"trial {k0}" if k0 == k1 else f"trials {k0}-{k1}"
        warn(f"the robot clock stepped during the session (note 'clock step' on {where} in trials.csv): trials "
             f"before it use sync_start only, trials after it use sync_end only (no drift correction), and {where} "
             "is skipped")
        info["kind"] = "knock (split at clock step)"

        def offset_for(row):
            k = int(pd.to_numeric(row.trial))
            if k0 <= k <= k1:
                return None, "robot clock stepped during or just before this trial (note 'clock step')"
            tag = "start" if k < k0 else "end"
            if tag not in offsets:
                return None, f"no usable sync_{tag} on this side of the robot clock step"
            return constant(offsets[tag]), None
    elif "start" in cands and "end" in cands and jump is None:
        (t1, c1), (t2, c2) = cands["start"], cands["end"]
        lim = MAX_DRIFT * abs(t2 - t1) + 2 * SYNC_TOL

        def pair_score(pair):
            """Most matched knocks, then fewest extra phone peaks, then the smallest drift."""
            a, b = pair
            return (a.n + b.n, -(a.extra + b.extra), -abs(b.offset - a.offset))
        pairs = sorted(((a, b) for a in c1 for b in c2 if abs(b.offset - a.offset) <= lim),
                       key=pair_score, reverse=True)
        if pairs:
            a, b = pairs[0]
            if len(pairs) > 1 and pair_score(pairs[1])[0] == pair_score(pairs[0])[0]:
                warn(f"ambiguous sync: runner-up offsets {pairs[1][0].offset:.3f} / {pairs[1][1].offset:.3f} s "
                     f"match as many knocks as the chosen {a.offset:.3f} / {b.offset:.3f} s")
            use("start", a); use("end", b)
            o1, o2 = a.offset, b.offset
            info["drift_ms"] = (o2 - o1) * 1000
            print(f"sync: start {a.n} and end {b.n} knocks matched; clock drift {(o2 - o1) * 1000:.1f} ms "
                  f"over {(t2 - t1) / 60:.1f} min")
            fn = lambda tr: o1 + (np.asarray(tr, float) - t1) * (o2 - o1) / (t2 - t1)
            info["kind"] = "knock (start+end)"
        else:
            warn("the start and end knock syncs disagree (no candidate pair with a plausible drift): one of them "
                 "is probably not in the phone record, or the robot clock stepped between them")
    if fn is None and offset_for is None and cands:
        if jump is not None:
            warn(f"the robot clock stepped by {jump:+.3f} s between sync_start and sync_end (wall - monotonic time "
                 "in sync_*/meta.json; or the Orin rebooted), but no trial note says where: one sync is used, so the "
                 "hand metrics of the trials on the other side of the step are shifted by that much")
        tag, c = best_single(list(cands), ok_rows)
        use(tag, c)
        others = [(t, k.n, round(k.offset, 3), k.extra) for t in cands for k in cands[t][1] if k is not c][:3]
        loud(f"using ONE knock sync (sync_{tag}: {c.n} knocks, offset {c.offset:.3f} s, "
             f"{inside(c.offset, ok_rows)}/{len(ok_rows)} trials inside the phone record); no drift correction. "
             f"Other candidates (sync, matched, offset s, extra peaks): {others}")
        fn = constant(c.offset)
        info["kind"] = "knock (single)"
    if fn is None and offset_for is None:
        if phone["t"][0] <= 1e9:
            sys.exit("No knock sync and the phone has no absolute time. Record sync_start.")
        loud("no knock sync; using wall clocks (NTP) as-is. Lags are approximate.")
        fn = constant(0.0)
        info["kind"] = "ntp"
    if offset_for is None:
        offset_for = lambda row: (fn, None)
    # phone side of each sync, for tuning the knock detector (T4)
    for tag, (a, b) in spans.items():
        s = info["syncs"][tag]
        o = s["offset"] if s["used"] else (float(fn(a)) if fn is not None else None)
        if o is None:
            continue
        s["phone_top_peaks"] = top5(pph[(ppt >= a + o) & (ppt <= b + o)])
        print(f"sync_{tag}: phone top peaks in the mapped sync window {s['phone_top_peaks']} "
              f"(phone threshold {pthr:.2f} m/s²)")
    info["phone_threshold"] = round(float(pthr), 2)
    return offset_for, info


# ------------------------------------------------------------------ per trial
def read_table(path):
    """CSV as a DataFrame sorted by t; None if missing or without rows."""
    try:
        df = pd.read_csv(path)
    except (FileNotFoundError, pd.errors.EmptyDataError):
        return None
    if not len(df) or "t" not in df.columns:
        return None
    return df.sort_values("t", kind="stable").reset_index(drop=True)


def row_note(row):
    note = row.get("note", "")
    return "" if pd.isna(note) else str(note).strip()


def row_problem(row):
    """Why a trials.csv row cannot be analysed, or None."""
    note = row_note(row)
    times = [pd.to_numeric(row[c], errors="coerce") for c in ("t_start", "t_turn_start", "t_turn_end", "t_end")]
    if note.lower().startswith(("aborted", "skipped")):
        # the pilot writes these with empty times; with complete times the note was typed to exclude the trial
        return note if any(pd.isna(x) for x in times) else "excluded by its note in trials.csv (times complete)"
    if pd.isna(pd.to_numeric(row.trial, errors="coerce")):
        return "no trial number"
    if any(pd.isna(x) for x in times):
        return "no event times (aborted or skipped)"
    if not times[0] < times[1] < times[2] < times[3]:
        return "event times out of order"
    return None


def held_above(t, y, thr, hold=0.25):
    """First time y reaches thr and stays there for `hold` s (so a sway or counter-swing does not count)."""
    n = max(1, int(round(hold * FS)))
    run = np.convolve((y >= thr).astype(int), np.ones(n, int), "valid") == n
    return t[np.argmax(run)] if run.any() else np.nan


def onset(t, y, frac=0.3, hold=0.25):
    """First time y (signed so the turn is positive, baseline removed) holds frac of its 95th-percentile peak."""
    pk = np.percentile(y, 95) if len(y) else np.nan
    if not np.isfinite(pk) or pk <= 0:
        return np.nan
    return held_above(t, y, frac * pk, hold)


def centroid_lag(t, u, y, min_frac=0.3):
    """Time between the centroids (time-weighted means) of y and u. For a linear response y = h * u this
    is the mean delay of h whatever the shape of u. NaN if y has too little area (no clear response)."""
    au, ay = np.sum(u), np.sum(y)
    if au <= 0 or ay < min_frac * au:
        return np.nan
    return float(np.sum(t * y) / ay - np.sum(t * u) / au)


def dot_sign(a, b):
    return float(np.sign(np.sum(a * b)))


def prepare_trial(session, row, phone, offset_fn):
    """Resample one trial onto a 100 Hz grid. Returns (data, None), or (None, reason to skip it)."""
    d = os.path.join(session, f"trial_{int(row.trial):03d}")
    cmd, sport, low = (read_table(os.path.join(d, f)) for f in ("cmd.csv", "sport.csv", "low.csv"))
    ts, te = float(row.t_start), float(row.t_end)
    tt0, tt1 = float(row.t_turn_start), float(row.t_turn_end)
    if cmd is None:
        return None, "cmd.csv missing or empty"
    if cmd.t.iloc[0] > tt0 + 0.1 or cmd.t.iloc[-1] < tt1 - 0.1:
        return None, "cmd.csv does not cover the trials.csv turn times (folder overwritten?)"
    a, b = tt0 - 2.0, tt1 + TAIL                            # pre-turn + turn windows
    usable = [x for x in (low, sport) if x is not None and max_gap(x.t.to_numpy(float), a, b) <= ROBOT_MAX_GAP]
    if not usable:
        return None, "no robot yaw rate (low.csv and sport.csv empty or with gaps)"
    yaw = usable[0]                                         # lowstate first: same stream as the knock sync
    t = np.arange(ts, te, 1 / FS)
    p = dict(row=row, t=t, tt0=tt0, tt1=tt1, yaw_src="low" if yaw is low else "sport")
    p["wz_cmd"] = np.interp(t, cmd.t, cmd.wz, left=0, right=0)
    p["gz"] = lp(np.interp(t, yaw.t, yaw.gz), 8)
    # Gyro bias from the 1 s 'still' phase, only if the robot really was still: any sway or stepping in place
    # would make the 1 s mean a turn-direction-signed error of mean x (4-5 s) in robot_turn_deg.
    still = yaw.gz[(yaw.t >= ts) & (yaw.t <= ts + 1.0)]
    p["gyro_still_std"] = float(np.std(still)) if len(still) >= 5 else np.nan
    p["gyro_bias"] = float(np.mean(still)) if p["gyro_still_std"] < GYRO_STILL_STD else 0.0
    if low is yaw:
        p["mech"] = np.interp(t, low.t, low.mech_power)
        p["elec"] = np.interp(t, low.t, low.power_v * low.power_a)
    else:
        p["mech"] = p["elec"] = np.full(len(t), np.nan)
        p["energy_note"] = "energy metrics missing: low.csv empty or with gaps (yaw rate from sport.csv)"

    tp = t + offset_fn(t)                       # robot time -> phone time
    if tp[0] < phone["t"][0] or tp[-1] > phone["t"][-1]:
        return None, "outside the phone recording"
    acc = np.column_stack([np.interp(tp, phone["t"], phone["acc"][:, i]) for i in range(3)])
    gyr = np.column_stack([np.interp(tp, phone["t"], phone["gyr"][:, i]) for i in range(3)])
    up = np.column_stack([np.interp(tp, phone["t"], phone["up"][:, i]) for i in range(3)])
    # raw phone gaps inside the analysis window (with 0.5 s for filter edges)
    w0 = tt0 - 2.5 + float(offset_fn(tt0))
    w1 = max(tt1 + TAIL, tt0 + JERK_WIN) + 0.5 + float(offset_fn(tt1))
    p["phone_gap"] = max(max_gap(rt, w0, w1) for rt in phone["raw_t"])
    p["hand_yaw"] = np.sum(gyr * up, axis=1)
    a_h = acc - np.sum(acc * up, axis=1, keepdims=True) * up
    p["a_h"] = lp(a_h, 10)
    p["jerk"] = np.linalg.norm(np.gradient(p["a_h"], 1 / FS, axis=0), axis=1)

    pre = p["pre"] = (t >= tt0 - 2.0) & (t < tt0)
    p["turn"] = (t >= tt0) & (t <= tt1 + TAIL)
    win = p["win"] = (t >= tt0 - LAG_PRE) & (t <= tt1 + TAIL)
    gz3, hy3 = lp(p["gz"], 3), lp(p["hand_yaw"], 3)
    p["gz3b"], p["hy3b"] = gz3 - np.mean(gz3[pre]), hy3 - np.mean(hy3[pre])
    # per-trial votes for the session's axis signs
    p["vote_gyro"] = dot_sign(p["gz3b"][win], p["wz_cmd"][win])
    p["vote_hand"] = dot_sign(p["hy3b"][win], p["gz3b"][win]) if p["phone_gap"] <= MAX_GAP else 0.0
    return p, None


def session_sign(votes, what):
    """+1/-1 from the median of per-trial correlation signs (never from an assumed axis convention)."""
    v = np.array([x for x in votes if np.isfinite(x) and x != 0])
    s = float(np.sign(np.median(v))) if len(v) else 0.0
    if s == 0:
        loud(f"could not decide the sign of {what} from the data; assuming +1")
        s = 1.0
    print(f"sign of {what}: {s:+.0f} ({int(np.sum(v == s))}/{len(v)} trials agree)")
    return s, int(np.sum(v == s)), len(v)


def trial_metrics(p, s_gyro, s_hand):
    row, t, tt0, tt1 = p["row"], p["t"], p["tt0"], p["tt1"]
    pre, turn, win = p["pre"], p["turn"], p["win"]
    jwin = (t >= tt0) & (t <= tt0 + JERK_WIN)
    s_cmd = np.sign(np.sum(p["wz_cmd"][turn])) or (1.0 if str(row.direction).upper().startswith("L") else -1.0)
    s_r = s_cmd * s_gyro                                     # orients robot yaw so the turn is positive
    u = s_cmd * p["wz_cmd"]
    y_r = s_r * p["gz3b"]
    y_h = s_r * s_hand * p["hy3b"]
    gz, mech, elec = p["gz"], p["mech"], p["elec"]

    c_on = onset(t[turn], u[turn], hold=0.0)
    r_on, h_on = onset(t[turn], y_r[turn]), onset(t[turn], y_h[turn])
    tw_rel = t[win] - tt0                                    # relative times keep the centroids precise
    tw = float(t[turn][-1] - t[turn][0])
    mech_walk, elec_walk = float(np.mean(mech[pre])), float(np.mean(elec[pre]))
    jerk = p["jerk"]
    res = dict(
        trial=int(row.trial), stick=row.stick, profile=row.profile, direction=row.direction,
        robot_turn_deg=float(np.degrees(s_r * trapz(gz[turn] - p["gyro_bias"], t[turn]))),
        robot_lag_s=centroid_lag(tw_rel, u[win], y_r[win]),
        robot_peak_yaw_acc=float(np.max(np.abs(np.gradient(p["gz3b"], 1 / FS)[turn]))),
        hand_lag_s=centroid_lag(tw_rel, y_r[win], y_h[win]),
        hand_peak_yaw=float(np.max(y_h[turn])),
        hand_peak_ah=float(np.max(np.linalg.norm(p["a_h"][turn], axis=1))),
        hand_jerk_turn=float(np.sqrt(np.mean(jerk[jwin] ** 2))),
        hand_jerk_pre=float(np.sqrt(np.mean(jerk[pre] ** 2))),
        mech_work_J=float(trapz(mech[turn], t[turn])),
        elec_energy_J=float(trapz(elec[turn], t[turn])),
        clarity=row.get("clarity_1to7", np.nan), comfort=row.get("comfort_1to7", np.nan),
    )
    res["hand_jerk_excess"] = res["hand_jerk_turn"] - res["hand_jerk_pre"]
    res.update(
        robot_onset_lag_s=r_on - c_on, hand_onset_lag_s=h_on - r_on,
        # when the hand can first feel the turn (absolute threshold, so it depends on the command shape)
        hand_cue_onset_s=held_above(t[turn], y_h[turn], CUE_THR, CUE_HOLD) - tt0,
        # work above the straight-walking baseline (mean power in the 2 s before the turn): unlike the
        # absolute integrals it does not grow with the window, which is 0.8 s longer for smooth
        mech_excess_J=float(trapz(mech[turn] - mech_walk, t[turn])),
        elec_excess_J=float(trapz(elec[turn] - elec_walk, t[turn])),
        mech_walk_W=mech_walk, elec_walk_W=elec_walk, turn_win_s=tw,
        robot_yaw_src=p["yaw_src"], gyro_bias=p["gyro_bias"], gyro_still_std=p["gyro_still_std"],
        robot_sway_pre=float(np.std(y_r[pre])), hand_sway_pre=float(np.sqrt(np.mean(y_h[pre] ** 2))),
        phone_max_gap_s=p["phone_gap"],
        robot_sign_ok=bool(p["vote_gyro"] == s_gyro),
        hand_sign_ok=bool(p["vote_hand"] == s_hand) if p["vote_hand"] else np.nan,
        note=row_note(row),
    )
    if p["phone_gap"] > MAX_GAP:
        for k in HAND_METRICS:
            res[k] = np.nan
        return res, None
    series = dict(t=t - tt0, cmd=u, robot=y_r, hand=y_h,
                  ah=np.linalg.norm(p["a_h"], axis=1), turn_len=tt1 - tt0)
    return res, series


# ------------------------------------------------------------------ plots
def style(ax):
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(MUTED)
    ax.tick_params(colors=INK2, labelsize=9)
    ax.grid(axis="y", color=GRID, lw=0.8)
    ax.set_axisbelow(True)


def stick_order(sticks):
    known = [s for s in ("rigid", "flexible") if s in sticks]
    return known + sorted(s for s in set(sticks) if s not in known)


def stick_color(s):
    return BLUE if s == "rigid" else ORANGE


def plot_examples(examples, out):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    sticks = stick_order({k[0] for k in examples})
    fig, axes = plt.subplots(len(sticks), 2, figsize=(10, 3.2 * len(sticks)), sharex=True,
                             sharey=True, squeeze=False)
    for i, s in enumerate(sticks):
        col = stick_color(s)
        for j, p in enumerate(("step", "smooth")):
            ax = axes[i][j]; style(ax)
            ser = examples.get((s, p))
            if ser is None:
                ax.set_visible(False); continue
            # series are already oriented so the turn is positive
            ax.plot(ser["t"], ser["cmd"], color=MUTED, lw=1.5, ls="--", label="command")
            ax.plot(ser["t"], ser["robot"], color=INK2, lw=2, label="robot body")
            ax.plot(ser["t"], ser["hand"], color=col, lw=2, label="hand (phone)")
            ax.axvspan(0, ser["turn_len"], color=GRID, alpha=0.5, lw=0)
            ax.set_title(f"{s} stick, {p} turn", fontsize=10, color=INK, loc="left")
            if j == 0:
                ax.set_ylabel("yaw rate (rad/s)", color=INK2, fontsize=9)
            if i == len(sticks) - 1:
                ax.set_xlabel("time from turn command (s)", color=INK2, fontsize=9)
            ax.set_xlim(-1.5, 5.5)
    for ax in (a for r in axes for a in r if a.get_visible()):
        ax.legend(frameon=False, fontsize=8, loc="upper right")
        break
    fig.suptitle("What the hand feels during a 90° turn (one representative trial each)",
                 x=0.01, ha="left", fontsize=11, color=INK)
    fig.tight_layout()
    fig.savefig(out, dpi=200)
    plt.close(fig)


def plot_metrics(df, out):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    metrics = [("hand_jerk_excess", "Hand jerk added by the turn\n(RMS, m/s³)"),
               ("hand_peak_ah", "Peak horizontal hand accel.\n(m/s²)"),
               ("hand_lag_s", "Hand cue lag after robot\n(s)"),
               ("mech_excess_J", "Robot joint work in the turn,\nabove walking (J)")]
    sticks = stick_order(set(df.stick))
    groups = [(s, p) for s in sticks for p in ("step", "smooth")
              if ((df.stick == s) & (df.profile == p)).any()]
    fig, axes = plt.subplots(1, len(metrics), figsize=(12, 3.6))
    rng = np.random.default_rng(0)
    for ax, (m, lab) in zip(axes, metrics):
        style(ax)
        for k, (s, p) in enumerate(groups):
            v = df[(df.stick == s) & (df.profile == p)][m].dropna().to_numpy()
            col = stick_color(s)
            x = k + rng.uniform(-0.12, 0.12, len(v))
            ax.scatter(x, v, s=36, facecolor=col if p == "smooth" else "white", edgecolor=col,
                       lw=1.5, zorder=3)
            if len(v):
                ax.plot([k - 0.28, k + 0.28], [np.median(v)] * 2, color=INK, lw=2, zorder=4)
        ax.set_xticks(range(len(groups)))
        ax.set_xticklabels([f"{s}\n{p}" for s, p in groups], fontsize=8, color=INK2)
        ax.set_title(lab, fontsize=9, color=INK, loc="left")
    colours = ", ".join(f"{'blue' if s == 'rigid' else 'orange'} = {s} stick" for s in sticks)
    fig.text(0.01, 0.01, "Dots: trials (hollow = step, filled = smooth). Bar: median. "
             f"{colours[0].upper() + colours[1:]}.", fontsize=8, color=INK2)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(out, dpi=200)
    plt.close(fig)


# ------------------------------------------------------------------ summary
def plural(n, word):
    return f"{n} {word}{'' if n == 1 else 's'}"


def fmt(x, nd=2):
    return "nan" if x is None or pd.isna(x) else f"{x:.{nd}f}"


def direction_section(df):
    """Profile x direction counts and the step - smooth difference averaged over turn directions."""
    lines = ["## Profile x direction", ""]
    dirs = sorted(df.direction.astype(str).unique())
    lines += ["| stick | profile | " + " | ".join(dirs) + " |", "|---|---|" + "---|" * len(dirs)]
    counts = df.groupby(["stick", "profile", df.direction.astype(str)]).size()
    for s in stick_order(set(df.stick)):
        for p in ("step", "smooth"):
            if ((df.stick == s) & (df.profile == p)).any():
                lines.append(f"| {s} | {p} | " + " | ".join(str(int(counts.get((s, p, d), 0))) for d in dirs) + " |")
    lines += ["", "Step - smooth difference of the medians, per turn direction and averaged over the directions "
              "(the averaged value is not biased by an unequal number of left and right turns per profile; "
              "cells hold only a few trials, so this is descriptive):", "",
              "| stick | metric | pooled | " + " | ".join(dirs) + " | mean over directions |",
              "|---|---|---|" + "---|" * len(dirs) + "---|"]
    for s in stick_order(set(df.stick)):
        ds = df[df.stick == s]
        if not {"step", "smooth"} <= set(ds.profile):
            continue
        for m, lab in DIFF_LABELS:
            pooled = ds[ds.profile == "step"][m].median() - ds[ds.profile == "smooth"][m].median()
            per = []
            for d in dirs:
                dd = ds[ds.direction.astype(str) == d]
                per.append(dd[dd.profile == "step"][m].median() - dd[dd.profile == "smooth"][m].median())
            ok = [x for x in per if pd.notna(x)]
            mean = float(np.mean(ok)) if ok else np.nan
            lines.append(f"| {s} | {lab} | {fmt(pooled)} | " + " | ".join(fmt(x) for x in per) + f" | {fmt(mean)} |")
    return lines


def t6_section(info, df, skipped, partial):
    """PASS/WARN lines for the T6 session checks."""
    lines = ["## T6 checks", ""]
    for tag in ("start", "end"):
        s = info["syncs"].get(tag)
        if s is None:
            lines.append(f"- WARN sync_{tag}: no sync_{tag}/low.csv" + (" (no drift correction)" if tag == "end" else ""))
            continue
        if not s["used"]:
            why = s.get("problem") or (f"{s['robot_knocks']} robot knocks, no consistent phone match"
                                       if s["robot_knocks"] else "no robot knocks found")
            lines.append(f"- WARN sync_{tag}: not used for the clock ({why})")
            continue
        ok = s["matched"] >= 4
        extra = f"; clock drift {info['drift_ms']:.1f} ms" if tag == "end" and info["drift_ms"] is not None else ""
        lines.append(f"- {'PASS' if ok else 'WARN'} sync_{tag}: {s['matched']} knocks matched (>= 4 needed){extra}")
    if info["kind"] == "ntp":
        lines.append("- WARN clock: no knock sync, wall clocks used as-is")
    lines += [f"- WARN clock: {w}" for w in info["warnings"]]
    for col, (lo, hi), unit in (("robot_turn_deg", TURN_OK, "deg"), ("hand_lag_s", HAND_LAG_OK, "s")):
        v = df[col] if len(df) else pd.Series(dtype=float)
        bad = df[~((v >= lo) & (v <= hi))] if len(df) else df
        if len(df) and not len(bad):
            lines.append(f"- PASS {col} within {lo:g}-{hi:g} {unit} in all {plural(len(df), 'analysed trial')}")
        else:
            lst = ", ".join(f"trial {int(r.trial)}: {fmt(r[col], 2)}" for _, r in bad.iterrows()) or "no trials analysed"
            lines.append(f"- WARN {col} outside {lo:g}-{hi:g} {unit}: {lst}")
    if skipped or partial:
        nums = ", ".join(str(k["trial"]) for k in skipped + partial)
        lines.append(f"- WARN {plural(len(skipped), 'trial')} skipped, {len(partial)} partly analysed "
                     f"(see 'Skipped trials'; note them in notes.md): {nums}")
    else:
        lines.append("- PASS no aborted, skipped or excluded trials")
    return lines


def skipped_section(skipped, partial):
    lines = ["## Skipped trials", ""]
    if not skipped and not partial:
        return lines + ["None: every row of trials.csv was analysed.", ""]
    def item(k):
        note = f" (note: {k['note']})" if k["note"] and k["note"] != k["reason"] else ""
        return f"- trial {k['trial']} ({k['stick']}, {k['profile']}, {k['direction']}): {k['reason']}{note}"
    lines += [item(k) for k in skipped] or ["No trial skipped."]
    if partial:
        lines += ["", "Partly analysed (the other metrics are kept):"] + [item(k) for k in partial]
    return lines + [""]


def sync_section(info):
    drift = f", drift {info['drift_ms']:.1f} ms between the syncs" if info["drift_ms"] is not None else ""
    lines = ["## Clock sync", "", f"Model: {info['kind']}{drift}."]
    for tag, s in info["syncs"].items():
        lines.append(f"- sync_{tag}: {s['robot_knocks']} robot knocks, {s['matched']} matched"
                     + (f", offset {s['offset']:.4f} s" if s["offset"] is not None else "")
                     + ("" if s["used"] else " (not used)")
                     + (f"; robot top peaks {s.get('robot_top_peaks')} (threshold {s.get('robot_threshold')} m/s²)"
                        if "robot_top_peaks" in s else "")
                     + (f"; phone top peaks {s.get('phone_top_peaks')}" if "phone_top_peaks" in s else "")
                     + (f"; candidates (matched, offset s, extra peaks) {s.get('candidates')}" if s.get("candidates") else "")
                     + (f"; {s['problem']}" if s.get("problem") else ""))
    return lines + [""]


# ------------------------------------------------------------------ main
def load_trials(session):
    f = os.path.join(session, "trials.csv")
    if not os.path.exists(f):
        sys.exit(f"{f} not found")
    try:
        trials = pd.read_csv(f)
    except pd.errors.EmptyDataError:
        sys.exit(f"{f} is empty (0 bytes). Write the header line: " + ",".join(TRIAL_COLS + ["clarity_1to7", "comfort_1to7", "note"]))
    missing = [c for c in TRIAL_COLS if c not in trials.columns]
    if missing:
        sys.exit(f"{f} has no proper header (missing {missing}). The first line must be: "
                 + ",".join(TRIAL_COLS + ["clarity_1to7", "comfort_1to7", "note"]))
    dup = trials.trial[trials.trial.duplicated()].tolist()
    if dup:
        loud(f"trial numbers repeated in trials.csv: {dup}. A later block may have overwritten those folders; "
             "rows whose times do not match the folder are skipped.")
    return trials


def main(session, robot_knock_floor=1.5):
    out = os.path.join(session, "results")
    os.makedirs(out, exist_ok=True)
    for name in OUTPUTS:                # never mix outputs of an earlier run with this one (results/ is zipped)
        if os.path.exists(os.path.join(out, name)):
            os.remove(os.path.join(out, name))
    trials = load_trials(session)
    phone = load_phone(os.path.join(session, "phone"))
    offset_for, sync = clock_model(session, phone, trials, robot_knock_floor)

    def describe(r, reason):
        return dict(trial=r.trial, stick=r.stick, profile=r.profile, direction=r.direction,
                    reason=reason, note=row_note(r))
    preps, skipped = [], []
    for _, r in trials.iterrows():
        reason = row_problem(r)
        p = None
        if reason is None:
            offset_fn, reason = offset_for(r)
        if reason is None:
            p, reason = prepare_trial(session, r, phone, offset_fn)
        if p is None:
            print(f"trial {r.trial}: skipped ({reason})")
            skipped.append(describe(r, reason))
        else:
            preps.append(p)

    s_gyro, gyro_votes, s_hand, hand_votes = 1.0, (0, 0), 1.0, (0, 0)
    if preps:
        s_gyro, *gyro_votes = session_sign([p["vote_gyro"] for p in preps], "robot yaw rate vs command")
        s_hand, *hand_votes = session_sign([p["vote_hand"] for p in preps], "hand yaw vs robot yaw")
    rows, examples, partial = [], {}, []           # partial: trials analysed with some metrics missing
    for p in preps:
        res, ser = trial_metrics(p, s_gyro, s_hand)
        rows.append(res)
        missing = [p["energy_note"]] if "energy_note" in p else []
        if ser is None:
            missing.append(f"hand metrics excluded: phone data gap of {p['phone_gap']:.2f} s (> {MAX_GAP} s)")
        else:
            examples.setdefault((res["stick"], res["profile"]), []).append((res, ser))
        if missing:                                # one entry per trial, whatever is missing
            print(f"WARNING trial {res['trial']}: {'; '.join(missing)}")
            partial.append(describe(p["row"], "; ".join(missing)))
    df = pd.DataFrame(rows, columns=RESULT_COLS)
    df.to_csv(os.path.join(out, "trials.csv"), index=False)

    src = df.robot_yaw_src.value_counts().to_dict()
    yaw_line = ("Robot yaw rate from " + ", ".join(
        f"{'lowstate (low.csv gz)' if k == 'low' else 'sportmodestate (sport.csv gz)'} in {plural(v, 'trial')}"
        for k, v in src.items()) + ".") if src else ""
    sign_line = (f"Axis signs from the data: robot gyro z vs command {s_gyro:+.0f} ({gyro_votes[0]}/{gyro_votes[1]} "
                 f"trials agree), hand yaw vs robot yaw {s_hand:+.0f} ({hand_votes[0]}/{hand_votes[1]}).")
    for col, what in (("robot_sign_ok", "robot yaw vs command"), ("hand_sign_ok", "hand yaw vs robot yaw")):
        odd = df.trial[df[col] == False].tolist()          # noqa: E712 (NaN = no vote)
        if odd:
            print(f"WARNING: {what} has the opposite sign in trials {odd}; check those trials")
            sign_line += f" Opposite {what} sign in trials {odd}."
    head = f"Clock alignment: {sync['kind']}. Trials analysed: {len(df)}."
    with open(os.path.join(out, "sync.json"), "w") as f:
        json.dump(sync, f, indent=1, default=float)

    if df.empty:
        with open(os.path.join(out, "summary.md"), "w") as f:
            f.write("\n".join([head, ""] + skipped_section(skipped, partial)
                              + t6_section(sync, df, skipped, partial) + [""] + sync_section(sync)))
        print(open(os.path.join(out, "summary.md")).read())
        sys.exit("No trial could be analysed (see the skipped trials above); no summary tables or figures.")

    # representative = trial closest to the group median of hand_jerk_excess
    rep = {}
    for k, lst in examples.items():
        med = np.median([x[0]["hand_jerk_excess"] for x in lst])
        rep[k] = min(lst, key=lambda x: abs(x[0]["hand_jerk_excess"] - med))[1]
    figures = [os.path.join(out, "metrics.png")]
    if rep:
        figures.insert(0, os.path.join(out, "example_trials.png"))
        plot_examples(rep, figures[0])
    else:
        print("example_trials.png not written: no trial has hand data")
    plot_metrics(df, figures[-1])

    has_hand = df[HAND_METRICS].notna().any(axis=1)          # False: phone gap, hand metrics excluded
    g = df.groupby(["stick", "profile"])[SUMMARY_COLS]
    summ = g.median().round(3)
    summ.insert(0, "n", g.size())
    summ["n_hand"] = has_hand.groupby([df.stick, df.profile]).sum()
    q1, q3 = g.quantile(0.25).round(3), g.quantile(0.75).round(3)
    summ.to_csv(os.path.join(out, "summary.csv"))
    gd = df.groupby(["stick", "profile", "direction"])[SUMMARY_COLS]
    sd = gd.median().round(3)
    sd.insert(0, "n", gd.size())
    sd["n_hand"] = has_hand.groupby([df.stick, df.profile, df.direction]).sum()
    sd.to_csv(os.path.join(out, "summary_by_direction.csv"))
    n_bias = int((df.gyro_still_std < GYRO_STILL_STD).sum())
    with open(os.path.join(out, "summary.md"), "w") as f:
        f.write(f"{head} {yaw_line}\n\n")
        f.write("| stick | profile | n | hand jerk added (m/s³) | peak hand accel (m/s²) | hand lag (s) | "
                "hand cue onset (s) | robot lag (s) | joint work above walking (J) | turn (deg) |\n")
        f.write("|---|---|---|---|---|---|---|---|---|---|\n")
        for (s, p), r in summ.iterrows():
            def cell(c):
                return f"{r[c]:.2f} [{q1.loc[(s, p), c]:.2f}–{q3.loc[(s, p), c]:.2f}]"
            n = f"{int(r.n)}" + (f" (hand {int(r.n_hand)})" if r.n_hand != r.n else "")
            f.write(f"| {s} | {p} | {n} | {cell('hand_jerk_excess')} | {cell('hand_peak_ah')} | "
                    f"{cell('hand_lag_s')} | {cell('hand_cue_onset_s')} | {cell('robot_lag_s')} | "
                    f"{cell('mech_excess_J')} | {r.robot_turn_deg:.0f} |\n")
        f.write("\nMedian [IQR]. n = trials per condition ('hand k': only k of them have phone data), one "
                "participant (the author), sighted, eyes open. Exploratory pilot, no inferential statistics.\n"
                f"Lags: difference of the signal centroids over [turn start - {LAG_PRE} s, turn end + {TAIL} s] "
                "(robot vs command, hand vs robot), on baseline-removed yaw rates signed so the turn is positive; "
                "for a linear response they do not depend on the command shape, so hand lag is the delay added "
                "by the stick, not when the hand first feels the turn. Hand cue onset: time from the turn command "
                f"until the hand yaw rate first holds >= {CUE_THR} rad/s for {CUE_HOLD} s; it does depend on the "
                "command shape. 30%-of-peak onset lags are in trials.csv (robot_onset_lag_s, hand_onset_lag_s). "
                f"Jerk window: {JERK_WIN} s from turn start for both profiles. "
                "Joint work above walking: integral of (power - mean power in the 2 s before the turn) over "
                f"turn + {TAIL} s; the absolute integrals (mech_work_J, elec_energy_J in trials.csv) grow with the "
                "window, which is 0.8 s longer for smooth. Turn: signed integral of the yaw rate, minus the gyro "
                "bias of the 1 s 'still' phase only if the robot was still there (yaw-rate std < "
                f"{GYRO_STILL_STD} rad/s): {n_bias} of {plural(len(df), 'trial')} (gyro_bias, gyro_still_std in "
                "trials.csv).\n"
                f"{sign_line}\n\n")
        f.write("\n".join(direction_section(df) + [""] + skipped_section(skipped, partial)
                          + t6_section(sync, df, skipped, partial) + [""] + sync_section(sync)))
    print(open(os.path.join(out, "summary.md")).read())
    print("figures:", *figures)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Analyse a Go2 guide-stick pilot session.")
    ap.add_argument("session", nargs="?", default="data/session1", help="session folder, e.g. data/s1")
    ap.add_argument("--robot-knock-floor", type=float, default=1.5, metavar="M_S2",
                    help="smallest knock peak accepted on the robot IMU in m/s² (default 1.5); the console "
                         "prints the top-5 peak heights of each sync to help choose it")
    a = ap.parse_args()
    main(a.session, a.robot_knock_floor)
