#!/usr/bin/env python3
"""
README figures and post-hoc turn-onset metrics for the Go2 guide-stick pilot.

    python3 make_figures.py data/s1            (writes figures/ next to this script)

Post hoc: these metrics were defined after looking at the session data, so they are
exploratory. The pre-registered metrics are the ones in analyze_pilot.py.

Self-contained (numpy, pandas, scipy, matplotlib). It reads only the raw session files:
trials.csv, trial_NNN/{cmd,low,sport}.csv, sync_start/ and sync_end/low.csv, phone/*.csv.

Signals (all turn-aligned at the turn command, t = 0):
  command : commanded yaw rate wz
  robot   : robot body yaw rate (IMU gyro z, lowstate by default)
  hand    : phone gyro projected on the gravity 'up' axis = yaw rate of the handle
Each is baseline-corrected (median of the 2 s of straight walking before the turn),
signed so the turn is positive, and low-passed at 3 Hz (zero-phase Butterworth).

Metrics, in the same window for both profiles (turn command to +4.12 s):
  onset_acc : peak of d/dt(yaw rate), rad/s^2, how sharply the turn starts
  t50       : first time the yaw rate reaches 50 % of its peak in the window, s
  walk_acc  : the onset_acc measure during straight walking, 2.0 to 0.5 s before the turn
              (gait sway level; 1.5 s window, so if anything an underestimate)
"""
import argparse, glob, os, sys
import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt, find_peaks

FS = 100.0
WIN = 4.12                      # shortest commanded turn (2.62 s) + 1.5 s
GRID = np.arange(-2.0, 5.0, 1 / FS)
FC = 3.0                        # low-pass for the turn cue, Hz

# colors: categorical slots 1-2 (validated light-mode pair); text stays in ink colors
STEP, SMOOTH = "#2a78d6", "#eb6834"
INK, INK2, MUTED, GRID_C, SURFACE = "#0b0b0b", "#52514e", "#8a8984", "#e4e3df", "#ffffff"
COL = {"step": STEP, "smooth": SMOOTH}


def lp(x, fc, fs=FS, order=2):
    b, a = butter(order, fc / (fs / 2))
    return filtfilt(b, a, x, axis=0)


# ------------------------------------------------------------------ phone
def _read_xyz(path):
    df = pd.read_csv(path)
    t = df["time"].to_numpy(float) / 1e9 if df["time"].iloc[0] > 1e15 else df["time"].to_numpy(float)
    return t, df[["x", "y", "z"]].to_numpy(float)


def load_phone(folder):
    ta, a = _read_xyz(os.path.join(folder, "Accelerometer.csv"))
    tg, g = _read_xyz(os.path.join(folder, "Gyroscope.csv"))
    tr, gr = _read_xyz(os.path.join(folder, "Gravity.csv"))
    t = np.arange(max(ta[0], tg[0], tr[0]), min(ta[-1], tg[-1], tr[-1]), 1 / FS)
    res = lambda tt, x: np.column_stack([np.interp(t, tt, x[:, i]) for i in range(3)])
    A, G, GR = res(ta, a), res(tg, g), res(tr, gr)
    if np.median(np.linalg.norm(GR, axis=1)) < 2.0:          # iOS: units of g
        A, GR = A * 9.81, GR * 9.81
    up = lp(GR, 1.0)
    up /= np.linalg.norm(up, axis=1, keepdims=True)
    return {"t": t, "acc": A, "gyr": G, "up": up}


# ------------------------------------------------------------------ clock sync (knocks)
def knocks(t, acc, fs, floor):
    mag = np.linalg.norm(acc, axis=1)
    hp = np.abs(mag - lp(mag, 2.0, fs))
    thr = np.median(hp) + 8 * (np.median(np.abs(hp - np.median(hp))) + 1e-6)
    idx, _ = find_peaks(hp, height=max(thr, floor), distance=int(0.25 * fs))
    return t[idx]


def match(robot_k, phone_k, prefer, tol=0.06):
    """offset with phone_time = robot_time + offset that matches the most knocks"""
    cands = []
    for r in robot_k:
        for p in phone_k:
            off = p - r
            n = sum(np.min(np.abs(phone_k - (x + off))) < tol for x in robot_k)
            cands.append((n, off))
    n = max(c[0] for c in cands)
    offs = [c[1] for c in cands if c[0] == n]
    off = min(offs) if prefer == "early" else max(offs)
    d = [phone_k[np.argmin(np.abs(phone_k - (x + off)))] - x for x in robot_k
         if np.min(np.abs(phone_k - (x + off))) < tol]
    return float(np.median(d)), n


def clock_model(session, phone, robot_floor=0.5):
    pk = knocks(phone["t"], phone["acc"], FS, floor=1.5)
    pts = []
    for tag, prefer in (("start", "early"), ("end", "late")):
        low = pd.read_csv(os.path.join(session, f"sync_{tag}", "low.csv"))
        fs = 1 / np.median(np.diff(low.t))
        rk = knocks(low.t.to_numpy(), low[["ax", "ay", "az"]].to_numpy(), fs, floor=robot_floor)
        off, n = match(rk, pk, prefer)
        print(f"sync_{tag}: {len(rk)} robot knocks, {n} matched, offset {off:.4f} s")
        if n < 4:
            sys.exit(f"sync_{tag}: only {n} knocks matched")
        pts.append((float(np.median(rk)), off))
    (t1, o1), (t2, o2) = pts
    print(f"clock drift {1000 * (o2 - o1):.1f} ms over {(t2 - t1) / 60:.1f} min")
    return lambda tr: o1 + (np.asarray(tr, float) - t1) * (o2 - o1) / (t2 - t1)


# ------------------------------------------------------------------ signals and metrics
def t50(x, win):
    pk = np.max(x[win])
    return float(GRID[np.argmax(win & (x >= 0.5 * pk))])


def onset_acc(x, win):
    return float(np.max(np.gradient(x, 1 / FS)[win]))


def trial_signals(session, row, phone, offset, gyro):
    d = os.path.join(session, f"trial_{int(row.trial):03d}")
    cmd = pd.read_csv(os.path.join(d, "cmd.csv"))
    rob = pd.read_csv(os.path.join(d, f"{gyro}.csv"))
    t = float(row.t_turn_start) + GRID
    wz = np.interp(t, cmd.t, cmd.wz, left=0, right=0)
    gz = np.interp(t, rob.t, rob.gz)
    tp = t + offset(t)
    gyr = np.column_stack([np.interp(tp, phone["t"], phone["gyr"][:, i]) for i in range(3)])
    up = np.column_stack([np.interp(tp, phone["t"], phone["up"][:, i]) for i in range(3)])
    return wz, gz, np.sum(gyr * up, axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session", nargs="?", default="data/s1")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "figures"))
    ap.add_argument("--gyro", choices=("low", "sport"), default="low",
                    help="robot yaw rate source: lowstate (default, as analyze_pilot.py) or sportmodestate")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    phone = load_phone(os.path.join(a.session, "phone"))
    offset = clock_model(a.session, phone)
    trials = pd.read_csv(os.path.join(a.session, "trials.csv"))
    raw = [(r, *trial_signals(a.session, r, phone, offset, a.gyro)) for _, r in trials.iterrows()]

    # hand yaw sign: one global sign, from the correlation with the robot yaw rate
    pre = GRID < 0
    win = (GRID >= 0) & (GRID < WIN)
    walk = (GRID >= -2.0) & (GRID < -0.5)       # away from the filter's edge at t = 0
    cors = [np.corrcoef(lp(h, FC)[GRID > -1], lp(g, FC)[GRID > -1])[0, 1] for _, _, g, h in raw]
    hs = -1.0 if np.sum(np.sign(cors)) < 0 else 1.0
    agree = int(np.sum(np.sign(cors) == np.sign(hs)))
    print(f"hand yaw sign vs robot: {hs:+.0f} ({agree}/{len(cors)} trials agree)")

    rows, curves = [], {"step": [], "smooth": []}
    for r, wz, gz, hy in raw:
        sg = np.sign(np.sum(wz)) or 1.0
        c = sg * wz
        g = sg * lp(gz - np.median(gz[pre]), FC)
        h = sg * hs * lp(hy - np.median(hy[pre]), FC)
        curves[r.profile].append((c, g, h))
        rows.append(dict(trial=int(r.trial), profile=r.profile, direction=r.direction,
                         robot_onset_acc=onset_acc(g, win), hand_onset_acc=onset_acc(h, win),
                         robot_walk_acc=onset_acc(g, walk), hand_walk_acc=onset_acc(h, walk),
                         cmd_t50=t50(c, win), robot_t50=t50(g, win), hand_t50=t50(h, win),
                         clarity_1to7=r.get("clarity_1to7"), comfort_1to7=r.get("comfort_1to7")))
    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(a.out, "posthoc_metrics.csv"), index=False, float_format="%.4f")

    # ------------------------------------------------------------ text summary
    cols = ["robot_onset_acc", "robot_walk_acc", "hand_onset_acc", "hand_walk_acc",
            "cmd_t50", "robot_t50", "hand_t50",
            "clarity_1to7", "comfort_1to7"]
    lines = [f"Post-hoc turn-onset metrics ({a.gyro} gyro for the robot), median [IQR], n per profile:"]
    for m in cols:
        cell = []
        for p in ("step", "smooth"):
            v = df[df.profile == p][m].dropna()
            cell.append(f"{p} {v.median():.2f} [{v.quantile(.25):.2f}-{v.quantile(.75):.2f}] (n={len(v)})")
        lines.append(f"  {m:16s} " + " | ".join(cell))
    by_dir = df.groupby(["direction", "profile"])[["robot_onset_acc", "hand_onset_acc", "hand_t50"]].median()
    lines.append("By turn direction (medians of 4 trials):\n" + by_dir.round(2).to_string())
    ens = {}
    for p, L in curves.items():
        m = [np.mean([c[i] for c in L], axis=0) for i in range(3)]
        ens[p] = m
        lines.append(f"Ensemble mean {p:6s}: t50 command {t50(m[0], win):.2f} s, robot {t50(m[1], win):.2f} s, "
                     f"hand {t50(m[2], win):.2f} s | onset acc robot {onset_acc(m[1], win):.2f}, "
                     f"hand {onset_acc(m[2], win):.2f} rad/s^2")
    txt = "\n".join(lines)
    print(txt)
    with open(os.path.join(a.out, "posthoc_summary.txt"), "w") as f:
        f.write(txt + "\n")
    pd.DataFrame({"t_s": GRID, **{f"{p}_{k}": ens[p][i] for p in ens for i, k in
                                   enumerate(("cmd", "robot", "hand"))}}).to_csv(
        os.path.join(a.out, "ensemble_mean_curves.csv"), index=False, float_format="%.4f")

    plot_curves(curves, ens, win, os.path.join(a.out, "fig1_yaw_rate.png"))
    plot_onsets(df, os.path.join(a.out, "fig2_turn_onset.png"))
    print("written:", a.out)


# ------------------------------------------------------------------ plots
def style(ax):
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(MUTED)
    ax.tick_params(colors=INK2, labelsize=9, length=3)
    ax.grid(axis="y", color=GRID_C, lw=0.8)
    ax.set_axisbelow(True)


def plot_curves(curves, ens, win, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    titles = ("Command", "Robot body", "Hand (phone on the handle)")
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.6), sharey=True, facecolor=SURFACE)
    for i, ax in enumerate(axes):
        style(ax)
        ax.axvline(0, color=MUTED, lw=0.8, ls=":")
        for p in ("step", "smooth"):
            for c in curves[p]:
                if i > 0:
                    ax.plot(GRID, c[i], color=COL[p], lw=0.6, alpha=0.25)
            ax.plot(GRID, ens[p][i], color=COL[p], lw=2.2, label=f"{p} (mean of {len(curves[p])})")
            if i > 0:
                t = t50(ens[p][i], win)
                y = np.interp(t, GRID, ens[p][i])
                ax.plot([t], [y], "o", ms=5, color=COL[p], mec=SURFACE, mew=1.2, zorder=5)
        ax.set_title(titles[i], loc="left", fontsize=10.5, color=INK)
        ax.set_xlim(-1.0, 4.6)
        ax.set_ylim(-0.35, 1.0)
        ax.set_xlabel("time from turn command (s)", fontsize=9, color=INK2)
    axes[0].set_ylabel("yaw rate in the turn direction (rad/s)", fontsize=9, color=INK2)
    # delay of the smooth profile at half of the peak (dots), written in empty space
    for i in (1, 2):
        d = t50(ens["smooth"][i], win) - t50(ens["step"][i], win)
        axes[i].text(-0.9, 0.98, f"smooth reaches half of the peak\n{d:.2f} s after step (dots)",
                     ha="left", va="top", fontsize=8.5, color=INK2, linespacing=1.3)
    axes[0].legend(frameon=False, fontsize=8.5, loc="upper right", labelcolor=INK2)
    fig.text(0.01, 0.005, "16 trials, one participant (the author). Thin lines: single trials. "
             "Thick lines: mean of 8. Low-passed at 3 Hz; baseline = straight walking before the turn.",
             fontsize=8, color=INK2)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(out, dpi=200, facecolor=SURFACE)
    plt.close(fig)


def plot_onsets(df, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    panels = [("robot_onset_acc", "Robot body:\nturn-onset yaw acceleration (rad/s²)"),
              ("hand_onset_acc", "Hand:\nturn-onset yaw acceleration (rad/s²)"),
              ("hand_t50", "Hand:\ntime to half of the peak yaw rate (s)")]
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.5), facecolor=SURFACE)
    rng = np.random.default_rng(0)
    for ax, (m, lab) in zip(axes, panels):
        style(ax)
        for k, p in enumerate(("step", "smooth")):
            sub = df[df.profile == p]
            x = k + rng.uniform(-0.1, 0.1, len(sub))
            for xi, (_, r) in zip(x, sub.iterrows()):
                ax.scatter(xi, r[m], s=34, color=COL[p], marker="o" if r.direction == "L" else "^",
                           edgecolor=SURFACE, lw=0.8, zorder=3)
            med = sub[m].median()
            ax.plot([k - 0.25, k + 0.25], [med, med], color=INK, lw=2, zorder=4)
            ax.text(k + 0.29, med, f"{med:.2f}", va="center", fontsize=8.5, color=INK)
        wcol = m.replace("onset", "walk") if "onset" in m else None
        if wcol:                                   # gait-sway reference, same y-scale for both
            w = df[wcol].median()
            ax.axhline(w, color=MUTED, lw=1.2, ls=(0, (4, 3)), zorder=2)
            ax.text(1.68, w - 0.06, f"walking: {w:.2f}", ha="right", va="top", fontsize=8,
                    color=INK2)
            ax.set_ylim(0, 1.08 * max(df.robot_onset_acc.max(), df.hand_onset_acc.max()))
        ax.set_xticks([0, 1])
        ax.set_xticklabels(["step", "smooth"], fontsize=9.5, color=INK2)
        ax.set_xlim(-0.5, 1.7)
        if not wcol:
            ax.set_ylim(0, 1.1 * df[m].max())
        ax.set_title(lab, loc="left", fontsize=10, color=INK)
    fig.text(0.01, 0.005, "Each marker is one trial (circle = left turn, triangle = right turn); "
             "bar and number = median of 8 trials. Dashed line: the same measure during straight "
             "walking before the turn. Post-hoc metrics.", fontsize=8, color=INK2)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(out, dpi=200, facecolor=SURFACE)
    plt.close(fig)


if __name__ == "__main__":
    main()
