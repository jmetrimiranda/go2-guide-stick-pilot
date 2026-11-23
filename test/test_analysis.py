#!/usr/bin/env python3
"""Offline tests of analyze_pilot.py on simulated sessions (no robot, no network, no pytest).

  .venv/bin/python test/test_analysis.py           (about a minute; sessions go to a fresh /tmp dir)
  .venv/bin/python test/test_analysis.py --keep    (keep the generated sessions for inspection)

Every scenario is written by test/simulate_session.py and analysed through the CLI, as Jorge runs it.
"""
import ast, json, os, shutil, subprocess, sys, tempfile, tokenize, traceback
import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import go2_turn_pilot as P                               # stdlib only at import time (the SDK is imported lazily)
SIM, ANA = os.path.join(HERE, "simulate_session.py"), os.path.join(ROOT, "analyze_pilot.py")
TMP = tempfile.mkdtemp(prefix="go2_analysis_test_")
ENV = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")     # leave no __pycache__ behind in the repo
OFF, DRIFT, T0 = 37.42, 2e-5, 1795000000.0              # clock truth used by the simulator
OUTPUTS = ("trials.csv", "summary.csv", "summary.md", "example_trials.png", "metrics.png",
           "summary_by_direction.csv", "sync.json", "ensemble_curves.csv")
ORIG_TRIAL_COLS = ["trial", "stick", "profile", "direction", "robot_turn_deg", "robot_lag_s",
                   "robot_peak_yaw_acc", "hand_lag_s", "hand_peak_yaw", "hand_peak_ah", "hand_jerk_turn",
                   "hand_jerk_pre", "mech_work_J", "elec_energy_J", "clarity", "comfort", "hand_jerk_excess"]
ORIG_SUMMARY_COLS = ["stick", "profile", "n", "robot_turn_deg", "robot_lag_s", "hand_lag_s", "hand_peak_yaw",
                     "hand_peak_ah", "hand_jerk_excess", "mech_work_J", "elec_energy_J"]
HAND = ["hand_lag_s", "hand_onset_lag_s", "hand_cue_onset_s", "hand_peak_yaw", "hand_peak_ah", "hand_jerk_turn",
        "hand_jerk_pre", "hand_jerk_excess", "hand_sway_pre", "hand_yaw_acc_onset", "hand_t50_s"]
HAND_LAG_TRUTH = {"rigid": 0.205, "flexible": 0.59}     # simulated mean hand delay: delay + tau + tau/2
ROBOT = ["robot_turn_deg", "robot_lag_s", "robot_onset_lag_s", "robot_peak_yaw_acc", "mech_work_J",
         "elec_energy_J", "mech_excess_J", "elec_excess_J", "robot_yaw_acc_onset", "cmd_t50_s", "robot_t50_s"]

_sessions = {}


def run(*cmd):
    return subprocess.run([sys.executable, *cmd], capture_output=True, text=True, env=ENV)


def session(*flags):
    """Simulate a scenario (once per flag set) and analyse it. Returns (folder, analysis process)."""
    if flags not in _sessions:
        d = os.path.join(TMP, "s%02d_" % len(_sessions) + ("_".join(f.strip("-") for f in flags) or "default")
                         .replace(":", "-").replace(",", "-"))
        sim = run(SIM, d, *flags)
        assert sim.returncode == 0, sim.stderr
        _sessions[flags] = (d, run(ANA, d))
    return _sessions[flags]


def copy_session(flags, name):
    """Fresh copy (without results) of a simulated scenario, for mutation tests."""
    src, _ = session(*flags)
    dst = os.path.join(TMP, name)
    shutil.copytree(src, dst, ignore=shutil.ignore_patterns("results"))
    return dst


def ok(proc):
    assert proc.returncode == 0 and "Traceback" not in proc.stderr, proc.stdout[-3000:] + proc.stderr[-3000:]


def results(d, name="trials.csv"):
    return pd.read_csv(os.path.join(d, "results", name))


def summary_md(d):
    with open(os.path.join(d, "results", "summary.md")) as f:
        return f.read()


def sync_info(d):
    with open(os.path.join(d, "results", "sync.json")) as f:
        return json.load(f)


def offset_error_ms(d, tag):
    s = sync_info(d)["syncs"][tag]
    return abs(s["offset"] - (OFF + DRIFT * (s["t_robot"] - T0))) * 1000


def step_minus_smooth(d, col):
    m = results(d).groupby(["stick", "profile"])[col].median()
    return {s: m[(s, "step")] - m[(s, "smooth")] for s in m.index.get_level_values(0).unique()}


def same(a, b, cols, atol=1e-9):
    for c in cols:
        x, y = a[c].to_numpy(float), b[c].to_numpy(float)
        assert np.allclose(x, y, rtol=0, atol=atol, equal_nan=True), (c, np.nanmax(np.abs(x - y)))


# ------------------------------------------------------------------ tests
def test_default_pipeline():
    d, p = session()
    ok(p)
    for f in OUTPUTS:
        assert os.path.getsize(os.path.join(d, "results", f)) > 0, f
    info = sync_info(d)
    assert info["kind"] == "knock (start+end)", info["kind"]
    assert offset_error_ms(d, "start") < 5, offset_error_ms(d, "start")
    assert offset_error_ms(d, "end") < 10, offset_error_ms(d, "end")       # phone sampled at 100 Hz
    df = results(d)
    assert len(df) == 20 and list(df.columns[:len(ORIG_TRIAL_COLS)]) == ORIG_TRIAL_COLS
    summ = results(d, "summary.csv")
    assert list(summ.columns[:len(ORIG_SUMMARY_COLS)]) == ORIG_SUMMARY_COLS, list(summ.columns)
    assert {"mech_excess_J", "elec_excess_J"} <= set(summ.columns)
    md = summary_md(d)
    for text in ("Clock alignment: knock (start+end). Trials analysed: 20.", "## T6 checks",
                 "## Profile x direction", "| stick | profile | L | R |", "mean over directions",
                 "## Skipped trials", "Robot yaw rate from lowstate"):
        assert text in md, text
    t6 = md.split("## T6 checks")[1].split("##")[0]
    assert "WARN" not in t6 and t6.count("PASS") == 5, t6
    # centroid lags recover the simulated mean delays (robot 0.08 + 0.12 s; hand delay + tau + tau/2)
    m = df.groupby("stick")[["robot_lag_s", "hand_lag_s"]].median()
    assert abs(m.robot_lag_s.rigid - 0.20) < 0.02 and abs(m.robot_lag_s.flexible - 0.20) < 0.02, m
    assert abs(m.hand_lag_s.rigid - 0.205) < 0.03 and abs(m.hand_lag_s.flexible - 0.59) < 0.04, m
    # cue onset (hand holds 0.1 rad/s) depends on the command shape; values of the noise-free simulation
    cue = df.groupby(["stick", "profile"]).hand_cue_onset_s.median()
    truth = {("rigid", "step"): 0.26, ("rigid", "smooth"): 0.58, ("flexible", "step"): 0.54,
             ("flexible", "smooth"): 0.89}
    for k, v in truth.items():
        assert abs(cue[k] - v) < 0.08, (k, cue[k])
    assert "| hand cue onset (s) |" in md and "hand_cue_onset_s" in summ.columns


def test_turn_deg_both_profiles():
    """The simulated robot sways while 'still', so no gyro bias may be subtracted: a 1 s mean of the sway
    would give L and R turns opposite errors and a spurious step - smooth difference (unequal L/R counts)."""
    d, _ = session()
    df = results(d)
    assert df.robot_turn_deg.between(89, 91).all(), df.robot_turn_deg.tolist()
    assert (df.gyro_bias == 0).all() and (df.gyro_still_std > 0.02).all()
    for stick, diff in step_minus_smooth(d, "robot_turn_deg").items():
        assert abs(diff) < 0.5, (stick, diff)
    lr = df.groupby("direction").robot_turn_deg.median()
    assert abs(lr.L - lr.R) < 0.5, lr


def test_gyro_bias_removed_when_still():
    d, p = session("--gyro-bias", "0.03", "--still-quiet")
    ok(p)
    df = results(d)
    assert df.robot_turn_deg.between(89, 91).all(), df.robot_turn_deg.tolist()
    assert np.allclose(df.gyro_bias, 0.03, atol=0.003), df.gyro_bias.tolist()
    assert "(yaw-rate std < 0.02 rad/s): 20 of 20 trials" in summary_md(d)


def test_equal_dynamics_no_profile_bias():
    """Same robot/hand dynamics and turn cost for both profiles: step - smooth must be ~0 on the
    corrected metrics, while the absolute joint work still shows the 0.8 s window bias."""
    d, p = session("--equal-dynamics", "--balanced", "--reps", "6")
    ok(p)
    # the peaky gait power (random phase per trial) gives the excess work ~5 J of per-trial noise, so a median
    # of 6 trials moves by a few J; the window bias these metrics remove is ~45 J (mech) and ~110 J (elec)
    tol = {"robot_lag_s": 0.02, "hand_lag_s": 0.03, "mech_excess_J": 8.0, "elec_excess_J": 8.0,
           "robot_turn_deg": 1.0}
    for col, t in tol.items():
        for stick, diff in step_minus_smooth(d, col).items():
            assert abs(diff) < t, (col, stick, diff)
    for stick, diff in step_minus_smooth(d, "mech_work_J").items():
        assert diff < -25, ("window bias no longer visible in mech_work_J", stick, diff)
    for stick, diff in step_minus_smooth(d, "elec_energy_J").items():
        assert diff < -50, ("window bias no longer visible in elec_energy_J", stick, diff)


def test_gyro_sign_inverted_same_metrics():
    d0, _ = session()
    d1, p = session("--gyro-sign", "-1")
    ok(p)
    a, b = results(d0), results(d1)
    same(a, b, HAND + ROBOT)
    same(a.assign(g=-a.gyro_bias), b.assign(g=b.gyro_bias), ["g", "gyro_still_std"])
    assert "robot gyro z vs command -1" in summary_md(d1)


def test_ios_sign_convention_same_metrics():
    """iOS negates acceleration and gravity, so gyro . up flips: the hand sign must come from the data."""
    d0, _ = session()
    d1, p = session("--ios")
    ok(p)
    same(results(d0), results(d1), HAND + ROBOT)
    assert "hand yaw vs robot yaw -1" in summary_md(d1)


def test_no_gravity_exits_clearly():
    _, p = session("--no-gravity")
    assert p.returncode != 0 and "Traceback" not in p.stderr, p.stderr
    assert "Enable Gravity in Sensor Logger" in p.stderr, p.stderr


def test_total_acceleration_instead_of_gravity():
    d0, _ = session()
    d1, p = session("--total-accel")
    ok(p)
    assert "no Gravity.csv; the vertical is estimated by low-passing TotalAcceleration.csv" in p.stdout
    a, b = results(d0), results(d1)
    assert len(b) == 20
    same(a, b, ROBOT)
    same(a, b, ["hand_lag_s"], atol=0.01)


def test_g_units():
    d0, _ = session()
    d1, p = session("--g-units")
    ok(p)
    same(results(d0), results(d1), HAND + ROBOT, atol=1e-6)


def test_no_sync_end_single():
    d, p = session("--no-sync-end")
    ok(p)
    assert sync_info(d)["kind"] == "knock (single)"
    assert offset_error_ms(d, "start") < 5
    md = summary_md(d)
    assert "Clock alignment: knock (single). Trials analysed: 20." in md
    assert "WARN sync_end: no sync_end/low.csv" in md


def test_phone_record_misses_end_sync():
    """sync_end exists on the robot but the phone stopped recording before it (old code: slope -1)."""
    d, p = session("--phone-cut-end")
    ok(p)
    assert sync_info(d)["kind"] == "knock (single)" and offset_error_ms(d, "start") < 5
    assert "start and end knock syncs disagree" in p.stdout
    assert len(results(d)) == 20 and "WARN sync_end: not used for the clock" in summary_md(d)


def test_phone_gap_only_that_trial():
    d0, _ = session()
    d1, p = session("--gap", "5:2.5")
    ok(p)
    a, b = results(d0), results(d1)
    gap = b.trial == 5
    assert b.loc[gap, HAND].isna().all(axis=None), b.loc[gap, HAND]
    assert b.loc[~gap, HAND].notna().all(axis=None)
    same(a, b, ROBOT)                                   # robot metrics of every trial are kept
    same(a[~gap], b[~gap], HAND)                        # other trials untouched
    assert b.loc[gap, "phone_max_gap_s"].iloc[0] > 2.4
    md = summary_md(d1)
    assert "trial 5 (" in md and "phone data gap" in md
    summ = results(d1, "summary.csv").set_index(["stick", "profile"])
    assert summ.loc[("rigid", "smooth"), "n"] == 5 and summ.loc[("rigid", "smooth"), "n_hand"] == 4
    assert "| rigid | smooth | 5 (hand 4) |" in md and "| rigid | step | 5 |" in md


def test_posthoc_metrics():
    """POST-HOC metrics (defined on 23 Nov 2026 after the s1 data had been seen): 50 % rise times of the
    3 Hz command, robot and hand yaw rates, signed onset yaw acceleration, ensemble curves."""
    d, p = session()
    ok(p)
    df = results(d)
    g = df.groupby("profile")[["cmd_t50_s", "robot_t50_s", "hand_t50_s", "robot_yaw_acc_onset",
                               "hand_yaw_acc_onset"]].median()
    # zero-phase filter: the step command crosses 50 % at ~0 s, the 0.8 s cosine ramp at ~0.4 s
    assert abs(g.cmd_t50_s.step) < 0.03 and abs(g.cmd_t50_s.smooth - 0.40) < 0.03, g
    assert g.robot_t50_s.step < g.robot_t50_s.smooth and g.hand_t50_s.step < g.hand_t50_s.smooth, g
    assert (g.robot_t50_s > g.cmd_t50_s).all() and (g.hand_t50_s > g.robot_t50_s).all(), g
    # signed maximum: positive (the turn starting), and larger for the step than for the ramp
    assert (df[["robot_yaw_acc_onset", "hand_yaw_acc_onset"]] > 0).all(axis=None)
    assert g.robot_yaw_acc_onset.step > g.robot_yaw_acc_onset.smooth, g
    e = results(d, "ensemble_curves.csv")
    groups = df.groupby(["stick", "profile"]).size()
    assert len(e) == 701 * len(groups) and e.t_s.min() == -2.0 and e.t_s.max() == 5.0, (len(e), groups)
    for (s, prof), n in groups.items():
        ge = e[(e.stick == s) & (e.profile == prof)].set_index("t_s")
        assert (ge.n_trials == n).all() and (ge.n_hand == n).all()
        assert abs(ge.cmd_median[1.5] - 0.6) < 1e-6 and abs(ge.cmd_median[-1.0]) < 1e-9
        assert (ge.robot_q25 <= ge.robot_median + 1e-9).all() and (ge.robot_median <= ge.robot_q75 + 1e-9).all()
    md = summary_md(d)
    assert "## Post-hoc" in md and "after the s1 data had been seen" in md and "| rigid | step |" in md.split("## Post-hoc")[1]


def test_spurious_knocks_and_missed_knock():
    d0, _ = session()
    d, p = session("--spurious-knocks", "--drop-phone-knock")
    ok(p)
    info = sync_info(d)
    assert info["kind"] == "knock (start+end)"
    assert info["syncs"]["start"]["matched"] == 4 and info["syncs"]["end"]["matched"] == 5, info["syncs"]
    assert offset_error_ms(d, "start") < 5 and offset_error_ms(d, "end") < 10
    same(results(d0), results(d), ["hand_lag_s", "robot_lag_s", "robot_turn_deg"], atol=0.02)


def test_aborted_and_skipped_rows_listed():
    d, p = session("--aborted-row")
    ok(p)
    md = summary_md(d)
    assert "Trials analysed: 20" in md
    assert "trial 21 (flexible, step, L): aborted: state stream stale" in md, md
    assert "trial 22 (flexible, step, L): skipped" in md
    assert "WARN 2 trials skipped" in md and "21, 22" in md
    assert "trial 21: skipped (aborted: state stream stale)" in p.stdout


def test_rigid_only_reps8():
    """Pilot design with the rigid stick only, in the order go2_turn_pilot.py run uses."""
    d, p = session("--sticks", "rigid", "--reps", "8", "--balanced")
    ok(p)
    for f in OUTPUTS:
        assert os.path.getsize(os.path.join(d, "results", f)) > 0, f
    summ = results(d, "summary.csv")
    assert set(summ.stick) == {"rigid"} and summ.n.tolist() == [8, 8], summ
    df = results(d)
    assert len(df) == 16 and df.profile.tolist() == P.block_order(8, 7)[0]
    assert (pd.crosstab(df.profile, df.direction) == 4).all(axis=None)


def test_other_stick_label_plots():
    d, p = session("--sticks", "test", "--reps", "1")
    ok(p)
    for f in ("metrics.png", "example_trials.png"):
        assert os.path.getsize(os.path.join(d, "results", f)) > 20000, f      # not an empty figure
    assert "| test | step | 1 |" in summary_md(d)


def test_direction_balanced_difference():
    """Right turns get 15% more robot yaw and the design is unbalanced (step 2L/3R, smooth 3L/2R):
    the pooled step - smooth turn difference is spurious, the within-direction mean is ~0."""
    d, p = session("--equal-dynamics", "--lr-asym", "0.15", "--sticks", "rigid")
    ok(p)
    pooled = step_minus_smooth(d, "robot_turn_deg")["rigid"]
    sd = results(d, "summary_by_direction.csv").set_index(["profile", "direction"])
    within = np.mean([sd.loc[("step", x), "robot_turn_deg"] - sd.loc[("smooth", x), "robot_turn_deg"]
                      for x in ("L", "R")])
    assert pooled > 5 and abs(within) < 1.0, (pooled, within)
    assert f"| rigid | turn (deg) | {pooled:.2f} |" in summary_md(d)


def test_graceful_failures():
    # header-only sync_end, empty low.csv in one trial, empty cmd.csv in another
    d = copy_session((), "mut_partial")
    for rel in ("sync_end/low.csv", "trial_003/low.csv", "trial_006/cmd.csv"):
        f = os.path.join(d, rel)
        with open(f) as fh:
            head = fh.readline()
        with open(f, "w") as fh:
            fh.write(head)
    p = run(ANA, d)
    ok(p)
    assert "sync_end: low.csv has 0 rows" in p.stdout and sync_info(d)["kind"] == "knock (single)"
    df = results(d).set_index("trial")
    assert 6 not in df.index and len(df) == 19
    assert df.loc[3, "robot_yaw_src"] == "sport" and np.isnan(df.loc[3, "mech_excess_J"])
    assert 85 < df.loc[3, "robot_turn_deg"] < 95
    md = summary_md(d)
    assert "trial 6 (" in md and "cmd.csv missing or empty" in md and "energy metrics missing" in md

    # nothing inside the phone record: clean exit, but summary.md and trials.csv are still written, and
    # the outputs of an earlier run are gone (results/ is zipped)
    d = copy_session((), "mut_no_trials")
    shutil.rmtree(os.path.join(d, "sync_end"))
    os.makedirs(os.path.join(d, "results"))
    for f in OUTPUTS:
        open(os.path.join(d, "results", f), "w").close()
    for f in os.listdir(os.path.join(d, "phone")):
        path = os.path.join(d, "phone", f)
        with open(path) as fh:
            lines = fh.readlines()[:2500]
        with open(path, "w") as fh:
            fh.writelines(lines)
    p = run(ANA, d)
    assert p.returncode == 1 and "Traceback" not in p.stderr and "No trial could be analysed" in p.stderr
    assert "outside the phone recording" in summary_md(d) and len(results(d)) == 0
    assert sorted(os.listdir(os.path.join(d, "results"))) == ["summary.md", "sync.json", "trials.csv"]

    # 0-byte trials.csv (process killed before the header was flushed)
    d = copy_session((), "mut_empty_trials")
    open(os.path.join(d, "trials.csv"), "w").close()
    p = run(ANA, d)
    assert p.returncode == 1 and "Traceback" not in p.stderr and "is empty (0 bytes)" in p.stderr


def test_clock_step_splits_the_session():
    """The robot clock steps +0.45 s before trial 11 (the pilot notes it), and the phone missed a start knock,
    so a group-shifted start candidate could pair with the end sync at a 'plausible' drift."""
    flags = ("--clock-step", "11:0.45", "--drop-phone-knock")
    d, p = session(*flags)
    ok(p)
    info, df = sync_info(d), results(d)
    assert info["kind"] == "knock (split at clock step)", info["kind"]
    assert 11 not in df.trial.tolist() and len(df) == 19
    err = (df.hand_lag_s - df.stick.map(HAND_LAG_TRUTH)).abs()
    assert err.max() < 0.05, df[["trial", "hand_lag_s"]]
    md = summary_md(d)
    assert "WARN clock: the robot clock stepped during the session" in md
    assert "trial 11 (flexible, step, L): robot clock stepped" in md

    # no note (step outside the trial loop): meta.json still shows it; one sync, loud T6 warning
    d = copy_session(flags, "clock_step_no_note")
    tr = pd.read_csv(os.path.join(d, "trials.csv"))
    tr["note"] = ""
    tr.to_csv(os.path.join(d, "trials.csv"), index=False)
    ok(run(ANA, d))
    assert sync_info(d)["kind"] == "knock (single)"
    assert "WARN clock: the robot clock stepped by +0.450 s between sync_start and sync_end" in summary_md(d)

    # and without meta.json (older pilot): never a start+end line through shifted knocks
    for tag in ("start", "end"):
        os.remove(os.path.join(d, f"sync_{tag}", "meta.json"))
    ok(run(ANA, d))
    assert sync_info(d)["kind"] == "knock (single)"
    assert "WARN clock: the start and end knock syncs disagree" in summary_md(d)


def test_repeated_sync_start_uses_the_last_attempt():
    """Two identical knock patterns in the phone record and no sync_end: the later one is sync_start/."""
    d, p = session("--repeat-sync-start", "60", "--no-sync-end")
    ok(p)
    assert sync_info(d)["kind"] == "knock (single)" and offset_error_ms(d, "start") < 5
    assert "WARN clock: ambiguous sync_start" in summary_md(d)


def test_hand_sway_centroid_lag():
    """Walking sway of 0.3 rad/s on the hand yaw rate (it broke the old 30%-onset lag)."""
    d, p = session("--hand-sway", "0.3")
    ok(p)
    df = results(d)
    err = df.hand_lag_s - df.stick.map(HAND_LAG_TRUTH)
    assert err.abs().max() < 0.12 and err.groupby(df.stick).median().abs().max() < 0.03, err.tolist()


def test_phone_csv_bad_cells():
    """A NaN and a non-numeric cell (e.g. a truncated last line) must not disable the knock sync."""
    d = copy_session((), "bad_cells")
    f = os.path.join(d, "phone", "Accelerometer.csv")
    x = pd.read_csv(f).astype({"x": object, "y": object})
    x.loc[30000, "x"], x.loc[30001, "y"] = np.nan, "-0.1x"
    x.to_csv(f, index=False)
    p = run(ANA, d)
    ok(p)
    assert "Accelerometer.csv: dropped 2 rows" in p.stdout
    assert sync_info(d)["kind"] == "knock (start+end)"
    same(results(session()[0]), results(d), HAND + ROBOT, atol=1e-6)


def test_partial_and_excluded_trials_listed_once():
    """Trial 5 has a phone gap and no low.csv data: one 'partly analysed' entry. Trial 7 has complete
    times but a note starting with 'skipped': excluded, with a reason that says why."""
    d = copy_session(("--gap", "5:2.5"), "partial_once")
    f = os.path.join(d, "trial_005", "low.csv")
    with open(f) as fh:
        head = fh.readline()
    with open(f, "w") as fh:
        fh.write(head)
    tr = pd.read_csv(os.path.join(d, "trials.csv"))
    tr["note"] = tr["note"].astype(object)
    tr.loc[tr.trial == 7, "note"] = "skipped the comfort rating"
    tr.to_csv(os.path.join(d, "trials.csv"), index=False)
    ok(run(ANA, d))
    md = summary_md(d)
    assert md.count("- trial 5 (") == 1 and "low.csv empty or with gaps (yaw rate from sport.csv); hand metrics" in md
    assert "WARN 1 trial skipped, 1 partly analysed" in md and ": 7, 5" in md
    assert "trial 7 (rigid, step, L): excluded by its note in trials.csv (times complete)" in md


def test_cli_and_knock_floor_flag():
    p = run(ANA, "--help")
    assert p.returncode == 0 and "--robot-knock-floor" in p.stdout and "session" in p.stdout
    d = copy_session((), "floor_high")
    p = run(ANA, d, "--robot-knock-floor", "50")
    ok(p)
    assert sync_info(d)["kind"] == "ntp" and "WARN clock: no knock sync" in summary_md(d)
    d = copy_session((), "floor_low")
    p = run(ANA, d, "--robot-knock-floor", "0.8")
    ok(p)
    assert sync_info(d)["kind"] == "knock (start+end)" and offset_error_ms(d, "start") < 5


def test_python310_syntax():
    for f in (ANA, SIM, os.path.abspath(__file__)):
        with open(f) as fh:
            src = fh.read()
        ast.parse(src, feature_version=(3, 10))
        if hasattr(tokenize, "FSTRING_START"):          # PEP 701 f-strings parse here but not on 3.10
            quotes = []
            with open(f, "rb") as fh:
                for tok in tokenize.tokenize(fh.readline):
                    if tok.type == tokenize.FSTRING_START:
                        q = tok.string.lstrip("rRfFbBuU")[0]
                        assert not quotes or q != quotes[-1], (f, tok.start)
                        quotes.append(q)
                    elif tok.type == tokenize.FSTRING_END:
                        quotes.pop()
                    elif quotes and tok.type == tokenize.STRING:
                        assert tok.string.lstrip("rRfFbBuU")[0] != quotes[-1], (f, tok.start)
                        assert "\\" not in tok.string, (f, tok.start)


def main():
    tests = [(k, v) for k, v in globals().items() if k.startswith("test_") and callable(v)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"PASS {name}")
        except Exception:
            failed.append(name)
            print(f"FAIL {name}\n{traceback.format_exc()}")
    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed; sessions in {TMP}")
    if failed or "--keep" in sys.argv:
        print("kept for inspection:", TMP)
    else:
        shutil.rmtree(TMP, ignore_errors=True)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
