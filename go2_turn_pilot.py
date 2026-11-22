#!/usr/bin/env python3
"""
Go2 EDU guide-stick pilot: does the shape of the turn command change what the hand feels?

Modes
  check   read robot state for 5 s and print rates, power, yaw rate, mode/gait, the clock and any
          commands sent by OTHER programs (robot does not move)
  sync    log the body IMU for N seconds while you knock on the base plate (phone taped to it)
  run     run a block of trials (step vs smooth 90-degree turns) with one stick
  dry     print the command profiles without a robot (for testing)

Example
  python3 go2_turn_pilot.py --iface eth0 --out data/s1 check
  python3 go2_turn_pilot.py --iface eth0 --out data/s1 sync --tag start
  python3 go2_turn_pilot.py --iface eth0 --out data/s1 run --stick rigid --reps 5
  python3 go2_turn_pilot.py --iface eth0 --out data/s1 run --stick flexible --reps 5
  python3 go2_turn_pilot.py --iface eth0 --out data/s1 sync --tag end

Safety: keep the remote in hand (L2+B = damping), clear 4 x 4 m floor, speeds are clamped.
Every exit path calls StopMove(), including SIGTERM/SIGHUP (kill, closed terminal); over ssh run
it inside tmux/screen anyway. SIGKILL or a power loss cannot be caught. Turn off obstacle
avoidance in the app so it does not edit commands. Before moving, `run` listens 2 s for commands
from other programs; if it sees any, it refuses to move unless you type 'continue'.
"""
import argparse, bisect, collections, csv, json, math, os, random, re, signal, statistics, sys, threading, time

# ------------------------------------------------------------------ profiles
VX = 0.30            # m/s forward speed while walking
W_MAX = 0.60         # rad/s peak commanded yaw rate
TURN = math.pi / 2   # 90 degrees
RAMP = 0.80          # s, cosine ramp for the smooth profile
HZ = 50              # command rate

VX_LIMIT, W_LIMIT = 0.5, 1.0

# ------------------------------------------------------------------ safety / timing
STALE_S = 0.3           # s, abort a trial if sportmodestate or lowstate is older than this
STALL_S = 0.25          # s, abort a trial if two control ticks are further apart than this
RPC_TIMEOUT = 1.0       # s, SDK timeout for Move/StopMove (10 s only for Init/BalanceStand)
STOP_TRIES = 3          # StopMove attempts until the robot confirms (code 0)
CLOCK_STEP_TOL = 0.05   # s, change of time.time() - time.monotonic() reported as a clock step
FOREIGN_LISTEN_S = 2.0  # s, passive listen for other programs' commands before `run` moves
KNOCK_MARGIN = 1.3      # sync knocks below this x the threshold may be missed by the analysis

# Request topics of the sport and obstacle-avoidance services. Anything seen here before our own
# SportClient exists was sent by another program (RollCall bridge, Nav2, app joystick ...).
REQUEST_TOPICS = ("rt/api/sport/request", "rt/api/obstacles_avoid/request")
API_NAMES = {
    "rt/api/sport/request": {1001: "Damp", 1002: "BalanceStand", 1003: "StopMove", 1004: "StandUp",
                             1005: "StandDown", 1006: "RecoveryStand", 1008: "Move"},
    "rt/api/obstacles_avoid/request": {1001: "SwitchSet", 1002: "SwitchGet", 1003: "Move",
                                       1004: "UseRemoteCommandFromApi"},
}
OTHER_COMMANDERS_ADVICE = (
    "   Stop RollCall, Nav2, the CMU stack and any app joystick before moving, and turn off\n"
    "   obstacle avoidance in the app.")


def turn_rate(profile, t):
    """Commanded yaw-rate magnitude t seconds after the turn starts. Returns (w, done)."""
    if profile == "step":
        T = TURN / W_MAX
        return (W_MAX if t < T else 0.0), t >= T
    # smooth: cosine ramp up, plateau, cosine ramp down; same peak and same total angle
    Tp = TURN / W_MAX - RAMP
    T = Tp + 2 * RAMP
    if t < RAMP:
        w = W_MAX * 0.5 * (1 - math.cos(math.pi * t / RAMP))
    elif t < RAMP + Tp:
        w = W_MAX
    elif t < T:
        w = W_MAX * 0.5 * (1 + math.cos(math.pi * (t - RAMP - Tp) / RAMP))
    else:
        w = 0.0
    return w, t >= T


def trial_schedule():
    """(phase, duration) list; the turn phase length is set by the profile."""
    return [("still", 1.0), ("accel", 1.0), ("straight", 2.5), ("turn", None),
            ("straight2", 1.5), ("decel", 1.0), ("still2", 1.0)]


def command_at(profile, direction, phase, tp):
    """(vx, vyaw) for a phase at time tp inside the phase."""
    sgn = 1.0 if direction == "L" else -1.0
    if phase in ("still", "still2"):
        return 0.0, 0.0
    if phase == "accel":
        return VX * min(tp / 1.0, 1.0), 0.0
    if phase == "decel":
        return VX * max(1.0 - tp / 1.0, 0.0), 0.0
    if phase in ("straight", "straight2"):
        return VX, 0.0
    if phase == "turn":
        w, _ = turn_rate(profile, tp)
        return VX, sgn * w
    raise ValueError(phase)


def block_order(reps, seed):
    """(profiles, directions) for one block of 2*reps trials.

    Directions alternate L,R,L,R,... (keeps the robot inside the 4 x 4 m area). Within every
    4 trials (L,R,L,R) 'step' goes to exactly one L and one R at random, so the profile x
    direction cells and the two halves of the block stay balanced. A last odd pair gets
    step/smooth in random order, which makes reps=1 identical to the old free shuffle.
    Blinding: trials 3-4 of every 4 are the complement of trials 1-2, and the same seed gives
    the same order for both sticks, so the profiles are never shown during the session; do
    not look at trials.csv before the last block."""
    rng = random.Random(seed)
    n = 2 * reps
    profiles = []
    for b in range(0, n, 4):
        if n - b >= 4:
            left, right = ["step", "smooth"], ["step", "smooth"]
            rng.shuffle(left)
            rng.shuffle(right)
            profiles += [left[0], right[0], left[1], right[1]]
        else:
            pair = ["step", "smooth"]
            rng.shuffle(pair)
            profiles += pair
    return profiles, ["L", "R"] * reps


# ------------------------------------------------------------------ logging
class Logger:
    def __init__(self):
        self.lock = threading.Lock()
        self.sport, self.low = [], []
        # time.monotonic() of the last sample, for the freshness checks (immune to clock steps)
        self.last_sport = self.last_low = float("-inf")
        self.last_rpy = (0.0, 0.0, 0.0)

    def on_sport(self, m):
        t = time.time()
        imu = m.imu_state
        row = (t, m.yaw_speed, imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2],
               imu.rpy[0], imu.rpy[1], imu.rpy[2], m.velocity[0], m.velocity[1],
               m.position[0], m.position[1], m.mode, m.gait_type)
        with self.lock:
            self.sport.append(row)
            self.last_sport = time.monotonic()
            self.last_rpy = tuple(imu.rpy)

    def on_low(self, m):
        t = time.time()
        imu = m.imu_state
        mech = sum(abs(m.motor_state[i].tau_est * m.motor_state[i].dq) for i in range(12))
        row = (t, imu.accelerometer[0], imu.accelerometer[1], imu.accelerometer[2],
               imu.gyroscope[2], m.power_v, m.power_a, mech)
        with self.lock:
            self.low.append(row)
            self.last_low = time.monotonic()

    def take(self):
        with self.lock:
            s, l = self.sport, self.low
            self.sport, self.low = [], []
        return s, l


SPORT_COLS = ["t", "yaw_speed", "gx", "gy", "gz", "roll", "pitch", "yaw", "vx", "vy",
              "px", "py", "mode", "gait"]
LOW_COLS = ["t", "ax", "ay", "az", "gz", "power_v", "power_a", "mech_power"]
CMD_COLS = ["t", "phase", "vx", "wz"]
TRIAL_COLS = ["trial", "stick", "profile", "direction", "t_start", "t_turn_start",
              "t_turn_end", "t_end", "clarity_1to7", "comfort_1to7", "note"]


def write_csv(path, cols, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        w.writerows(rows)


def say(msg):
    """print() that cannot break a cleanup path: after SIGHUP the terminal may be gone (EIO)."""
    try:
        print(msg, flush=True)
    except (OSError, ValueError):
        pass


def connect(iface):
    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_, LowState_
    ChannelFactoryInitialize(0, iface)
    log = Logger()
    s1 = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    s1.Init(log.on_sport, 10)
    s2 = ChannelSubscriber("rt/lowstate", LowState_)
    s2.Init(log.on_low, 10)
    return log, (s1, s2)


def sport_client():
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    c = SportClient()
    c.SetTimeout(10.0)   # generous only for Init/BalanceStand; do_run/safe_stop then use RPC_TIMEOUT
    c.Init()
    return c


class RequestWatch:
    """Passive listener on the sport / obstacle-avoid request topics. close() must be called
    BEFORE this script creates its SportClient, so it only ever records other programs' commands."""

    def __init__(self):
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_api.msg.dds_ import Request_
        self.lock = threading.Lock()
        self.listening = True
        self.seen = []    # (topic, api_id, parameter)
        self.subs = []
        for topic in REQUEST_TOPICS:
            sub = ChannelSubscriber(topic, Request_)
            sub.Init(lambda m, topic=topic: self._on_request(topic, m), 10)
            self.subs.append((topic, sub))

    def _on_request(self, topic, m):
        with self.lock:
            if self.listening:
                self.seen.append((topic, m.header.identity.api_id, m.parameter))

    def close(self):
        """Stop recording. The subscribers stay alive (self.subs) like the state ones: the SDK's
        Close() drops the DataReader, and if a callback is running at that moment Cyclone deletes
        the reader from inside its own listener and the receive thread can hang."""
        with self.lock:
            self.listening = False

    def report(self):
        if not self.seen:
            print("  none seen (expected)")
            return
        for topic in REQUEST_TOPICS:
            reqs = [r for r in self.seen if r[0] == topic]
            if not reqs:
                continue
            names = API_NAMES.get(topic, {})
            ids = ", ".join(f"{i} ({names.get(i, '?')}) x{c}"
                            for i, c in sorted(collections.Counter(r[1] for r in reqs).items()))
            print(f"  {topic}: {len(reqs)} requests, api_id {ids}; "
                  f"sample parameter: {str(reqs[-1][2])[:120]}")


# ------------------------------------------------------------------ clock
def wall_minus_mono():
    return time.time() - time.monotonic()


def clock_sanity():
    """Print the wall clock; warn if it is obviously wrong (the Orin falls back to 1970 after boot)."""
    now = time.time()
    stamp = time.strftime("%Y-%m-%d %H:%M:%S UTC", time.gmtime(now))
    if time.gmtime(now).tm_year < 2024:
        print(f"!! system clock reads {stamp}: it is WRONG (the Orin falls back to 1970 after boot).\n"
              "   Fix it (timedatectl / NTP on the robot) BEFORE sync_start; if it gets corrected\n"
              "   between sync_start and sync_end the clock sync of the session is broken.")
        return False
    print(f"clock: {stamp}   (wall - monotonic = {wall_minus_mono():.3f} s)")
    return True


class ClockWatch:
    """Detects steps of the wall clock (NTP, timedatectl) by watching time.time() - time.monotonic().
    Control timing uses time.monotonic(), so a step does not change the motion, but it breaks the
    knock-based sync of the logged time.time() stamps."""

    def __init__(self):
        self.ref = wall_minus_mono()

    def step(self):
        """Clock step in s since the last reported one (0.0 if none); warns loudly."""
        d = wall_minus_mono() - self.ref
        if abs(d) <= CLOCK_STEP_TOL:
            return 0.0
        self.ref += d
        say(f"\n!! system clock stepped by {d:+.3f} s: clock sync is broken, "
            "run sync again / check timedatectl")
        return d

    def note(self):
        d = self.step()
        return f"clock step {d:+.3f} s" if d else ""


# ------------------------------------------------------------------ operator safety
_exit_signal = None   # last SIGINT/SIGTERM/SIGHUP received; it means "exit as soon as it is safe"


def _signal_text(signum):
    return "Ctrl-C" if signum == signal.SIGINT else f"signal {signum}"


def _on_exit_signal(signum, frame):
    global _exit_signal
    _exit_signal = signum
    if signum != signal.SIGINT:   # one-shot: a 2nd kill cannot cut the cleanup (Ctrl-C stays usable)
        for s in (signal.SIGTERM, signal.SIGHUP):
            signal.signal(s, signal.SIG_IGN)
    raise KeyboardInterrupt(_signal_text(signum))


def install_exit_signals():
    """Ctrl-C, SIGTERM and SIGHUP (kill, closed terminal, dropped ssh) raise KeyboardInterrupt, so
    the finally blocks still send StopMove and save the data, and are recorded for exit_requested()."""
    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(s, _on_exit_signal)


def exit_requested():
    """Re-raise a signal whose KeyboardInterrupt was swallowed by the SDK: its reply wait
    (utils/future.py) catches every exception and just returns a timeout code."""
    if _exit_signal is not None:
        raise KeyboardInterrupt(_signal_text(_exit_signal))


def safe_stop(client):
    """StopMove until the robot confirms it (code 0); shout if it never does.
    A signal during an attempt does not cut the retries and is not raised here: it stays recorded
    and the caller re-raises it with exit_requested() once the data is saved."""
    global _exit_signal
    client.SetTimeout(RPC_TIMEOUT)   # a lost link costs ~1 s per attempt, not 10-20 s
    code = None
    for i in range(1, STOP_TRIES + 1):
        try:
            code = client.StopMove()
        except KeyboardInterrupt:
            code = "interrupted"
            _exit_signal = _exit_signal or signal.SIGINT   # normally already set by _on_exit_signal
        except Exception as e:   # an SDK error must not skip the retries
            code = repr(e)
        if code == 0:
            break
        # 3104 = no reply in time: the request went out, so the robot may well have stopped
        why = ": no reply, the stop may still have arrived" if code == 3104 else ""
        say(f"\n!! StopMove not confirmed (code {code}{why}), attempt {i}/{STOP_TRIES} - "
            "be ready to stop the robot with the remote")
    else:
        say("\n!! StopMove NOT CONFIRMED - stop the robot with the remote NOW (L2+B)")
    return code == 0


def stdin_is_terminal():
    try:
        return sys.stdin.isatty()
    except Exception:   # no stdin, or closed
        return False


def flush_stdin(settle=0.0):
    """Drop keys typed ahead (e.g. Enter pressed while the robot was moving). Terminal only."""
    try:
        import termios
        if stdin_is_terminal():
            time.sleep(settle)   # let a burst of stray Enters arrive, then drop it
            termios.tcflush(sys.stdin, termios.TCIFLUSH)
    except Exception:   # best effort: no termios, closed stdin ...
        pass


def ask_start(prompt):
    """Only Enter (start), s (skip) or q (quit) are accepted; anything else asks again.
    End of input counts as q. Never starts a trial on unknown input."""
    flush_stdin(settle=1.0)
    while True:
        try:
            ans = input(prompt).strip().lower()
        except EOFError:
            print("\n(no more input: quitting)")
            return "q"
        if ans in ("", "s", "q"):
            return ans
        print("  type only Enter (start), s (skip) or q (quit)")


def ask(prompt):
    """Rating prompt; end of input counts as an empty answer."""
    try:
        return input(prompt).strip()
    except EOFError:
        return ""


def next_trial_number(out, tfile):
    """1 + the largest trial number already used in trials.csv or by a trial_NNN folder, so a
    skipped, aborted or interrupted trial never gets its number (and its data) reused."""
    used = [0]
    if os.path.exists(tfile):
        with open(tfile, newline="") as f:
            rows = list(csv.reader(f))
        if rows and rows[0] != TRIAL_COLS:
            print(f"!! {tfile} does not start with the expected header; fix it by hand "
                  "before the analysis")
        used += [int(r[0]) for r in rows if r and r[0].strip().isdigit()]
    for name in os.listdir(out):
        m = re.fullmatch(r"trial_(\d+)", name)
        if m and os.path.isdir(os.path.join(out, name)):
            used.append(int(m.group(1)))
    return max(used) + 1


def open_trials(tfile):
    """Open trials.csv for appending; a new or 0-byte file gets the header, flushed at once."""
    f = open(tfile, "a", newline="")
    w = csv.writer(f)
    if f.tell() == 0:
        w.writerow(TRIAL_COLS)
        f.flush()
    else:
        with open(tfile, "rb") as g:   # hand-edited and saved without a final newline?
            g.seek(-1, os.SEEK_END)
            if g.read(1) != b"\n":
                f.write("\r\n")   # else the next row would be glued onto the last line
    return f, w


def listen_for_other_commanders(seconds):
    """Passively listen for requests from other programs. Closed before SportClient exists."""
    watch = RequestWatch()
    print(f"Listening {seconds:.0f} s for commands sent by other programs ...", flush=True)
    try:
        time.sleep(seconds)
    finally:
        watch.close()
    return watch


# ------------------------------------------------------------------ modes
def do_check(args):
    log, _ = connect(args.iface)
    watch = RequestWatch()   # passive: is any other program commanding the robot?
    try:
        time.sleep(5.0)
    finally:
        watch.close()
    s, l = log.take()
    print(f"sportmodestate: {len(s)/5:.0f} Hz   lowstate: {len(l)/5:.0f} Hz")
    if l:
        p = [r[5] * r[6] for r in l]
        print(f"battery power: {sum(p)/len(p):.1f} W   mech power: {sum(r[7] for r in l)/len(l):.1f} W")
    if s:
        print(f"yaw rate now: {s[-1][4]:+.3f} rad/s   rpy: {[round(x,3) for x in s[-1][5:8]]}")
        print(f"mode: {s[-1][12]}   gait_type: {s[-1][13]}")
    print("commands from other programs on " + ", ".join(REQUEST_TOPICS) + ":")
    watch.report()
    if watch.seen:
        print(OTHER_COMMANDERS_ADVICE)
    clock_sanity()
    if not s or not l:
        print("No data. Check --iface (the interface on 192.168.123.x) and that no other SDK holds the robot.")


def knock_report(rows, t0, floor=1.5):
    """Print knock-like peaks in |a| of low.csv rows, stdlib only. Close to the
    analysis: |a| minus a baseline, peaks at least 0.25 s apart and above max(median + 8 MAD, floor).
    The baseline here is a running median over +-0.25 s; the analysis uses a 2 Hz low-pass, which
    absorbs part of each spike, so near the floor it can find fewer knocks than reported here."""
    if len(rows) < 20:
        print("knocks: too few lowstate rows to look for knocks")
        return
    t = [r[0] for r in rows]
    mag = [math.sqrt(r[1] ** 2 + r[2] ** 2 + r[3] ** 2) for r in rows]
    hp, win, lo, hi = [], [], 0, 0
    for i, ti in enumerate(t):
        while hi < len(t) and t[hi] <= ti + 0.25:
            bisect.insort(win, mag[hi])
            hi += 1
        while t[lo] < ti - 0.25:
            win.pop(bisect.bisect_left(win, mag[lo]))
            lo += 1
        hp.append(abs(mag[i] - win[len(win) // 2]))
    med = statistics.median(hp)
    noise = med + 8 * statistics.median([abs(x - med) for x in hp])
    thr = max(noise, floor)
    # local maxima above the noise level; the highest wins when two are closer than 0.25 s
    cand = [i for i in range(len(hp)) if hp[i] > noise
            and (i == 0 or hp[i] >= hp[i - 1]) and (i == len(hp) - 1 or hp[i] >= hp[i + 1])]
    peaks = []
    for i in sorted(cand, key=lambda i: -hp[i]):
        if all(abs(t[i] - t[j]) >= 0.25 for j in peaks):
            peaks.append(i)
    kept = sorted(i for i in peaks if hp[i] >= thr)
    print(f"knocks: {len(kept)} peaks above {thr:.2f} m/s^2 in |a| (expected 5: 3 knocks, pause, 2 knocks)")
    if kept:
        print("  at t = " + ", ".join(f"{t[i] - t0:.2f}" for i in kept) + " s after the start")
    if peaks:
        top = sorted((hp[i] for i in peaks), reverse=True)[:5]
        print("  top-5 peak heights: " + ", ".join(f"{h:.2f}" for h in top) + " m/s^2")
    if len(peaks) > len(kept):
        print(f"  {len(peaks) - len(kept)} weaker peak(s) between the noise level {noise:.2f} and "
              f"the {floor:.1f} m/s^2 floor of the analysis")
    if len(kept) < 4:
        print("  !! fewer than 4 clear knocks: knock harder or longer and run sync again")
    elif sum(hp[i] >= KNOCK_MARGIN * thr for i in kept) < 4:
        print(f"  !! fewer than 4 knocks reach {KNOCK_MARGIN * thr:.2f} m/s^2 ({KNOCK_MARGIN:g} x the threshold): "
              "the analysis may match fewer than 4. Knock harder and run sync again, or plan a lower "
              "analyze_pilot.py --robot-knock-floor")


def do_sync(args):
    install_exit_signals()
    os.makedirs(args.out, exist_ok=True)
    d = os.path.join(args.out, f"sync_{args.tag}")
    if os.path.exists(d):   # never overwrite a sync: the session's clock alignment depends on it
        # The analysis reads only sync_<tag>, so an accidental re-run (e.g. 'start' from the shell
        # history at the end of the session) must not silently take its place.
        print(f"!! {d} already exists (written {(time.time() - os.path.getmtime(d)) / 60:.0f} min ago); "
              f"the analysis uses only the newest sync_{args.tag}.")
        if ask("Type 'replace' to record a new one (the old one is kept as *_old_<time>); "
               "anything else exits: ") != "replace":
            sys.exit("Nothing recorded.")
        old = f"{d}_old_{int(time.time())}"
        k = 2
        while os.path.exists(old):
            old, k = f"{d}_old_{int(time.time())}_{k}", k + 1
        os.rename(d, old)
        print(f"renamed the old one to {old}")
    log, _ = connect(args.iface)
    clock_sanity()
    os.makedirs(d)
    print(f"Phone taped flat on the base plate, recording. Knock on the plate: 3 knocks, pause, 2 knocks.")
    print(f"Logging {args.seconds:.0f} s ... (Ctrl-C stops early and still saves)", flush=True)
    t_start, off_start = time.time(), wall_minus_mono()
    interrupted = False
    try:
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        interrupted = True
    finally:   # save whatever was logged, also on Ctrl-C / SIGTERM / SIGHUP
        s, l = log.take()
        write_csv(os.path.join(d, "sport.csv"), SPORT_COLS, s)
        write_csv(os.path.join(d, "low.csv"), LOW_COLS, l)
        off_end = wall_minus_mono()
        with open(os.path.join(d, "meta.json"), "w") as f:
            json.dump({"tag": args.tag, "t_saved": time.time(), "t_start": t_start,
                       "seconds_requested": args.seconds, "interrupted": interrupted,
                       "wall_minus_monotonic_s": off_start, "wall_minus_monotonic_end_s": off_end}, f)
        say(f"{'stopped early; ' if interrupted else ''}saved {len(l)} lowstate rows in {d}")
    if abs(off_end - off_start) > CLOCK_STEP_TOL:
        print(f"!! system clock stepped by {off_end - off_start:+.3f} s during the sync: "
              "run sync again / check timedatectl")
    knock_report(l, t_start)


def run_trial(client, log, profile, direction, tdir):
    """One open-loop trial. Always sends StopMove first, then saves what was recorded (also when
    the trial is aborted or interrupted). A signal that arrives only during the final StopMove
    stays pending (exit_requested), so a completed trial is still returned and logged as such."""
    os.makedirs(tdir, exist_ok=True)
    log.take()  # drop old samples
    cmd_rows, events, abort = [], {}, None
    dt = 1.0 / HZ
    last_tick = None
    try:
        for phase, dur in trial_schedule():
            t0 = time.monotonic()                    # control timing: monotonic clock
            events[phase + "_start"] = time.time()   # logged times: wall clock (data contract)
            while True:
                tick = time.monotonic()
                tp = tick - t0
                if phase == "turn":
                    _, done = turn_rate(profile, tp)
                    if done:
                        break
                elif tp >= dur:
                    break
                vx, wz = command_at(profile, direction, phase, tp)
                vx = max(-VX_LIMIT, min(VX_LIMIT, vx))
                wz = max(-W_LIMIT, min(W_LIMIT, wz))
                # safety: stalled loop, stale state streams or large tilt
                if last_tick is not None and tick - last_tick > STALL_S:
                    raise RuntimeError(f"control loop stalled {tick - last_tick:.2f} s")
                last_tick = tick
                if tick - log.last_sport > STALE_S:
                    raise RuntimeError("sportmodestate stream stale")
                if tick - log.last_low > STALE_S:
                    raise RuntimeError("lowstate stream stale")
                r, p, _ = log.last_rpy
                if abs(r) > 0.35 or abs(p) > 0.35:
                    raise RuntimeError(f"tilt too large roll={r:.2f} pitch={p:.2f}")
                now = time.time()
                code = client.Move(vx, 0.0, wz)
                if code != 0:
                    raise RuntimeError(f"Move not sent (code {code})")
                cmd_rows.append((now, phase, vx, wz))
                time.sleep(max(0.0, dt - (time.monotonic() - tick)))
            events[phase + "_end"] = time.time()
    except BaseException as e:
        abort = "interrupted" if isinstance(e, KeyboardInterrupt) else (str(e) or repr(e))
        raise
    finally:
        try:
            safe_stop(client)
        finally:   # nested, so StopMove always comes first and the data is saved even if it raises
            s, l = log.take()
            write_csv(os.path.join(tdir, "cmd.csv"), CMD_COLS, cmd_rows)
            write_csv(os.path.join(tdir, "sport.csv"), SPORT_COLS, s)
            write_csv(os.path.join(tdir, "low.csv"), LOW_COLS, l)
            with open(os.path.join(tdir, "events.json"), "w") as f:
                json.dump({"profile": profile, "direction": direction, "aborted": abort,
                           "events": events,
                           "rows": {"cmd": len(cmd_rows), "sport": len(s), "low": len(l)}}, f, indent=1)
            if cmd_rows or s or l:   # no row counts here: they would reveal the profile
                say(f"  saved {tdir}")
            else:
                say(f"  no data recorded for this trial ({tdir} has header-only CSVs)")
    return events


def do_run(args):
    install_exit_signals()
    if not stdin_is_terminal():
        print("!! stdin is not a terminal (docker without -t, ssh without -t, a pipe): keys pressed "
              "during a trial\n   cannot be discarded and will answer the next prompts, even start "
              "the next trial.\n   Run it with docker run/exec -it or ssh -t.")
    os.makedirs(args.out, exist_ok=True)
    log, _ = connect(args.iface)
    clock_sanity()
    clock = ClockWatch()
    # Before our SportClient exists: is any other program commanding the robot?
    watch = listen_for_other_commanders(FOREIGN_LISTEN_S)
    if watch.seen:
        print("!! Another program is sending commands to the robot:")
        watch.report()
        print(OTHER_COMMANDERS_ADVICE)
        ans = ask("Type 'continue' to move anyway; anything else exits WITHOUT moving: ")
        if ans != "continue":
            sys.exit("Exiting without moving.")
    else:
        print("commands from other programs: none seen")
    client = sport_client()
    tfile = os.path.join(args.out, "trials.csv")
    f = None
    try:
        time.sleep(1.0)
        order, dirs = block_order(args.reps, args.seed)
        first = next_trial_number(args.out, tfile)
        f, w = open_trials(tfile)

        def write_row(n, prof, d, ev, clar, comf, *notes):
            w.writerow([n, args.stick, prof, d, ev.get("still_start", ""), ev.get("turn_start", ""),
                        ev.get("turn_end", ""), ev.get("still2_end", ""), clar, comf,
                        "; ".join(x for x in notes if x)])
            f.flush()

        code = client.BalanceStand()
        exit_requested()   # a signal during the reply wait is swallowed by the SDK
        if code != 0:
            print(f"!! BalanceStand returned code {code}: make sure the robot is standing "
                  "(use the remote) before you start a trial")
        client.SetTimeout(RPC_TIMEOUT)   # trial loop and stops: a lost link blocks ~1 s, not 10 s
        # Participant blinding: the profile is never printed (see block_order); trials.csv has it.
        print(f"Block: stick={args.stick}, seed={args.seed}, {len(order)} trials "
              f"({order.count('step')} step, {order.count('smooth')} smooth, turns alternate L/R); "
              "profiles are not shown: do not open trials.csv before the last block")
        for k, (prof, d) in enumerate(zip(order, dirs)):
            n = first + k
            ans = ask_start(f"\n[{k+1}/{len(order)}] trial {n:03d}  turn={d}. "
                            f"Hold the handle, Enter to start (s=skip, q=quit): ")
            if ans == "q":
                break
            if ans == "s":
                write_row(n, prof, d, {}, "", "", "skipped")
                print("  skipped")
                continue
            tdir = os.path.join(args.out, f"trial_{n:03d}")
            try:
                for c in (3, 2, 1):
                    print(c, end=" ", flush=True); time.sleep(0.7)
                print("go")
                ev, ok = run_trial(client, log, prof, d, tdir), ""
            except RuntimeError as e:
                ev, ok = {}, f"aborted: {e}"
                print("ABORTED:", e)
            except BaseException as e:
                # Ctrl-C / SIGTERM / SIGHUP (StopMove already sent): annotate with EMPTY times so
                # the analysis skips it, then exit.
                why = "interrupted" if isinstance(e, KeyboardInterrupt) else repr(e)
                write_row(n, prof, d, {}, "", "", f"aborted: {why}", clock.note())
                say(f"\nABORTED: {why}")
                raise
            clock_note = clock.note()
            clar = comf = note = ""
            try:
                exit_requested()   # a signal during the trial's final StopMove: keep its row, then exit
                flush_stdin()
                clar = ask("  cue clarity 1-7 (Enter to skip): ")
                comf = ask("  comfort 1-7 (Enter to skip): ")
                note = ask("  note (Enter to skip): ")
                if not (ok or clock_note) and note.lower().startswith(("aborted", "skipped")):
                    print("  (a note starting with 'aborted' or 'skipped' leaves this trial out of the analysis)")
            finally:   # the row is written even on Ctrl-C at the prompts (fill what is known)
                write_row(n, prof, d, ev, clar, comf, ok, clock_note, note)
            print("  Reposition the robot with the remote, then continue.")
    finally:
        try:
            safe_stop(client)
        finally:
            if f is not None:
                f.close()
            clock.step()   # also report a step during the last ratings
    exit_requested()   # a signal during the last StopMove of a block that ended normally


def do_dry(args):
    for prof in ("step", "smooth"):
        rows, t = [], 0.0
        for phase, dur in trial_schedule():
            tp = 0.0
            while True:
                if phase == "turn":
                    if turn_rate(prof, tp)[1]:
                        break
                elif tp >= dur:
                    break
                rows.append((round(t, 2), phase) + command_at(prof, "L", phase, tp))
                tp += 1 / HZ; t += 1 / HZ
        ang = sum(r[3] for r in rows) / HZ
        print(f"{prof:6s}: duration {t:.2f} s, commanded turn {math.degrees(ang):.1f} deg")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--out", default="data/session1")
    # --iface/--out are also accepted after the mode ('run --stick test --out data/teste');
    # SUPPRESS keeps the value given before the mode, or the default, when they are not repeated.
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--iface", default=argparse.SUPPRESS)
    common.add_argument("--out", default=argparse.SUPPRESS)
    sub = ap.add_subparsers(dest="mode", required=True)
    sub.add_parser("check", parents=[common])
    s = sub.add_parser("sync", parents=[common]); s.add_argument("--tag", default="start"); s.add_argument("--seconds", type=float, default=20)
    r = sub.add_parser("run", parents=[common]); r.add_argument("--stick", required=True); r.add_argument("--reps", type=int, default=5)
    r.add_argument("--seed", type=int, default=7)
    sub.add_parser("dry", parents=[common])
    args = ap.parse_args()
    try:
        {"check": do_check, "sync": do_sync, "run": do_run, "dry": do_dry}[args.mode](args)
    except KeyboardInterrupt as e:
        say(f"\nInterrupted ({str(e) or 'Ctrl-C'}).")
        sys.exit(130)


if __name__ == "__main__":
    main()
