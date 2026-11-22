#!/usr/bin/env python3
"""Offline tests of go2_turn_pilot.py against the FAKE unitree_sdk2py in test/fake_sdk.

    .venv/bin/python test/test_pilot.py              # all tests (~1-2 min, run in parallel)
    .venv/bin/python test/test_pilot.py -k sync      # only tests whose name contains 'sync'

No robot and no network: the fake SDK is pure Python and is put on the PYTHONPATH of the
subprocesses started here only. Everything is written to a fresh temporary directory.
"""
import argparse, ast, concurrent.futures, csv, hashlib, inspect, json, math, os, pty, random, re
import shutil, signal, subprocess, sys, tempfile, threading, time, traceback

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(REPO, "go2_turn_pilot.py")
FAKE_SDK = os.path.join(REPO, "test", "fake_sdk")
PY = sys.executable

TRIAL_COLS = ["trial", "stick", "profile", "direction", "t_start", "t_turn_start", "t_turn_end",
              "t_end", "clarity_1to7", "comfort_1to7", "note"]
CMD_COLS = ["t", "phase", "vx", "wz"]
SPORT_COLS = ["t", "yaw_speed", "gx", "gy", "gz", "roll", "pitch", "yaw", "vx", "vy", "px", "py", "mode", "gait"]
LOW_COLS = ["t", "ax", "ay", "az", "gz", "power_v", "power_a", "mech_power"]
T_FIELDS = ["t_start", "t_turn_start", "t_turn_end", "t_end"]
MID_TURN_MOVES = 300   # Move #225 starts the turn (1 + 1 + 2.5 s at 50 Hz)

sys.dont_write_bytecode = True   # keep the repo free of __pycache__ from test runs
sys.path.insert(0, REPO)
import go2_turn_pilot as P   # must not import unitree_sdk2py (checked in test_contract_and_imports)
PILOT_IMPORTED_SDK = "unitree_sdk2py" in sys.modules


# ------------------------------------------------------------------ helpers
def check(cond, msg):
    if not cond:
        raise AssertionError(msg)


class Pilot:
    """go2_turn_pilot.py in a subprocess, with the fake SDK and its call log."""

    def __init__(self, work, args, stdin="", env=None, out="out", tty=False, name="pilot"):
        # never start the script without the fake SDK in front: a real one would open DDS on the network
        check(os.path.isfile(os.path.join(FAKE_SDK, "unitree_sdk2py", "_fake_robot.py")), "fake SDK missing")
        self.work, self.out = work, os.path.join(work, out)
        self.calls_path = os.path.join(work, f"{name}_calls.jsonl")
        full_env = {k: v for k, v in os.environ.items() if not k.startswith("FAKE_")}
        full_env.update(PYTHONPATH=FAKE_SDK, PYTHONUNBUFFERED="1", PYTHONDONTWRITEBYTECODE="1",
                        FAKE_CALLS=self.calls_path)
        full_env.update(env or {})
        cmd = [PY, SCRIPT, "--iface", "fake0", "--out", self.out] + args
        self.master = None
        if tty:   # stdin is a real terminal (pty), so the script's tcflush is active
            self.master, slave = pty.openpty()
            self.proc = subprocess.Popen(cmd, stdin=slave, stdout=subprocess.PIPE,
                                         stderr=subprocess.STDOUT, env=full_env, cwd=work)
            os.close(slave)
            threading.Thread(target=self._drain_master, daemon=True).start()
        else:
            self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         stderr=subprocess.STDOUT, env=full_env, cwd=work)
        self.buf, self.lock = [], threading.Lock()
        threading.Thread(target=self._read, daemon=True).start()
        self.t_start = time.monotonic()
        if stdin:
            self.send(stdin)

    def _read(self):
        for chunk in iter(lambda: self.proc.stdout.read1(4096), b""):
            with self.lock:
                self.buf.append(chunk.decode(errors="replace"))

    def _drain_master(self):   # echo of the typed keys; keep the pty from filling up
        try:
            while os.read(self.master, 4096):
                pass
        except OSError:
            pass

    @property
    def output(self):
        with self.lock:
            return "".join(self.buf)

    def send(self, text):
        if self.master is not None:
            os.write(self.master, text.encode())
        else:
            self.proc.stdin.write(text.encode())
            self.proc.stdin.flush()

    def close_stdin(self):
        if self.master is None:
            self.proc.stdin.close()

    def calls(self, name=None):
        if not os.path.exists(self.calls_path):
            return []
        out = []
        with open(self.calls_path) as f:
            for line in f:
                try:
                    out.append(json.loads(line))
                except ValueError:   # last line still being written
                    pass
        return [c for c in out if name is None or c["call"] == name]

    def moves(self):
        return [c for c in self.calls("Move") if c["ret"] == 0]

    def wait_for(self, pred, what, timeout=40):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            if pred():
                return
            if self.proc.poll() is not None and not pred():
                raise AssertionError(f"pilot exited ({self.proc.returncode}) before: {what}\n{self.tail()}")
            time.sleep(0.05)
        raise AssertionError(f"timeout waiting for: {what}\n{self.tail()}")

    def wait_text(self, text, timeout=40):
        self.wait_for(lambda: text in self.output, repr(text), timeout)

    def kill(self, sig):
        self.proc.send_signal(sig)

    def finish(self, timeout=90):
        try:
            rc = self.proc.wait(timeout)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            raise AssertionError(f"pilot did not exit within {timeout} s\n{self.tail()}")
        time.sleep(0.05)   # let the reader thread take the last bytes
        if self.master is not None:
            os.close(self.master)
        return rc

    def tail(self, n=2500):
        return "----- pilot output (tail) -----\n" + self.output[-n:]


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.reader(f))


def trials(out):
    rows = read_csv(os.path.join(out, "trials.csv"))
    check(rows and rows[0] == TRIAL_COLS, f"trials.csv header is {rows[:1]}")
    return [dict(zip(TRIAL_COLS, r)) for r in rows[1:]]


def md5(path):
    with open(path, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()


def assert_stopped_after_last_move(p, confirmed=True):
    """StopMove follows the last Move, and the very last client call is a StopMove."""
    calls = p.calls()
    client = [c for c in calls if c["call"] in ("Move", "StopMove", "BalanceStand")]
    check(client and client[-1]["call"] == "StopMove", f"last client call is not StopMove: {client[-3:]}")
    idx = max([i for i, c in enumerate(calls) if c["call"] == "Move"], default=-1)
    stops = [c for c in calls[idx + 1:] if c["call"] == "StopMove"]
    check(stops, "no StopMove after the last Move")
    if confirmed:
        check(any(c["ret"] == 0 for c in stops), f"no confirmed StopMove after the last Move: {stops}")
    check(all(c["timeout"] == P.RPC_TIMEOUT for c in stops), f"StopMove with timeout != 1 s: {stops}")
    return stops


def assert_trial_files(tdir, min_cmd=1):
    for name, cols in (("cmd.csv", CMD_COLS), ("sport.csv", SPORT_COLS), ("low.csv", LOW_COLS)):
        rows = read_csv(os.path.join(tdir, name))
        check(rows[0] == cols, f"{tdir}/{name} header {rows[0]} != {cols}")
        if name == "cmd.csv":
            check(len(rows) - 1 >= min_cmd, f"{tdir}/cmd.csv has {len(rows) - 1} rows")
        else:
            check(len(rows) > 1, f"{tdir}/{name} has no data rows")
    return json.load(open(os.path.join(tdir, "events.json")))


def commanded_turn_deg(tdir):
    rows = read_csv(os.path.join(tdir, "cmd.csv"))[1:]
    t = [float(r[0]) for r in rows]
    wz = [float(r[3]) for r in rows]
    return math.degrees(sum(wz[i] * (t[i + 1] - t[i]) for i in range(len(t) - 1)) + wz[-1] / P.HZ)


def first_line_with(text, needle):
    for line in text.splitlines():
        if needle in line:
            return line
    return None


# ------------------------------------------------------------------ static / in-process tests
def test_py38_syntax_and_stdlib_only(work):
    src = open(SCRIPT).read()
    tree = ast.parse(src, feature_version=(3, 8))
    top = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            top |= {a.name.split(".")[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom):
            top.add(node.module.split(".")[0])
    check(top <= set(sys.stdlib_module_names), f"non-stdlib top-level imports: {top - set(sys.stdlib_module_names)}")
    nested = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)) and node not in tree.body:
            mod = node.module if isinstance(node, ast.ImportFrom) else node.names[0].name
            nested.add(mod.split(".")[0])
    check(nested <= set(sys.stdlib_module_names) | {"unitree_sdk2py"}, f"unexpected lazy imports {nested}")
    # the same check the operator runs by hand
    subprocess.run([PY, "-c", "import ast,sys; ast.parse(open(sys.argv[1]).read(), feature_version=(3,8))",
                    SCRIPT], check=True)
    # ast with feature_version does NOT reject 3.12-only f-strings, so check them on the tokens
    check(fstring_violations('f"{x["a"]}"') and fstring_violations("f'{\"\\\\n\".join(a)}'"), "guard broken")
    check(not fstring_violations("f\"{x['a']:>{w}}\\n\""), "guard too strict")
    bad = fstring_violations(src)
    check(not bad, f"f-strings that Python < 3.12 rejects: {bad}")
    newer = [api for api in NEWER_THAN_38 if re.search(api, src)]
    check(not newer, f"stdlib APIs newer than Python 3.8: {newer}")


NEWER_THAN_38 = [r"\.removeprefix\(", r"\.removesuffix\(", r"zip\([^)]*strict=", r"\bmath\.lcm\b",
                 r"insort\([^)]*key=", r"bisect_\w+\([^)]*key=", r"\bpairwise\b", r"\.bit_count\(",
                 r"\bzoneinfo\b", r"\bgraphlib\b", r"\btomllib\b", r"randbytes", r"functools\.cache\b",
                 r"\bmatch\s+\w+\s*:", r":\s*(list|dict|tuple|set)\[", r"->\s*(list|dict|tuple|set)\[",
                 r"\w+\s*\|\s*None\b", r"datetime\.UTC\b", r"contextlib\.chdir", r"anext\(", r"aiter\("]


def fstring_violations(src):
    """f-string parts that only Python >= 3.12 accepts: a nested string reusing an enclosing quote,
    a backslash or a comment inside a replacement field."""
    import io, tokenize
    bad, quotes = [], []   # quote chars of the f-strings we are inside
    for tok in tokenize.generate_tokens(io.StringIO(src).readline):
        if tok.type == tokenize.FSTRING_START:
            q = tok.string.lstrip("fFrRbBuU")[0]
            if q in quotes:
                bad.append((tok.start, tok.line.strip()))
            quotes.append(q)
        elif tok.type == tokenize.FSTRING_END:
            quotes.pop()
        elif quotes and tok.type == tokenize.STRING:
            if tok.string.lstrip("fFrRbBuU")[0] in quotes or "\\" in tok.string:
                bad.append((tok.start, tok.line.strip()))
        elif len(quotes) > 1 and tok.type == tokenize.FSTRING_MIDDLE and "\\" in tok.string:
            bad.append((tok.start, tok.line.strip()))
        elif quotes and tok.type == tokenize.COMMENT:
            bad.append((tok.start, tok.line.strip()))
    return bad


def test_contract_and_imports(work):
    check(not PILOT_IMPORTED_SDK, "importing go2_turn_pilot imported unitree_sdk2py")
    check(P.SPORT_COLS == SPORT_COLS and P.LOW_COLS == LOW_COLS, "SPORT_COLS / LOW_COLS changed")
    check(P.TRIAL_COLS == TRIAL_COLS and P.CMD_COLS == CMD_COLS, "TRIAL_COLS / CMD_COLS changed")
    check(P.trial_schedule() == [("still", 1.0), ("accel", 1.0), ("straight", 2.5), ("turn", None),
                                 ("straight2", 1.5), ("decel", 1.0), ("still2", 1.0)], "trial_schedule changed")
    check(str(inspect.signature(P.turn_rate)) == "(profile, t)", "turn_rate signature changed")
    check(str(inspect.signature(P.command_at)) == "(profile, direction, phase, tp)", "command_at signature changed")
    check((P.VX, P.W_MAX, P.VX_LIMIT, P.W_LIMIT) == (0.30, 0.60, 0.5, 1.0), "speed limits changed")
    check(P.turn_rate("step", 0.0) == (0.6, False) and P.turn_rate("smooth", 0.0) == (0.0, False), "turn_rate")
    check(P.command_at("step", "R", "turn", 1.0) == (0.3, -0.6), "command_at sign for R")


def offline_env():
    """Environment for scripts run without the fake SDK; no __pycache__ written into the repo."""
    env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    return env


def test_dry_prints_90_deg(work):
    r = subprocess.run([PY, SCRIPT, "dry"], capture_output=True, text=True, timeout=30, cwd=work,
                       env=offline_env())
    check(r.returncode == 0, r.stderr)
    for prof, dur in (("step", "10.62"), ("smooth", "11.42")):
        m = re.search(rf"^{prof}\s*: duration (\S+) s, commanded turn (\S+) deg$", r.stdout, re.M)
        check(m, f"dry output has no line for {prof}: {r.stdout}")
        check(m.group(1) == dur and abs(float(m.group(2)) - 90) < 0.5, f"dry {prof}: {m.group(0)}")


def test_block_order_balance(work):
    def old_shuffle(reps, seed):
        order = ["step", "smooth"] * reps
        random.Random(seed).shuffle(order)
        return order

    check(P.block_order(1, 7) == (["step", "smooth"], ["L", "R"]), f"reps 1 seed 7: {P.block_order(1, 7)}")
    for reps in (1, 2, 3, 5, 8):
        for seed in range(1, 21):
            prof, dirs = P.block_order(reps, seed)
            check(prof == P.block_order(reps, seed)[0], "not deterministic")
            check(len(prof) == 2 * reps and dirs == ["L", "R"] * reps, f"reps {reps}: length/dirs")
            check(prof.count("step") == reps, f"reps {reps} seed {seed}: {prof.count('step')} step")
            cells = {(p, d): sum(1 for a, b in zip(prof, dirs) if (a, b) == (p, d))
                     for p in ("step", "smooth") for d in "LR"}
            check(abs(cells["step", "L"] - cells["step", "R"]) == reps % 2,
                  f"reps {reps} seed {seed}: cells {cells}")
            for b in range(0, 2 * reps - 3, 4):   # each full block of 4: one step-L and one step-R
                blk = list(zip(prof[b:b + 4], dirs[b:b + 4]))
                check(blk.count(("step", "L")) == 1 and blk.count(("step", "R")) == 1,
                      f"reps {reps} seed {seed}: block {blk}")
            half = prof[:reps].count("step")
            if reps == 8:
                check(cells == {("step", "L"): 4, ("step", "R"): 4, ("smooth", "L"): 4, ("smooth", "R"): 4}
                      and half == 4, f"reps 8 seed {seed}: {cells}, first half {half} step")
            if reps == 5:
                check(sorted(cells.values()) == [2, 2, 3, 3] and abs(2 * half - reps) <= 1,
                      f"reps 5 seed {seed}: {cells}, first half {half}")
            if reps == 1:
                check(prof == old_shuffle(1, seed), f"reps 1 seed {seed} differs from the old order")


def test_knock_report_noise_and_weak_knocks(work):
    import contextlib, io
    rng = random.Random(3)
    knocks = [3.0, 3.45, 3.9, 6.5, 6.9]

    def report(amp):
        t0, rows = 1.8e9, []
        for i in range(20 * 500):
            t = t0 + i / 500
            spike = sum(amp * math.exp(-((t - t0 - k) / 0.004) ** 2) for k in knocks)
            rows.append((t, rng.gauss(0, 0.05), rng.gauss(0, 0.05), 9.81 + spike + rng.gauss(0, 0.05),
                         0.0, 28.5, 2.0, 30.0))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            P.knock_report(rows, t0)
        return buf.getvalue()

    strong = report(8.0)
    check("knocks: 5 peaks" in strong and "3.00, 3.45, 3.90, 6.50, 6.90" in strong and "!!" not in strong, strong)
    weak = report(1.0)   # visible above the noise, but under the 1.5 m/s^2 floor of the analysis
    check("knocks: 0 peaks" in weak and "5 weaker peak(s)" in weak and "fewer than 4 clear" in weak, weak)
    # just above the floor: counted here, but the analysis's low-pass baseline may lose some of them
    near = report(1.7)
    check("knocks: 5 peaks" in near and "fewer than 4 knocks reach 1.95" in near, near)
    check("!!" not in report(2.5), "near-floor warning for knocks well above the floor")


# ------------------------------------------------------------------ run mode
def test_run_normal_block(work):
    p = Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="\n5\n4\nfirst\n\n6\n3\nskipped lunch\n")
    rc = p.finish()
    check(rc == 0, f"exit code {rc}\n{p.tail()}")
    out, text = p.out, p.output
    rows = trials(out)
    check([r["trial"] for r in rows] == ["1", "2"], f"rows {rows}")
    expected = P.block_order(1, 7)[0]
    for r, prof, d in zip(rows, expected, "LR"):
        check(r["stick"] == "test" and r["profile"] == prof and r["direction"] == d, f"row {r}")
        ts = [float(r[k]) for k in T_FIELDS]
        check(ts == sorted(ts) and abs(ts[0] - time.time()) < 600, f"t_* not epoch/ordered: {ts}")
        ev = assert_trial_files(os.path.join(out, f"trial_{int(r['trial']):03d}"), min_cmd=500)
        check(ev["aborted"] is None and ev["profile"] == prof, f"events.json {ev}")
        check(abs(float(r["t_turn_start"]) - ev["events"]["turn_start"]) < 1e-6, "events.json != trials.csv")
        ang = commanded_turn_deg(os.path.join(out, f"trial_{int(r['trial']):03d}"))
        check(abs(abs(ang) - 90) < 3 and (ang > 0) == (d == "L"), f"trial {r['trial']} turned {ang:.1f} deg")
    check((rows[0]["clarity_1to7"], rows[0]["comfort_1to7"], rows[0]["note"]) == ("5", "4", "first"), rows[0])
    check((rows[1]["clarity_1to7"], rows[1]["comfort_1to7"], rows[1]["note"]) == ("6", "3", "skipped lunch"), rows[1])
    # the operator is told that such a note excludes the trial from the analysis (only for that trial)
    check(text.count("leaves this trial out of the analysis") == 1
          and text.index("leaves this trial out") > text.index("trial 002"), p.tail())
    check("stdin is not a terminal" in text, "no warning that typed-ahead keys cannot be dropped (pipe)")
    # SDK usage: 10 s timeout only for Init/BalanceStand, 1 s for every Move and StopMove
    calls = p.calls()
    names = [c["call"] for c in calls]
    check(names.count("BalanceStand") == 1 and p.calls("BalanceStand")[0]["timeout"] == 10.0, "BalanceStand")
    check(all(c["timeout"] == P.RPC_TIMEOUT for c in p.calls("Move")), "a Move ran with timeout != 1 s")
    check(len(p.moves()) > 1000, f"only {len(p.moves())} Move calls")
    assert_stopped_after_last_move(p)
    # each trial ends with a confirmed StopMove before the next trial's first Move
    move_idx = [i for i, c in enumerate(calls) if c["call"] == "Move"]
    gaps = [i for a, i in zip(move_idx, move_idx[1:]) if i - a > 1]
    for i in gaps:
        check("StopMove" in names[move_idx[0]:i], "no StopMove between the two trials")
    # request listeners are never Close()d (see RequestWatch.close), and saw nothing from other programs
    check(not p.calls("ChannelSubscriber.Close"), f"a subscriber was closed: {p.calls('ChannelSubscriber.Close')}")
    check("none seen" in text, "no 'none seen' for other programs")
    # blinding: the profile is never shown (block_order makes trials 3-4 the complement of 1-2),
    # only the counts in the Block line
    block = first_line_with(text, "Block:")
    check(block and "seed=7" in block and "order" not in block and "1 step, 1 smooth" in block, block)
    after = text[text.index(block) + len(block):].replace(work, "WORK")   # paths are not output
    check(not re.search(r"step|smooth", after), f"profile visible during the session: {after!r}")
    segs = []
    for k in (1, 2):
        seg = text[text.index(f"trial {k:03d}  turn="):]
        seg = seg[:seg.index("note (Enter to skip)")]
        segs.append(re.sub(r"turn=[LR]", "turn=X", re.sub(r"trial[ _]\d{3}", "trial N", seg)))
    # nothing else (row counts, durations ...) may differ between the step and the smooth trial
    check(segs[0] == segs[1], f"operator output differs by profile before the ratings: {segs}")
    check("clock:" in text, "no clock line")


def test_out_and_iface_also_after_the_mode(work):
    # 'run --stick test --reps 1 --out data/teste'; given twice, the one after the mode wins
    p = Pilot(work, ["run", "--stick", "test", "--reps", "1", "--out", os.path.join(work, "teste"),
                     "--iface", "fake1"], stdin="q\n")
    check(p.finish() == 0, p.tail())
    check(trials(os.path.join(work, "teste")) == [] and not os.path.exists(p.out), f"{os.listdir(work)}\n{p.tail()}")
    check(p.calls("ChannelFactoryInitialize")[0]["args"] == ["fake1"], "--iface after the mode ignored")


def test_unknown_answers_never_start(work):
    words = ["sair", "quit", "skip", "exit", "no", "pare", "go", "yes", " Sair "]
    p = Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="\n".join(words) + "\nQ\n")
    rc = p.finish()
    check(rc == 0, f"exit {rc}\n{p.tail()}")
    check(not p.calls("Move"), f"{len(p.calls('Move'))} Move calls after unknown answers")
    check(p.output.count("type only Enter (start), s (skip) or q (quit)") == len(words), p.tail())
    check(not os.path.exists(os.path.join(p.out, "trial_001")), "trial folder created")
    check(trials(p.out) == [], "rows written")
    assert_stopped_after_last_move(p)


def test_eof_at_start_prompt_quits(work):
    p = Pilot(work, ["run", "--stick", "test", "--reps", "1"])
    p.close_stdin()
    rc = p.finish()
    check(rc == 0 and not p.calls("Move"), f"exit {rc}, moves {len(p.calls('Move'))}\n{p.tail()}")
    check("no more input" in p.output, p.tail())
    assert_stopped_after_last_move(p)


def test_skip_and_numbering_never_reused(work):
    a = Pilot(work, ["run", "--stick", "rigid", "--reps", "1"], stdin="s\n\n\n\n\n", name="a")
    check(a.finish() == 0, a.tail())
    rows = trials(a.out)
    check([(r["trial"], r["note"]) for r in rows] == [("1", "skipped"), ("2", "")], f"block A rows {rows}")
    check(all(rows[0][k] == "" for k in T_FIELDS), f"skipped row has times: {rows[0]}")
    check(not os.path.exists(os.path.join(a.out, "trial_001")), "skipped trial has a folder")
    before = md5(os.path.join(a.out, "trial_002", "cmd.csv"))
    # an orphan folder (e.g. left by a SIGKILL) must not be reused either
    os.makedirs(os.path.join(a.out, "trial_004"))
    open(os.path.join(a.out, "trial_004", "cmd.csv"), "w").write("orphan\n")
    b = Pilot(work, ["run", "--stick", "flexible", "--reps", "1"], stdin="\n\n\n\nq\n", name="b")
    check(b.finish() == 0, b.tail())
    check("trial 005  turn=L" in b.output, f"block B did not start at 005\n{b.tail()}")
    rows = trials(b.out)
    nums = [r["trial"] for r in rows]
    check(nums == ["1", "2", "5"] and len(set(nums)) == len(nums), f"numbers {nums}")
    check(rows[2]["stick"] == "flexible" and rows[2]["t_turn_start"], f"row {rows[2]}")
    check(md5(os.path.join(a.out, "trial_002", "cmd.csv")) == before, "trial_002 was overwritten")
    check(open(os.path.join(a.out, "trial_004", "cmd.csv")).read() == "orphan\n", "orphan folder touched")
    assert_stopped_after_last_move(b)


def test_header_flushed_and_zero_byte_file(work):
    p = Pilot(work, ["run", "--stick", "test", "--reps", "1"], name="a")
    p.wait_text("Enter to start")
    tfile = os.path.join(p.out, "trials.csv")
    check(open(tfile, newline="").read() == ",".join(TRIAL_COLS) + "\r\n", "header not on disk before the first trial")
    p.kill(signal.SIGKILL)
    p.finish()
    check(read_csv(tfile) == [TRIAL_COLS], "header lost after SIGKILL")
    q = Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="q\n", name="b")
    check(q.finish() == 0 and "trial 001  turn=L" in q.output, q.tail())
    check(read_csv(tfile) == [TRIAL_COLS], "header duplicated")
    # a 0-byte trials.csv (left by the old script) is treated as new
    out2 = os.path.join(work, "out2")
    os.makedirs(out2)
    open(os.path.join(out2, "trials.csv"), "w").close()
    z = Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="q\n", out="out2", name="c")
    check(z.finish() == 0 and "trial 001  turn=L" in z.output, z.tail())
    check(read_csv(os.path.join(out2, "trials.csv")) == [TRIAL_COLS], "0-byte file did not get the header")
    # a hand-edited trials.csv saved without a final newline: the next row must not be glued on
    out3 = os.path.join(work, "out3")
    os.makedirs(out3)
    with open(os.path.join(out3, "trials.csv"), "w", newline="") as f:
        f.write(",".join(TRIAL_COLS) + "\r\n1,rigid,step,L,,,,,,,skipped")
    e = Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="s\nq\n", out="out3", name="d")
    check(e.finish() == 0 and "trial 002  turn=L" in e.output, e.tail())
    check([(r["trial"], r["note"]) for r in trials(os.path.join(work, "out3"))] == [("1", "skipped"), ("2", "skipped")],
          f"rows {read_csv(os.path.join(out3, 'trials.csv'))}")


def _signal_in_final_stop(work, sig):
    """A signal while the END-of-trial StopMove waits for its reply: the completed trial keeps its row."""
    p = Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="\n", env={"FAKE_STOP_DELAY": "0.6"})
    p.wait_for(lambda: p.calls("StopMove.begin"), "end-of-trial StopMove", timeout=60)
    time.sleep(0.2)
    p.kill(sig)
    rc = p.finish(30)
    check(rc == 130, f"exit code {rc} (expected 130)\n{p.tail()}")
    check(len(p.moves()) > 500, f"the stop was not the end-of-trial one ({len(p.moves())} Moves)")
    rows = trials(p.out)
    check(len(rows) == 1 and all(rows[0][k] for k in T_FIELDS) and not rows[0]["note"], f"completed row lost: {rows}")
    ev = assert_trial_files(os.path.join(p.out, "trial_001"), min_cmd=500)
    check(ev["aborted"] is None and abs(float(rows[0]["t_end"]) - ev["events"]["still2_end"]) < 1e-6, f"events {ev}")
    check("ABORTED" not in p.output and "cue clarity" not in p.output, p.tail())
    check("no reply, the stop may still have arrived" in p.output, p.tail())
    assert_stopped_after_last_move(p)


def test_sigterm_in_final_stop_keeps_completed_trial(work):
    _signal_in_final_stop(work, signal.SIGTERM)


def test_sigint_in_final_stop_keeps_completed_trial(work):
    _signal_in_final_stop(work, signal.SIGINT)


def _interrupt_mid_trial(work, sig):
    p = Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="\n")
    p.wait_for(lambda: len(p.moves()) >= MID_TURN_MOVES, "mid-turn")
    p.kill(sig)
    rc = p.finish(30)
    check(rc == 130, f"exit code {rc} (expected 130)\n{p.tail()}")
    assert_stopped_after_last_move(p)
    rows = trials(p.out)
    check(len(rows) == 1 and rows[0]["note"] == "aborted: interrupted", f"rows {rows}")
    check(all(rows[0][k] == "" for k in T_FIELDS), f"aborted row has times {rows[0]}")
    ev = assert_trial_files(os.path.join(p.out, "trial_001"), min_cmd=MID_TURN_MOVES)
    check(ev["aborted"] == "interrupted" and "turn_start" in ev["events"], f"events.json {ev}")
    check("ABORTED: interrupted" in p.output, p.tail())


def test_sigterm_mid_trial_stops_and_saves(work):
    _interrupt_mid_trial(work, signal.SIGTERM)


def test_sigint_mid_trial_stops_and_saves(work):
    _interrupt_mid_trial(work, signal.SIGINT)


def test_sighup_mid_trial_stops_and_saves(work):
    _interrupt_mid_trial(work, signal.SIGHUP)


def test_repeated_sigterm_cannot_cut_the_stop(work):
    # StopMove reply takes 0.5 s; a second and third SIGTERM arrive during the cleanup
    p = Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="\n", env={"FAKE_STOP_DELAY": "0.5"})
    p.wait_for(lambda: len(p.moves()) >= MID_TURN_MOVES, "mid-turn")
    p.kill(signal.SIGTERM)
    p.wait_for(lambda: p.calls("StopMove.begin"), "first StopMove")
    p.kill(signal.SIGTERM)
    time.sleep(0.1)
    p.kill(signal.SIGHUP)
    rc = p.finish(30)
    check(rc == 130, f"exit {rc}\n{p.tail()}")
    stops = assert_stopped_after_last_move(p)
    check(stops[0]["ret"] == 0, f"first StopMove was cut short: {stops}")
    check(trials(p.out)[0]["note"] == "aborted: interrupted", "row")


def test_interrupt_at_ratings_keeps_completed_trial(work):
    p = Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="\n7\n")
    p.wait_text("comfort 1-7")
    p.kill(signal.SIGINT)
    rc = p.finish(30)
    check(rc == 130, f"exit {rc}\n{p.tail()}")
    rows = trials(p.out)
    check(len(rows) == 1 and all(rows[0][k] for k in T_FIELDS), f"completed row lost: {rows}")
    check(rows[0]["clarity_1to7"] == "7" and rows[0]["note"] == "", f"row {rows[0]}")
    assert_trial_files(os.path.join(p.out, "trial_001"), min_cmd=500)
    assert_stopped_after_last_move(p)


def test_interrupt_in_countdown_writes_aborted_row(work):
    p = Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="\n")
    p.wait_text("q=quit): 3 ")
    p.kill(signal.SIGINT)
    check(p.finish(30) == 130, p.tail())
    check(not p.calls("Move"), "moved")
    rows = trials(p.out)
    check(len(rows) == 1 and rows[0]["note"] == "aborted: interrupted" and not rows[0]["t_start"], f"{rows}")
    assert_stopped_after_last_move(p)


def _aborted_trial(work, env, reason, min_cmd=1):
    p = Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="\n\n\nmine\nq\n", env=env)
    rc = p.finish()
    check(rc == 0, f"exit {rc}\n{p.tail()}")
    rows = trials(p.out)
    check(len(rows) == 1 and rows[0]["note"].startswith(f"aborted: {reason}") and rows[0]["note"].endswith("mine"),
          f"rows {rows}")
    check(all(rows[0][k] == "" for k in T_FIELDS), f"aborted row has times {rows[0]}")
    check(f"ABORTED: {reason}" in p.output, p.tail())
    ev = assert_trial_files(os.path.join(p.out, "trial_001"), min_cmd=min_cmd)
    check(ev["aborted"].startswith(reason), f"events.json {ev}")
    assert_stopped_after_last_move(p)
    return p


def test_move_failure_aborts_trial(work):
    p = _aborted_trial(work, {"FAKE_MOVE_FAIL_AFTER": "100"}, "Move not sent (code 3102)", min_cmd=100)
    cmd = read_csv(os.path.join(p.out, "trial_001", "cmd.csv"))
    check(len(cmd) - 1 == 100, f"cmd.csv logged {len(cmd) - 1} commands, 100 were sent")
    failed = [c for c in p.calls("Move") if c["ret"] != 0]
    check(len(failed) == 1 and failed[0]["timeout"] == P.RPC_TIMEOUT, f"failed Moves {failed}")


def test_stale_lowstate_aborts_trial(work):
    p = _aborted_trial(work, {"FAKE_LOW_STOP_AFTER_MOVES": "150"}, "lowstate stream stale", min_cmd=150)
    check(len(p.moves()) < 150 + 0.4 * P.HZ, f"{len(p.moves())} Moves: stale lowstate noticed too late")


def test_stalled_loop_aborts_trial(work):
    p = _aborted_trial(work, {"FAKE_MOVE_DELAY": "100:0.4"}, "control loop stalled 0.4", min_cmd=100)
    check(len(p.moves()) == 101, f"{len(p.moves())} Moves: the loop went on after the stall")


def test_stale_sportmodestate_aborts_trial(work):
    _aborted_trial(work, {"FAKE_SPORT_STOP_AFTER_MOVES": "120"}, "sportmodestate stream stale", min_cmd=120)


def test_tilt_aborts_trial(work):
    _aborted_trial(work, {"FAKE_TILT_AFTER_MOVES": "80"}, "tilt too large", min_cmd=80)


def test_stopmove_retries_and_warns(work):
    p = Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="q\n", env={"FAKE_STOP_FAIL": "3"}, name="a")
    check(p.finish() == 0, p.tail())
    stops = p.calls("StopMove")
    check([c["ret"] for c in stops] == [3104] * 3, f"StopMove attempts {stops}")
    check(all(c["timeout"] == P.RPC_TIMEOUT for c in stops), "StopMove timeout != 1 s")
    check("StopMove NOT CONFIRMED - stop the robot with the remote NOW" in p.output, p.tail())
    check("attempt 1/3" in p.output.split("NOT CONFIRMED")[0], "no warning on the first failure")
    q = Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="q\n", env={"FAKE_STOP_FAIL": "2"},
              out="out2", name="b")
    check(q.finish() == 0, q.tail())
    check([c["ret"] for c in q.calls("StopMove")] == [3104, 3104, 0], "third attempt should succeed")
    check("NOT CONFIRMED" not in q.output, "NOT CONFIRMED printed although the 3rd StopMove worked")


def test_balancestand_warning_and_signals_during_it(work):
    p = Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="q\n", env={"FAKE_BALANCE_CODE": "7"}, name="a")
    check(p.finish() == 0 and "BalanceStand returned code 7" in p.output, p.tail())
    # SIGTERM or Ctrl-C while BalanceStand waits for its reply: the SDK swallows the KeyboardInterrupt
    for sig, name in ((signal.SIGTERM, "b"), (signal.SIGINT, "c")):
        q = Pilot(work, ["run", "--stick", "test", "--reps", "1"], env={"FAKE_BALANCE_DELAY": "4"},
                  out="out_" + name, name=name)
        q.wait_for(lambda: q.calls("BalanceStand.begin"), "BalanceStand")
        time.sleep(0.3)
        t_sig = time.monotonic()
        q.kill(sig)
        rc = q.finish(30)
        check(rc == 130 and time.monotonic() - t_sig < 3,
              f"{sig!r}: exit {rc} after {time.monotonic() - t_sig:.1f} s\n{q.tail()}")
        check(not q.calls("Move") and "Enter to start" not in q.output, f"{sig!r}: went on to the trials\n{q.tail()}")
        assert_stopped_after_last_move(q)


def test_other_program_commands_refuse_to_move(work):
    env = {"FAKE_FOREIGN": "sport:1008"}
    p = Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="yes\n\n", env=env, name="a")
    rc = p.finish()
    check(rc == 1 and "Exiting without moving" in p.output, f"exit {rc}\n{p.tail()}")
    check("rt/api/sport/request" in p.output and "1008 (Move)" in p.output and "RollCall" in p.output, p.tail())
    names = [c["call"] for c in p.calls()]
    check("SportClient.__init__" not in names, "SportClient created although the operator refused")
    check(not os.path.exists(os.path.join(p.out, "trials.csv")), "trials.csv created")
    # end of input at the 'continue' prompt also refuses
    q = Pilot(work, ["run", "--stick", "test", "--reps", "1"], env=env, out="out2", name="b")
    q.close_stdin()
    check(q.finish() == 1 and not any(c["call"] == "SportClient.__init__" for c in q.calls()), q.tail())


def test_other_program_commands_continue(work):
    p = Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="continue\nq\n",
              env={"FAKE_FOREIGN": "sport:1008,obstacles_avoid:1003"})
    check(p.finish() == 0, p.tail())
    names = [c["call"] for c in p.calls()]
    # foreign requests keep arriving: no listener may be Close()d under them (RequestWatch.close)
    check("ChannelSubscriber.Close" not in names, "a request listener was closed")
    check("BalanceStand" in names and not p.calls("Move"), "should stand and then quit without moving")
    check("rt/api/obstacles_avoid/request" in p.output, p.tail())
    assert_stopped_after_last_move(p)


def test_request_watch_ignores_everything_after_close(work):
    # in a subprocess with the fake SDK: our own SportClient requests (sent after close()) are not recorded
    code = "\n".join([
        "import go2_turn_pilot as P",
        "from unitree_sdk2py.core.channel import ChannelFactoryInitialize",
        "from unitree_sdk2py._fake_robot import ROBOT",
        "ChannelFactoryInitialize(0, 'fake0')",
        "w = P.RequestWatch()",
        "ROBOT.request(1008, '{}', 1)",
        "w.close()",
        "ROBOT.request(1003, '{}', 2)",
        "print('seen', [r[1] for r in w.seen])"])
    env = dict(offline_env(), PYTHONPATH=os.pathsep.join([FAKE_SDK, REPO]),
               FAKE_CALLS=os.path.join(work, "calls.jsonl"))
    r = subprocess.run([PY, "-c", code], capture_output=True, text=True, timeout=30, cwd=work, env=env)
    check(r.returncode == 0 and "seen [1008]" in r.stdout, f"{r.stdout}\n{r.stderr}")


# ------------------------------------------------------------------ check / sync / clock
def test_check_reports(work):
    p = Pilot(work, ["check"], name="a")
    check(p.finish() == 0, p.tail())
    t = p.output
    for s in ("sportmodestate:", "battery power:", "mode: 1   gait_type: 0", "none seen (expected)", "clock: 20"):
        check(s in t, f"check output lacks {s!r}\n{p.tail()}")
    check(not any(c["call"] == "SportClient.__init__" for c in p.calls()), "check created a SportClient")
    q = Pilot(work, ["check"], env={"FAKE_FOREIGN": "sport:1008,obstacles_avoid:1003"}, name="b")
    check(q.finish() == 0, q.tail())
    for s in ("rt/api/sport/request:", "api_id 1008 (Move)", "rt/api/obstacles_avoid/request:",
              "api_id 1003 (Move)", 'sample parameter: {"x": 0.2', "RollCall"):
        check(s in q.output, f"check output lacks {s!r}\n{q.tail()}")


def test_clock_1970_warning_in_all_modes(work):
    env = {"FAKE_WALL_OFFSET": "-1.7e9"}
    runs = [Pilot(work, ["check"], env=env, name="a"),
            Pilot(work, ["sync", "--tag", "t", "--seconds", "1"], env=env, name="b"),
            Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="q\n", env=env, out="out2", name="c")]
    for p in runs:
        check(p.finish() == 0, p.tail())
        check("!! system clock reads 197" in p.output, f"no 1970 warning\n{p.tail()}")


def test_clock_step_is_reported_and_noted(work):
    p = Pilot(work, ["run", "--stick", "test", "--reps", "1"], stdin="\n\n\n\nq\n",
              env={"FAKE_CLOCK_STEP": "300:1.5"})
    check(p.finish() == 0, p.tail())
    check("system clock stepped by +1.500 s" in p.output and "timedatectl" in p.output, p.tail())
    row = trials(p.out)[0]
    check("clock step +1.500 s" in row["note"] and row["t_turn_start"], f"row {row}")
    # control timing is monotonic: the trial still lasts ~10.6 s and turns ~90 deg
    moves = p.moves()
    dur = moves[-1]["mono"] - moves[0]["mono"]
    ang = math.degrees(sum(a["args"][2] * (b["mono"] - a["mono"]) for a, b in zip(moves, moves[1:])))
    check(abs(dur - 10.6) < 0.3 and abs(ang - 90) < 3, f"duration {dur:.2f} s, turn {ang:.1f} deg")


def test_sync_saves_renames_and_finds_knocks(work):
    env = {"FAKE_KNOCKS": "1.0,1.45,1.9,3.3,3.7"}
    a = Pilot(work, ["sync", "--tag", "start", "--seconds", "5"], env=env, name="a")
    check(a.finish() == 0, a.tail())
    d = os.path.join(a.out, "sync_start")
    low = read_csv(os.path.join(d, "low.csv"))
    check(low[0] == LOW_COLS and len(low) > 1000, f"low.csv {len(low)} rows")
    check(read_csv(os.path.join(d, "sport.csv"))[0] == SPORT_COLS, "sport.csv header")
    meta = json.load(open(os.path.join(d, "meta.json")))
    check({"tag", "t_saved", "wall_minus_monotonic_s"} <= set(meta) and meta["interrupted"] is False, f"meta {meta}")
    m = re.search(r"knocks: (\d+) peaks", a.output)
    check(m and m.group(1) == "5", f"knock diagnostic\n{a.tail()}")
    times = [float(x) for x in re.search(r"at t = ([\d., ]+) s after", a.output).group(1).split(",")]
    check(all(abs(x - y) < 0.15 for x, y in zip(times, [1.0, 1.45, 1.9, 3.3, 3.7])), f"knock times {times}")
    check(re.search(r"top-5 peak heights: (1[45]\.\d+, ){4}1[45]\.\d+ m/s", a.output), a.tail())
    first = md5(os.path.join(d, "low.csv"))
    # an accidental re-run of the same tag (e.g. 'start' from the shell history) records nothing ...
    for name, stdin in (("b", "\n"), ("c", "yes\n"), ("d", "")):
        x = Pilot(work, ["sync", "--tag", "start", "--seconds", "1"], stdin=stdin, name=name)
        x.close_stdin()
        check(x.finish() == 1 and "already exists" in x.output and "Nothing recorded" in x.output, x.tail())
        check(not any(c["call"] == "ChannelFactoryInitialize" for c in x.calls()), "connected before asking")
    check(md5(os.path.join(d, "low.csv")) == first and sorted(os.listdir(a.out)) == ["sync_start"],
          f"sync_start touched: {os.listdir(a.out)}")
    # ... unless 'replace' is typed; the old recording is kept as sync_start_old_*
    e = Pilot(work, ["sync", "--tag", "start", "--seconds", "1"], stdin="replace\n", name="e")
    check(e.finish() == 0, e.tail())
    olds = [x for x in os.listdir(a.out) if x.startswith("sync_start_old_")]
    check(len(olds) == 1 and "renamed the old one to" in e.output, f"{os.listdir(a.out)}\n{e.tail()}")
    check(md5(os.path.join(a.out, olds[0], "low.csv")) == first, "old sync data changed")
    check(md5(os.path.join(d, "low.csv")) != first, "new sync not written")
    check("fewer than 4 clear knocks" in e.output, "no hint for a sync without knocks")


def _sync_interrupted(work, sig):
    p = Pilot(work, ["sync", "--tag", "x", "--seconds", "30"])
    p.wait_text("Logging")
    time.sleep(1.5)
    p.kill(sig)
    rc = p.finish(15)
    check(rc == 0 and time.monotonic() - p.t_start < 10, f"exit {rc}\n{p.tail()}")
    d = os.path.join(p.out, "sync_x")
    check(len(read_csv(os.path.join(d, "low.csv"))) > 200, "low.csv empty")
    meta = json.load(open(os.path.join(d, "meta.json")))
    check(meta["interrupted"] is True and "stopped early" in p.output and "knocks:" in p.output, p.tail())


def test_sync_sigint_still_saves(work):
    _sync_interrupted(work, signal.SIGINT)


def test_sync_sigterm_still_saves(work):
    _sync_interrupted(work, signal.SIGTERM)


def test_typeahead_on_a_terminal_is_dropped(work):
    p = Pilot(work, ["run", "--stick", "test", "--reps", "1"], tty=True)
    p.wait_text("trial 001  turn=L")
    check("stdin is not a terminal" not in p.output, "terminal reported as not a terminal")
    p.send("\r")
    p.wait_for(lambda: len(p.moves()) >= 100, "trial running")
    p.send("\r\r\r\r")   # impatient Enters while the robot moves
    p.wait_text("cue clarity")
    n_moves = len(p.moves())
    time.sleep(2.5)
    check(len(p.moves()) == n_moves and "comfort 1-7" not in p.output,
          f"stray Enters were used as answers\n{p.tail()}")
    p.send("5\r4\rok\r\r\r\r\r")   # ratings, then a burst of Enters right after
    p.wait_text("trial 002  turn=R")
    time.sleep(2.0)
    check(len(p.moves()) == n_moves and "3 2 1" not in p.output.split("trial 002")[1],
          f"a stray Enter started trial 2\n{p.tail()}")
    p.send("q\r")
    check(p.finish(30) == 0, p.tail())
    rows = trials(p.out)
    check(len(rows) == 1 and (rows[0]["clarity_1to7"], rows[0]["comfort_1to7"], rows[0]["note"]) == ("5", "4", "ok"),
          f"rows {rows}")
    assert_stopped_after_last_move(p)


# ------------------------------------------------------------------ offline pipeline
def test_simulator_and_analysis_still_work(work):
    sim = os.path.join(work, "sim")
    env = offline_env()   # simulate_session.py imports go2_turn_pilot: no .pyc in the repo
    r = subprocess.run([PY, os.path.join(REPO, "test", "simulate_session.py"), sim], capture_output=True,
                       text=True, timeout=300, env=env)
    check(r.returncode == 0, f"simulate_session.py failed:\n{r.stdout[-1500:]}\n{r.stderr[-1500:]}")
    check(len(trials(sim)) == 20, "simulated trials.csv")
    r = subprocess.run([PY, os.path.join(REPO, "analyze_pilot.py"), sim], capture_output=True, text=True,
                       timeout=600, env=env)
    check(r.returncode == 0, f"analyze_pilot.py failed:\n{r.stdout[-1500:]}\n{r.stderr[-1500:]}")
    check(os.path.exists(os.path.join(sim, "results", "trials.csv")), "no results/trials.csv")


# ------------------------------------------------------------------ runner
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-k", default="", help="run only tests whose name contains this")
    ap.add_argument("-j", type=int, default=8, help="tests run in parallel (default 8)")
    ap.add_argument("--keep", action="store_true", help="keep the scratch directory")
    args = ap.parse_args()
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and args.k in n]
    root = tempfile.mkdtemp(prefix="go2_pilot_tests_")
    print(f"{len(tests)} tests, scratch dir {root}", flush=True)

    def run(item):
        name, fn = item
        work = os.path.join(root, name)
        os.makedirs(work)
        t0 = time.monotonic()
        try:
            fn(work)
            return name, None, time.monotonic() - t0
        except Exception:
            return name, traceback.format_exc(), time.monotonic() - t0

    failed = []
    with concurrent.futures.ThreadPoolExecutor(args.j) as ex:
        for name, err, dt in ex.map(run, tests):
            print(f"{'PASS' if err is None else 'FAIL'}  {name}  ({dt:.1f} s)", flush=True)
            if err:
                failed.append(name)
                print("    " + err.replace("\n", "\n    "), flush=True)
    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed" + (f"; FAILED: {', '.join(failed)}" if failed else ""))
    if not failed and not args.keep:
        shutil.rmtree(root, ignore_errors=True)
    else:
        print(f"scratch kept in {root}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
