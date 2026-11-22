"""Simulated Go2 behind the FAKE unitree_sdk2py (tests only; pure Python, no sockets, no DDS).

A thread publishes SportModeState_ / LowState_ to the fake subscribers; SportClient calls change
the commanded velocity and are all recorded. Behaviour is set with environment variables:

  FAKE_CALLS=path               append every SDK call as one JSON line (call, args, ret, t, mono, timeout)
  FAKE_RATE_HZ=250              state publishing rate
  FAKE_SPORT_STOP_AFTER_MOVES=N stop publishing sportmodestate after N Move calls
  FAKE_LOW_STOP_AFTER_MOVES=N   stop publishing lowstate after N Move calls
  FAKE_MOVE_FAIL_AFTER=N        after N good Moves, Move blocks for the client timeout and returns 3102
  FAKE_MOVE_DELAY=N:S           Move number N+1 takes S s but succeeds (a stalled control loop)
  FAKE_STOP_FAIL=N              the first N StopMove calls time out (wait the client timeout, return 3104)
  FAKE_STOP_DELAY=S             StopMove reply arrives after S s
  FAKE_BALANCE_DELAY=S          BalanceStand reply arrives after S s
  FAKE_BALANCE_CODE=C           BalanceStand return code (default 0)
  FAKE_TILT_AFTER_MOVES=N       roll jumps to 0.5 rad after N Move calls
  FAKE_FOREIGN=sport:1008,...   another program publishes these requests (service:api_id) at 20 Hz
  FAKE_KNOCKS=1.0,1.45,...      lowstate accelerometer spikes (+15 m/s^2) at these s after init
  FAKE_CLOCK_STEP=N:S           time.time() jumps by S s after N Move calls (monotonic does not)
  FAKE_WALL_OFFSET=S            time.time() is shifted by S s from import (e.g. -1.7e9 -> 1970)

Like the real SDK (utils/future.py), a KeyboardInterrupt during a reply wait is swallowed and
the call returns 3104.
"""
import json, math, os, sys, threading, time

RPC_ERR_CLIENT_SEND = 3102
RPC_ERR_CLIENT_API_TIMEOUT = 3104


def _env(name, cast, default=None):
    v = os.environ.get(name, "")
    return cast(v) if v.strip() else default


CFG = {
    "calls": os.environ.get("FAKE_CALLS"),
    "rate": _env("FAKE_RATE_HZ", float, 250.0),
    "sport_stop": _env("FAKE_SPORT_STOP_AFTER_MOVES", int),
    "low_stop": _env("FAKE_LOW_STOP_AFTER_MOVES", int),
    "move_fail": _env("FAKE_MOVE_FAIL_AFTER", int),
    "move_delay": _env("FAKE_MOVE_DELAY", lambda v: (int(v.split(":")[0]), float(v.split(":")[1]))),
    "stop_fail": _env("FAKE_STOP_FAIL", int, 0),
    "stop_delay": _env("FAKE_STOP_DELAY", float, 0.0),
    "balance_delay": _env("FAKE_BALANCE_DELAY", float, 0.0),
    "balance_code": _env("FAKE_BALANCE_CODE", int, 0),
    "tilt": _env("FAKE_TILT_AFTER_MOVES", int),
    "foreign": [x.split(":") for x in os.environ.get("FAKE_FOREIGN", "").split(",") if x.strip()],
    "knocks": [float(x) for x in os.environ.get("FAKE_KNOCKS", "").split(",") if x.strip()],
    "clock_step": _env("FAKE_CLOCK_STEP", lambda v: (int(v.split(":")[0]), float(v.split(":")[1]))),
    "wall_offset": _env("FAKE_WALL_OFFSET", float, 0.0),
}

# --- optional wall-clock manipulation (time.time only; time.monotonic is never touched)
_real_time = time.time
_wall = {"offset": CFG["wall_offset"]}
if CFG["wall_offset"] or CFG["clock_step"]:
    time.time = lambda: _real_time() + _wall["offset"]


class Robot:
    def __init__(self):
        self.lock = threading.Lock()
        self.log_lock = threading.Lock()
        self.handlers = {}          # topic -> list of handlers
        self.initialized = False
        self.t_init = None
        self.cmd = (0.0, 0.0, 0.0)  # commanded vx, vy, wz, held until the next Move / StopMove
        self.yaw = self.px = self.py = 0.0
        self.moves = 0
        self.stops = 0

    # ------------------------------------------------------------ call log
    def log(self, call, args=(), ret=None, timeout=None):
        if not CFG["calls"]:
            return
        rec = {"call": call, "args": list(args), "ret": ret, "t": time.time(),
               "mono": time.monotonic(), "timeout": timeout}
        with self.log_lock, open(CFG["calls"], "a") as f:
            f.write(json.dumps(rec) + "\n")

    # ------------------------------------------------------------ pub/sub
    def start(self, iface):
        with self.lock:
            if self.initialized:
                return
            self.initialized = True
            self.t_init = time.monotonic()
        self.log("ChannelFactoryInitialize", [iface])
        threading.Thread(target=self._state_loop, name="fake_robot", daemon=True).start()
        if CFG["foreign"]:
            threading.Thread(target=self._foreign_loop, name="fake_other_program", daemon=True).start()

    def subscribe(self, topic, handler):
        with self.lock:
            self.handlers.setdefault(topic, []).append(handler)

    def unsubscribe(self, topic, handler):
        with self.lock:
            self.handlers[topic].remove(handler)

    def publish(self, topic, msg):
        with self.lock:
            hs = list(self.handlers.get(topic, []))
        for h in hs:
            try:
                h(msg)
            except Exception as e:   # keep publishing, like a separate reader thread would
                print(f"[fake robot] handler error on {topic}: {e!r}", file=sys.stderr)

    # ------------------------------------------------------------ simulated robot
    def _state_loop(self):
        from .idl.unitree_go.msg.dds_ import SportModeState_, LowState_
        dt = 1.0 / CFG["rate"]
        nxt = time.monotonic()
        while True:
            with self.lock:
                vx, _, wz = self.cmd
                self.yaw += wz * dt
                self.px += vx * math.cos(self.yaw) * dt
                self.py += vx * math.sin(self.yaw) * dt
                moves, yaw, px, py = self.moves, self.yaw, self.px, self.py
            walking = abs(vx) > 1e-6 or abs(wz) > 1e-6
            roll = 0.5 if CFG["tilt"] is not None and moves >= CFG["tilt"] else 0.01
            since = time.monotonic() - self.t_init
            if CFG["sport_stop"] is None or moves < CFG["sport_stop"]:
                m = SportModeState_()
                m.mode, m.gait_type = (3, 1) if walking else (1, 0)
                m.yaw_speed = wz
                m.imu_state.gyroscope = [0.0, 0.0, wz]
                m.imu_state.rpy = [roll, -0.01, yaw]
                m.velocity = [vx, 0.0, 0.0]
                m.position = [px, py, 0.3]
                self.publish("rt/sportmodestate", m)
            if CFG["low_stop"] is None or moves < CFG["low_stop"]:
                m = LowState_()
                knock = any(abs(since - k) < 0.008 for k in CFG["knocks"])
                m.imu_state.accelerometer = [0.02, -0.01, 9.81 + (15.0 if knock else 0.0)]
                m.imu_state.gyroscope = [0.0, 0.0, wz]
                m.power_v, m.power_a = 28.5, (3.0 if walking else 1.5)
                for i in range(12):
                    m.motor_state[i].tau_est = 4.0 if walking else 2.0
                    m.motor_state[i].dq = 2.0 if walking else 0.01
                self.publish("rt/lowstate", m)
            nxt += dt
            time.sleep(max(0.0, nxt - time.monotonic()))

    def _foreign_loop(self):
        from .idl.unitree_api.msg.dds_ import Request_
        k = 0
        while True:
            for service, api_id in CFG["foreign"]:
                self.publish(f"rt/api/{service}/request",
                             Request_.make(int(api_id), json.dumps({"x": 0.2, "y": 0.0, "z": 0.1}), k))
                k += 1
            time.sleep(0.05)

    # ------------------------------------------------------------ sport service
    def request(self, api_id, parameter, client_id):
        """Our own client's request is delivered to any local reader, as DDS would do."""
        from .idl.unitree_api.msg.dds_ import Request_
        self.publish("rt/api/sport/request", Request_.make(api_id, parameter, client_id))

    def move(self, vx, vy, wz, timeout):
        with self.lock:
            fail = CFG["move_fail"] is not None and self.moves >= CFG["move_fail"]
        if fail:   # writer has no matched reader: the SDK busy-waits `timeout`, then gives up
            time.sleep(timeout)
            return RPC_ERR_CLIENT_SEND
        if CFG["move_delay"] and self.moves == CFG["move_delay"][0]:
            time.sleep(CFG["move_delay"][1])
        with self.lock:
            self.cmd = (vx, vy, wz)
            self.moves += 1
            n = self.moves
        if CFG["clock_step"] and n == CFG["clock_step"][0]:
            _wall["offset"] += CFG["clock_step"][1]
        return 0

    def stop_move(self, timeout):
        with self.lock:
            self.stops += 1
            fail = self.stops <= CFG["stop_fail"]
        if fail:   # no reply ever comes (and the robot did not get it)
            reply_wait(timeout)
            return RPC_ERR_CLIENT_API_TIMEOUT
        with self.lock:   # the SDK writes the request first, then waits for the reply
            self.cmd = (0.0, 0.0, 0.0)
        delay = CFG["stop_delay"]
        if delay and (not reply_wait(min(delay, timeout)) or delay > timeout):
            return RPC_ERR_CLIENT_API_TIMEOUT
        return 0

    def balance_stand(self, timeout):
        delay = CFG["balance_delay"]
        if delay and (not reply_wait(min(delay, timeout)) or delay > timeout):
            return RPC_ERR_CLIENT_API_TIMEOUT
        return CFG["balance_code"]


def reply_wait(seconds):
    """Wait like utils/future.py Future.__Wait: any exception (Ctrl-C) is swallowed -> False."""
    try:
        time.sleep(seconds)
        return True
    except BaseException:
        print("[Future] future wait error", flush=True)
        return False


ROBOT = Robot()
