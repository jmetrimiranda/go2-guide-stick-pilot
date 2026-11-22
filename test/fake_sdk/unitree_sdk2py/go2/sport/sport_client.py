"""FAKE unitree_sdk2py.go2.sport.sport_client.SportClient: records every call (FAKE_CALLS)."""
import json, time

from ..._fake_robot import ROBOT

SPORT_API_ID_BALANCESTAND = 1002
SPORT_API_ID_STOPMOVE = 1003
SPORT_API_ID_MOVE = 1008


class SportClient:
    def __init__(self, enableLease: bool = False):
        if not ROBOT.initialized:
            raise RuntimeError("fake SDK: ChannelFactoryInitialize must be called first")
        self._timeout = 1.0   # SDK default (rpc/client_base.py)
        self._id = 0
        ROBOT.log("SportClient.__init__")
        time.sleep(0.05)

    def _req(self, api_id, parameter):
        self._id += 1
        ROBOT.request(api_id, parameter, self._id)

    def SetTimeout(self, timeout: float):
        self._timeout = timeout
        ROBOT.log("SetTimeout", [timeout])

    def Init(self):
        ROBOT.log("Init", timeout=self._timeout)

    def Move(self, vx: float, vy: float, vyaw: float):
        code = ROBOT.move(vx, vy, vyaw, self._timeout)
        if code == 0:
            self._req(SPORT_API_ID_MOVE, json.dumps({"x": vx, "y": vy, "z": vyaw}))
        ROBOT.log("Move", [vx, vy, vyaw], code, self._timeout)
        return code

    def StopMove(self):
        ROBOT.log("StopMove.begin", timeout=self._timeout)
        self._req(SPORT_API_ID_STOPMOVE, "{}")
        code = ROBOT.stop_move(self._timeout)
        ROBOT.log("StopMove", [], code, self._timeout)
        return code

    def BalanceStand(self):
        ROBOT.log("BalanceStand.begin", timeout=self._timeout)
        self._req(SPORT_API_ID_BALANCESTAND, "{}")
        code = ROBOT.balance_stand(self._timeout)
        ROBOT.log("BalanceStand", [], code, self._timeout)
        return code
