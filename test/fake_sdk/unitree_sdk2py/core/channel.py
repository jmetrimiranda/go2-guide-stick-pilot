"""FAKE unitree_sdk2py.core.channel: same API as the SDK, delivered by the in-process fake robot."""
from .._fake_robot import ROBOT


def ChannelFactoryInitialize(id: int = 0, networkInterface: str = None):
    ROBOT.start(networkInterface)


class ChannelSubscriber:
    def __init__(self, name, type):
        if not ROBOT.initialized:
            raise RuntimeError("fake SDK: ChannelFactoryInitialize must be called first")
        self.name, self.type = name, type
        self.handler = None
        self.closed = False

    def Init(self, handler=None, queueLen=0):
        if self.handler is None and handler is not None:
            self.handler = handler
            ROBOT.subscribe(self.name, handler)
            ROBOT.log("ChannelSubscriber.Init", [self.name])

    def Close(self):
        # the real Close() does `del self.__reader`, so a second Close() raises AttributeError
        if self.closed:
            raise AttributeError("fake SDK: ChannelSubscriber closed twice")
        self.closed = True
        if self.handler is not None:
            ROBOT.unsubscribe(self.name, self.handler)
        ROBOT.log("ChannelSubscriber.Close", [self.name])

    def Read(self, timeout=None):
        raise NotImplementedError("fake SDK: Read() is not used by go2_turn_pilot.py")
