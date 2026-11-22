"""FAKE unitree_api.msg.dds_ request types (same field names as the SDK IDL)."""


class RequestIdentity_:
    def __init__(self, id=0, api_id=0):
        self.id, self.api_id = id, api_id


class RequestLease_:
    def __init__(self, id=0):
        self.id = id


class RequestPolicy_:
    def __init__(self, priority=0, noreply=False):
        self.priority, self.noreply = priority, noreply


class RequestHeader_:
    def __init__(self, identity=None, lease=None, policy=None):
        self.identity = identity or RequestIdentity_()
        self.lease = lease or RequestLease_()
        self.policy = policy or RequestPolicy_()


class Request_:
    def __init__(self, header=None, parameter="", binary=None):
        self.header = header or RequestHeader_()
        self.parameter = parameter
        self.binary = binary or []

    @classmethod
    def make(cls, api_id, parameter, id=0):
        return cls(RequestHeader_(RequestIdentity_(id, api_id)), parameter, [])
