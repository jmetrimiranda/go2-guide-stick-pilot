"""FAKE unitree_go.msg.dds_ messages: plain objects with the fields of the SDK IDL."""


class IMUState_:
    def __init__(self):
        self.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.gyroscope = [0.0, 0.0, 0.0]
        self.accelerometer = [0.0, 0.0, 0.0]
        self.rpy = [0.0, 0.0, 0.0]
        self.temperature = 30


class MotorState_:
    def __init__(self):
        self.mode = 0
        self.q = self.dq = self.ddq = self.tau_est = 0.0


class SportModeState_:
    def __init__(self):
        self.imu_state = IMUState_()
        self.mode = 0
        self.gait_type = 0
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.yaw_speed = 0.0


class LowState_:
    def __init__(self):
        self.imu_state = IMUState_()
        self.motor_state = [MotorState_() for _ in range(20)]
        self.power_v = 0.0
        self.power_a = 0.0
        self.tick = 0
