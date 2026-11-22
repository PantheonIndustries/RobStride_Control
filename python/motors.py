import math
import struct

from robstride_dynamics import RobstrideBus, ParameterType, CommunicationType
from robstride_dynamics import Motor as BusMotor

# all degrees
BAND = 360 / 7.75
SAFETY_MARGIN = 5

# Nm
TORQUE_LIMIT = 5

# position mode (CSP) top speed, degrees/s
CSP_SPEED_LIMIT = 360

KNEE_CAN_ID = 1
KNEE_BOOT_ENCODER = math.degrees(0.029)
KNEE_BOOT_IK_ANGLE = 154.55
KNEE_DIRECTION = -1
KNEE_MIN_IK_ANGLE = -135.45   
KNEE_MAX_IK_ANGLE = 154.55

HIP_CAN_ID = 2
HIP_BOOT_ENCODER = math.degrees(-0.7106)
HIP_BOOT_IK_ANGLE = 0
HIP_DIRECTION = 1
HIP_MIN_IK_ANGLE = -97
HIP_MAX_IK_ANGLE = 178

class Motor:
    def __init__(self, bus, name, can_id, boot_encoder, boot_ik_angle, direction, min_ik_angle, max_ik_angle, model="rs-01"):
        self.bus = bus
        self.name = name
        self.can_id = can_id
        self.model = model
        self.boot_encoder = boot_encoder
        self.boot_ik_angle = boot_ik_angle
        self.direction = direction
        self.min_ik_angle = min_ik_angle
        self.max_ik_angle = max_ik_angle
        self.safe_min_ik_angle = min_ik_angle + SAFETY_MARGIN
        self.safe_max_ik_angle = max_ik_angle - SAFETY_MARGIN

        # last known IK angle of this joint
        self.ik_angle = None

        bus.motors[name] = BusMotor(can_id, model)

    def encoder_to_ik_angle(self, encoder):
        turned = encoder - self.boot_encoder
        return self.boot_ik_angle + self.direction * turned

    def ik_angle_to_encoder(self, ik_angle):
        turned = ik_angle - self.boot_ik_angle
        return self.boot_encoder + self.direction * turned

    def fit_in_range(self, ik_angle):
        for turn in (0, 360, -360):
            if self.min_ik_angle - SAFETY_MARGIN <= ik_angle + turn <= self.max_ik_angle + SAFETY_MARGIN:
                return ik_angle + turn
        return None

    def read_encoder(self):
        return math.degrees(self.bus.read(self.name, ParameterType.MECHANICAL_POSITION))

    def calibrate(self):
        encoder = self.read_encoder()
        self.ik_angle = self.encoder_to_ik_angle(encoder)

    def enable(self, mit=False):
        if mit:
            self.bus.write(self.name, ParameterType.MODE, 0)
            self.bus.enable(self.name)
        else:
            # target = where it is right now, so it doesn't jump to an old target the moment it turns on
            here = math.radians(self.read_encoder())
            self.bus.write(self.name, ParameterType.MODE, 5)
            self.bus.write(self.name, ParameterType.POSITION_TARGET, here)
            self.bus.enable(self.name)
            self.bus.write(self.name, ParameterType.VELOCITY_LIMIT, math.radians(CSP_SPEED_LIMIT))
            self.bus.write(self.name, ParameterType.POSITION_TARGET, here)

    def disable(self):
        self.bus.disable(self.name)

    def get_ik_angle(self):
        self.ik_angle = self.encoder_to_ik_angle(self.read_encoder())
        return self.ik_angle

    # position mode: sends the target, returns where the joint is right now
    def move_to_ik_angle(self, target_ik_angle):
        # clamp to the safe range, but if the joint is already past it (knee on its hard stop at boot)
        # let it stay where it is instead of snapping inside all at once
        low = min(self.safe_min_ik_angle, self.ik_angle)
        high = max(self.safe_max_ik_angle, self.ik_angle)
        target_ik_angle = min(max(target_ik_angle, low), high)
        encoder = self.ik_angle_to_encoder(target_ik_angle)

        # same as bus.write, but keeps the motor's reply instead of throwing it away
        data = struct.pack("<HHf", ParameterType.POSITION_TARGET[0], 0, math.radians(encoder))
        self.bus.transmit(CommunicationType.WRITE_PARAMETER, self.bus.host_id, self.can_id, data)
        position, velocity, torque, temperature = self.bus.receive_status_frame(self.name)

        self.ik_angle = self.encoder_to_ik_angle(math.degrees(position))
        return self.ik_angle
