import math
import time

# convention
# x = forward/back, y = sideways, z = up/down, all in inches
# theta0 = tilt, theta1 = hip, theta2 = knee, all in degrees
# neutral: theta0 = 0, theta1 and theta2 about 45, foot at (0, 0, 0)

# slightly silly cordinate system
PIVOT_Y = 8.586
TOP_LINK = 6.142
BOTTOM_LINK = 6.0
HIP_OFFSET = 2.895

# deadzones
MIN_REACH = 2.3
MAX_REACH = TOP_LINK + BOTTOM_LINK
TILT_MIN = -20
TILT_MAX = 90

# seconds between commands sent to the motors, 0.01 = 100 per second
STEP_TIME = 0.01

class Leg:
    def __init__(self, hip, knee):
        self.hip = hip
        self.knee = knee

        # where the foot was last sent, inches
        self.foot_x = None
        self.foot_y = None

    # accepts degrees, returns (bx, by, ax, ay) in inches
    def forward(self, theta1, theta2):

        theta1 = math.radians(theta1)
        theta2 = math.radians(theta2)

        bx = -TOP_LINK * math.sin(theta1)
        by = PIVOT_Y - TOP_LINK * math.cos(theta1)
        ax = bx + BOTTOM_LINK * math.sin(theta2)
        ay = by - BOTTOM_LINK * math.cos(theta2)
        return bx, by, ax, ay

    # accepts (tx, ty) in inches, returns (theta1, theta2) in degrees
    def inverse(self, tx, ty):

        d = math.sqrt(tx**2 + (PIVOT_Y - ty)**2)
        if d < MIN_REACH or d > MAX_REACH:
            print("can't go there")
            return None

        beta = math.acos((TOP_LINK**2 - BOTTOM_LINK**2 + d**2) / (2 * TOP_LINK * d))
        alpha = math.atan2(-tx, PIVOT_Y - ty)

        best = None
        for side in [1, -1]:
            theta1 = alpha + side * beta
            bx = -TOP_LINK * math.sin(theta1)
            by = PIVOT_Y - TOP_LINK * math.cos(theta1)
            theta2 = math.atan2(tx - bx, by - ty)
            theta1 = math.degrees(theta1)
            theta2 = math.degrees(theta2)

            # no motors attached, nothing to check against
            if self.hip is None:
                return theta1, theta2

            # the knee motor turns by the bend, theta1 + theta2
            theta1 = self.hip.fit_in_range(theta1)
            if theta1 is None:
                continue
            bend = self.knee.fit_in_range(theta1 + theta2)
            if bend is None:
                continue
            theta2 = bend - theta1

            change = abs(theta1 - self.hip.ik_angle) + abs(bend - self.knee.ik_angle)
            if best is None or change < best[0]:
                best = (change, theta1, theta2)

        if best is None:
            print("can't go there")
            return None
        return best[1], best[2]

    def calibrate(self):
        self.hip.calibrate()
        self.knee.calibrate()

        theta1, theta2 = self.get_angles()
        bx, by, self.foot_x, self.foot_y = self.forward(theta1, theta2)

    def enable(self):
        self.hip.enable()
        self.knee.enable()

    def disable(self):
        self.hip.disable()
        self.knee.disable()

    def get_angles(self):
        theta1 = self.hip.get_ik_angle()
        knee = self.knee.get_ik_angle()
        return theta1, knee - theta1

    # position mode: accepts inches, returns the target (theta1, theta2) in degrees
    def move_to(self, tx, ty):
        angles = self.inverse(tx, ty)
        if angles is None:
            return None
        theta1, theta2 = angles

        self.hip.move_to_ik_angle(theta1)
        self.knee.move_to_ik_angle(theta1 + theta2)
        self.foot_x = tx
        self.foot_y = ty
        return theta1, theta2

    # position mode: straight line (tx, ty) at speed inches/s
    def smooth_move_to(self, tx, ty, speed=5):
        start_x = self.foot_x
        start_y = self.foot_y

        distance = math.sqrt((tx - start_x)**2 + (ty - start_y)**2)
        steps = max(1, int(distance / speed / STEP_TIME))

        next_tick = time.time()
        for i in range(1, steps + 1):
            s = 3 * (i / steps)**2 - 2 * (i / steps)**3
            x = start_x + (tx - start_x) * s
            y = start_y + (ty - start_y) * s
            if self.move_to(x, y) is None:
                return None

            next_tick = next_tick + STEP_TIME
            time.sleep(max(0, next_tick - time.time()))

        return tx, ty