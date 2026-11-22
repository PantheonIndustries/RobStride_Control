import math

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
MIN_REACH = 3.375
MAX_REACH = TOP_LINK + BOTTOM_LINK
TILT_MIN = -20
TILT_MAX = 90

class Leg:
    def __init__(self, hip, knee):
        self.hip = hip
        self.knee = knee

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
        theta1 = alpha + beta

        bx = -TOP_LINK * math.sin(theta1)
        by = PIVOT_Y - TOP_LINK * math.cos(theta1)
        theta2 = math.atan2(tx - bx, by - ty)
        return math.degrees(theta1), math.degrees(theta2)

    # accepts degrees, returns foot (x, y, z) in inches
    def forward_3d(self, theta0, theta1, theta2):
        bx, by, ax, ay = self.forward(theta1, theta2)
        h = PIVOT_Y - ay

        theta0 = math.radians(theta0)
        x = ax
        y = HIP_OFFSET * math.cos(theta0) + h * math.sin(theta0)
        z = PIVOT_Y + HIP_OFFSET * math.sin(theta0) - h * math.cos(theta0)
        return x, y - HIP_OFFSET, z

    # accepts foot (x, y, z) in inches, returns (theta0, theta1, theta2) in degrees
    def inverse_3d(self, x, y, z):
        y = y + HIP_OFFSET

        h = math.sqrt(y**2 + (PIVOT_Y - z)**2 - HIP_OFFSET**2)
        theta0 = math.degrees(math.atan2(y, PIVOT_Y - z) - math.atan2(HIP_OFFSET, h))
        if theta0 < TILT_MIN or theta0 > TILT_MAX:
            print("can't go there")
            return None

        angles = self.inverse(x, PIVOT_Y - h)
        if angles is None:
            return None
        
        theta1, theta2 = angles
        return theta0, theta1, theta2

    # accepts degrees and the foot push (fx, fy) in newtons, returns (hip, knee) torque in Nm
    def foot_force_to_torque(self, theta1, theta2, fx, fy):
        theta1 = math.radians(theta1)
        theta2 = math.radians(theta2)

        top_link_m = TOP_LINK * 0.0254 # to meters from inches
        bottom_link_m = BOTTOM_LINK * 0.0254

        hip_torque = top_link_m * (-math.cos(theta1) * fx + math.sin(theta1) * fy)
        knee_torque = bottom_link_m * (math.cos(theta2) * fx + math.sin(theta2) * fy)
        return hip_torque, knee_torque

    # accepts degrees and the foot push (fx, fy, fz) in newtons, returns (tilt, hip, knee) torque in Nm
    def foot_force_to_torque_3d(self, theta0, theta1, theta2, fx, fy, fz):
        bx, by, ax, ay = self.forward(theta1, theta2)
        h = PIVOT_Y - ay

        h_m = h * 0.0254 # to meters from inches
        offset_m = HIP_OFFSET * 0.0254

        theta0 = math.radians(theta0)
        tilt_torque = fy * (h_m * math.cos(theta0) - offset_m * math.sin(theta0)) + fz * (h_m * math.sin(theta0) + offset_m * math.cos(theta0))

        fy_in_plane = fz * math.cos(theta0) - fy * math.sin(theta0)
        hip_torque, knee_torque = self.foot_force_to_torque(theta1, theta2, fx, fy_in_plane)
        return tilt_torque, hip_torque, knee_torque

    # TODO: send the angles to the motors
    def move_to(self, tx, ty):
        pass
