import math

# slightly silly cordinate system
PIVOT_Y = 8.586
TOP_LINK = 6.142
BOTTOM_LINK = 6.0

# deadzones
MIN_REACH = 3.375
MAX_REACH = TOP_LINK + BOTTOM_LINK

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

    # TODO: send the angles to the motors
    def move_to(self, tx, ty):
        pass
