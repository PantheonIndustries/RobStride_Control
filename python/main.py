import time

from robstride_dynamics import RobstrideBus
from motors import *
from ik import Leg

# foot (x, y) in inches, (0, 0) is the neutral foot spot, hip is at (0, 8.586)
FOOT_POSITIONS = [
    (0, 0), (0, -3.4), (0, 0),             # straight down, almost a straight leg
    (8.3, 0), (-8.3, 0), (0, 0),           # far forward, far back
    (0, 5), (0, 0),                        # straight up, knee almost folded
    (-7, -1), (7, -1), (-7, -1), (7, -1), (0, 0),   # big side to side swings
    (8, 2), (6, 5), (-5, 5), (-8, 2), (0, 0),       # over the top, front to back
    (8, 2), (4, 14), (8, 2), (0, 0),       # up above the hip in front, hip near -88
    (-8, 2), (-8, 11), (-8, 2), (0, 0),    # up above the hip behind, hip near 153
]
SPEED = 8   # foot speed, inches per second

if __name__ == "__main__":
    bus = RobstrideBus("can0", {})
    bus.connect()
    hip = Motor(bus, "hip", HIP_CAN_ID, HIP_BOOT_ENCODER, HIP_BOOT_IK_ANGLE, HIP_DIRECTION, HIP_MIN_IK_ANGLE, HIP_MAX_IK_ANGLE)
    knee = Motor(bus, "knee", KNEE_CAN_ID, KNEE_BOOT_ENCODER, KNEE_BOOT_IK_ANGLE, KNEE_DIRECTION, KNEE_MIN_IK_ANGLE, KNEE_MAX_IK_ANGLE)
    leg = Leg(hip, knee)

    # hip hanging straight down, knee folded on its hard stop
    leg.calibrate()
    print("starts with foot at", round(leg.foot_x, 2), round(leg.foot_y, 2))
    input("press enter to start")
    leg.enable()

    try:
        # first move from the folded boot pose
        leg.smooth_move_to(0, 0, speed=2)
        time.sleep(0.5)

        for tx, ty in FOOT_POSITIONS:
            leg.smooth_move_to(tx, ty, speed=SPEED)
            time.sleep(0.3)

            theta1, theta2 = leg.get_angles()
            bx, by, ax, ay = leg.forward(theta1, theta2)
            print("asked foot", (tx, ty), "at", (round(ax, 2), round(ay, 2)))
    finally:
        leg.disable()
