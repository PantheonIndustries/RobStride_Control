import math
import time

from robstride_dynamics import RobstrideBus
from motors import *
from ik import Leg

# foot (x, y) in inches, (0, 0) is the neutral foot spot, hip is at (0, 8.586)
SPEED = 3   # foot speed, inches per second

# hop, foot heights in inches, straight leg is -3.55
STAND_Y = -1          # standing on the ground
CROUCH_Y = 2.5        # crouch before the push, 5 inches above where the push stops
PUSH_UNTIL_Y = -2.5   # stop pushing here so it doesn't slam straight
LAND_Y = -0.5         # bent a bit to soak up the landing
HOP_FORCE = 10       # newtons, foot pushing the ground down, 5 Nm limit caps it near 37 from this crouch
PUSH_TIME_MAX = 0.3   # seconds, gives up pushing after this
PUSH_KP = 20          # how hard it keeps the foot straight under the hip during the push

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
        leg.smooth_move_to(0, STAND_Y, speed=SPEED)
        input("foot on the ground, press enter to hop")

        # 1. crouch
        leg.smooth_move_to(0, CROUCH_Y, speed=2)
        time.sleep(0.5)

        # 2. push: torque pushes the foot down, a spring keeps it straight under the hip
        start = time.time()
        while leg.foot_y > PUSH_UNTIL_Y and time.time() - start < PUSH_TIME_MAX:
            theta1 = hip.ik_angle
            theta2 = knee.ik_angle - theta1
            hip_torque, knee_torque = leg.foot_force_to_torque(theta1, theta2, 0, -HOP_FORCE)

            # target is under the hip at the height the foot is at right now,
            # so the spring only pulls it back sideways, it doesn't hold the push back
            target1, target2 = leg.inverse(0, leg.foot_y)
            hip.move_to_ik_angle(target1, kp=PUSH_KP, kd=0, torque=hip_torque)
            knee.move_to_ik_angle(target1 + target2, kp=PUSH_KP, kd=0, torque=knee_torque)

            theta1 = hip.ik_angle
            theta2 = knee.ik_angle - theta1
            bx, by, leg.foot_x, leg.foot_y = leg.forward(theta1, theta2)
        print("pushed for", round(time.time() - start, 3), "s, foot got to", round(leg.foot_x, 2), round(leg.foot_y, 2))

        # 3. flight and landing: spring back to a bent pose and hold it
        leg.move_to(0, LAND_Y)
        time.sleep(1)

        # 4. stand back up
        leg.smooth_move_to(0, STAND_Y, speed=SPEED)
        time.sleep(0.5)
    finally:
        leg.disable()

# small moves around neutral, to see how MIT holds and lags
# FOOT_POSITIONS = [
#     (0, 0), (0, -2), (0, 0),               # down and back
#     (3, 0), (-3, 0), (0, 0),               # forward, back
#     (0, 2), (0, 0),                        # up
# ]
#
#        for tx, ty in FOOT_POSITIONS:
#            leg.smooth_move_to(tx, ty, speed=SPEED)
#            time.sleep(0.5)
#
#            theta1, theta2 = leg.get_angles()
#            bx, by, ax, ay = leg.forward(theta1, theta2)
#            print("asked foot", (tx, ty), "at", (round(ax, 2), round(ay, 2)))

# full limits tour, CSP ran it at SPEED 8
# FOOT_POSITIONS = [
#     (0, 0), (0, -3.4), (0, 0),             # straight down, almost a straight leg
#     (8.3, 0), (-8.3, 0), (0, 0),           # far forward, far back
#     (0, 5), (0, 0),                        # straight up, knee almost folded
#     (-7, -1), (7, -1), (-7, -1), (7, -1), (0, 0),   # big side to side swings
#     (8, 2), (6, 5), (-5, 5), (-8, 2), (0, 0),       # over the top, front to back
#     (8, 2), (4, 14), (8, 2), (0, 0),       # up above the hip in front, hip near -88
#     (-8, 2), (-8, 11), (-8, 2), (0, 0),    # up above the hip behind, hip near 153
# ]
