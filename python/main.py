import math
import time

from robstride_dynamics import RobstrideBus, ParameterType
from motors import *
from ik import Leg

# foot (x, y) in inches, (0, 0) is the neutral foot spot, hip is at (0, 8.586)
SPEED = 3   # foot speed, inches per second

# hop, foot heights in inches, straight leg is -3.55
STAND_Y = -1          # standing on the ground
CROUCH_Y = 5          # crouch before the push, a bent knee moves the foot fastest so the push lives up here
PUSH_UNTIL_Y = 0      # stop pushing here, below this the knee spins fast but the foot barely speeds up
LAND_Y = -2.0         # close to where the push ends, so it doesn't yank the leg back mid air
LAND_KP = 15          # soft spring for landing, gives on touchdown instead of slamming
LAND_KD = 1.5         # damping for landing, soaks up the touchdown
HOP_FORCE = 60        # newtons, foot pushing the ground down, 10 Nm limit caps it near 68 from this crouch
PUSH_TIME_MAX = 0.3   # seconds, gives up pushing after this
PUSH_KP = 40          # how hard the hip keeps the foot straight under itself during the push, knee gets 0

if __name__ == "__main__":
    bus = RobstrideBus("can0", {})
    bus.connect()
    hip = Motor(bus, "hip", HIP_CAN_ID, HIP_BOOT_ENCODER, HIP_BOOT_IK_ANGLE, HIP_DIRECTION, HIP_MIN_IK_ANGLE, HIP_MAX_IK_ANGLE)
    knee = Motor(bus, "knee", KNEE_CAN_ID, KNEE_BOOT_ENCODER, KNEE_BOOT_IK_ANGLE, KNEE_DIRECTION, KNEE_MIN_IK_ANGLE, KNEE_MAX_IK_ANGLE)
    leg = Leg(hip, knee)

    # hip hanging straight down, knee folded on its hard stop
    leg.calibrate()
    print("starts with foot at", round(leg.foot_x, 2), round(leg.foot_y, 2))
    # raw readings in degrees, in the boot pose these are HIP_BOOT_ENCODER and KNEE_BOOT_ENCODER
    print("hip encoder", round(hip.read_encoder(), 3), "knee encoder", round(knee.read_encoder(), 3))
    # the motor's own current limit in amps, the landing log looked capped near 3 Nm
    print("current limit hip", bus.read("hip", ParameterType.CURRENT_LIMIT), "knee", bus.read("knee", ParameterType.CURRENT_LIMIT))
    input("press enter to start")
    leg.enable()

    try:
        # first move from the folded boot pose
        hip.phase = knee.phase = "start"
        leg.smooth_move_to(0, 0, speed=2)
        leg.smooth_move_to(0, STAND_Y, speed=SPEED)
        input("foot on the ground, press enter to hop")

        # 1. crouch, then hold it, sending the same target so the log keeps recording
        hip.phase = knee.phase = "crouch"
        leg.smooth_move_to(0, CROUCH_Y, speed=2)
        end = time.time() + 0.5
        while time.time() < end:
            leg.move_to(0, CROUCH_Y)

        # 2. push: torque pushes the foot down, the hip's spring keeps it straight under the hip,
        # the knee has no spring because its target always trails a little and pulls against the push
        hip.phase = knee.phase = "push"
        hip.log_current = knee.log_current = False
        start = time.time()
        while leg.foot_y > PUSH_UNTIL_Y and time.time() - start < PUSH_TIME_MAX:
            theta1 = hip.ik_angle
            theta2 = knee.ik_angle - theta1
            hip_torque, knee_torque = leg.foot_force_to_torque(theta1, theta2, 0, -HOP_FORCE)

            # target is under the hip at the height the foot is at right now,
            # so the spring only pulls it back sideways, it doesn't hold the push back
            target1, target2 = leg.inverse(0, leg.foot_y)
            hip.move_to_ik_angle(target1, kp=PUSH_KP, kd=0, torque=hip_torque)
            knee.move_to_ik_angle(target1 + target2, kp=0, kd=0, torque=knee_torque)

            theta1 = hip.ik_angle
            theta2 = knee.ik_angle - theta1
            bx, by, leg.foot_x, leg.foot_y = leg.forward(theta1, theta2)
        print("pushed for", round(time.time() - start, 3), "s, foot got to", round(leg.foot_x, 2), round(leg.foot_y, 2))

        # 3. flight and landing: soft spring to a nearly straight pose and hold it for 1 s,
        # sending it over and over as fast as the bus goes so the log catches the landing
        hip.phase = knee.phase = "land"
        hip.log_current = knee.log_current = True
        target1, target2 = leg.inverse(0, LAND_Y)
        end = time.time() + 1
        while time.time() < end:
            hip.move_to_ik_angle(target1, kp=LAND_KP, kd=LAND_KD)
            knee.move_to_ik_angle(target1 + target2, kp=LAND_KP, kd=LAND_KD)
        leg.foot_x = 0
        leg.foot_y = LAND_Y

        # 4. stand back up
        hip.phase = knee.phase = "stand"
        leg.smooth_move_to(0, STAND_Y, speed=SPEED)
        time.sleep(0.5)
    finally:
        # a motor that dropped off the bus can't answer the disable, carry on so the log still gets written
        for motor in (hip, knee):
            try:
                motor.disable()
            except Exception as error:
                print("couldn't disable", motor.name, error)

        # every motor command and reply, both motors in time order
        rows = [("hip",) + row for row in hip.log] + [("knee",) + row for row in knee.log]
        rows.sort(key=lambda row: row[1])
        with open("hop_log.txt", "w") as f:
            f.write("HOP_FORCE " + str(HOP_FORCE) + " PUSH_KP " + str(PUSH_KP) + " LAND_Y " + str(LAND_Y) + " LAND_KP " + str(LAND_KP) + " LAND_KD " + str(LAND_KD) + " TORQUE_LIMIT " + str(TORQUE_LIMIT)
                    + " MIT_KP " + str(MIT_KP) + " MIT_KD " + str(MIT_KD) + "\n")
            f.write("# angles in IK degrees (knee = bend), speed in IK deg/s, torque in Nm along the IK angle, current in A, volts = supply, current and volts are 0 during the push (not read, keeps the push fast)\n")
            f.write("motor\tt\tphase\ttarget\tkp\tkd\ttorque_sent\tangle\tspeed\ttorque_measured\tcurrent\tvolts\n")
            for motor, t, phase, target, kp, kd, torque, angle, speed, measured, current, volts in rows:
                f.write(f"{motor}\t{t - rows[0][1]:.4f}\t{phase}\t{target:.2f}\t{kp}\t{kd}\t{torque:.3f}\t{angle:.2f}\t{speed:.1f}\t{measured:.3f}\t{current:.2f}\t{volts:.1f}\n")
        print("wrote", len(rows), "rows to hop_log.txt")

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
