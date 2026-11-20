from src.position_control import PositionControllerMIT

motor_ids = [2,127]
pose_1 = [0,30]
pose_2 = [90,-20]

motors = []
for id in motor_ids:
    motors.append(PositionControllerMIT(id))

for motor in motors:
    if not motor.connect():
        print(f"couldn't connect to motor {motor.motor_id}")
        exit()

    motor.set_kp(35)# the default is 30 Nm/rad
    motor.set_torque(3)

input("press enter to go to the first position")

for i in range(len(motors)):
    motors[i].set_angle(pose_1[i])

input("press enter to go to the second position")

for i in range(len(motors)):
    motors[i].set_angle(pose_2[i])

input("okie dokie press enter and its done now")