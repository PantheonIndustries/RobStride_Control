from src.position_control import PositionControllerMIT
motor_ids = []
while True:
    try:
        motor_id = input("what's the motor's ID? input 'y' when done. ")
        motor_id = int(motor_id)
        motor_ids.append(motor_id)
    except:
        if motor_id == "y":
            break
        print("thats not a integer")

if not motor_ids:
    print("very funny")
    exit()

motors = []
for id in motor_ids:
    motors.append(PositionControllerMIT(id))

for motor in motors:
    if not motor.connect():
        print(f"couldn't connect to motor {motor.motor_id}")
        exit()

    motor.set_kp(10)# the default is 30 Nm/rad

while True:
    for motor in motors:
        print(f"trying to go to {motor.target_position} rn")
        motor_angle = float(input("gimmie an angle in degrees "))
        motor.set_angle(motor_angle)
    # motor.run_interactive