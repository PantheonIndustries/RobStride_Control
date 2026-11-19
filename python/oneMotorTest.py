from src.position_control import PositionControllerMIT
while True:
    try:
        motor_id = int(input("what's the motor's ID? "))
        break
    except:
        print("thats not a integer")
motor = PositionControllerMIT(motor_id)

if not motor.connect():
    print("couldn't connect")
    exit()

motor.set_kp(10)# the default is 30 Nm/rad

while True:
    print(f"trying to go to {motor.target_position} rn")
    while True:
        try:
            motor_angle = float(input("gimmie an angle in degrees "))
            break
        except:
            print("thats not a number probably")
    motor.set_angle(motor_angle)
    # motor.run_interactive