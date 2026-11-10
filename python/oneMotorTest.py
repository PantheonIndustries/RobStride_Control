from src.position_control import PositionControllerMIT

motor_id = int(input("what's the motor's ID? "))
motor = PositionControllerMIT(motor_id)

if not motor.connect():
    print("couldn't connect")
    exit()

while True:
    print(f"trying to go to {motor.target_position} rn")
    motor_angle = float(input("gimmie an angle in degrees "))
    motor.set_angle(motor_angle)