from src.position_control import PositionControllerMIT

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink, Link

import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D

import os

# evil_chain = Chain(name='leg', links=[
#     OriginLink(),
#     URDFLink(
#              name="hip",
#              origin_translation=[0,0,-2.45],
#              origin_orientation=[0,0,0],
#              rotation=[0,1,0]),
#     URDFLink(
#             name="yeag",
#             origin_translation=[0,0,-5.45],
#             origin_orientation=[0,0,0],
#             rotation=[0,1,0]
#     )
# ])

# evil_chain = Chain.from_urdf_file("leg.URDF", active_links_mask=[True, True, True, True, False])
evil_chain = Chain.from_urdf_file(f"{os.path.dirname(os.path.realpath(__file__))}/leg2Motor.URDF", active_links_mask=[True, True, True, False])



ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

# ik units are in meters.
ik_result = evil_chain.inverse_kinematics([0.1, 0, 0.1])
print(ik_result)
evil_chain.plot(ik_result, ax)
matplotlib.pyplot.show()

motor_ids = [2,127]

motors = []
for id in motor_ids:
    motors.append(PositionControllerMIT(id))

for motor in motors:
    if not motor.connect():
        print(f"couldn't connect to motor {motor.motor_id}")
        exit()

    motor.set_kp(30)# the default is 30 Nm/rad
    # motor.set_torque(3)

input("press enter to go to the first position")

for i in range(len(motors)):
    motors[i].set_angle(ik_result[i+1])


# # 创建控制器
# controller = PositionControllerMIT(motor_id=11)

# # 连接电机
# controller.connect()

# # 设置位置 (角度)
# controller.set_angle(90.0)

# # 调整控制参数
# controller.set_kp(30.0)  # 位置增益
# controller.set_kd(0.5)   # 阻尼增益
