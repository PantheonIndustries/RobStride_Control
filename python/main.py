from src.position_control import PositionControllerMIT

# 创建控制器
controller = PositionControllerMIT(motor_id=11)

# 连接电机
controller.connect()

# 设置位置 (角度)
controller.set_angle(90.0)

# 调整控制参数
controller.set_kp(30.0)  # 位置增益
controller.set_kd(0.5)   # 阻尼增益
