from robstride_dynamics import RobstrideBus, Motor



bus = RobstrideBus("can0",{
    "hip": Motor(1,"rs-01"),
})

motors = bus.scan_channel("can0")
print(f"Found motors: {motors}")

# try:
#     for motorName in bus.motors:
#         bus.disable(motorName)
#         bus.write(motorName,)

