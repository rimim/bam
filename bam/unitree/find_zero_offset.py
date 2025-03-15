import time
import sys
sys.path.append('lib')
from unitree_actuator_sdk import *


serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()
GEAR_RATIO = queryGearRatio(MotorType.GO_M8010_6)

alpha = 0.01  
filtered_pos = 0.0

start_time = time.time()
while time.time() - start_time < 5:
    data.motorType = MotorType.GO_M8010_6
    cmd.motorType = MotorType.GO_M8010_6
    cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.BRAKE)
    cmd.id   = 0
    cmd.q    = 0.0
    cmd.dq   = 0
    cmd.kp   = 0
    cmd.kd   = 0
    cmd.tau  = 0.0
    serial.sendRecv(cmd, data)
    motor_pos = data.q / GEAR_RATIO
    motor_vel = data.dq / GEAR_RATIO
    filtered_pos = alpha * motor_pos + (1 - alpha) * filtered_pos
    # print(f"pos: {motor_pos} vel: {motor_vel} temp: {data.temp} error: {data.merror}")
    time.sleep(0.0002) # 200 us

print(f"Offset from zero: {filtered_pos:.5}")
