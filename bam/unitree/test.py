import time
import sys
sys.path.append('lib')
from unitree_actuator_sdk import *


serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()
GEAR_RATIO = queryGearRatio(MotorType.GO_M8010_6)

while True:
    data.motorType = MotorType.GO_M8010_6
    cmd.motorType = MotorType.GO_M8010_6
    cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
    cmd.id   = 0
    cmd.q    = 0.39466 * GEAR_RATIO
    cmd.dq   = 0 ##6.3 * GEAR_RATIO
    cmd.kp   = 10 / (GEAR_RATIO*GEAR_RATIO)
    cmd.kd   = 0.01 / (GEAR_RATIO*GEAR_RATIO)
    cmd.tau  = 0.0
    serial.sendRecv(cmd, data)
    motor_pos = data.q / GEAR_RATIO
    motor_vel = data.dq / GEAR_RATIO
    print(f"pos: {motor_pos} vel: {motor_vel} temp: {data.temp} error: {data.merror}")
    time.sleep(0.0002) # 200 us
