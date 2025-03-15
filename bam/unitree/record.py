import json
import datetime
import os
import sys
import numpy as np
import argparse
import time
from bam.trajectory import *

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lib'))
from unitree_actuator_sdk import *

#offset 0.39466
arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--offset", type=float, required=True, help="Offset in radians for the zero position")
arg_parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
arg_parser.add_argument("--id", type=int, default=0)
arg_parser.add_argument("--masskg", type=float)
arg_parser.add_argument("--masslbs", type=float)
arg_parser.add_argument("--arm_mass", type=float, default=0.078) #kg
arg_parser.add_argument("--length", type=float, required=True) #mm
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--trajectory", type=str, default="sin_time_square")
arg_parser.add_argument("--motor", type=str, required=True)
arg_parser.add_argument("--kp", type=float, default=10.0)
arg_parser.add_argument("--damping", type=float, default=0.3)
args = arg_parser.parse_args()

if args.masskg != None:
    mass = args.masskg
elif args.masslbs != None:
    # Convert bullshit to kg
    mass = args.masslbs * 0.45359237
else:
    print("Must either specify mass in kg or lbs")
    sys.exit(1)

if args.trajectory not in trajectories:
    raise ValueError(f"Unknown trajectory: {args.trajectory}")

def angle_wrap(radian_angle):
    return (radian_angle + np.pi) % (2 * np.pi) - np.pi

trajectory = trajectories[args.trajectory]

serial = SerialPort(args.port)
GEAR_RATIO = queryGearRatio(MotorType.GO_M8010_6)

cmd = MotorCmd()
motorData = MotorData()

def go8010_position_control(rad, kp, kd):
    global cmd, motorData
    motorData.motorType = MotorType.GO_M8010_6
    cmd.motorType = MotorType.GO_M8010_6
    cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
    cmd.id   = args.id
    cmd.q    = rad * GEAR_RATIO
    cmd.dq   = 0
    cmd.kp   = kp / (GEAR_RATIO * GEAR_RATIO)
    cmd.kd   = kd / (GEAR_RATIO * GEAR_RATIO)
    cmd.tau  = 0.0
    serial.sendRecv(cmd, motorData)

def go8010_disable_torque():
    global cmd, motorData
    motorData.motorType = MotorType.GO_M8010_6
    cmd.motorType = MotorType.GO_M8010_6
    cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.BRAKE)
    cmd.id   = args.id
    cmd.q    = 0
    cmd.dq   = 0
    cmd.kp   = 0
    cmd.kd   = 0
    cmd.tau  = 0.0
    serial.sendRecv(cmd, motorData)

goal_position, torque_enable = trajectory(0)
go8010_position_control(args.offset + goal_position, 1, .01)
time.sleep(1)

#eth.goto_safe(0, args.offset + goal_position)

start = time.time()
data = {
    "mass": mass,
    "arm_mass": args.arm_mass,
    "length": args.length,
    "kp": args.kp,
    "motor": args.motor,
    "damping": args.damping,
    "trajectory": args.trajectory,
    "entries": []
}

while time.time() - start < trajectory.duration:
    t = time.time() - start
    goal_position, torque_enable = trajectory(t)

    if torque_enable:
        go8010_position_control(args.offset + goal_position, args.kp, args.damping)
    else:
        go8010_disable_torque()

    motor_pos = motorData.q / GEAR_RATIO - args.offset
    motor_vel = motorData.dq / GEAR_RATIO
    # print(f"requested: {goal_position} reported: {motor_pos} wrapped: {angle_wrap(motor_pos)}")
    entry = {
        "position": round(angle_wrap(motor_pos), 5),
        "speed": round(motor_vel, 5),
        "torque_demand": round(motorData.tau, 5),
        # "control": status["current"],
        "timestamp": time.time() - start,
        "goal_position": goal_position,
        "torque_enable": torque_enable,
    }

    data["entries"].append(entry)

go8010_disable_torque()

#Format YYYY-MM-DD_HH:mm:ss"
date = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")

filename = f"{args.logdir}/{date}.json"
json.dump(data, open(filename, "w"))
