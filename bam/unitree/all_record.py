import argparse
import os
import time


arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--offset", type=float, required=True, help="Offset in radians for the zero position")
arg_parser.add_argument("--masskg", type=float)
arg_parser.add_argument("--masslbs", type=float)
arg_parser.add_argument("--arm_mass", type=float, default=0.078) #kg
arg_parser.add_argument("--length", type=float, required=True)
arg_parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
arg_parser.add_argument("--id", type=int, default=0)
arg_parser.add_argument("--motor", type=str, default="unitree_go1")
arg_parser.add_argument("--logdir", type=str, required=True)
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

# kps = [130, 275, 550, 1100]
kps = [10, 12, 14, 16]
trajectories = ["brutal", "sin_sin", "lift_and_drop", "up_and_down", "sin_time_square"]

command_base = (
    f"python3 -m bam.unitree.record --masskg {mass} --arm_mass {args.arm_mass} --length {args.length} --offset {args.offset}"
)
command_base += f" --port {args.port} --logdir {args.logdir} --id {args.id} --motor {args.motor}"


for kp in kps:
    for trajectory in trajectories:
        sentence = f"Kp {kp}, trajectory {trajectory.replace('_', ' ')}"
        print(sentence)

        command = f"{command_base} --kp {kp} --damping {args.damping} --trajectory {trajectory}"
        os.system(command)

        if trajectory == "sin_time_square":
            time.sleep(3)
        else:
            time.sleep(1)
