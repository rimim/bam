import argparse
import mujoco
import numpy as np
import mujoco.viewer
import sys
import time
import math
import os

import matplotlib.pyplot as plt

from bam.model import load_model
from bam.mujoco import MujocoController

import puddleduck.puddleduck as pd

SCRIPT_PATH = os.path.dirname(os.path.abspath(__file__))

def equivalent_inertial_mass(
    arm_length: float,
    arm_mass: float,
    payload_mass: float,
    motor_type: str = "unitree_go1",
    payload_radius: float = 0.0
) -> float:
    motors = {
        "unitree_go1": {
            "mass": 0.530,       # kg
            "ratio": 6.33,       # gear ratio
            "rotor_diameter": 0.050,
            "rotor_fraction": 0.20 # rotor mass percentage of motor mass
        },
    }
    motor_type = motor_type.lower()
    if motor_type not in motors:
        raise ValueError(f"motor_type must be one of {list(motors)}")
    motor = motors[motor_type]
    m_motor = motor["mass"]
    gear_ratio = motor["ratio"]
    r_rotor = motor["rotor_diameter"] / 2
    m_rotor = m_motor * motor["rotor_fraction"]

    # Estimate rotor inertia
    J_rotor = 0.5 * m_rotor * r_rotor**2
    J_rotor *= gear_ratio**2
    J_arm = (1/3) * arm_mass * arm_length**2

    # payload inertia
    if payload_radius > 0.0:
        J_disk = 0.5 * payload_mass * payload_radius**2
        J_payload = J_disk + payload_mass * arm_length**2
    else:
        # pure point mass at distance arm_length
        J_payload = payload_mass * arm_length**2

    # Total inertia and back to equivalent mass
    J_total = J_rotor + J_arm + J_payload
    m_eq    = J_total / (arm_length**2)
    return m_eq

class MujocoRig:
    def __init__(self, motor_type, masskg, kp, kv, fr_min, fr_max, damping, armature, frictionloss, bam=False, model_path=None, control_decimation=10):

        self.bam = bam
        if self.bam and model_path is None:
            raise ValueError("BAM mode requires model_path to be set")
        # use template.xml to generate our model with our values
        inertial_mass = equivalent_inertial_mass(0.150, 0.1, masskg, motor_type, 0.140 / 2)
        xml_path = f"{SCRIPT_PATH}/assets/rig_0_150m/"
        xml = open(f"{xml_path}/template.xml").read()
        xml = xml.replace('<WEIGHT/>', f'"{inertial_mass}"')
        if bam:
            xml = xml.replace('<MOTOR/>', '<motor name="MOTOR" joint="MOTOR"/>')
            # xml = xml.replace('<MOTORCLASS/>', '')
        else:
            xml = xml.replace('<MOTOR/>', f'<position name="MOTOR" joint="MOTOR" kp="{kp}" kv="{kv}" forcerange="{fr_min} {fr_max}"/>')
        if damping is not None:
            xml = xml.replace('<MOTORCLASS/>', f'<joint damping="{damping}" armature="{armature}" frictionloss="{frictionloss}"/>')
        else:
            xml = xml.replace('<MOTORCLASS/>', '')

        if bam:
            # torque controller
            xml_scene = f"{xml_path}/scene_bam.xml"
        else:
            # position controller
            xml_scene = f"{xml_path}/scene_mujoco.xml"
        with open(xml_scene, 'w') as f:
            f.write(xml)

        self.model = mujoco.MjModel.from_xml_path(xml_scene)
        self.model.opt.timestep = 0.002
        self.control_decimation = control_decimation
        self.data = mujoco.MjData(self.model)

        # 3) recompute kinematics
        mujoco.mj_forward(self.model, self.data)

        if self.bam:
            unitree_model = load_model(model_path)
            self.mujoco_controller = MujocoController(
                unitree_model, "MOTOR", self.model, self.data
            )
            self.mujoco_controller.model.actuator.kp = kp

        # control & logging state:
        self.goal_position = 0.0
        self.torque_enabled = True

    def set_goal_position(self, pos):
        self.goal_position = pos

    def step_controller(self, actuator, actuator_rad):
        if self.bam:
            # your custom controller
            self.mujoco_controller.set_q_target("MOTOR", self.goal_position)
            self.mujoco_controller.update()
        else:
            # direct torque = goal_position
            # ensure ctrl is the right shape:
            self.data.ctrl[0] = self.goal_position

        # --------------------------
        # drive actuator
        if actuator:
            actuator.raw_radians = actuator_rad
            actuator.stiff()     # torque on
            actuator.update()

    def get_sim_position(self):
        return self.data.qpos[0]

    def get_sim_velocity(self):
        return self.data.qvel[0]

def humanize(s: str) -> str:
    return s.replace('_', ' ').title()

def main():
    motor_choices = [
        "unitree_go1"
    ]
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--port", type=str, default=None)
    arg_parser.add_argument("--motor", choices=motor_choices, required=True)
    arg_parser.add_argument("--kp", type=float, required=True)
    arg_parser.add_argument("--kd", type=float, required=True)
    arg_parser.add_argument("--masskg", type=float, default=1)
    arg_parser.add_argument(
        "--amplitude",
        type=float,
        default=90,
        help="Amplitude of sinewave in degrees")
    arg_parser.add_argument(
        "--period",
        type=float,
        default=5,
        help="Sinewave period in seconds")
    arg_parser.add_argument(
        "--duration",
        type=float,
        default=10,
        help="Duration of test in seconds")
    arg_parser.add_argument(
        "--bam",
        action="store_true",
        help="Use BAM controller.",
    )
    arg_parser.add_argument(
        "--plot",
        action="store_true",
        help="Optionally plot sim vs real positions.",
    )
    arg_parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Verbose output.",
    )
    args = arg_parser.parse_args()

    motorType = None
    damping=None
    armature=None
    frictionloss=None
    # TODO: load these from the bam model
    if args.motor == "unitree_go1":
        motorType = pd.MotorType.Go1
        baud = 4000000
        fr_min=-23.7
        fr_max=23.7
        # damping = 0
        # armature = 0
        # frictionloss = 0
        damping=1.252572806829929
        armature=0.008898889300028003
        frictionloss=0.13703684028833626
    else:
        print(f"Unsupported motor type: {args.motor}")
        sys.exit(1)

    # Actuator setup
    actuator = None
    start_offset = 0
    if args.port:
        bus = pd.Bus("MyBus", pd.BusType.RS485, args.port, baud, 0, 2)
        if not bus.isOpen:
            print("Failed to open bus"); sys.exit(1)

        actuator = pd.Actuator.create(0, motorType, "MyMotor")
        actuator.bus = bus
        actuator.relax(); actuator.update()
        if not actuator.responding:
            print("Failed to find actuator"); sys.exit(1)
        time.sleep(1)
        actuator.update()
        actuator.kp = args.kp
        actuator.kd = args.kd

        # record starting offset
        start_offset = actuator.raw_radians
    actuator_rad = start_offset

    # --------------------------
    rig = MujocoRig(
        args.motor,
        masskg=args.masskg,
        kp=args.kp,
        kv=args.kd,
        fr_min=fr_min,
        fr_max=fr_max,
        damping=damping,
        armature=armature,
        frictionloss=frictionloss,
        bam=args.bam,
        model_path=f"params/{args.motor}/m1.json",
        control_decimation=10
    )

    # --------------------------
    duration  = args.duration    # total run time [s]
    period    = args.period      # sine period [s]
    amplitude = args.amplitude   # degrees

    if args.plot:
        time_list     = []
        sim_positions = []
        real_positions= []

    with mujoco.viewer.launch_passive(rig.model, rig.data,
                                      show_left_ui=False,
                                      show_right_ui=False) as viewer:

        start_time = time.time()
        step_count = 0
        while viewer.is_running():
            t = time.time() - start_time
            if t > duration:
                break

            step_start = time.time()

            # 1) compute new goal
            target_rad = math.radians(amplitude * math.sin(2*math.pi*t/period))
            rig.set_goal_position(target_rad)
            actuator_rad = start_offset + target_rad

            # 2) MuJoCo physics step
            mujoco.mj_step(rig.model, rig.data)
            step_count += 1

            # 3) every Nth step run your controller + real actuator
            if step_count % rig.control_decimation == 0:
                rig.step_controller(actuator, actuator_rad)

            # 4) logging
            sim_pos  = rig.get_sim_position()
            if actuator:
                real_pos = actuator.raw_radians - start_offset
            if args.plot:
                time_list.append(t)
                sim_positions.append(sim_pos)
                if actuator:
                    real_positions.append(real_pos)
            if args.verbose:
                print(f"t={t:.3f} | sim={sim_pos:.4f}  real={real_pos:.4f}")

            # 5) sync render + enforce fixed timestep
            viewer.sync()
            dt = rig.model.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)

    # --------------------------
    # Remove torque from actuator
    if actuator:
        actuator.relax()
        actuator.update()

    if args.plot:
        if args.bam:
            plot_type = "BAM"
        else:
            plot_type = "Mujoco"
        actuator_name = humanize(args.motor)
        plt.figure(figsize=(10,6))
        plt.plot(time_list, sim_positions, label=plot_type)
        if actuator:
            plt.plot(time_list, real_positions,label=actuator_name)
        plt.xlabel('Time [s]')
        plt.ylabel('Position [rad]')
        if actuator:
            plt.title(f'{plot_type} vs. {actuator_name}')
        else:
            plt.title(f'{plot_type}')
        plt.legend(); plt.grid(True)
        plt.savefig("actuator_simulation_plot.png")
        plt.show()

if __name__ == "__main__":
    main()
