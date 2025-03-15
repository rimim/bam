import argparse
import os
import numpy as np
import matplotlib.pyplot as plt

from .model import load_model, DummyModel
from .actuator import actuators
from . import simulate
from . import logs

# Set up the argument parser including an output directory for plots
arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--params", type=str, default=["params.json"], nargs="+")
arg_parser.add_argument("--actuator", type=str, required=True)
arg_parser.add_argument("--reset_period", default=None, type=float)
arg_parser.add_argument("--sim", action="store_true")
arg_parser.add_argument("--output_dir", type=str, default="plots", 
                        help="Directory to save generated plots and HTML page.")
args = arg_parser.parse_args()

# Create the output directory if it doesn't exist
if not os.path.exists(args.output_dir):
    os.makedirs(args.output_dir)

plot_files = []  # List to store the names of saved plot files

logs = logs.Logs(args.logdir)

if args.sim:
    model_names = args.params

plot_index = 0  # Counter for naming plot files

for log in logs.logs:
    print(log["filename"])
    all_sim_q = []
    all_sim_speeds = []
    all_sim_controls = []
    all_names = []

    if args.sim:
        for model_name in model_names:
            model = load_model(model_name)
            all_names.append(model.name)
            simulator = simulate.Simulator(model)
            sim_q, sim_speed, sim_controls = simulator.rollout_log(
                log, reset_period=args.reset_period, simulate_control=True
            )
            all_sim_q.append(np.array(sim_q))
            all_sim_speeds.append(np.array(sim_speed))
            all_sim_controls.append(np.array(sim_controls))

    ts = np.arange(len(log["entries"])) * log["dt"]
    q = [entry["position"] for entry in log["entries"]]
    goal_q = [entry["goal_position"] for entry in log["entries"]]
    speed = [entry["speed"] if "speed" in entry else 0.0 for entry in log["entries"]]
    has_speed = any("speed" in entry for entry in log["entries"])

    dummy = DummyModel()
    dummy.set_actuator(actuators[args.actuator]())
    simulator = simulate.Simulator(dummy)
    _, __, controls = simulator.rollout_log(log, simulate_control=False)
    torque_enable = np.array([entry["torque_enable"] for entry in log["entries"]])

    # Create subplots: three subplots if speed is available, otherwise two.
    if has_speed:
        f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
    else:
        f, (ax1, ax3) = plt.subplots(2, sharex=True)

    # Plot joint angles and goal positions
    ax1.plot(ts, q, label="q")
    ax1.plot(ts, goal_q, label="goal_q", color="black", linestyle="--")
    if args.sim:
        for model_name, sim_q in zip(all_names, all_sim_q):
            ax1.plot(ts, sim_q, label=f"{model_name}_q")
    ax1.legend()
    title = f'{log["motor"]}, {log["trajectory"]}, m={log["mass"]}, l={log["length"]}, k={log["kp"]}'
    ax1.set_title(title)
    ax1.set_ylabel("angle [rad]")
    ax1.grid()

    # Plot speed if available
    if has_speed:
        ax2.plot(ts, speed, label="speed")
        if args.sim:
            for model_name, sim_speeds in zip(all_names, all_sim_speeds):
                ax2.plot(ts, sim_speeds, label=f"{model_name}_speed")
        ax2.set_ylabel("speed [rad/s]")
        ax2.grid()
        ax2.legend()

    # Plot control signals and highlight areas with torque off
    ax3.plot(ts, controls, label=dummy.actuator.control_unit())
    if args.sim:
        for model_name, sim_controls in zip(all_names, all_sim_controls):
            ax3.plot(ts, sim_controls, label=f"{model_name}_{dummy.actuator.control_unit()}")
    # Use fill_between to shade regions where torque is off
    control_values = [0.0 if c is None else c for c in controls]
    ax3.fill_between(
        ts,
        min(control_values) - 0.02,
        max(control_values) + 0.02,
        where=[not torque for torque in torque_enable],
        color="red",
        alpha=0.3,
        label="torque off",
    )
    ax3.set_ylabel(f"{dummy.actuator.control_unit()}")
    ax3.legend()
    plt.xlabel("time [s]")
    plt.grid()

    # Save the plot as a PNG file
    plot_filename = f"plot_{plot_index}.png"
    plot_filepath = os.path.join(args.output_dir, plot_filename)
    f.savefig(plot_filepath)
    plt.close(f)
    plot_files.append(plot_filename)
    plot_index += 1

# Generate an HTML page that displays all plots in a grid layout.
html_content = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Generated Plots</title>
    <style>
        body { font-family: Arial, sans-serif; }
        .grid-container {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            grid-gap: 10px;
        }
        .grid-item {
            text-align: center;
        }
        .grid-item img {
            width: 100%;
            height: auto;
            border: 1px solid #ccc;
        }
    </style>
</head>
<body>
    <h1>Generated Plots</h1>
    <div class="grid-container">
"""

for filename in plot_files:
    html_content += f"""
        <div class="grid-item">
            <a href="{filename}" target="_blank">
                <img src="{filename}" alt="{filename}">
            </a>
        </div>
    """

html_content += """
    </div>
</body>
</html>
"""

html_filepath = os.path.join(args.output_dir, "index.html")
with open(html_filepath, "w") as html_file:
    html_file.write(html_content)

print(f"Plots and HTML page saved in directory: {args.output_dir}")
