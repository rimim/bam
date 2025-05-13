## Installation

```bash
uv pip install mujoco puddleduck
```

If you want the matplotlib to display the graphs at the end instead of just saving a PNG:

```bash
uv pip install pyqt5
```

## Running sim test (sim only)
```bash
uv run -m bam.mujoco_rig.unitree --motor unitree_go1 --kp 10 --kd 0.3 --plot
```

## Running real actuator and simulator (position actuator)
```bash
uv run -m bam.mujoco_rig.unitree --motor unitree_go1 --port /dev/ttyUSB0 --kp 10 --kd 0.3 --plot
```
