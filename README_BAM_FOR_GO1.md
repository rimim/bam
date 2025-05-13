# Installation

uv pip install puddleduck

# Find zero offset

uv run -m bam.unitree.find_zero_offset \
	--port /dev/ttyUSB0 \
	--id 0 \
	--motor unitree_go1

# Sampling data 

mkdir -p data_raw_go1 data_processed_go1 data_plots_go1

# 0kg
uv run -m bam.unitree.all_record \
	--port /dev/ttyUSB0 \
	--id 0 \
	--offset 0.20868 \
	--kp 10 \
	--damping 0.3 \
	--arm_mass 0.078 \
	--masskg 0 \
	--length 0.150 \
	--logdir data_raw_go1 \
	--motor unitree_go1

# 1kg
uv run -m bam.unitree.all_record \
	--port /dev/ttyUSB0 \
	--id 0 \
	--offset 0.20868 \
	--kp 10 \
	--damping 0.3 \
	--arm_mass 0.078 \
	--masskg 1 \
	--length 0.150 \
	--logdir data_raw_go1 \
	--motor unitree_go1

# 2kg
uv run -m bam.unitree.all_record \
	--port /dev/ttyUSB0 \
	--id 0 \
	--offset 0.20868 \
	--kp 10 \
	--damping 0.3 \
	--arm_mass 0.078 \
	--masskg 2 \
	--length 0.150 \
	--logdir data_raw_go1 \
	--motor unitree_go1

# 3kg
uv run -m bam.unitree.all_record \
	--port /dev/ttyUSB0 \
	--id 0 \
	--offset 0.20868 \
	--kp 10 \
	--damping 0.3 \
	--arm_mass 0.078 \
	--masskg 3 \
	--length 0.150 \
	--logdir data_raw_go1 \
	--motor unitree_go1

## Post-processing

uv run -m bam.process \
	--raw data_raw_go1 \
	--logdir data_processed_go1 \
	--dt 0.005

## Plotting

uv run -m bam.plot_all \
	--actuator unitree_go1 \
	--logdir data_processed_go1 \
	--output_dir data_plots_go1

## Model fitting

uv run -m bam.fit \
	--actuator unitree_go1 \
	--model m1 \
	--logdir data_processed_go1 \
	--method cmaes \
	--trials 2000

uv run -m bam.to_mujoco --params params.json --kp 10
