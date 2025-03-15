# Sampling data 

python bam/unitree/all_record.py \
	--port /dev/ttyUSB0 \
	--id 0 \
	--offset 0.39466 \
	--damping 0.3 \
	--arm_mass 0.078 \
	--masskg 1.035 \
	--length 0.150 \
	--logdir data_raw

## Post-processing

python -m bam.process \
	--raw data_raw \
	--logdir data_processed \
	--dt 0.005

## Plotting

python -m bam.plot_all \
	--actuator unitree_go1 \
	--logdir data_processed \
	--output_dir data_plots

## Model fitting

python -m bam.fit \
	--actuator unitree_go1 \
	--model m1 \
	--logdir data_processed \
	--method cmaes \
	--trials 2000