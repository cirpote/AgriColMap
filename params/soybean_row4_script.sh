#! /bin/bash

# RUN THIS SCRIPT FROM THE MAIN WORKSPACE FOLDER
PARAMS_FILE_PATH="src/uav_ugv_collaboration_module/params/aligner_soybean_params_row4.yaml"

for Scale in $(seq 0 5 30):
do
	for YNoise in $(seq 0 50 250):
	do
		for ExpID in $(seq 0 1 9):
		do	
			echo ${PARAMS_FILE_PATH} ${Scale} 0 ${YNoise} ${ExpID}
			rosrun uav_ugv_collaboration_module registration_node ${PARAMS_FILE_PATH} ${Scale} 0 ${YNoise} ${ExpID}
		done
	done
done

