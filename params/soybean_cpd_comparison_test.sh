#! /bin/bash

# RUN THIS SCRIPT FROM THE MAIN WORKSPACE FOLDER
PARAMS_FILE_PATH="src/uav_ugv_collaboration_module/params/aligner_params.yaml"

for z in $(seq 0 1 199):
do
	echo ${x} ${y} ${z}
	rosrun uav_ugv_collaboration_module comparison_node ${PARAMS_FILE_PATH} 70 70 ${z}
done


