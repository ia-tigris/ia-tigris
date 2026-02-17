# echo "Running task number in container $SLURM_ARRAY_TASK_ID"

# cd /workspace/

# source devel/setup.bash
# cd src/ipp_metrics/scripts/
# roscore &

# # Run the script. Modify based on run id
# python3 parameter_sweep.py "/storage2/datasets/bradym/test_results/tests1/" "100" "100" "100"
echo "Running task number in container $SLURM_ARRAY_TASK_ID"

WORKSPACE_DIR="ipp_ws/"
echo "Workspace directory: $WORKSPACE_DIR"
cd $WORKSPACE_DIR

# Zero-pad SLURM_ARRAY_TASK_ID to have at least 4 digits
PADDED_TASK_ID=$(printf "%02d" $SLURM_ARRAY_TASK_ID)

echo "Before ROS_MASTER_URI: $ROS_MASTER_URI"
# Construct the ROS_MASTER_URI
# export ROS_MASTER_URI="http://localhost:500${PADDED_TASK_ID}"
echo "After ROS_MASTER_URI: $ROS_MASTER_URI"

# Print the result
echo "Ros master URI: $ROS_MASTER_URI"

echo "print roscor"
ps aux | grep roscore

source /opt/ros/noetic/setup.bash
source devel/setup.bash
echo "sourced workspace"

#############################
# Script for Param Sweep
#############################

# cd src/ipp_metrics/scripts/
# roscore &

# sleep 10
# echo "Launched roscore"

# 15 Jan 2023
# Select arguments based on SLURM_ARRAY_TASK_ID
# case $SLURM_ARRAY_TASK_ID in
#     1) ARGS='"100, 250, 400, 500" "100" "100"';;
#     2) ARGS='"100, 250, 400, 500" "100" "250"';;
#     3) ARGS='"100, 250, 400, 500" "100" "400"';;
#     4) ARGS='"100, 250, 400, 500" "250" "100"';;
#     5) ARGS='"100, 250, 400, 500" "250" "250"';;
#     6) ARGS='"100, 250, 400, 500" "250" "400"';;
#     7) ARGS='"100, 250, 400, 500" "500" "100"';;
#     8) ARGS='"100, 250, 400, 500" "500" "250"';;
#     9) ARGS='"100, 250, 400, 500" "500" "400"';;
#     10) ARGS='"100, 250, 400, 500" "1000" "100"';;
#     11) ARGS='"100, 250, 400, 500" "1000" "250"';;
#     12) ARGS='"100, 250, 400, 500" "1000" "400"';;
#     13) ARGS='"100, 250, 400, 500" "1500" "100"';;
#     14) ARGS='"100, 250, 400, 500" "1500" "250"';;
#     15) ARGS='"100, 250, 400, 500" "1500" "400"';;
#     16) ARGS='"100, 250, 400, 500" "2000" "100"';;
#     17) ARGS='"100, 250, 400, 500" "2000" "250"';;
#     18) ARGS='"100, 250, 400, 500" "2000" "400"';;
#     *) echo "Invalid SLURM_ARRAY_TASK_ID"; exit 1;;
# esac

# 16 Jan 2023
# Select arguments based on SLURM_ARRAY_TASK_ID 4 each gives 8.5 hours
# case $SLURM_ARRAY_TASK_ID in
#     1) ARGS='"50" "250, 500, 750" "100"';;
#     2) ARGS='"100" "600, 750, 900" "100"';;
#     3) ARGS='"1000" "2000" "100, 250, 400"';;
#     4) ARGS='"2000" "1000" "100, 250, 400"';;
#     5) ARGS='"2000" "1500" "100, 250, 400"';;
#     6) ARGS='"50" "250, 500, 750" "250"';;
#     7) ARGS='"250" "200" "600, 1000, 2000"';;
#     8) ARGS='"2000" "2000" "250, 1000, 2000"';;
#     *) echo "Invalid SLURM_ARRAY_TASK_ID"; exit 1;;
# esac

# Debug
# Select arguments based on SLURM_ARRAY_TASK_ID 4 each gives 8.5 hours
# case $SLURM_ARRAY_TASK_ID in
#     1) ARGS='"50" "250, 750" "100"';;
#     2) ARGS='"100" "600, 750" "100"';;
#     3) ARGS='"1000" "2000" "100, 250"';;
#     *) echo "Invalid SLURM_ARRAY_TASK_ID"; exit 1;;
# esac

# Param Sweep 1 May 2024
# case $SLURM_ARRAY_TASK_ID in
#     1) ARGS='"100" "100, 500" "50, 150, 400, 600"';;
#     2) ARGS='"100" "1000, 1500" "50, 150, 400, 600"';;
#     3) ARGS='"100" "3000, 5000" "50, 150, 400, 600"';;
#     4) ARGS='"1500" "1000, 1500" "50, 150, 400, 600"';;
#     5) ARGS='"1500" "3000, 5000" "50, 150, 400, 600"';;
#     6) ARGS='"3000" "1000, 1500" "50, 150, 400, 600"';;
#     7) ARGS='"3000" "3000, 5000" "50, 150, 400, 600"';;
#     8) ARGS='"5000" "3000, 5000" "50, 150, 400, 600"';;
#     9) ARGS='"500" "100, 500" "50, 150, 400, 600"';;
#     10) ARGS='"500" "1000, 1500" "50, 150, 400, 600"';;
#     11) ARGS='"500" "3000, 5000" "50, 150, 400, 600"';;
#     12) ARGS='"1000" "100, 500" "400, 600"';;
#     13) ARGS='"1000" "1000, 1500" "400, 600"';;
#     14) ARGS='"1000" "3000, 5000" "400, 600"';;
#     15) ARGS='"1500" "100, 500" "400, 600"';;
#     *) echo "Invalid SLURM_ARRAY_TASK_ID"; exit 1;;
# esac

    

# # Param Sweep 2 May 2024
# case $SLURM_ARRAY_TASK_ID in
#     1) ARGS='"100" "100" "50, 150, 400, 600"';;
#     2) ARGS='"100" "500" "50, 150, 400, 600"';;
#     3) ARGS='"100" "1000" "50, 150, 400, 600"';;
#     4) ARGS='"100" "1500" "50, 150, 400, 600"';;
#     5) ARGS='"100" "3000" "50, 150, 400, 600"';;
#     6) ARGS='"100" "5000" "50, 150, 400, 600"';;
#     7) ARGS='"500" "100" "50, 150, 400, 600"';;
#     8) ARGS='"500" "500" "50, 150, 400, 600"';;
#     9) ARGS='"500" "1000" "50, 150, 400, 600"';;
#     10) ARGS='"500" "1500" "50, 150, 400, 600"';;
#     11) ARGS='"500" "3000" "50, 150, 400, 600"';;
#     12) ARGS='"500" "5000" "50, 150, 400, 600"';;
#     13) ARGS='"1000" "100" "50, 150, 400, 600"';;
#     14) ARGS='"1000" "500" "50, 150, 400, 600"';;
#     15) ARGS='"1000" "1000" "50, 150, 400, 600"';;
#     16) ARGS='"1000" "1500" "50, 150, 400, 600"';;
#     17) ARGS='"1000" "3000" "50, 150, 400, 600"';;
#     18) ARGS='"1000" "5000" "50, 150, 400, 600"';;
#     19) ARGS='"1500" "100" "50, 150, 400, 600"';;
#     20) ARGS='"1500" "500" "50, 150, 400, 600"';;
#     21) ARGS='"1500" "1000" "50, 150, 400, 600"';;
#     22) ARGS='"1500" "1500" "50, 150, 400, 600"';;
#     23) ARGS='"1500" "3000" "50, 150, 400, 600"';;
#     24) ARGS='"1500" "5000" "50, 150, 400, 600"';;
#     25) ARGS='"3000" "100" "50, 150, 400, 600"';;
#     26) ARGS='"3000" "500" "50, 150, 400, 600"';;
#     27) ARGS='"3000" "1000" "50, 150, 400, 600"';;
#     28) ARGS='"3000" "1500" "50, 150, 400, 600"';;
#     29) ARGS='"3000" "3000" "50, 150, 400, 600"';;
#     30) ARGS='"3000" "5000" "50, 150, 400, 600"';;
#     31) ARGS='"5000" "100" "50, 150, 400, 600"';;
#     32) ARGS='"5000" "500" "50, 150, 400, 600"';;
#     33) ARGS='"5000" "1000" "50, 150, 400, 600"';;
#     34) ARGS='"5000" "1500" "50, 150, 400, 600"';;
#     35) ARGS='"5000" "3000" "50, 150, 400, 600"';;
#     36) ARGS='"5000" "5000" "50, 150, 400, 600"';;
#     *) echo "Invalid SLURM_ARRAY_TASK_ID"; exit 1;;
# esac

# Param Sweep 4 May 2024
# case $SLURM_ARRAY_TASK_ID in
#     1) ARGS='"100" "100" "600"';;
#     2) ARGS='"100" "500" "400"';;
#     3) ARGS='"100" "1000" "600"';;
#     4) ARGS='"100" "1500" "400"';;
#     5) ARGS='"100" "3000" "600"';;
#     6) ARGS='"100" "5000" "600"';;
#     7) ARGS='"1500" "100" "50"';;
#     8) ARGS='"1500" "500" "150"';;
#     9) ARGS='"1500" "500" "400"';;
#     10) ARGS='"1500" "500" "50"';;
#     11) ARGS='"1500" "500" "600"';;
#     12) ARGS='"1500" "1000" "600"';;
#     13) ARGS='"1500" "1500" "400"';;
#     14) ARGS='"1500" "3000" "400"';;
#     15) ARGS='"3000" "100, 500, 1000" "600"';;
#     16) ARGS='"3000" "1500, 3000" "400"';;
#     17) ARGS='"3000" "5000" "600"';;
#     18) ARGS='"5000" "100" "600"';;
#     19) ARGS='"5000" "1000" "400"';;
#     20) ARGS='"5000" "3000" "600"';;
#     21) ARGS='"5000" "5000" "600"';;
#     *) echo "Invalid SLURM_ARRAY_TASK_ID"; exit 1;;
# esac

# eval python3 parameter_sweep.py "/storage2/datasets/bradym/test_results/param_sweep_v40/" $ARGS

#############################
# Script for MC testing Random Plan
#############################
# FOLDER_NAME="/storage2/datasets/bradym/test_results/full_tests_v37"

# mkdir -p ${FOLDER_NAME}/${SLURM_ARRAY_TASK_ID}/

# # Run the script with the selected arguments
# roslaunch metrics sim_mc_runs.launch \
# rviz:=false \
# include_cpu_mem_monitor:=true \
# cpu_mem_csv_file:=${FOLDER_NAME}/${SLURM_ARRAY_TASK_ID}/cpu_mem_metrics.csv \
# log_plan_metrics:=true \
# plan_metrics_csv_directory:=${FOLDER_NAME}/${SLURM_ARRAY_TASK_ID}/ \
# search:=true \
# track:=false \
# mc_config:=mc_testing_search1_random_v33.yaml \
# sim:=simple

#############################
# Script for MC testing from plan request
#############################
# FOLDER_NAME="/storage2/datasets/bradym/test_results/full_tests_v8"

# echo "Created directory: ${FOLDER_NAME}/${SLURM_ARRAY_TASK_ID}/"
# mkdir -p ${FOLDER_NAME}/${SLURM_ARRAY_TASK_ID}/
# echo "FOLDER_NAME: $FOLDER_NAME"
# echo "SLURM_ARRAY_TASK_ID: $SLURM_ARRAY_TASK_ID"


# cd src/ipp_metrics/scripts/
# roscore &

# sleep 5
# echo "Launched roscore"

# eval python3 mc_testing_with_plan_request.py ${FOLDER_NAME}/${SLURM_ARRAY_TASK_ID}/

#############################
# Script for Test Sweep
#############################

# cd src/ipp_metrics/scripts/
# roscore &

# sleep 10
# echo "Launched roscore"

# 30 April 2024
# Select arguments based on SLURM_ARRAY_TASK_ID
# case $SLURM_ARRAY_TASK_ID in
#     1) ARGS='"4, 8" "2"';;
#     2) ARGS='"12, 16" "2"';;
#     3) ARGS='"20" "2, 6"';;
#     4) ARGS='"4, 8" "6"';;
#     5) ARGS='"12, 16" "6"';;
#     6) ARGS='"4, 8" "10"';;
#     7) ARGS='"12, 16" "10"';;
#     8) ARGS='"20" "10, 20"';;
#     9) ARGS='"4, 8" "20"';;
#     10) ARGS='"12, 16" "20"';;
#     *) echo "Invalid SLURM_ARRAY_TASK_ID"; exit 1;;
# esac

# 6 May 2024. 4-20 targets. 5 planners each. 
# Select arguments based on SLURM_ARRAY_TASK_ID
# case $SLURM_ARRAY_TASK_ID in
#     1) ARGS='"4"';;
#     2) ARGS='"5"';;
#     3) ARGS='"6"';;
#     4) ARGS='"7"';;
#     5) ARGS='"8"';;
#     6) ARGS='"9"';;
#     7) ARGS='"10"';;
#     8) ARGS='"11"';;
#     9) ARGS='"12"';;
#     10) ARGS='"13"';;
#     11) ARGS='"14"';;
#     12) ARGS='"15"';;
#     13) ARGS='"16"';;
#     14) ARGS='"17"';;
#     15) ARGS='"18"';;
#     16) ARGS='"19"';;
#     17) ARGS='"20"';;
#     *) echo "Invalid SLURM_ARRAY_TASK_ID"; exit 1;;
# esac

# eval python3 test_sweep_targets.py "/storage2/datasets/bradym/test_results/full_tests_v39_sweep/" $ARGS