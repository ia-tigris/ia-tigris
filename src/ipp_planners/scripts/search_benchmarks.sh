#!/bin/bash
# This script ajfdklsdfj
folder_path=$1
set -m
catkin_devel_path="${CATKIN_DEVEL_PATH:-devel}"
source "/opt/ros/noetic/setup.bash"
source "$catkin_devel_path/setup.bash"


############################################
# Search Scenario 1
############################################


mkdir -p $folder_path/search_scenario_1

# Launch planner and metrics saving
roslaunch metrics sim_mc_runs.launch \
rviz:=false \
include_cpu_mem_monitor:=true \
cpu_mem_csv_file:=$folder_path/search_scenario_1/cpu_mem_metrics.csv \
log_plan_metrics:=true \
plan_metrics_csv_directory:=$folder_path/search_scenario_1/ \
mc_config:=mc_testing_search1.yaml \
sim:=false \
> $folder_path/search_scenario_1/stdout.txt &

# roslaunch metrics sim_mc_runs.launch \
# rviz:=false \
# include_cpu_mem_monitor:=true \
# cpu_mem_csv_file:=$folder_path/search_scenario_1/cpu_mem_metrics.csv \
# mc_config:=mc_testing_search1.yaml \
# sim:=false &

# Wait for everything to be running
sleep 5

# Send plan request
rosrun planner_map_interfaces pub_plan_request_from_yaml.py src/planner_map_interfaces/config/onr/plan_requests/benchmarks/search_scenario_1.yaml {} ""&

# Bring main process forward
fg %1

if [ $? -eq 0 ]; then
    echo ""
    echo ""
    echo "Search Benchmarks 1 ran successfully"
else
    echo ""
    echo "Search Benchmarks 1 failed to run"
    exit 1
fi

pkill rosmaster

############################################
# Search Scenario 2
############################################

mkdir -p $folder_path/search_scenario_2

# Launch planner and metrics saving
roslaunch metrics sim_mc_runs.launch \
rviz:=false \
include_cpu_mem_monitor:=true \
cpu_mem_csv_file:=$folder_path/search_scenario_2/cpu_mem_metrics.csv \
log_plan_metrics:=true \
plan_metrics_csv_directory:=$folder_path/search_scenario_2/ \
mc_config:=mc_testing_search1.yaml \
sim:=false \
> $folder_path/search_scenario_2/stdout.txt &

# roslaunch metrics sim_mc_runs.launch \
# rviz:=false \
# include_cpu_mem_monitor:=true \
# cpu_mem_csv_file:=$folder_path/search_scenario_2/cpu_mem_metrics.csv  \
# mc_config:=mc_testing_search1.yaml \
# sim:=false &

# Wait for everything to be running
sleep 5

# Send plan request
rosrun planner_map_interfaces pub_plan_request_from_yaml.py src/planner_map_interfaces/config/onr/plan_requests/benchmarks/search_scenario_2.yaml {} ""&

# Bring main process forward
jobs
fg %3

if [ $? -eq 0 ]; then
    echo ""
    echo ""
    echo "Search Benchmarks 2 ran successfully"
else
    echo ""
    echo "Search Benchmarks 2 failed to run"
    exit 1
fi
pkill rosmaster


############################################
# Search Scenario 3
############################################

mkdir -p $folder_path/search_scenario_3

# Launch planner and metrics saving
roslaunch metrics sim_mc_runs.launch \
rviz:=false \
include_cpu_mem_monitor:=true \
cpu_mem_csv_file:=$folder_path/search_scenario_3/cpu_mem_metrics.csv \
log_plan_metrics:=true \
plan_metrics_csv_directory:=$folder_path/search_scenario_3/ \
mc_config:=mc_testing_search1.yaml \
sim:=false \
> $folder_path/search_scenario_3/stdout.txt &

# roslaunch metrics sim_mc_runs.launch \
# rviz:=false \
# include_cpu_mem_monitor:=true \
# cpu_mem_csv_file:=$folder_path/search_scenario_3/cpu_mem_metrics.csv  \
# mc_config:=mc_testing_search1.yaml \
# sim:=false &

# Wait for everything to be running
sleep 5

# Send plan request
rosrun planner_map_interfaces pub_plan_request_from_yaml.py src/planner_map_interfaces/config/onr/plan_requests/benchmarks/search_scenario_3.yaml {} ""&

# Bring main process forward
jobs
fg %5

if [ $? -eq 0 ]; then
    echo ""
    echo ""
    echo "Search Benchmarks 3 ran successfully"
else
    echo ""
    echo "Search Benchmarks 3 failed to run"
    exit 1
fi
pkill rosmaster


############################################
# Search Random Scenario 1
############################################

mkdir -p $folder_path/search_random_1

# Launch planner and metrics saving
roslaunch metrics sim_mc_runs.launch \
rviz:=false \
include_cpu_mem_monitor:=true \
cpu_mem_csv_file:=$folder_path/search_random_1/cpu_mem_metrics.csv \
log_plan_metrics:=true \
plan_metrics_csv_directory:=$folder_path/search_random_1/ \
mc_config:=mc_testing_search1_random.yaml \
sim:=false \
> $folder_path/search_random_1/stdout.txt &

# Wait for everything to be running
sleep 5

# Send plan request
rosrun planner_map_interfaces pub_plan_request_from_yaml.py src/planner_map_interfaces/config/onr/plan_requests/benchmarks/search_scenario_3.yaml {} ""&

# Bring main process forward
jobs
fg %7

if [ $? -eq 0 ]; then
    echo ""
    echo ""
    echo "Search Random 1 ran successfully"
else
    echo ""
    echo "Search Random 1 failed to run"
    exit 1
fi
pkill rosmaster

############################################
# Search Scenario 1 with collisions
############################################

mkdir -p $folder_path/search_collisions_1

# Launch planner and metrics saving
roslaunch metrics sim_mc_runs.launch \
rviz:=true \
include_cpu_mem_monitor:=true \
cpu_mem_csv_file:=$folder_path/search_collisions_1/cpu_mem_metrics.csv \
log_plan_metrics:=true \
plan_metrics_csv_directory:=$folder_path/search_collisions_1/ \
mc_config:=mc_testing_search1.yaml \
sim:=false \
> $folder_path/search_collisions_1/stdout.txt &

# Wait for everything to be running
sleep 5

# Send plan request
rosrun planner_map_interfaces pub_plan_request_from_yaml.py src/planner_map_interfaces/config/research/plan_requests/feature_tests/keep_out_zones_test.yaml {} ""&

# Bring main process forward
jobs
fg %9

if [ $? -eq 0 ]; then
    echo ""
    echo ""
    echo "Search with keep out zones 1 ran successfully"
else
    echo ""
    echo "Search with keep out zones 1 failed to run"
    exit 1
fi
pkill rosmaster


exit
