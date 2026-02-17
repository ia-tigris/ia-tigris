#!/bin/bash
# This script ajfdklsdfj
folder_path=$1
set -m
source devel/setup.bash


############################################
# Search Scenario 1
############################################


mkdir -p $folder_path/search_track_scenario_1

# Launch planner and metrics saving
roslaunch metrics sim_mc_runs.launch \
rviz:=false \
include_cpu_mem_monitor:=true \
cpu_mem_csv_file:=$folder_path/search_track_scenario_1/cpu_mem_metrics.csv \
log_plan_metrics:=true \
plan_metrics_csv_directory:=$folder_path/search_track_scenario_1/ \
search:=true \
track:=true \
mc_config:=mc_testing_track1.yaml \
sim:=false \
> $folder_path/search_track_scenario_1/stdout.txt &

# roslaunch metrics sim_mc_runs.launch \
# rviz:=false \
# include_cpu_mem_monitor:=true \
# cpu_mem_csv_file:=$folder_path/search_scenario_1/cpu_mem_metrics.csv \
# search:=true \
# track:=false \
# mc_config:=mc_testing_search1.yaml \
# sim:=false &

# Wait for everything to be running
sleep 5

# Send plan request
rosrun planner_map_interfaces pub_plan_request_from_yaml.py src/planner_map_interfaces/config/onr/plan_requests/benchmarks/search-track_scenario.yaml {} ""&

# Bring main process forward
fg %1

if [ $? -eq 0 ]; then
    echo ""
    echo ""
    echo "Search/Track Benchmarks 1 ran successfully"
else
    echo ""
    echo "Search/Track Benchmarks 1 failed to run"
    exit 1
fi

pkill rosmaster


exit
