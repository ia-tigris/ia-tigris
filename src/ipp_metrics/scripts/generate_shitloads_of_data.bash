#!/bin/bash
set -e

parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
ros_ws_path=`readlink -f "$parent_path/../../.."`

# move the existing folder if it has files
if [ -d "$HOME/.ros/train_data" ] && find "$HOME/.ros/train_data"  -mindepth 1 -maxdepth 1 | read; then
    echo; echo;
    new_path="$HOME/.ros/train_data_$( date +%T )"
    echo "MOVING ~/.ros/train_data to $new_path"
    mv $HOME/.ros/train_data $new_path || echo "No train_data to move, moving on"
fi

echo "GENERATING SHITLOADS OF DATA UNDER ~/.ros/train_data"
mkdir -p ~/.ros/train_data

# copy configs
echo "COPYING YAML FILES AND GIT HASHES TO ~/.ros/train_data"
set -x
cp "$ros_ws_path"/src/ipp_metrics/config/*.yaml ~/.ros/train_data
cp  "$ros_ws_path"/src/ipp_planners/config/*mcts.yaml ~/.ros/train_data
cp  "$ros_ws_path/src/planner_map_interfaces/config/sensor_params.yaml" ~/.ros/train_data
cp  "$ros_ws_path/src/planner_map_interfaces/config/sensor_model_0.csv" ~/.ros/train_data
vcs export --exact > ~/.ros/train_data/onr_ws_exact.repos
set +x

# build
echo; echo;
echo "BUILDING IN RELEASE MODE WITH -O3"
echo; echo;
set -x
catkin build --cmake-args "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-O4 -Wall -Wextra"

# run
source "$ros_ws_path/devel/setup.bash"
set +x

echo "STARTING TRIALS"

counter=1
while true; do
    echo; echo; echo;
    echo "------ RUNNING EXECUTION $counter ------"
    set -x
    roslaunch metrics sim_mc_runs.launch config:=mc_gen_train_data.yaml gen_train_data:=true "$@"
    set +x
    echo "------ COMPLETED EXECUTION $counter ------"
    echo "Sleeping for 3 seconds to make sure everything shuts down"
    sleep 3;   # sleep just to make sure everything shuts down okay
    counter=$((counter+1))
done

