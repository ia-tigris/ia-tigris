#!/bin/bash
# This script runs the unit tests and benchmarks
source devel/setup.bash
folder_path=$1
echo "Folder is $folder_path"
mkdir -p $folder_path/gtests

roscore &
sleep 2

# Run benchmarks and save results
./devel/lib/ipp_planners/ipp_planners-benchmark --benchmark_out=$folder_path/gtests/ipp_planners_google_benchmarks.json --benchmark_out_format=json
./devel/lib/trochoids/trochoids-benchmark --benchmark_out=$folder_path/gtests/trochoids_google_benchmarks.json --benchmark_out_format=json

# Run unit tests and save results
rosrun trochoids trochoids-test > $folder_path/gtests/trochoids_unit_tests.txt
rosrun ipp_planners ipp_planners-test > $folder_path/gtests/ipp_planners_unit_tests.txt

pkill rosmaster
exit





