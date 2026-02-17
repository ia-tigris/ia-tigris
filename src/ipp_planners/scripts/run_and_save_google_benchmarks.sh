#!/bin/bash
# This script runs the benchmarks and saves the results in a file with the date, time, and commit hash
commit_id=$(git log --format="%h" -n 1)
curr_date=$(date +"%Y-%m-%d_%T")
result_file_name="`dirname $0`/../test_results/commit_google_benchmarks/benchmark_${curr_date}_${commit_id}.json"
echo "Saving benchmarks to $result_file_name"

./"`dirname $0`"/../../../devel/lib/ipp_planners/ipp_planners-benchmark --benchmark_out=$result_file_name --benchmark_out_format=json
