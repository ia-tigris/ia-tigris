#!/bin/bash
# This script builds a dev docker image, runs unit tests, and benchmarks your current codebase. 
# See flags for only running portions of the script
build_image=true 
unit_tests=true 
full_tests=true
local_test=false

while getopts b:u:t:l: flag
do
    case "${flag}" in
        b) build_image=${OPTARG};;
        u) unit_tests=${OPTARG};;
        t) full_tests=${OPTARG};;
        l) local_test=${OPTARG};;
    esac
done
echo "Script options:";
echo "Build Docker image: $build_image";
echo "Run Unit Tests: $unit_tests";
echo "Run Full Tests: $full_tests";
echo "Test locally (no Docker): $local_test";


# Builds the Docker Image
if [ "$build_image" = true ] ; then
    build_file="`dirname $0`/../Docker/dev.Dockerfile"
    build_context="`dirname $0`/../../../"
    echo "Building Docker image of $build_file on 18.04" 
    echo "Build context is $build_context"
    docker build -t ipp-dev-18-04 -f $build_file $build_context

    if [ $? -eq 0 ]; then
        echo ""
        echo ""
        echo "Docker image built successfully on 18.04"
    else
        echo ""
        echo "Docker image failed to build on 18.04"
        exit 1
    fi
fi

# Make folder for results
if [ "$unit_tests" = true ] || [ "$full_tests" = true ] ; then
    curr_date=$(date +"%Y-%m-%d_%T")
    result_dir_name="`dirname $0`/../test_results/${curr_date}"
    mkdir -p $result_dir_name
    echo "Saving to folder $result_dir_name"
    results_mount_folder="$(realpath $(pwd)/$result_dir_name)"
    echo "Mount folder is $results_mount_folder"
    cd $results_mount_folder/../../../../
    touch $results_mount_folder/notes.txt
    vcs diff > $results_mount_folder/vcs_diff.txt
    vcs export > $results_mount_folder/ws.repos --exact
fi

# Runs the unit tests and google benchmarks
if [ "$unit_tests" = true ] ; then
    test_file="src/ipp_planners/scripts/tests.sh"
    echo ""
    echo ""
    echo "Running google unit tests and benchmarks of $test_file"
    if [ "$local_test" = true ] ; then
        cd $results_mount_folder/../../../../
        echo "Running locally"
        $test_file $results_mount_folder
    else
        echo "Running in Docker"
        docker run -it \
        --memory=8g \
        --cpus=4 \
        --rm \
        --name run_those_tests \
        --mount type=bind,source="$results_mount_folder",target=/test_results \
        ipp-dev-18-04 \
        /bin/bash $test_file /test_results
    fi
fi

# Run full benchmarks
if [ "$full_tests" = true ] ; then
    test_file="src/ipp_planners/scripts/search_benchmarks.sh"
    echo ""
    echo ""
    echo "Running benchmarks from script $test_file"
    echo ""
    if [ "$local_test" = true ] ; then
        cd $results_mount_folder/../../../../
        echo "Running locally"
        $test_file $results_mount_folder
    else
        echo "Running in Docker"
        docker run -it \
        --memory=8g \
        --cpus=4 \
        --rm \
        --name run_those_tests \
        --mount type=bind,source="$results_mount_folder",target=/test_results \
        ipp-dev-18-04 \
        /bin/bash $test_file /test_results
    fi

    test_file="src/ipp_planners/scripts/search_track_benchmarks.sh"
    echo ""
    echo ""
    echo "Running benchmarks from script $test_file"
    echo ""
    if [ "$local_test" = true ] ; then
        cd $results_mount_folder/../../../../
        echo "Running locally"
        $test_file $results_mount_folder
    else
        echo "Running in Docker"
        docker run -it \
        --memory=8g \
        --cpus=4 \
        --rm \
        --name run_those_tests \
        --mount type=bind,source="$results_mount_folder",target=/test_results \
        ipp-dev-18-04 \
        /bin/bash $test_file /test_results
    fi

    test_file="src/ipp_planners/scripts/track_benchmarks.sh"
    echo ""
    echo ""
    echo "Running benchmarks from script $test_file"
    echo ""
    if [ "$local_test" = true ] ; then
        cd $results_mount_folder/../../../../
        echo "Running locally"
        $test_file $results_mount_folder
    else
        echo "Running in Docker"
        docker run -it \
        --memory=8g \
        --cpus=4 \
        --rm \
        --name run_those_tests \
        --mount type=bind,source="$results_mount_folder",target=/test_results \
        ipp-dev-18-04 \
        /bin/bash $test_file /test_results
    fi
fi


