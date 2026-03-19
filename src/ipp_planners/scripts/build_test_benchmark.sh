#!/bin/bash
# This script builds an isolated test docker image, runs unit tests, and benchmarks.
# The image copies source into the container and builds there (no host pollution).
# See flags for only running portions of the script.
build_image=true
unit_tests=true
full_tests=true
local_test=false
workspace_root="$(realpath "$(dirname "$0")/../../../")"
docker_image="ipp-test"

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


# Builds the isolated test Docker image (multi-stage target)
if [ "$build_image" = true ] ; then
    echo "Building isolated test image '$docker_image' from workspace root"
    docker build \
        --memory=8g \
        -t "$docker_image" \
        --target test \
        -f src/ipp_planners/Dockerfile \
        "$workspace_root"

    if [ $? -eq 0 ]; then
        echo ""
        echo "Docker test image built successfully"
    else
        echo ""
        echo "Docker test image failed to build"
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
        echo "Running in isolated Docker container"
        docker run -it \
            --memory=8g \
            --cpus=4 \
            --rm \
            --name run_those_tests \
            --mount type=bind,source="$results_mount_folder",target=/test_results \
            "$docker_image" \
            /bin/bash $test_file /test_results
    fi
fi

# Run full benchmarks
if [ "$full_tests" = true ] ; then
    benchmark_scripts=(
        "src/ipp_planners/scripts/search_benchmarks.sh"
    )

    for test_file in "${benchmark_scripts[@]}"; do
        echo ""
        echo ""
        echo "Running benchmarks from script $test_file"
        echo ""

        if [ ! -f "$workspace_root/$test_file" ]; then
            echo "Skipping missing benchmark script: $test_file"
            continue
        fi

        if [ "$local_test" = true ] ; then
            cd $results_mount_folder/../../../../
            echo "Running locally"
            $test_file $results_mount_folder
        else
            echo "Running in isolated Docker container"
            docker run -it \
                --memory=8g \
                --cpus=4 \
                --rm \
                --name run_those_tests \
                --mount type=bind,source="$results_mount_folder",target=/test_results \
                "$docker_image" \
                /bin/bash $test_file /test_results
        fi
    done
fi


