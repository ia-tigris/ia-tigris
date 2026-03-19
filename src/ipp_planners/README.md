# ipp_planners

Informative path planning package used in the `ia-tigris` ROS Noetic workspace.

## Start Here

Most setup and run instructions now live at the workspace level:

- [Workspace README](../../README.md)

For Docker details specific to this package:

- [Docker notes](Docker/README.md)
- [Compose file](docker-compose.yml)
- [Dockerfile](Dockerfile)

## Package-Specific Usage

After the workspace is built and sourced (see top-level README), launch the planner:

```bash
roslaunch ipp_planners main.launch \
    config:=fixed-wing \
    planner:=tigris \
    sim:=simple
```

Publish an example request:

```bash
source devel/setup.bash
rosrun planner_map_interfaces pub_plan_request_from_yaml.py \
    $(rospack find planner_map_interfaces)/config/fixed-wing/plan_requests/aug_workshop_demos/search-track_scenario.yaml
```

## Tests

Build and run unit tests:

```bash
catkin build --make-args tests -- ipp_planners
rosrun ipp_planners ipp_planners-test
```

Alternative launch-based test entrypoint:

```bash
roslaunch ipp_planners unit_test.launch
```

## Benchmarks

Run benchmark helper scripts from this package:

```bash
src/ipp_planners/scripts/run_and_save_google_benchmarks.sh
```

For isolated Docker-based test/benchmark runs:

```bash
src/ipp_planners/scripts/build_test_benchmark.sh
```
 $ sudo apt-get remove docker docker-engine docker.io

 ```

3. Check if the system is up-to-date using the following command 

 ```
