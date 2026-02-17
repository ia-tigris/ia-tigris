# ia-tigris

ROS1 (Noetic) workspace for planner, simple simulation, and metrics/benchmark runs.

## Recommended Visualization Approach

- Ubuntu: run ROS natively and use RViz (best experience).
- macOS: run in Docker and use Foxglove Bridge (most reliable).
- macOS RViz via noVNC is available, but treat it as optional/experimental.

## Workspace Layout

This repository is the catkin workspace root:

- `src/` contains ROS packages
- `build/`, `devel/`, `logs/` are generated artifacts

After cloning:

```bash
git submodule update --init --recursive
```

## Ubuntu (native) Setup

Prereqs: Ubuntu 20.04 + ROS Noetic.

From repo root:

```bash
source /opt/ros/noetic/setup.bash
catkin config --extend /opt/ros/noetic
catkin build --no-status --summarize
source devel/setup.bash
```

## macOS Setup (Docker)

```bash
cd src/ipp_planners
docker compose up -d ros
docker compose exec ros bash
```

Use `up + exec` as the default workflow so all terminals use the same running ROS container.
Container is configured with an entrypoint that auto-sources ROS (`/opt/ros/noetic/setup.bash`).
Build once inside the container:

```bash
cd /workspace
catkin config --extend /opt/ros/noetic
catkin build --no-status --summarize
```

## Run: Simple Sim Demo

### Ubuntu (RViz default)

RViz is enabled by default in `main.launch`.

```bash
roslaunch ipp_planners main.launch \
  config:=onr \
  planner:=tigris \
  sim:=simple \
  search:=true \
  track:=false
```

In a second terminal:

```bash
source devel/setup.bash
rosrun planner_map_interfaces pub_plan_request_from_yaml.py \
  $(rospack find planner_map_interfaces)/config/onr/plan_requests/aug_workshop_demos/search-track_scenario.yaml
```

### macOS (Foxglove recommended)

Terminal 1:

```bash
docker compose exec ros bash
roslaunch ipp_planners main.launch \
  config:=onr \
  planner:=tigris \
  sim:=simple \
  search:=true \
  track:=false \
  rviz:=false \
  foxglove:=true
```

Then connect Foxglove to:

- `ws://localhost:8765`

To publish an example request:

```bash
docker compose exec ros bash
rosrun planner_map_interfaces pub_plan_request_from_yaml.py \
  $(rospack find planner_map_interfaces)/config/onr/plan_requests/aug_workshop_demos/search-track_scenario.yaml
```

### macOS Optional: RViz via noVNC

From `src/ipp_planners` on host:

```bash
ENABLE_HEADLESS_DISPLAY=1 docker compose --profile vnc up -d
docker compose exec ros bash
```

Notes:

- `ENABLE_HEADLESS_DISPLAY=1` is required for RViz/noVNC mode.
- Apple Silicon (M1/M2/M3): the `novnc` image is `linux/amd64`, so it runs via emulation.
If Docker asks, enable x86/amd64 emulation (Rosetta) in Docker Desktop settings.

Inside container:

```bash
roslaunch ipp_planners main.launch \
  config:=onr \
  planner:=tigris \
  sim:=simple \
  search:=true \
  track:=false
```

Open:

- `http://localhost:8080`

## Metrics / Monte Carlo Sweeps

```bash
docker compose exec ros bash
roslaunch metrics sim_mc_runs.launch \
  config:=research \
  sim:=simple \
  search:=true \
  track:=false \
  include_cpu_mem_monitor:=true
```

Default output directory is configured in:

- `src/ipp_metrics/launch/sim_mc_runs.launch`
