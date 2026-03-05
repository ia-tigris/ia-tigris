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

Clone with submodules:

```bash
git clone --recurse-submodules <repo-url>
```

If already cloned, run:

```bash
git submodule update --init --recursive
```

## Ubuntu (native) Setup

Prereqs: Ubuntu 20.04 + ROS Noetic.

From repo root:

```bash
source /opt/ros/noetic/setup.bash
sudo apt update
sudo apt install -y \
  python3-empy \
  python3-catkin-tools \
  python3-rosdep \
  python3-vcstool
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF -Wno-dev
catkin build --no-status --summarize
source devel/setup.bash
```

## Docker Setup (macOS / Ubuntu 24.04 / Headless)

For platforms without native ROS Noetic or when running headless, use Docker with Foxglove for visualization.

Install Foxglove first:

- [Foxglove Desktop download](https://foxglove.dev/download)
- [Foxglove Web App](https://app.foxglove.dev/)

**Start the container:**
```bash
cd /home/moon/code/ia-tigris
docker compose -f src/ipp_planners/docker-compose.yml up -d ros
```

**Build the workspace (first time only):**
```bash
docker compose -f src/ipp_planners/docker-compose.yml exec ros bash
# Inside container:
cd /workspace
catkin config --extend /opt/ros/noetic
catkin build --no-status --summarize
source devel/setup.bash
```

**Run the demo:**
```bash
# Inside the container (or in a new exec session):
roslaunch ipp_planners main.launch \
  config:=onr \
  planner:=tigris \
  sim:=simple \
  rviz:=false \
  foxglove:=true
```

**Connect Foxglove to:** `ws://localhost:8765`

In Foxglove:

1. Open Foxglove Desktop (or Web App).
2. Click `Open connection`.
3. Choose `WebSocket`.
4. Enter `ws://localhost:8765`.
5. Start streaming and add panels as needed (for example, 3D and Raw Messages).

**Send a plan request (in another terminal):**
```bash
docker compose -f src/ipp_planners/docker-compose.yml exec ros bash -c \
  "source /workspace/devel/setup.bash && rosrun planner_map_interfaces pub_plan_request_from_yaml.py \$(rospack find planner_map_interfaces)/config/onr/plan_requests/aug_workshop_demos/search_scenario_1.yaml"
```

The workspace is mounted as a volume (`../../:/workspace`), so build artifacts (`build/`, `devel/`, `logs/`) appear on the host.



## Run: Simple Sim Demo

### Ubuntu (RViz default)

RViz is enabled by default in `main.launch`.

```bash
roslaunch ipp_planners main.launch \
  config:=onr \
  planner:=tigris \
  sim:=simple
```

In a second terminal:

```bash
source devel/setup.bash
rosrun planner_map_interfaces pub_plan_request_from_yaml.py \
  $(rospack find planner_map_interfaces)/config/onr/plan_requests/aug_workshop_demos/search-track_scenario.yaml
```

### Docker (Foxglove)

See [Docker Setup](#docker-setup-macos--ubuntu-2404--headless) section above for the complete workflow.

Key points:
- Use `rviz:=false foxglove:=true` when launching
- Connect to `ws://localhost:8765` in Foxglove
- Foxglove bridge is already exposed via compose on port 8765

### macOS Optional: RViz via noVNC

From repo root on host:

```bash
ENABLE_HEADLESS_DISPLAY=1 docker compose -f src/ipp_planners/docker-compose.yml --profile vnc up -d
docker compose -f src/ipp_planners/docker-compose.yml exec ros bash
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
  sim:=simple
```

Open:

- `http://localhost:8080`

## Metrics / Monte Carlo Sweeps

```bash
docker compose -f src/ipp_planners/docker-compose.yml exec ros bash
roslaunch metrics sim_mc_runs.launch \
  config:=research \
  sim:=simple \
  include_cpu_mem_monitor:=true
```

## Benchmarks & Testing

For isolated testing, reproducibility, or CI (platforms without native ROS), use the benchmark script:

```bash
src/ipp_planners/scripts/build_test_benchmark.sh
```

This builds an isolated Docker image with the full workspace (no host artifacts), runs unit tests and benchmarks, and saves results to `src/ipp_planners/test_results/<timestamp>/`.

Options:
- `-b true/false` - Build image from scratch
- `-u true/false` - Run unit tests
- `-t true/false` - Run benchmarks
- `-l true/false` - Run tests locally (no Docker)

## ipp_planners Dockerfiles (Current State)

- Active image: [src/ipp_planners/Dockerfile](src/ipp_planners/Dockerfile) (used by [src/ipp_planners/docker-compose.yml](src/ipp_planners/docker-compose.yml) and [src/ipp_planners/scripts/build_test_benchmark.sh](src/ipp_planners/scripts/build_test_benchmark.sh)).

Default output directory is configured in:

- `src/ipp_metrics/launch/sim_mc_runs.launch`


TODO
- Remove extra configs. Keep research, m600, maybe devel and journal
- Remove ONR references
- Good set of example plan requests
- Check the metrics scripts to replicate paper results. Clear instructions in readme.
- Check default rviz configurations
- Have someone else try this readme and instructions. 
- Update top of readme to look like a paper reaadme with title, authors, image. Paper ciatation. Video link, website link, etc. 
- fix cgal warning about needing to build in release. ARen't we in release already? 
- default foxglove config?
  - Check foxglove visualizations. Border, etc. 
- grid cells updating 