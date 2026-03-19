# ipp_planners Docker

The Dockerfile is at [src/ipp_planners/Dockerfile](../Dockerfile) (the compose context root).

## Multi-stage Build

Two stages serve different purposes:

### `ipp_planner` stage (interactive dev with compose)
- Base image with ROS dependencies, tools, and entrypoint
- **No source code copied** (workspace mounted from host instead)
- Used by `docker compose` for interactive development
- You run `catkin build` inside the container; artifacts write to mounted workspace

### `test` stage (isolated testing/CI)
- Extends `ipp_planner` with `COPY src/` and full `catkin build`
- Self-contained: builds everything inside the image
- Used by `src/ipp_planners/scripts/build_test_benchmark.sh`
- No pollution of the host filesystem (useful for CI/benchmarking)

## Workflows

**Interactive development**:
```bash
docker compose -f src/ipp_planners/docker-compose.yml up -d ros
docker compose -f src/ipp_planners/docker-compose.yml exec ros bash

# Inside container:
cd /workspace
catkin config --extend /opt/ros/noetic
catkin build --no-status --summarize
source devel/setup.bash
roslaunch ipp_planners main.launch config:=fixed-wing planner:=tigris sim:=simple
```

Build artifacts (`build/`, `devel/`, `logs/`) appear on the host through the volume mount.

**Isolated build + test** (no host build artifacts):
```bash
src/ipp_planners/scripts/build_test_benchmark.sh
```

Perfect for CI, reproducibility, or platforms without native ROS (e.g., macOS, Ubuntu 24.04).

## Notes

- The entrypoint script is inlined in the Dockerfile (no separate file needed)
- `.dockerignore` at workspace root excludes 135MB+ of build artifacts from Docker context for the test stage

