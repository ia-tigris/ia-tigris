# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Types of changes
- `Added` for new features.
- `Changed` for changes in existing functionality.
- `Deprecated` for soon-to-be removed features.
- `Removed` for now removed features.
- `Fixed` for any bug fixes.
- `Security` in case of vulnerabilities.


## [Unreleased]

### Fixed

- Fixed sending the new planning height to the search map update service.

## [1.12] - 2025-01-17

### Added

- Added support for gimbal: local planner can accept camera pose instead of drone pose, and global planner can use wider FOV to mimic sweeping camera. Added use_gimbal boolean to PlannerParams.msg.
- Added configs for DSTA.
- Added separate function for checking if point in poly, convex or concave. Used when updating grid priors.
- Added debug statements to identify issues in camera projection and search map updates on some hardware tests.
- Added proportional search map grid size to ONR.
- Added flight height to search map setup service.
- Added an extend only flag for plan.
- Added a pursuing target id in plan msg

### Changed

- Moved no_info_on_turns to search_map.yaml.
- Adjusted viewpoint goal for ONR.
- Replaced CMU sensor model with Arete Synthetic data one.
- Commented out the new tracking variables in the plan request for now to keep backward compatibility.
- Removed the max range for sensor model 1 from ONR sensor params.

### Fixed

- Fixed the sampling radius when planning with a horizon.

## [1.11] - 2024-07-21

### Added

- Added a feature that enables skips odometry update when the drone is turning.
- Added parameter 'no_info_on_turns' to pause info gain update on turns with the above feature.
- Added math utility package in ipp planners.
- Added vtol config for field testing
- Added tree recycle feature for primitive tree and branch and bound planners
- Added multiple rollout types for MCTS search planner.

### Changed

- Changed list of subscribers when simple sim is not launched

### Fixed

- Fixed a bug causing crashes in branch and bound baseline.
- Fixed a memory issue of motion primitive tree and branch and bound planners.
- Fixed rollout value calculation for MCTS search planner.

## [1.10] - 2024-04-05

### Added

- Save the wind from the plan request to enable trochoidal path generation
- Added parameter `trochoid_min_wind` to `planner.yaml`. Wind speeds above this 
value will kick on using trochoidal paths. Otherwise Dubins paths are used.
- Added motion primitive baselines for comparisons: BNB, MCTS.
- Added filter for when node to extend to is same as current node.
- Add check in extend_from_start_node_toward_goal if the states are the same, then
return the same state to save computation.
-  Add condition in extend_from_start_node_toward_goal_node for if the trochoid 
path solver returns a path of one point.
- Checks for the feasible node on if it should be kept. Prune condition as well
 as not colocated with parent
- Before extending to neighbor, now checks if they are colocated and skip
- Check if the extended node to neighbor doesn't have an incremental cost above
 0.01. Otherwise delete
- Clear keep out zone visualization every time
- Only process new plan requests when there is a new plan request to process

### Changed
- extend_from_start_node_toward_goal no longer returns nullptr if trapped. This 
is so that the function can be used in other places.
- Planner loop rate changed from 1 to 10 Hz
- ONR config for publishing the search map is now set to publish every 20 loops, 
and loop is at 2 Hz. This is to reduce the size of the bag files.

### Fixed

- Fixed case in extending the tree, when it is "trapped", no new node is created.
- Added collisions to be in the closed set on expansion.
- Fixed edge case when nodes at the end of budget weren't being added to the closed set.
- Fixed issue on add_node_to_tree_or_prune where it could exit with tree on a nullptr
- Fix angle wrapping check in find_closest_tree_node_to_pose
- Clear hashmap when recycling search tree
- use_plan_horizon for the sampling radius not correctly uses the pitch
- Explicitly delete old nodes when the tree is recycling

## [1.9.1] - 2023-10-23

### Fixed

- Issue where plan requests received during planning might be lost and affect future plans.

### Added

- Added a scenario number to the plan request that matches the scenario number of the returned plan.


## [1.9] - 2023-10-05

### Fixed

- Fixed bug for search map where boundary of search map wasn't being updated as "seen"
- Fixed bug in replanning where new tree root node wasn't being updated with correct remaining budget
- Allowed for extend_radius to be less than extend_distance

### Added

- Added failure heartbeat message when fetching of search map belief fails
- Visualizations for waypoints and waypoint numbers
- Added a ros spin when the altitude is too high so realtime_search_map node doesn't die
- Added final_path_discretization_distance to plan requests

### Changed

- Modified "agent_odom_topic" in planner.yaml in the onr config directory to "/agent_pose"
- Removed hedge and endurance from all codebase

## [1.8.1] - 2023-07-28

### Fixed

- Fixed bug in checking cells from the bbox, the for loop wasn't inclusive of the last elements.
- Fixed bug in the hashmap key to index that shows up when search area has a width less than height

## [1.8] - 2023-07-17

### Added

- Added local_origin the PlanRequest.msg by request. For easier association of origin for each plan request.
- Implementation of GreedySearch planner

### Changed

- Changed the way we interface with the trochoid solver. There is no longer any OMPL objects in the trochoid solver
which speeds up computation time. The trochoid solver now returns a vector of points which get converted into pathGeometric objects.
- Modified the heartbeat message for the planner to be a enum type and to be more descriptive of the state of the planner. 
Enum is defined in PlannerStatus.msg. Also publishes less often to reduce message rate and rate limited to 10 hz in the planner loop.

### Fixed

- Bug in realtime_search_map where it wasn't taking into account the roll and pitch when updating the map

## [1.7.1] - 2023-06-06

### Added

- Added script for recording CMU parameters for every plan request
- Added flight_height and max_kappa parameters to plan request
- Save directory argument and save parameters argument to record bag launch

### Changed

- Reduced visualization scale for search map frustum visualization

## [1.7] - 2023-05-24

### Added

- Added uav1 namespace to all nodes
- tf2_eigen was added as a dependency for the ipp_belief package

### Changed

- Renamed the packages from onr_ipp->ipp_planners, ipp_belief->ipp_beliefs.
- Changed message definition for FilteredTargets

## [1.6] - 2023-04-04

### Added

- Added public and private nodehandles to enable launching of multiple namespaced ipp_planners nodes. An example namespaced launch file can be found in multi_ipp.launch within the launch/ directory
- Visualization markers for generated paths are scaled on a scale of 0 - 1 based off of the visualization_scale parameter within planner.yaml
- InfoMapSearch now uses hashmap search map embeddings within each node for faster evaluations and tree recycling. Can turn off with the `use_hashmap_for_search` parameter in `planner.yaml`
- Added `search_belief_publishing_every_n_loops` parameter to `search_map.yaml` to limit the rate of publishing the search map state. Useful to limit the bag sizes on long runs or when there is limited bandwidth.

## [1.5] - 2022-12-13

### Added

- Added Google Benchmark. If CATKIN_ENABLE_TESTING is true, then install Google Benchmark with `sudo apt install libbenchmark-dev`
- Two heartbeat messages. One for the planner and one for planner node. Topic names set in `planner.yaml`. The planner node heartbeat publishes every ros spin. The planner heartbeat publishes every planner loop as well as within each extend in Tigris.
- Added analytical solutions for Right-Straight-Right and Left-Straight-Left trajectories in trochoids package
- InfoMapSearch checks if strict_stay_in_bounds is true and and starting point is within bounds. If not, strict_stay_in_bounds is set to false so a plan is still able to be returned. 
- strict_stay_in_bounds is now part of the planner_params that can be sent in the PlanRequest.msg. So if the header of planner_params is set, then the values will be updated from the plan request.
- Keep out regions added to the plan request and are checked within InfoMapSearch

### Changed

- `record_bag.launch` no longer records the visualization messages for belief spaces. They were taking up too much space in rosbag.
- Restructuring of the launch files to a main.launch


## [1.4.2] - 2022-10-05

### Fixed

- Bug when a search and track both on but plan request didn't have any target priors. 
- Make save_plan_request_params() return bool so JHU no longer has runtime errors


## [1.4] - 2022-09-29

### Added

- You can now pass in more parameters in a plan request. The planner parameters weighting the rewards and sampling of track vs search can be set in the `PlannerParams.msg` that is in the `PlanRequest.msg`. In order for the values to be read in, the header stamp needs to be set. This way these parameters are optional and are ignored if the stamp is the default of 0.
- Added some safety checks for parameters to give feedback to users.

### Changed

- The returned path is now a discretized path of waypoints rather than just the sparse set of waypoints. You can set the distance between each waypoint in the `planner.yaml` under `final_path_discretization_distance`.
- The search bounds are no longer used as a method of setting prior information. The search bounds will be used purely as the bounds of the plan. So the initial confidence is no longer used. The entire area in the bounds will just we set to an extremely low value and low priority. All search prior needs to be passed in through the plan requests. This way the confidence, priority, and sensor model are explicitly set by the user. This way the user can reason a bit better when deciding the priority of searching empty space vs higher priority regions.
- `tigris.yaml` was split to have a `planner.yaml` for generic planner parameters.
- `PlanningRequest.msg` is now `PlanRequest.msg`.

### Fixed

- The every-other-plan-being-empty-when-keep-in-bounds-is-on bug is now fixed.
- Counter detection radius for tracked targets is working again. It is a simple version for now where it stays away from where the targets are at the start of the plan.
- Fixed some bugs in trochoid paths where it would create unnecessary loops occasionally.


[Unreleased]: https://github.com/castacks/ipp_planners/compare/v1.12...develop
[1.12]: https://github.com/castacks/ipp_planners/compare/v1.11...v1.12
[1.11]: https://github.com/castacks/ipp_planners/compare/v1.10...v1.11
[1.10]: https://github.com/castacks/ipp_planners/compare/v1.9.1...v1.10
[1.9.1]: https://github.com/castacks/ipp_planners/compare/v1.9...v1.9.1
[1.9]: https://github.com/castacks/ipp_planners/compare/v1.8.1...v1.9
[1.8.1]: https://github.com/castacks/ipp_planners/compare/v1.8...v1.8.1
[1.8]: https://github.com/castacks/ipp_planners/compare/v1.7.1...v1.8
[1.7.1]: https://github.com/castacks/ipp_planners/compare/v1.7...v1.7.1
[1.7]: https://github.com/castacks/ipp_planners/compare/v1.6...v1.7
[1.6]: https://github.com/castacks/ipp_planners/compare/v1.5...v1.6
[1.5]: https://github.com/castacks/ipp_planners/compare/v1.4.2...v1.5
[1.4.2]: https://github.com/castacks/ipp_planners/compare/v1.4...v1.4.2
[1.4]: https://github.com/castacks/ipp_planners/compare/04b7aa0b76207e929a37da7fa66f10e43b2a72c7...v1.4
