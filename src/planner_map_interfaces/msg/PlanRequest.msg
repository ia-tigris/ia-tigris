# Request time and serial number
Header header 

# Initial condition information:
geometry_msgs/Pose start_pose #Plan from position and orientation
geometry_msgs/Vector3 current_speed #Start speed in m/s, x-east, y-north, z-up
geometry_msgs/Vector3 wind_speed #Wind speed in m/s
float32 max_planning_time #seconds
float32 maximum_range #budget

#Constraints and objectives:
float32 desired_speed #Desired flight speed in m/s
# Bounds of the area that might contain the target. Set to low priority and low confidence.
# Also can be used as hard limit for flight boundary.
geometry_msgs/Polygon search_bounds 
float32 counter_detection_range #meters
float32 min_loiter_radius #meters
SensorConstraint sensor_constraints

#Prior information:
TargetPrior[] target_priors 
KeepOutZone[] keep_out_zones

# Replan flag
bool clear_tree
# bool reset_trackers

# # Debug vis
# bool msg_debug_vis

# Set the header time for parameters that have been updated
# If header not set, parameters will be ignored 
PlannerParams planner_params
# RimctsParams rimcts_params
# GreedyParams greedy_params
# MctsParams mcts_params
# Search_MapParams search_map_params
# TigrisParams tigris_params

# Scenario
uint32 scenario # To match up with the correct plan
geographic_msgs/GeoPoint local_origin #Origin of local coordinate system
