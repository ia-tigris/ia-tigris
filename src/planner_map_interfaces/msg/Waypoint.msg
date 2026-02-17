Header stamp 
uint8 WAYPOINT_FLYBY = 1
uint8 WAYPOINT_FLYOVER = 2
uint8 waypoint_type
geometry_msgs/Pose position
float32 command_speed # Commanded airspeed in m/s
geometry_msgs/Vector3 direction # Viewing direction of sensor (artificial or physical) with respect to true north
float32 field_of_view_horizontal # Zoom level (artificial or physical)
float32 field_of_view_vertical # Zoom level (artificial or physical)