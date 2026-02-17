Header header

uint8 POLYGON_PRIOR = 1 # must be convex polygon
uint8 LINE_SEG_PRIOR = 2
uint8 POINT_PRIOR = 3
uint8 grid_prior_type

geometry_msgs/Polygon bounds # must be convex polygon

float32 confidence # initial confidence value for the polygon, line seg, or point
# - 0.5 means unexplored, we have no idea if there are unaccounted objects
# - 0.0 means total confidence that there is nothing there
# - 1.0 means total confidence that all accounted for there

float64 priority # Higher values mean higher priority. If empty then the priority is set to 1.0
uint8 sensor_model_id # the type of sensor model for that region type