Header header

uint16 GLOBAL_ID_FOR_UNKNOWN=0
uint16 global_id  # set to 0 (don't set) if unknown, then belief will estimate based on global reassociation
uint16 local_id  # id of the target in a local region. the perception kalman filter uses this to associate targets in image frame
uint16 class_id  # numeric class_id of the object
string class_label # corresponding string label of the object class e.g. car, person, ship

# State
float64 x
float64 y
float64 xdot
float64 ydot
# Covariance
float64[16] covariance # x, y, xdot, ydot

float64 priority # Higher values mean higher priority. If empty then the priority is set to 1.0
uint8 sensor_model_id # the type of sensor model for that region type
