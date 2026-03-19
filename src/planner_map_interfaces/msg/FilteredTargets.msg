# this message exists because it represents ONE complete observation by the drone
# in this way we won't overly update our particle fiilters with duplicate negative observations
# Really this message should be renamed "Observation" and include the sensor parameters and camera bounds
Header header

FilteredTarget[] targets