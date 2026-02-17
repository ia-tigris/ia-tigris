import utm
import argparse
import math
import yaml
import pprint
from bs4 import BeautifulSoup

# pip install utm lxml


def parse_folder(folder):
    names = folder.find_all("name")

    origin_lat, origin_lon = find_placemark_lat_lon(names, "origin")

    start_lat, start_lon = find_placemark_lat_lon(names[1:], "start_pose")

    origin_x, origin_y, _, _ = utm.from_latlon(origin_lat, origin_lon)
    start_x, start_y, _, _ = utm.from_latlon(start_lat, start_lon)
    start_x_relative_to_origin = float(start_x - origin_x)
    start_y_relative_to_origin = float(start_y - origin_y)
    
    target_priors_dict = {"target_priors": []}
    
    for name in names[1:]:
        if name.next_element == "search_bounds":
            print("Found search bounds")
            placemark = name.parent
            points_dict = extract_cartesian_points(placemark, origin_lat, origin_lon)
            search_bounds_dict = {"search_bounds": points_dict}
            # pprint.pprint(search_bounds_dict)
        elif "grid_prior_" in name.next_element:
            confidence = float(name.next_element[22:]) # extract 'grid_prior_confidence='
            placemark = name.parent
            points_dict = extract_cartesian_points(placemark, origin_lat, origin_lon)
            grid_prior_dict = {
                "grid_prior":{
                    "header": {"frame_id": "local_enu"},
                    "grid_prior_type": 1,
                    "bounds": points_dict,
                    "confidence": confidence,
                    "priority": 1.0,
                    "sensor_model_id": 0
                }
            }
            target_priors_dict["target_priors"].append(grid_prior_dict)
        elif name.next_element == "grid_prior":
            confidence = 0.49
            placemark = name.parent
            lon_lat_alt_list = str(placemark.find("coordinates").next_element).strip().split(" ")
            points_dict = {"points": []}
            lon, lat, alt = lon_lat_alt_list[0].split(",")
            lon, lat, alt = float(lon), float(lat), float(alt)
            
            x, y = approx_cartesian_offset_meters(start_lat, start_lon, lat, lon)
            points_dict["points"].append({"x": float(x), "y":float(y), "z": 0})
            grid_prior_dict = {
                "grid_prior":{
                    "header": {"frame_id": "local_enu"},
                    "grid_prior_type": 3,
                    "bounds": points_dict,
                    "confidence": confidence,
                    "priority": 1.0,
                    "sensor_model_id": 0
                }
            }
            target_priors_dict["target_priors"].append(grid_prior_dict)
    ret = {
        "start_pose": {
            "position": {
                "x": start_x_relative_to_origin, "y": start_y_relative_to_origin, "z": args.flight_height
            },
            "orientation": {"x": 0, "y": 0, "z": 1.0, "w": 0}
        }
    }
    ret.update(search_bounds_dict)
    ret.update(target_priors_dict)
    return ret

def approx_cartesian_offset_meters(lat1, lon1, lat2, lon2):
    """
    lat1: origin latitude
    lon1: origin longitude
    """
    # https://stackoverflow.com/a/14176034
    east1, north1, _, _ = lat1_utm = utm.from_latlon(lat1, lon1)
    east2, north2, _, _ = lat1_utm = utm.from_latlon(lat2, lon2)

    # already in meters
    x = east2 - east1
    y = north2 - north1

    # # https://stackoverflow.com/a/19356480
    # latMid = lat1 * math.pi /180
    
    # m_per_deg_lat = 111132.954 - 559.822 * math.cos( 2 * latMid ) + 1.175 * math.cos( 4 * latMid);
    # m_per_deg_lon = 111132.954 * math.cos ( latMid );
    
    # # https://gis.stackexchange.com/a/260673
    
    # x = (lon2 - lon1) * m_per_deg_lon
    # y = (lat2 - lat1) * m_per_deg_lat
    
    return float(x), float(y)

def extract_cartesian_points(placemark, start_lat, start_lon):
    lon_lat_alt_list = str(placemark.find("coordinates").next_element).strip().split(" ")
    ret = {"points": []}
    for lon_lat_alt in lon_lat_alt_list[:-1]:  # leave off the last one because it's a repeat of the first
        lon, lat, alt = lon_lat_alt.split(",")
        lon, lat, alt = float(lon), float(lat), float(alt)
        
        x, y = approx_cartesian_offset_meters(start_lat, start_lon, lat, lon)
        ret["points"].append({"x": float(x), "y":float(y), "z": 0})
    return ret

def find_placemark_lat_lon(names, search_name):
    latitude = None
    longitude = None
    
    for name in names:
        if name.next_element == search_name:
            placemark = name.parent
            longitude, latitude, altitude = str(placemark.find("coordinates").next_element).strip().split(",")
            longitude, latitude, altitude = float(longitude), float(latitude), float(altitude)
            
#             altitutde = float(placemark.find("altitute").next_element)
            print("Found lat lon for %s at %f,%f" % (search_name, latitude, longitude))
            break
    else:
        raise ValueError("Could not find %s lat lon" % search_name)
    
    return latitude, longitude

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("kml_file_path", type=str)
    parser.add_argument("--clear_tree", type=bool, default=True)
    parser.add_argument("--counter_detection_range", type=float, default=1)
    parser.add_argument("--desired_speed", type=float, default=5)
    parser.add_argument("--flight_height", type=float, default=50)
    parser.add_argument("--max_planning_time", type=float, default=10)
    parser.add_argument("--maximum_range", type=float, default=10000000000)
    args = parser.parse_args()

    with open(args.kml_file_path, "r") as f:
        kml_file = f.read()
    
    soup = BeautifulSoup(kml_file, "xml")
    folders = soup.find_all("Folder")

    print("Found {} mission folders".format(len(folders)))

    for folder in folders:
        mission_name = str(folder.find("name").next_element)
        print("\n===============================")
        print("Writing mission " + mission_name + " to yaml file")
        plan_request_dict = parse_folder(folder)
        plan_request_dict["clear_tree"] = args.clear_tree
        plan_request_dict["counter_detection_range"] = args.counter_detection_range
        plan_request_dict["desired_speed"] = args.desired_speed
        # flight height set above with start_pose.position.z
        plan_request_dict["max_planning_time"] = args.max_planning_time
        plan_request_dict["maximum_range"] = args.maximum_range


        pprint.pprint(plan_request_dict)
        with open(mission_name + ".yaml", 'w') as outfile:
            yaml.dump(plan_request_dict, outfile, default_flow_style=False)
        print("===============================\n")

