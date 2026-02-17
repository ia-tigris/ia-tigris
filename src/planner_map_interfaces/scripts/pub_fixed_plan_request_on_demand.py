#!/usr/bin/python

import rospy
import yaml

import argparse
from planner_map_interfaces.msg import PlanRequest, TargetPrior
from geometry_msgs.msg import Point32
from std_msgs.msg import Empty

yaml_file = "/home/wildfire/m600_ws/src/ipp/planner_map_interfaces/config/journal_m600/plan_requests/ipp_tro.yaml"

parser = argparse.ArgumentParser(description='Read yaml file')
parser.add_argument('file', metavar='f', type=str, help='the file to publish from')
parser.add_argument("overrides", type=str, default="", nargs="?", help="pass in a dict as a string to override values in the yaml file")

def callback(empty_msg):
    global publisher
    global pr_yaml

    plan_request = PlanRequest()
    load_sub_dicts_as_obj_properties(pr_yaml, plan_request)

    rospy.sleep(1)
    # print("Publish plan request:\n{}".format(plan_request))
    publisher.publish(plan_request)

    exit(0)

def load_sub_dicts_as_obj_properties(sub_dicts, obj):
    for key, value in sub_dicts.items():
        # print("reading key: {}".format(key))
        if isinstance(value, dict):
            load_sub_dicts_as_obj_properties(value, getattr(obj, key))
        elif isinstance(value, list) and key == "points":
            for i, item in enumerate(value):
                p = Point32()
                p.x = item["x"]
                p.y = item["y"]
                p.z = item["z"]
                getattr(obj, key).append(p)
        elif isinstance(value, list) and key == "target_priors":
            for i, item in enumerate(value):
                getattr(obj, key).append(TargetPrior())
                load_sub_dicts_as_obj_properties(item, getattr(obj, key)[i])
        else:
            setattr(obj, key, value)

def main():
    global publisher
    global pr_yaml

    rospy.init_node('pub_fixed_plan_request_on_demand', anonymous=True)
    publisher = rospy.Publisher('planner/plan_request', PlanRequest, queue_size=10, latch=True)
    rospy.Subscriber("generate_ipp_plan_request", Empty, callback)

    with open(yaml_file, "r") as f:
        pr_yaml = yaml.safe_load(f)
        
    rospy.spin()

if __name__ == '__main__':
    main()
    