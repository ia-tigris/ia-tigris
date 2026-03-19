#!/usr/bin/python

import rospy
import yaml

import argparse
from planner_map_interfaces.msg import PlanRequest, TargetPrior, KeepOutZone, MultiPlanRequest
from geometry_msgs.msg import Point32, Pose

parser = argparse.ArgumentParser(description='Read yaml file')
parser.add_argument('file', metavar='f', type=str, help='the file to publish from')
parser.add_argument('--centralized', action='store_true', help='if we want to use centralized plan request', default=False)
parser.add_argument("overrides", type=str, default="", nargs="?", help="pass in a dict as a string to override values in the yaml file")
parser.add_argument('robot_name', type=str, default="uav1", nargs='?', const='', help="name of agent we want to publish to")

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
        elif isinstance(value, list) and key == "keep_out_zones":
            for i, item in enumerate(value):
                getattr(obj, key).append(KeepOutZone())
                load_sub_dicts_as_obj_properties(item, getattr(obj, key)[i])
        #multi-agent (multiple start poses)
        elif isinstance(value, list) and key == "start_poses":
            for i, item in enumerate(value):
                getattr(obj, key).append(Pose())
                load_sub_dicts_as_obj_properties(item, getattr(obj, key)[i])
        else:
            setattr(obj, key, value)

def main():
    rospy.init_node('pub_plan_request_from_yaml', anonymous=True)
    args = parser.parse_args()

    robot_name = ""
    if len(args.robot_name) > 0:
        robot_name = "/" + args.robot_name

    if args.centralized:
        publisher = rospy.Publisher(robot_name + '/planner/plan_request', MultiPlanRequest, queue_size=10, latch=True)
        plan_request_msg = MultiPlanRequest()
    else:
        publisher = rospy.Publisher(robot_name + '/planner/plan_request', PlanRequest, queue_size=10, latch=True)
        plan_request_msg = PlanRequest()

    with open(args.file, "r") as f:
        pr_yaml = yaml.safe_load(f)
    
    if args.overrides:
        overrides = eval(args.overrides)
        pr_yaml.update(overrides)

    load_sub_dicts_as_obj_properties(pr_yaml, plan_request_msg)

    rospy.sleep(1)
    print("Published plan request:\n{}".format(plan_request_msg))
    publisher.publish(plan_request_msg)

if __name__ == '__main__':
    main()