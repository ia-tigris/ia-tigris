#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import os


def kill_all_nodes(msg):
    nodes = os.popen("rosnode list").readlines()
    for i in range(len(nodes)):
        nodes[i] = nodes[i].replace("\n","")

    for node in nodes:
        if node != "kill_all_nodes":  # don't shutdown myself until I'm done
            os.system("rosnode kill "+ node)

    rospy.signal_shutdown("Killed all nodes, shutting down")


if __name__ == "__main__":
    rospy.init_node("kill_all_nodes")
    subscriber = rospy.Subscriber("/kill_all_nodes", String, kill_all_nodes)