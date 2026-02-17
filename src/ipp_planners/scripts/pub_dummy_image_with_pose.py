#!/usr/bin/env python2

'''
Publishes the image with drone pose. The image is dummy and just a circle or heart
'''
import argparse
import tf
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from planner_map_interfaces.msg import ImageWithPose
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion, quaternion_from_euler

height = 240
width = 320

camera_odometry = None
camera_pose_stamped = None
agent_pose_stamped = None

def update_camera_odometry(msg):
    global camera_odometry
    global camera_pose_stamped
    camera_pose_stamped = PoseStamped()
    camera_pose_stamped.header = msg.header
    camera_pose_stamped.pose = msg.pose.pose
    camera_odometry = msg

def update_agent_pose(msg):
    global agent_pose_stamped
    agent_pose_stamped = msg

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", help="Mode of the image. Can be 'circle' or 'heart'", choices=["circle", "heart"])


    bridge = CvBridge()

    rospy.init_node("pub_dummy_image_with_pose")
    sub = rospy.Subscriber("/simulator/camera_pose", Odometry, update_camera_odometry)
    agent_pose_sub = rospy.Subscriber("/simulator/agent_pose", PoseStamped, update_agent_pose)
    image_with_pose_pub = rospy.Publisher("/simulator/image_to_project", ImageWithPose, queue_size=1)
    camera_pose_pub = rospy.Publisher("/debug/camera_pose", PoseStamped, queue_size=1)
    listener = tf.TransformListener("thermal_cam_listener")
    rate = rospy.Rate(10)
    for i in range(10):
        rate.sleep()
    # listener.waitForTransform("/local_enu", "/camera_coords", rospy.Time().now(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        if camera_pose_stamped is not None:
            msg = ImageWithPose()
            # trans, rot = listener.lookupTransform("/local_enu", "/camera_coords", rospy.Time(0))
            # transformed_pose_stamped = listener.transformPose("/camera_coords", camera_pose_stamped)
            # thermal_cam_pose_stamped = listener.transformPose("/thermal/camera_link", agent_pose_stamped)
            (trans,rot) = listener.lookupTransform('/local_enu', '/simulator/thermal/camera_link', rospy.Time(0))
            msg.pose = Pose()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = trans
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = rot
            # msg.pose = camera_pose_stamped.pose
            # euler_coords = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            # euler_coords = (euler_coords[0], -euler_coords[1], euler_coords[2])
            # msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quaternion_from_euler(euler_coords[0], euler_coords[1], euler_coords[2])
            # print("euler_rot is: ", euler_coords)
            # print("Frame of transformed pose stamped:", transformed_pose_stamped.header.frame_id)
            # msg.pose = transformed_pose_stamped.pose
            # camera_pose_pub.publish(transformed_pose_stamped)
            # msg.pose = thermal_cam_pose_stamped.pose

            # image = np.zeros((height, width, 1), dtype=np.uint8)
            image = np.ones((height, width, 1), dtype=np.uint8) * 255
            cv2.circle(image, (height, (int) (width/2)), 100, (0,), -1)
            # image[height/2-50:height/2+50, width/2-50:width/2+50] = 0
            image_message = bridge.cv2_to_imgmsg(image, encoding="passthrough")
            msg.image = image_message
            msg.image.header.stamp = rospy.Time.now()
            msg.image.header.frame_id = "local_enu"
            image_with_pose_pub.publish(msg)
        rate.sleep()
