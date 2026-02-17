#!/usr/bin/env python

import os
import rospy
import random
import numpy as np
from rospkg import RosPack
import tf2_ros
# import tf2_geometry_msgs
from planner_map_interfaces.msg import (
    Plan,
    PlanRequest,
    GroundTruthTargets,
    GroundTruthTarget,
    MultiPlanRequest
)
from ipp_simple_sim.environment import *
from geometry_msgs.msg import (
    PoseStamped,
    Point,
    Pose,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from std_msgs.msg import ColorRGBA, Header
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8, UInt32, Float32
from ipp_simple_sim.msg import Detections
from ipp_planners.srv import SearchMapUpdate
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from visualization_msgs.msg import Marker, MarkerArray

package = RosPack()
package_path = package.get_path("ipp_simple_sim")

visualization_scale = 1.0

# /ipp_planners_node/visualization_scale

# https://sashamaps.net/docs/resources/20-colors/
COLORS = [
    [230, 25, 75],
    [60, 180, 75],
    [255, 225, 25],
    [0, 130, 200],
    [245, 130, 48],
    [145, 30, 180],
    [70, 240, 240],
    [240, 50, 230],
    [210, 245, 60],
    [250, 190, 212],
    [0, 128, 128],
    [220, 190, 255],
    [170, 110, 40],
    [255, 250, 200],
    [128, 0, 0],
    [170, 255, 195],
    [128, 128, 0],
    [255, 215, 180],
    [0, 0, 128],
    [128, 128, 128],
    [255, 255, 255],
    [0, 0, 0],
]


def get_color(index):
    r, g, b = COLORS[index % len(COLORS)]
    ros_color = ColorRGBA()
    ros_color.r = r / 255.0
    ros_color.g = g / 255.0
    ros_color.b = b / 255.0
    ros_color.a = 1.0
    return ros_color


class SimManager:
    def __init__(self, robot_names):
        self.robot_names = robot_names
        self.num_agents = len(robot_names)
        self.planner_path_topic = rospy.get_param("~planner_path")
        self.sim_env = self.sim_manager_node()
        self.agent_traj_list = [[] for i in range(self.num_agents)]
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pause_while_planning = rospy.get_param(
            "sim_manager_node/pause_while_planning"
        )
        self.waiting_for_plan = False
        self.centralized = rospy.get_param("~centralized")

        global visualization_scale
        visualization_scale = rospy.get_param("~visualization_scale")

        rospy.loginfo("simulating " + str(self.num_agents) + " agent(s)")

    def sim_manager_node(self):
        # ships
        targets_list = rospy.get_param("~targets", [])

        # drone state
        init_x = rospy.get_param("~init_x")
        init_y = rospy.get_param("~init_y")
        init_z = rospy.get_param("~init_z")
        init_yaw = rospy.get_param("~init_yaw")

        max_omega = rospy.get_param("~max_omega")
        max_zvel = rospy.get_param("~max_zvel")

        agent_l = rospy.get_param("~agent_l")

        hvel = rospy.get_param("~hvel")
        vvel = rospy.get_param("~vvel")

        n_rand_targets = rospy.get_param("~n_rand_targets")

        del_t = rospy.get_param("~del_t")

        K_p = rospy.get_param("~K_p")
        K_p_z = rospy.get_param("~K_p_z")

        waypoint_threshold = rospy.get_param("~waypoint_threshold")
        # assume we're always simulating atleast one agent. Default is ""
        robot_name = self.robot_names[0]
        sensor_focal_length = rospy.get_param(robot_name + "/sensor/focal_length")
        sensor_width = rospy.get_param(robot_name + "/sensor/width")
        sensor_height = rospy.get_param(robot_name + "/sensor/height")
        sensor_pitch = rospy.get_param(robot_name + "/sensor/pitch")
        sensor_max_range = max(rospy.get_param(robot_name + "/sensor/max_range"))

        return Environment(
            targets_list,
            max_omega,
            max_zvel,
            init_x,
            init_y,
            init_z,
            init_yaw,
            K_p,
            K_p_z,
            self.num_agents,
            agent_l,
            hvel,
            vvel,
            # the below is UNIFORM sampling of targets. for weighted sampling from search map, see below
            0
            if rospy.get_param("~sample_additional_true_targets_from_search_prior")
            else n_rand_targets,
            del_t,
            waypoint_threshold,
            sensor_focal_length,
            sensor_width,
            sensor_height,
            sensor_pitch,
            sensor_max_range,
        )

    def get_agent_odometry(self, time, frame):
        agent_odom_list = []
        for id_num in range(self.num_agents):
            agent_odom = Odometry()
            agent_odom.header.frame_id = frame
            agent_odom.header.stamp = time
            agent_odom.pose.pose.position.x = self.sim_env.agent[id_num].x
            agent_odom.pose.pose.position.y = self.sim_env.agent[id_num].y
            agent_odom.pose.pose.position.z = self.sim_env.agent[id_num].z
            quat = quaternion_from_euler(0, 0, self.sim_env.agent[id_num].yaw)
            agent_odom.pose.pose.orientation.x = quat[0]
            agent_odom.pose.pose.orientation.y = quat[1]
            agent_odom.pose.pose.orientation.z = quat[2]
            agent_odom.pose.pose.orientation.w = quat[3]
            agent_odom_list.append(agent_odom)
        return agent_odom_list

    def get_target_positions(self, time, frame):
        target_poses = GroundTruthTargets()
        target_poses.header.frame_id = frame
        target_poses.header.stamp = time

        for target in self.sim_env.targets:
            target_pose = GroundTruthTarget()

            target_pose.id = target.id
            target_pose.x = target.x
            target_pose.y = target.y
            target_pose.heading = target.heading
            target_pose.linear_speed = target.linear_speed
            target_pose.angular_speed = target.angular_speed
            target_pose.is_detected = target.is_detected

            # print target_pose
            target_poses.targets.append(target_pose)

        return target_poses

    def get_camera_pose(self, time, frame):
        camera_pose_list = []
        for id_num in range(self.num_agents):
            camera_pose = Odometry()
            camera_pose.header.frame_id = frame
            camera_pose.header.stamp = time
            camera_pose.pose.pose.position.x = self.sim_env.agent[id_num].x
            camera_pose.pose.pose.position.y = self.sim_env.agent[id_num].y
            camera_pose.pose.pose.position.z = self.sim_env.agent[id_num].z
            quat = quaternion_from_euler(
                0, self.sim_env.sensor_pitch, self.sim_env.agent[id_num].yaw
            )
            camera_pose.pose.pose.orientation.x = quat[0]
            camera_pose.pose.pose.orientation.y = quat[1]
            camera_pose.pose.pose.orientation.z = quat[2]
            camera_pose.pose.pose.orientation.w = quat[3]
            cov_matrix = np.zeros((6, 6))
            cov_matrix[0, 0] = self.sim_env.get_agent_uncertainty(id_num)[0]
            cov_matrix[1, 1] = self.sim_env.get_agent_uncertainty(id_num)[1]
            cov_matrix[2, 2] = self.sim_env.get_agent_uncertainty(id_num)[2]
            cov_matrix[3, 3] = self.sim_env.get_agent_uncertainty(id_num)[3]
            cov_matrix[4, 4] = self.sim_env.get_agent_uncertainty(id_num)[4]
            cov_matrix[5, 5] = self.sim_env.get_agent_uncertainty(id_num)[5]
            camera_pose.pose.covariance = cov_matrix.flatten().tolist()
            camera_pose_list.append(camera_pose)
        return camera_pose_list

    def get_waypoint_num(self):
        waypoint_num_list = []
        for id_num in range(self.num_agents):
            waypoint_number = UInt32()
            waypoint_number.data = self.sim_env.curr_waypoint_num[id_num]
            waypoint_num_list.append(waypoint_number)
        return waypoint_num_list

    def get_remaining_budget(self):
        remaining_budget_list = []
        for id_num in range(self.num_agents):
            remaining_budget = Float32()
            remaining_budget.data = self.sim_env.remaining_budget[id_num]
            remaining_budget_list.append(remaining_budget)
        return remaining_budget_list

    def get_target_detections(self, time, frame):
        detections_msg_list = []
        detected_targets, camera_projection = self.sim_env.get_sensor_measurements()
        for i in range(self.num_agents):
            detection_msg = Detections()
            detection_msg.header.frame_id = frame
            detection_msg.header.stamp = time
            for target in detected_targets[i]:
                detection_msg.headings.append(
                    self.sim_env.get_target_heading_noise(target.heading)
                )
                target_camera_unit_vector = Point()
                range_to_target = np.linalg.norm(
                    np.array([target.x, target.y, 0])
                    - np.array(
                        [
                            self.sim_env.agent[i].x,
                            self.sim_env.agent[i].y,
                            self.sim_env.agent[i].z,
                        ]
                    )
                )
                i_hat = (target.x - self.sim_env.agent[i].x) / range_to_target
                j_hat = (target.y - self.sim_env.agent[i].y) / range_to_target
                k_hat = -self.sim_env.agent[i].z / range_to_target
                R = np.matmul(
                    self.sim_env.sensor.Rz(self.sim_env.agent[i].yaw),
                    self.sim_env.sensor.Ry(self.sim_env.sensor_pitch),
                )
                R_inv = np.linalg.inv(R)
                camera_frame_pose = np.matmul(R_inv, [i_hat, j_hat, k_hat])
                target_camera_unit_vector.x = camera_frame_pose[0]
                target_camera_unit_vector.y = camera_frame_pose[1]
                target_camera_unit_vector.z = camera_frame_pose[2]
                detection_msg.target_camera_vectors.append(target_camera_unit_vector)
                detection_msg.target_ids.append(target.id)
            detections_msg_list.append(detection_msg)

        return detections_msg_list, camera_projection

    def planner_callback(self, msg, id_num):
        self.sim_env.update_waypoints(msg, id_num)
        self.waiting_for_plan = False

    def get_ocean_marker(self, time, frame):
        ocean_marker = Marker()
        ocean_marker.header.frame_id = frame
        ocean_marker.ns = "ocean"
        ocean_marker.header.stamp = time
        ocean_marker.id = 0
        ocean_marker.type = Marker.CUBE
        ocean_marker.action = Marker.ADD
        ocean_marker.lifetime = rospy.Duration()
        ocean_marker.color.r = 0
        ocean_marker.color.b = 1.0
        ocean_marker.color.g = 0.8
        ocean_marker.color.a = 1
        ocean_marker.scale.x = 10000
        ocean_marker.scale.y = 10000
        ocean_marker.scale.z = 1
        ocean_marker.pose.position.z = -1
        return ocean_marker

    def get_agent_marker(self, time, frame, agent_odom):
        agent_marker_list = MarkerArray()
        for id_num in range(self.num_agents):
            odom = agent_odom[id_num]
            agent_marker = Marker()
            agent_marker.header.frame_id = frame
            agent_marker.header.stamp = time
            agent_marker.ns = "agent_mesh"
            agent_marker.id = id_num
            agent_marker.type = Marker.MESH_RESOURCE
            agent_marker.action = Marker.ADD
            agent_marker.mesh_use_embedded_materials = False
            agent_marker.mesh_resource = (
                "package://ipp_simple_sim/meshes/vtol_to_scale.dae"
            )
            agent_marker.lifetime = rospy.Duration()
            agent_marker.pose.position = odom.pose.pose.position
            agent_marker.pose.orientation = odom.pose.pose.orientation
            agent_marker.color.r = 0.8
            agent_marker.color.g = 0.95
            agent_marker.color.b = 1.0
            agent_marker.color.a = 0.99
            agent_marker.scale.x = 1.0 * visualization_scale
            agent_marker.scale.y = 1.0 * visualization_scale
            agent_marker.scale.z = 1.0 * visualization_scale
            agent_marker_list.markers.append(agent_marker)
        return agent_marker_list

    def get_agent_trajectory_marker(self, time, frame, agent_odom):
        trajectory_list = MarkerArray()
        for id_num in range(self.num_agents):
            odom = agent_odom[id_num]
            agent_traj = self.agent_traj_list[id_num]

            trajectory_marker = Marker()
            trajectory_marker.header.frame_id = frame
            trajectory_marker.header.stamp = time
            trajectory_marker.ns = "agent_trajectory"
            trajectory_marker.id = 0
            trajectory_marker.type = Marker.LINE_STRIP
            trajectory_marker.action = Marker.ADD
            trajectory_marker.lifetime = rospy.Duration(10.0)

            agent_traj.append(
                [
                    odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    odom.pose.pose.position.z,
                ]
            )
            if len(agent_traj) > 500:  # setting traj length to 100
                agent_traj.pop(0)

            trajectory_marker.pose.position.x = 0
            trajectory_marker.pose.position.y = 0
            trajectory_marker.pose.position.z = 0

            trajectory_marker.pose.orientation.w = 1.0

            for i in range(1, len(agent_traj)):
                trajectory_marker.points.append(
                    Point(agent_traj[i][0], agent_traj[i][1], agent_traj[i][2])
                )

            trajectory_marker.color.r = 1
            trajectory_marker.color.g = 69 / 255
            trajectory_marker.color.b = 0
            trajectory_marker.color.a = 0.3
            trajectory_marker.scale.x = 1 * visualization_scale
            trajectory_marker.scale.y = 1 * visualization_scale
            trajectory_marker.scale.z = 1 * visualization_scale

            trajectory_list.markers.append(trajectory_marker)

        return trajectory_list

    """
    Four points of the sensor footprint polygon. 
    """

    def get_projection_points_marker(self, time, frame, agent_odom, camera_projection):
        projection_points_list = MarkerArray()
        for id_num in range(self.num_agents):
            odom = agent_odom[id_num]
            projection = camera_projection[id_num]

            projection_points_marker = Marker()
            projection_points_marker.header.frame_id = frame
            projection_points_marker.header.stamp = time
            projection_points_marker.ns = "projection_marker"
            projection_points_marker.id = id_num
            projection_points_marker.type = Marker.POINTS
            projection_points_marker.action = Marker.ADD
            projection_points_marker.color.r = 1.0
            projection_points_marker.color.g = 1.0
            projection_points_marker.color.b = 0.0
            projection_points_marker.color.a = 1.0
            projection_points_marker.scale.x = 0.1 * visualization_scale
            projection_points_marker.scale.y = 0.1 * visualization_scale
            projection_points_marker.scale.z = 0.1 * visualization_scale

            points = []
            for np_point in projection:
                ros_point = Point()
                ros_point.x = np_point[0]
                ros_point.y = np_point[1]
                ros_point.z = np_point[2]
                points.append(ros_point)
            projection_points_marker.points = points

            projection_points_list.markers.append(projection_points_marker)

        return projection_points_list

    def get_projection_marker(self, time, frame, agent_odom, camera_projection):
        marker_list = MarkerArray()
        for id_num in range(self.num_agents):
            odom = agent_odom[id_num]
            projection = camera_projection[id_num]

            projection_marker = Marker()
            projection_marker.header.frame_id = frame
            projection_marker.header.stamp = time
            projection_marker.ns = "projection_marker"
            projection_marker.id = id_num
            projection_marker.type = Marker.LINE_LIST
            projection_marker.action = Marker.ADD
            projection_marker.color.r = 1
            projection_marker.color.g = 69 / 255
            projection_marker.color.b = 0
            projection_marker.color.a = 1.0
            projection_marker.scale.x = 0.3 * visualization_scale  # in meters
            projection_marker.scale.y = 0.3 * visualization_scale
            projection_marker.scale.z = 0. * visualization_scale

            points = []
            agent_point = Point()
            agent_point.x = odom.pose.pose.position.x
            agent_point.y = odom.pose.pose.position.y
            agent_point.z = odom.pose.pose.position.z

            # connect the projected camera bounds

            for edge in range(len(projection)):
                point_a = Point()
                point_a.x = projection[edge][0]
                point_a.y = projection[edge][1]
                point_a.z = projection[edge][2]

                point_b = Point()
                point_b.x = projection[(edge + 1) % len(projection)][0]
                point_b.y = projection[(edge + 1) % len(projection)][1]
                point_b.z = projection[(edge + 1) % len(projection)][2]

                points.append(point_a)
                points.append(point_b)
                points.append(point_a)
                points.append(agent_point)

            projection_marker.points = points
            marker_list.markers.append(projection_marker)

        return marker_list

    def get_targets_marker(self, time, frame, target_positions):
        targets_marker_array = MarkerArray()

        for idx, target in enumerate(target_positions.targets):
            target_marker = Marker()
            target_marker.header.frame_id = frame
            target_marker.header.stamp = time
            target_marker.ns = "target_pose"
            target_marker.id = idx
            target_marker.type = Marker.MESH_RESOURCE
            target_marker.action = Marker.ADD
            target_marker.mesh_use_embedded_materials = False
            target_marker.mesh_resource = os.path.join(
                "package://ipp_simple_sim",
                rospy.get_param("~target_mesh"),
            )

            target_marker.lifetime = rospy.Duration()
            quat = quaternion_from_euler(0, 0, target.heading)
            target_marker.pose = Pose(
                Point(
                    target.x, target.y, 0
                ),  # z offset to make it appear above grid-map
                Quaternion(quat[0], quat[1], quat[2], quat[3]),
            )

            target_marker.color = get_color(target.id)
            target_marker.scale.x = 0.2 * visualization_scale
            target_marker.scale.y = 0.2 * visualization_scale
            target_marker.scale.z = 0.2 * visualization_scale
            targets_marker_array.markers.append(target_marker)

        return targets_marker_array

    def plan_request_callback(self, plan_request, id_num):
        self.waiting_for_plan = True
        self.sim_env.agent[id_num].vel = plan_request.desired_speed
        self.sim_env.hvel = plan_request.desired_speed
        self.sim_env.remaining_budget[id_num] = plan_request.maximum_range

        priority_list = {}
        for target in plan_request.target_priors:
            if target.target.header.frame_id != "":
                priority_list[str(target.target.local_id)] = target.target.priority
        # print(priority_list)
        rospy.set_param("~priority", priority_list)

        if rospy.get_param("~set_agent_pose_to_plan_request"):
            rospy.loginfo("Teleporting agent to plan request position")
            self.agent_traj_list[id_num] = []
            agent_pose = plan_request.start_pose
            self.sim_env.agent[id_num].x = agent_pose.position.x
            self.sim_env.agent[id_num].y = agent_pose.position.y
            self.sim_env.agent[id_num].z = agent_pose.position.z
            self.sim_env.prev_agentxyz[id_num] = [
                agent_pose.position.x,
                agent_pose.position.y,
                agent_pose.position.z,
            ]
            # https://github.com/ros/geometry/issues/109#issuecomment-344702754
            explicit_quat = [
                agent_pose.orientation.x,
                agent_pose.orientation.y,
                agent_pose.orientation.z,
                agent_pose.orientation.w,
            ]
            roll, pitch, yaw = euler_from_quaternion(explicit_quat)
            self.sim_env.agent[id_num].yaw = yaw  # yaw angle
            self.sim_env.global_waypoint_list[id_num] = Plan()

        if rospy.get_param("~set_true_targets_to_target_prior_means"):
            rospy.loginfo(
                "Set true simulated target states from plan request prior distributions"
            )
            self.init_true_targets_from_target_prior_means(plan_request.target_priors)
        elif rospy.get_param("~sample_true_targets_from_target_priors"):
            rospy.loginfo(
                "Sampling true simulated target states from plan request prior distributions"
            )
            self.sample_true_targets_from_target_priors(plan_request.target_priors)

        if rospy.get_param("~sample_additional_true_targets_from_search_prior"):
            rospy.loginfo("Sampling true simulated target states from search map prior")
            self.sample_additional_true_targets_from_search_prior()
        
    def multi_plan_request_callback(self, multi_plan_request):
        self.waiting_for_plan = True
        for idx in range(len(multi_plan_request.start_poses)):
            self.sim_env.agent[idx].vel = multi_plan_request.desired_speed
            self.sim_env.hvel = multi_plan_request.desired_speed
            self.sim_env.remaining_budget[idx] = multi_plan_request.maximum_range
        
        priority_list = {}
        priority_list = {}
        for target in multi_plan_request.target_priors:
            if target.target.header.frame_id != "":
                priority_list[str(target.target.local_id)] = target.target.priority
        # print(priority_list)
        rospy.set_param("~priority", priority_list)

        for idx in range(len(multi_plan_request.start_poses)):
            rospy.loginfo("teleporting agent " + str(idx) + " to plan request position")
            self.agent_traj_list[idx] = []
            agent_pose = multi_plan_request.start_poses[idx]
            self.sim_env.agent[idx].x = agent_pose.position.x
            self.sim_env.agent[idx].y = agent_pose.position.y
            self.sim_env.agent[idx].z = agent_pose.position.z
            self.sim_env.prev_agentxyz[idx] = [
                agent_pose.position.x,
                agent_pose.position.y,
                agent_pose.position.z,
            ]
            # https://github.com/ros/geometry/issues/109#issuecomment-344702754
            explicit_quat = [
                agent_pose.orientation.x,
                agent_pose.orientation.y,
                agent_pose.orientation.z,
                agent_pose.orientation.w,
            ]
            roll, pitch, yaw = euler_from_quaternion(explicit_quat)
            self.sim_env.agent[idx].yaw = yaw  # yaw angle
            self.sim_env.global_waypoint_list[idx] = Plan()

    def sample_additional_true_targets_from_search_prior(self):
        # don't want to bother to reconstruct the search map from plan request. instead, make a service call to the search belief and get the search map from there
        rospy.loginfo("IPP Simple Sim waiting for search map update service")
        rospy.sleep(2)  # explicitly wait
        service_name = f"/{self.robot_names[0]}/realtime_search_map/search_map_update"
        rospy.wait_for_service(service_name)
        search_map_update_srv = rospy.ServiceProxy(service_name, SearchMapUpdate)
        search_map_params = search_map_update_srv()
        # SearchMap.h stores the map as rows (first axis) as x, cols (second axis) as y
        n_rows = int(
            (search_map_params.x_end - search_map_params.x_start)
            / search_map_params.map_resolution
        )
        n_cols = int(
            (search_map_params.y_end - search_map_params.y_start)
            / search_map_params.map_resolution
        )

        search_map = np.array(search_map_params.map_values).reshape(n_rows, n_cols)

        linear_idxs = np.random.choice(
            search_map.size,
            size=rospy.get_param("~n_rand_targets"),
            p=search_map.ravel() / float(search_map.sum()),
        )
        rows, cols = np.unravel_index(linear_idxs, search_map.shape)

        id_to_start_from = (
            max(target.id for target in self.sim_env.targets) + 1
            if len(self.sim_env.targets) > 0
            else 0
        )
        for idx, (r, c) in enumerate(zip(rows, cols)):
            x = search_map_params.x_start + r * search_map_params.map_resolution
            y = search_map_params.y_start + c * search_map_params.map_resolution
            target = Target(
                id=idx + id_to_start_from,
                init_x=x,
                init_y=y,
                heading=np.random.uniform(*rospy.get_param("~rand_heading_range")),
                linear_speed=np.random.uniform(
                    *rospy.get_param("~rand_linear_speed_range")
                ),
                angular_speed=np.random.normal(
                    *rospy.get_param("~rand_angular_speed_range")
                ),
                linear_speed_std=rospy.get_param("~rand_linear_speed_std"),
                angular_speed_std=rospy.get_param("~rand_angular_speed_std"),
            )
            rospy.loginfo(f"Sampled target from search prior: {target}")
            self.sim_env.targets.append(target)

    def sample_true_targets_from_target_priors(self, target_priors):
        """
        Given the list of target priors, sample the true simulated target state from the multivariate Gaussian distribution.
        """
        self.sim_env.targets[:] = []
        for prior in target_priors:
            t = prior.target
            if t and not (t.x == 0 and t.y == 0 and t.xdot == 0 and t.ydot == 0):
                prior_heading = np.arctan2(t.ydot, t.xdot)
                prior_speed = np.sqrt(t.xdot**2 + t.ydot**2)
                jacobian = np.array(
                    [
                        [1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [
                            0,
                            0,
                            -t.ydot / (t.xdot * t.xdot + t.ydot * t.ydot),
                            t.xdot / (t.xdot * t.xdot + t.ydot * t.ydot),
                        ],
                        [0, 0, t.xdot / prior_speed, t.ydot / prior_speed],
                        [0, 0, 0, 0],
                    ]
                )
                covs = np.array(t.covariance).reshape(5, 5)[0:4, 0:4]
                covs = jacobian @ covs @ jacobian.T
                # covs[2, 2]  = covs[3,3] = covs[4,4] = 0.0001  # ignore heading, speed, angle
                target_state = np.random.multivariate_normal([0, 0, 0, 0, 0], covs, 1)[
                    0
                ]
                # target_state *= 3/4  # scale down the variance
                sim_target = Target(
                    id=t.id,
                    init_x=target_state[0] + t.x,
                    init_y=target_state[1] + t.y,
                    heading=target_state[2] + t.heading,
                    linear_speed=target_state[3] + t.linear_speed,
                    angular_speed=target_state[4] + t.angular_speed,
                    linear_speed_std=0.00,
                    angular_speed_std=0.000,
                )
                self.sim_env.targets.append(sim_target)
        rospy.loginfo("Added " + str(len(self.sim_env.targets)) + " simulated targets")

    def init_true_targets_from_target_prior_means(self, target_priors):
        """
        Given the list of target priors, init the true simulated target state.
        """
        self.sim_env.targets[:] = []
        for prior in target_priors:
            t = prior.target
            if t and not (t.x == 0 and t.y == 0 and t.xdot == 0 and t.ydot == 0):
                prior_heading = np.arctan2(t.ydot, t.xdot)
                prior_speed = np.sqrt(t.xdot**2 + t.ydot**2)
                sim_target = Target(
                    id=t.local_id,
                    init_x=t.x,
                    init_y=t.y,
                    heading=prior_heading,
                    linear_speed=prior_speed,
                    angular_speed=0,
                    linear_speed_std=0.0,
                    angular_speed_std=0.0,
                )
                self.sim_env.targets.append(sim_target)
        rospy.loginfo("Added " + str(len(self.sim_env.targets)) + " simulated targets")

    def main(self):
        odom_pubs = [[] for i in range(self.num_agents)]
        sensor_detection_pubs = [[] for i in range(self.num_agents)]
        camera_pose_pubs = [[] for i in range(self.num_agents)]
        remaining_budget_pubs = [[] for i in range(self.num_agents)]
        waypoint_num_pubs = [[] for i in range(self.num_agents)]

        if self.centralized:
            rospy.Subscriber("/planner/plan_request", MultiPlanRequest, self.multi_plan_request_callback)

        for idx in range(self.num_agents):
            odom_pubs[idx] = rospy.Publisher(
                robot_names[idx] + "/odom", Odometry, queue_size=10
            )
            sensor_detection_pubs[idx] = rospy.Publisher(
                robot_names[idx] + "/sensor_measurement", Detections, queue_size=10
            )
            camera_pose_pubs[idx] = rospy.Publisher(
                robot_names[idx] + "/camera_pose", Odometry, queue_size=10
            )
            remaining_budget_pubs[idx] = rospy.Publisher(
                robot_names[idx] + "/remaining_budget", Float32, queue_size=10
            )
            if not self.centralized:
                rospy.Subscriber(
                    robot_names[idx] + "/planner/plan_request",
                    PlanRequest,
                    self.plan_request_callback,
                    (idx),
                )
            rospy.Subscriber(
                robot_names[idx] + self.planner_path_topic,
                Plan,
                self.planner_callback,
                (idx),
            )
            waypoint_num_pubs[idx] = rospy.Publisher(
                robot_names[idx] + "/waypoint_num", UInt32, queue_size=10
            )
        target_pose_pub = rospy.Publisher(
            "simulator/target_poses", GroundTruthTargets, queue_size=10
        )

        # visualization publishers
        ocean_marker_pub = rospy.Publisher(
            "simulator/markers/ocean_plane", Marker, queue_size=2
        )
        agent_marker_pub = rospy.Publisher(
            "simulator/markers/agent_mesh", MarkerArray, queue_size=10
        )
        projection_marker_pub = rospy.Publisher(
            "simulator/markers/camera_projection", MarkerArray, queue_size=10
        )
        projection_points_marker_pub = rospy.Publisher(
            "simulator/markers/camera_projection_points", MarkerArray, queue_size=10
        )
        targets_marker_pub = rospy.Publisher(
            "simulator/markers/targets", MarkerArray, queue_size=10
        )
        agent_trajectory_pub = rospy.Publisher(
            "simulator/markers/agent_trajectory", MarkerArray, queue_size=10
        )

        rate = rospy.Rate(1.0 / self.sim_env.del_t)
        broadcaster = tf2_ros.TransformBroadcaster()
        counter = 0

        # filename = "./data/" + rospy.get_param('/experiment', 'blank_sim_manager') + "_target_positions.csv"
        # with open(filename, 'w') as f:
        #     f.write("time_stamp,target_id,x,y,heading,linear_speed,angular_speed\n")

        # visualization markers
        projection_marker_pub = rospy.Publisher(
            "simulator/markers/camera_projection", MarkerArray, queue_size=10
        )

        start_time = rospy.Time.now()
        print("\nSim is ready to go\n")

        while not rospy.is_shutdown():
            if self.pause_while_planning and self.waiting_for_plan:
                pass  # do nothing while waiting for plan
            else:
                counter += 1
                self.sim_env.update_states()

            time = rospy.Time.now()
            frame = "local_enu"
            agent_odom = self.get_agent_odometry(time, frame)
            target_positions = self.get_target_positions(time, frame)
            target_detections, camera_projections = self.get_target_detections(
                time, frame
            )
            # self.get_detections_marker(target_detections)
            camera_pose = self.get_camera_pose(time, frame)
            remaining_budget = self.get_remaining_budget()
            waypoint_num = self.get_waypoint_num()

            for id_num in range(self.num_agents):
                # agent body frame
                agent_pose = agent_odom[id_num].pose.pose
                body_transform = TransformStamped(
                    header=Header(frame_id="local_enu", stamp=rospy.Time.now()),
                    child_frame_id=self.robot_names[id_num] + "/base_link",
                    transform=Transform(
                        translation=Vector3(
                            agent_pose.position.x,
                            agent_pose.position.y,
                            agent_pose.position.z,
                        ),
                        rotation=agent_pose.orientation,
                    ),
                )
                broadcaster.sendTransform(body_transform)
                # agent camera frame
                camera_quat = quaternion_from_euler(0, self.sim_env.sensor_pitch, 0.0)
                camera_transform = TransformStamped(
                    header=Header(
                        frame_id=self.robot_names[id_num] + "/base_link",
                        stamp=rospy.Time.now(),
                    ),
                    child_frame_id=self.robot_names[id_num] + "/camera",
                    transform=Transform(
                        translation=Vector3(0.0, 0.0, 0.0),
                        rotation=Quaternion(
                            camera_quat[0],
                            camera_quat[1],
                            camera_quat[2],
                            camera_quat[3],
                        ),
                    ),
                )
                broadcaster.sendTransform(camera_transform)
                # publish agent info topics
                odom_pubs[id_num].publish(agent_odom[id_num])
                camera_pose_pubs[id_num].publish(camera_pose[id_num])
                sensor_detection_pubs[id_num].publish(target_detections[id_num])
                remaining_budget_pubs[id_num].publish(remaining_budget[id_num])
                waypoint_num_pubs[id_num].publish(waypoint_num[id_num])

            target_pose_pub.publish(target_positions)

            if counter % 10 == 0:
                # visualizations
                agent_trajectory_pub.publish(
                    self.get_agent_trajectory_marker(time, frame, agent_odom)
                )
                ocean_marker_pub.publish(self.get_ocean_marker(time, frame))
                agent_marker_pub.publish(self.get_agent_marker(time, frame, agent_odom))
                projection_marker_pub.publish(
                    self.get_projection_marker(
                        time, frame, agent_odom, camera_projections
                    )
                )
                targets_marker_pub.publish(
                    self.get_targets_marker(time, frame, target_positions)
                )
                projection_points_marker_pub.publish(
                    self.get_projection_points_marker(
                        time, frame, agent_odom, camera_projections
                    )
                )

            # calculate the velocity of the target
            # curr_time  = rospy.get_time()
            # if self.prev_time != -1 and len(self.prev_target_positions.targets) != 0:
            #     for i in range(len(target_positions.targets)):
            #         # print(target_positions)
            #         # print()
            #         delta_x = target_positions.targets[i].x - self.prev_target_positions.targets[i].x
            #         delta_y = target_positions.targets[i].y - self.prev_target_positions.targets[i].y

            #         delta_t = curr_time - self.prev_time
            #         velocity = np.sqrt(delta_x**2 + delta_y**2) / delta_t
            #         print("Target " + str(i) + " velocity: " + str(velocity))

            # self.prev_time = curr_time
            # self.prev_target_positions = target_positions

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("sim_manager_node")
    robot_names = rospy.get_param("~robot_names")
    if len(robot_names) == 0:
        robot_names = [""]
    obj = SimManager(robot_names)
    obj.main()
