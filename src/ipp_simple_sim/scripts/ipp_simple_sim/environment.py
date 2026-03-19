import math
import random
from turtle import heading
import numpy as np
from ipp_simple_sim.agent import *
from ipp_simple_sim.target import *
from ipp_simple_sim.sensor import *
from planner_map_interfaces.msg import Plan

class Environment:
    def __init__(self, list_of_target_dicts=[], max_omega=5, max_zvel=5,
                 init_x=None, init_y=None, init_z=None, init_yaw=None,
                 K_p=0.01, K_p_z=0.01,num_agents=1,
                 agent_l=3, hvel=5, vvel=2, n_rand_targets=-1, del_t=0.02,
                 waypoint_threshold=5,
                 sensor_focal_length=5, sensor_width=10, sensor_height=10,
                 sensor_pitch=20, sensor_max_range=500):
        '''
        Setup simulation environment
        '''
        # if initial position not specified, randomly spawn agent between (50, 1000)
        init_x = random.randrange(50, 1000) if init_x is None else init_x
        init_y = random.randrange(50, 1000) if init_y is None else init_y
        init_z = random.randrange(20, 120,
                                  20) if init_z is None else init_z  # discretized by step-size 20
        init_yaw = random.uniform(0, np.pi) if init_yaw is None else init_yaw

        # drone pose
        self.init_x = init_x
        self.init_y = init_y
        self.init_z = init_z
        self.init_yaw = init_yaw

        self.del_t = del_t
        self.num_agents = num_agents #number of agents to simulate

        self.agent_l = agent_l  # agent length
        self.hvel = hvel  # horizontal velocity
        self.vvel = vvel  # vertical velocity
        self.max_omega = max_omega  # max angular velocity
        self.max_zvel = max_zvel  # max vertical velocity

        self.sensor_focal_length = sensor_focal_length
        self.sensor_width = sensor_width
        self.sensor_height = sensor_height
        self.sensor_pitch = sensor_pitch
        self.sensor_max_range = sensor_max_range

        # if targets not specified, randomly generate between 1-10 targets
        self.n_rand_targets = random.randrange(1, 10) if not list_of_target_dicts and n_rand_targets == -1 else n_rand_targets

        self.targets = self.generate_targets(list_of_target_dicts, self.n_rand_targets)

        self.waypoint_threshold = waypoint_threshold

        self.K_p = K_p  # x-y proportionality constant for PID controller
        self.K_p_z = K_p_z  # z-axis proportionality constant for PID controller
        self.sensor = self.init_sensor()

        self.global_waypoint_list = [Plan() for i in range(self.num_agents)]
        self.agent = [self.init_agent(i) for i in range(self.num_agents)]
        self.prev_agentxyz = [[0.0, 0.0, 0.0] for i in range(self.num_agents)]
        self.curr_waypoint_num = [0 for i in range(self.num_agents)]
        self.remaining_budget = [0 for i in range(self.num_agents)]

        # self.prev_time = -1

    def generate_targets(self, list_of_target_dicts, n_rand_targets=None):
        '''
        Generates ships with initial positions
        '''
        # when no targets specified
        if not list_of_target_dicts:
            if n_rand_targets is None:
                raise ValueError(
                    "Passed in no targets but didn't pass in n_rand_targets")
            targets = [
                Target(
                    id=idx,
                    init_x=np.random.uniform(-600, 600),
                    init_y=np.random.uniform(-600, 600),
                    heading=np.random.uniform(*rospy.get_param("~rand_heading_range")),
                    linear_speed=np.random.normal(*rospy.get_param("~rand_linear_speed_range")),
                    angular_speed=np.random.normal(*rospy.get_param("~rand_angular_speed_range")),
                    linear_speed_std=rospy.get_param("~rand_linear_speed_std"),
                    angular_speed_std=rospy.get_param("~rand_angular_speed_std")
                )
                for idx in range(n_rand_targets)
            ]
        # when targets are specified
        else:
            targets = [
                Target(
                    id=target["id"],
                    init_x=target["x"], init_y=target["y"], heading=target["heading"],
                    linear_speed=target["linear_speed"],
                    angular_speed=target["angular_speed"],
                    linear_speed_std=target["linear_speed_std"],
                    angular_speed_std=target["angular_speed_std"]
                ) for target in list_of_target_dicts
            ]
        return targets

    def init_agent(self,id_num):
        return Agent(agent_num=id_num,
                        init_x=self.init_x,
                        init_y=self.init_y,
                        init_z=self.init_z,
                        init_yaw=self.init_yaw,
                        agent_l=self.agent_l,
                        hvel=self.hvel,
                        vvel=self.vvel)

    def init_sensor(self):
        return SensorModel(
            self.sensor_focal_length,
            self.sensor_width, self.sensor_height, self.sensor_pitch, self.sensor_max_range
        )

    def get_ground_intersect(self, agent_pos, pitch, yaw):
        return self.sensor.project_camera_bounds_to_plane(agent_pos,
                                                self.sensor.rotated_camera_fov(
                                                    pitch=pitch, yaw=yaw))

    def get_sensor_measurements(self):
        '''
        Get sensor measurements from camera sensor
        '''
        detected_targets_list = []
        camera_projections_list = []
        for i in range(self.num_agents):
            camera_projection = self.get_ground_intersect(
            [self.agent[i].x, self.agent[i].y, self.agent[i].z], self.sensor_pitch,
            self.agent[i].yaw)
            detected_targets = []
            for target in self.targets:
                if self.sensor.is_point_inside_camera_projection([target.x, target.y],
                                                             camera_projection):
                    range_to_target = np.linalg.norm(
                    np.array([target.x, target.y, 0]) - np.array(
                        [self.agent[i].x, self.agent[i].y, self.agent[i].z]))
                    # is_detected = self.sensor.get_detection(range_to_target)
                    detection_prob = np.random.random()
                    sensor_tpr = self.sensor.tpr(range_to_target)
                    if detection_prob < sensor_tpr:
                        target.is_detected = True
                        detected_targets.append(target)
            detected_targets_list.append(detected_targets)
            camera_projections_list.append(camera_projection)
        return detected_targets_list, camera_projections_list

    def traverse(self, delta_t):
        '''
        Waypoint manager and agent state update- moves agent towards waypoints as long as waypoints exist in global_waypoint_list
        '''
        for i in range(self.num_agents):
            agent_waypoint_list = self.global_waypoint_list[i]
            if len(agent_waypoint_list.plan) == 0:
                continue
            else:
                next_position = np.array(
                    [agent_waypoint_list.plan[0].position.position.x,
                    agent_waypoint_list.plan[0].position.position.y,
                    agent_waypoint_list.plan[0].position.position.z])
                dist_to_waypoint = np.linalg.norm(
                    [self.agent[i].x, self.agent[i].y, self.agent[i].z] - next_position)
                # update waypoint list if reached waypoint
                if dist_to_waypoint < self.waypoint_threshold:
                    self.curr_waypoint_num[i] += 1
                    agent_waypoint_list.plan.pop(0)
                omega, z_d = self.agent[i].go_to_goal(self.max_omega, self.max_zvel,
                                                    next_position, self.K_p,
                                                    self.K_p_z)
                self.agent[i].yaw += delta_t * omega
                self.agent[i].x += delta_t * self.hvel * math.cos(self.agent[i].yaw)
                self.agent[i].y += delta_t * self.hvel * math.sin(self.agent[i].yaw)
                self.agent[i].z += delta_t * z_d
                delta_dist = np.linalg.norm(
                        np.array([self.agent[i].x, self.agent[i].y, self.agent[i].z]) - np.array(self.prev_agentxyz[i]))
                if delta_dist > self.agent[i].vel*2:
                    print("Jump for agent ", i, " of ", delta_dist, "m")
                    print("Ignoring jump and not changing remaining budget")
                else:
                    self.remaining_budget[i] -= delta_dist
                self.prev_agentxyz[i] = [self.agent[i].x, self.agent[i].y, self.agent[i].z]
        

    def update_waypoints(self, new_wpts, id_num):
        '''
        Receive new waypoints and send them to waypoint manager
        '''
        # self.global_waypoint_list.append(new_wpts)
        self.global_waypoint_list[id_num] = new_wpts
        self.curr_waypoint_num[id_num] = 0

    def update_states(self):
        '''
        Updates the environment states
        '''
        self.traverse(self.del_t)
        # update the states for all ships in the environment
        for target in self.targets:
            target.propagate(self.del_t)

    def get_agent_uncertainty(self,id_num):
        return self.agent[id_num].position_uncertainty()

    # function that gets target heading and return heading with gaussian noise
    def get_target_heading_noise(self, heading):
        # gaussian noise model for now
        return heading + np.random.normal(0, 0.05)
