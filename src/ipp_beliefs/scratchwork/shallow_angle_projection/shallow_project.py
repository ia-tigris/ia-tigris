from operator import pos
import os
import numpy as np
import matplotlib.pyplot as plt
from math import *

os.makedirs("figures", exist_ok=True)
# defining constants
camera_width = 640
camera_height = 640
camera_fov = 90
fx = camera_width / (2 * tan(radians(camera_fov)/2))
fy = fx
cx = (camera_width + 1)/2
cy = (camera_height + 1)/2
intrinsics_matrix = np.array([[fx, 0, cx, 0],
                              [0, fy, cy, 0],
                              [0, 0,  1,  0],
                              [0, 0,  0,  1]])
K_inverse = np.linalg.inv(intrinsics_matrix)

# for camera facing x by default
# inter_matrix = np.array([[0, 1, 0, 0],
#                          [0, 0, 1, 0],
#                          [1, 0, 0, 0],
#                          [0, 0, 0, 1]])

# for camera facing z by default
inter_matrix = np.array([[0, 1, 0, 0],
                         [-1, 0, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

inter_inverse = np.linalg.inv(inter_matrix)
num_points = 50000
pos_variance = 1
rot_variance = 0.1
distr_x = np.random.normal(0, pos_variance, num_points)
distr_y = np.random.normal(0, pos_variance, num_points)
distr_roll = np.random.normal(0, 0, num_points) # roll around z
distr_yaw = np.random.normal(0, rot_variance, num_points) # yaw around x

z_vals = [-10, -20, -40, -80, -160, -320, -640]
# pitch_degs = [-10, -15, -20, -25, -30, -90]

alphas = np.zeros(num_points, dtype=float) + (1/255)

# equivalent angles for camera facing z by default
pitch_degs = [80, 75, 70, 65, 60, 0]
pitch_vals = [radians(d) for d in pitch_degs]


def xyz_to_rot_matrix(psi, theta, phi):
    res = np.array([[cos(phi)*cos(theta), -sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi), sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi)],
                    [sin(phi)*cos(theta), cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi),  -cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi)],
                    [-sin(theta),           cos(theta)*sin(psi),                               cos(theta)*cos(psi)]])
    return res

def get_projection_from_pose(x, y, z, roll, pitch, yaw):
    rot = xyz_to_rot_matrix(yaw, pitch, roll)
    camera_to_world = np.zeros((4, 4))
    camera_to_world[0:3, 0:3] = rot
    camera_to_world[0, 3] = x
    camera_to_world[1, 3] = y
    camera_to_world[2, 3] = z
    camera_to_world[3, 3] = 1
    pixel_to_world = np.matmul(camera_to_world, np.matmul(inter_inverse, K_inverse))
    center_pixel = np.array([camera_width/2, camera_height/2, 1, 1])
    center_pixel_shifted = center_pixel * 2
    center_pixel_shifted[3] = 1
    world_pos = np.matmul(pixel_to_world, center_pixel)
    world_pos_shifted = np.matmul(pixel_to_world, center_pixel_shifted)

    world_pos = world_pos[0:3]
    world_pos_shifted = world_pos_shifted[0:3]
    diff = np.subtract(world_pos_shifted, world_pos)
    t = -world_pos[2]/diff[2]
    ans = world_pos + t * diff
    return ans[0], ans[1], t >= 0
    
for i in range(len(z_vals)):
    for j in range(len(pitch_vals)):
        title = f"Points={num_points}, z={z_vals[i]}, pitch={pitch_degs[j]}"
        print("Generating", title)
        distr_z = np.random.normal(z_vals[i], pos_variance, num_points)
        distr_pitch = np.random.normal(pitch_vals[j], rot_variance, num_points)
        projected_points_x = []
        projected_points_y = []
        for k in range(num_points):
            x, y, z, roll, pitch, yaw = distr_x[k], distr_y[k], distr_z[k], distr_roll[k], distr_pitch[k], distr_yaw[k]
            proj_x, proj_y, valid = get_projection_from_pose(x, y, z, roll, pitch, yaw)
            if (valid):
                projected_points_x.append(proj_x)
                projected_points_y.append(proj_y)
        plt.scatter(projected_points_x, projected_points_y, s=1, alpha=alphas, color="red")
        axes=plt.gca()
        axes.set_aspect("equal") #, adjustable='datalim')
        axes.set_title(title)
        axes.set_xlim([-100, 1900])
        axes.set_ylim([-1000, 1000])
        plt.savefig("figures/Z_{0}_P_{1}.png".format(abs(z_vals[i]), int(round(degrees(pitch_vals[j])))))
        plt.clf()
        
# x -> phi
# y -> theta
# z -> psi
# xyz all around a fixed frame




