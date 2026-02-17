from constants import *
from glob import glob
import numpy as np
import torch.utils.data
import re

from torch.utils.data import Dataset
from torch.utils.data.dataset import T_co

torch.utils.data.Dataset


def rand_rot(x, y, theta):
    rand_rot = np.random.uniform(-np.pi, np.pi)

    rot_matrix = np.array(
        [
            [np.cos(rand_rot), -np.sin(rand_rot)],
            [np.sin(rand_rot), np.cos(rand_rot)],
        ]
    )
    x, y = rot_matrix @ np.array([x, y])
    theta += rand_rot
    return x, y, theta


def center(x, y, theta):
    x = x - (x.max() + x.min()) / 2
    y = y - (y.max() + y.min()) / 2
    return x, y, theta

def rand_x_flip(x, y, theta):
    if np.random.randint(0, 2) == 0:
        x = -x
        theta = np.pi - theta
    return x, y, theta

def rand_shift(x, y, theta):
    x += np.random.uniform(-5000, 5000)
    y += np.random.uniform(-5000, 5000)
    return x, y, theta

def rand_y_flip(x, y, theta):
    if np.random.randint(0, 2) == 0:
        y = -y
        theta = np.pi - theta
    return x, y, theta


class MctsStateValueDataset(Dataset):
    NUM_PARTICLES = NUM_PARTICLES
    NUM_TRACKERS = MAX_NUM_TRACKERS

    def __init__(self, folder_root):
        print(f"USING {NUM_PARTICLES=} AND {MAX_NUM_TRACKERS=}")
        self.folder_root = folder_root

        # recursively glob for all csv files
        self.particle_files = glob(
            folder_root + "/**/tracker_list*.csv", recursive=True
        )
        self.state_files = glob(folder_root + "/**/state_teardown*.csv", recursive=True)
        assert len(self.particle_files) == len(self.state_files), "Mismatched files"

    def __len__(self):
        return len(self.particle_files)

    def __getitem__(self, index) -> T_co:
        """
        :return:
        """
        # TODO: fill in blanks for remaining trackers
        particle_file = self.particle_files[index]
        n_visits = int(particle_file[particle_file.find("=") + 1:-4])
        drone_file = particle_file.replace("tracker_list", "state")
        drone_file = re.sub("_n_visits=\d+", "", drone_file)

        my_array = []

        with open(particle_file, "r") as f:
            prev_id = None
            file_lines = f.readlines()[1:]  # skip the header
            assert len(file_lines) <= self.NUM_PARTICLES * self.NUM_TRACKERS, f"File {particle_file} has too many lines: {len(file_lines)}"
            for line in file_lines:
                id, x, y, theta, speed, _ = line.replace(" ", "").strip().split(",")
                if (
                    prev_id is not None
                    and id != prev_id
                    and len(my_array) % self.NUM_PARTICLES > 0
                ):
                    self.pad_to_num_particles(my_array)
                    assert len(my_array) % self.NUM_PARTICLES == 0, f"Not a multiple of {self.NUM_PARTICLES}"
                my_array.append([float(x), float(y), float(theta), float(speed)])
                prev_id = id

        # pad the last one
        if (len(my_array) % self.NUM_PARTICLES) > 0:
            self.pad_to_num_particles(my_array)

        num_trackers = len(my_array) // self.NUM_PARTICLES

        # pad if there are less trackers than NUM_TRACKERS
        for _ in range((self.NUM_TRACKERS - num_trackers) * self.NUM_PARTICLES):
            my_array.append([0., 0., 0., 0.])

        try:
            with open(drone_file, "r") as f:
                all_lines = f.readlines()
                drone_state_line = all_lines[1]
                x, y, z, heading, budget, initial_var = drone_state_line.split(",")
                my_array.append([float(x), float(y), float(heading), -1])

                result_line = all_lines[2].strip()
                future_value = float(result_line)
                # rollout_value, mcts_tree_value = result_line
        except Exception as e:
            print(e)
            raise ValueError(f"Error reading file: {drone_file}")

        future_value /= n_visits

        my_array = np.array(my_array)

        xs, ys, thetas = my_array[:, 0], my_array[:, 1], my_array[:, 2]
        xs, ys, thetas = center(xs, ys, thetas)
        xs, ys, thetas = rand_rot(xs, ys, thetas)
        xs, ys, thetas = rand_shift(xs, ys, thetas)
        xs, ys, thetas = rand_x_flip(xs, ys, thetas)
        xs, ys, thetas = rand_y_flip(xs, ys, thetas)
        my_array[:, 0], my_array[:, 1], my_array[:, 2] = xs, ys, thetas

        drone_array = np.concatenate(
            (my_array[-1, :3], np.array([float(budget), float(initial_var)]))
        )
        drone_state = torch.tensor(drone_array, dtype=torch.float32)
        particle_states = torch.tensor(my_array[:-1].flatten(), dtype=torch.float32)
        value = torch.tensor(np.array([float(future_value)]), dtype=torch.float32)
        assert len(drone_state == 5)
        assert len(particle_states) == self.NUM_TRACKERS * self.NUM_PARTICLES * 4, f"file {particle_file} {len(particle_states)} != {self.NUM_TRACKERS * self.NUM_PARTICLES * 4}"
        assert len(value) == 1
        return drone_state, particle_states, value

    def pad_to_num_particles(self, my_array):
        num_blanks = self.NUM_PARTICLES - len(my_array) % self.NUM_PARTICLES
        for _ in range(num_blanks):
            my_array.append([0., 0., 0., 0.])
