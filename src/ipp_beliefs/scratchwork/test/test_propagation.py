from propagation import propagate
import numpy as np
import unittest


class TestPropagation(unittest.TestCase):
    def test_only_x_propagation(self):
        """
        Tests a component moving in the x direction
        :return:
        """
        odometry_belief_map = np.zeros((5, 5, 2), dtype=float)
        # set the coordinate [0,0], with component[0] (x compoennt) to have 1
        odometry_belief_map[0, 0, 0] = 1

        for _ in range(3):
            odometry_belief_map = propagate(odometry_belief_map, None)
            odometry_belief_map = propagate(odometry_belief_map, None)
            odometry_belief_map = propagate(odometry_belief_map, None)

    def test_only_y_propagation(self):
        pass