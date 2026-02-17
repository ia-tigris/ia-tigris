import numpy as np
import matplotlib
import matplotlib.colors
import matplotlib.pyplot as plt


# have a map of vector
# have a 2D array for [x, y]
# each position will need an object ID

# operation: for each point [x, y], calculate where the vector component points to, with its degree of uncertainty;
# distribute the vector component to its neighboring cells accordingly

# 2 degrees of uncertainty here: magnitude, direction. we should be able to demonstrate these degrees independently.
# each degree of uncertainty should have a mean and variance.

# test: a vector with no uncertainty should propagate forward with no spread.
# a vector with uncertainty should disperse over time in certainty
#
# have one map layer per object ID. let's start with 1 object for now

def propagate(odometry_belief_means: np.ndarray, odometry_belief_variances: np.ndarray, meters_per_grid: float = 1.0):
    """

    :param meters_per_grid: smaller means higher resolution. E.g. 0.1 means 0.1 meters are represented per cell
    :param odometry_belief_means: a H*W*[x,y] map representing odometry in METERS
    :param odometry_belief_variances:
    :return:
    """

    xys, height, width = odometry_belief_means.shape
    # let's handle just the means for iteration 1
    new_odometry_belief_map = np.zeros_like(odometry_belief_means)

    for y_coord in range(height):
        for x_coord in range(width):
            # component meaning the vector component
            x_component_mean = odometry_belief_means[y_coord, x_coord, 0]
            y_component_mean = odometry_belief_means[y_coord, x_coord, 1]
            if x_component_mean != 0 or y_component_mean != 0:
                new_x_component_mean = (1 / meters_per_grid) * x_component_mean + x_coord
                new_y_component_mean = (1 / meters_per_grid) * y_component_mean + y_coord
                # TODO: what to do about rounding error? evenly distribute?
                #  we can choose how fine resolution to make our map; i.e if each cell represents a meter, 0.5 meters, etc.
                #  then we also need the map scale. since we can choose the resolution, then it's fine to round because we
                #  acknowledge we're losing accuracy under that resolution

                # TODO: uncertainty will be based on the variance; we will weight distributing the value over cells. for now,
                #  let's assume no variance, so everything propagates atomically
                intround = lambda x: int(round(x))

                new_odometry_belief_map[
                    intround(new_y_component_mean), intround(new_x_component_mean), 0] = x_component_mean
                new_odometry_belief_map[
                    intround(new_y_component_mean), intround(new_x_component_mean), 1] = y_component_mean

    return new_odometry_belief_map


if __name__ == "__main__":
    odometry_belief_map = np.zeros((5, 5, 2), dtype=float)
    odometry_belief_map[0, 0, 0] = 1
    odometry_belief_map = propagate(odometry_belief_map, None)
    odometry_belief_map = propagate(odometry_belief_map, None)
    odometry_belief_map = propagate(odometry_belief_map, None)
