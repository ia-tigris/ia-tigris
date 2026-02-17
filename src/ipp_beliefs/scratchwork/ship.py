"""
Requirements

a ship propagates according to some motion primitivies, randomly.

There's some smoothness in its direction. Only change direction occasionally

We have a Kalman Filter on it to propagate and track its expected movement

Visualize the ship in one screen. Visualize the belief in another screen
Plot both

Ship motion primitives

acceleration
velocity
position
"""

import numpy as np
from matplotlib import animation
from matplotlib import pyplot as plt


def convert_angle_to_0_2pi_interval(angle):
    new_angle = np.arctan2(np.sin(angle), np.cos(angle))
    if new_angle < 0:
        new_angle = abs(new_angle) + 2 * (np.pi - abs(new_angle))
    return new_angle


class MovingObject:

    def __init__(self):
        self.position = np.array([0, 0], dtype=float)
        self.heading = np.pi/8  # between 0 and pi; the turning angle. positive is counter clockwise
        self.odometry = np.random.rand(2).astype(float)
        self.odometry = self.odometry / np.linalg.norm(self.odometry) / 1000
        self.heading_update_prob = 0.1
        # self.heading_perturb_range = -np.pi / 8, np.pi / 8
        self.heading_options = self.heading + np.linspace(-np.pi/8, np.pi/8, 5, endpoint=True)

    def __str__(self):
        return f"MovingObject({self.position=}, {self.heading=}, {self.odometry=}, linear_velocity={np.linalg.norm(self.odometry)})"

    def propagate(self):
        # propagate the position
        self.position += self.odometry
        dx, dy = self.odometry
        # update the odometry with the heading
        self.odometry[0] = np.cos(self.heading * dx) - np.sin(self.heading * dy)
        self.odometry[1] = np.sin(self.heading * dx) + np.cos(self.heading * dy)

        # update the heading with some low randomness
        if np.random.rand() < self.heading_update_prob:
            # low, hi = self.heading_perturb_range
            self.heading = np.random.choice(self.heading_options)

        print("Propagate called", self)

    def get_state(self):
        pass


if __name__ == "__main__":
    # First set up the figure, the axis, and the plot element we want to animate
    fig = plt.figure()
    ax = plt.axes(xlim=(-1000, 1000), ylim=(-1000, 1000))
    line, = ax.plot([], [], lw=2)

    particle, = plt.plot([], [], marker='o', color='r')

    ship = MovingObject()

    # animation function.  This is called sequentially
    def animate(i):
        x, y = ship.position
        particle.set_data(x, y)
        ship.propagate()
        return particle

    # call the animator.  blit=True means only re-draw the parts that have changed.
    anim = animation.FuncAnimation(fig, animate, frames=100, interval=50)

    # save the animation as an mp4.  This requires ffmpeg or mencoder to be
    # installed.  The extra_args ensure that the x264 codec is used, so that
    # the video can be embedded in html5.  You may need to adjust this for
    # your system: for more information, see
    # http://matplotlib.sourceforge.net/api/animation_api.html
    anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

    plt.show()
