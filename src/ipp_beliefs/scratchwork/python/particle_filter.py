import numpy as np


def make_normal_distribution_fn(mean, std):
    def normal_distribution_fn(num):
        return std * np.random.randn(num) + mean

    return normal_distribution_fn


class ParticleFilter:
    def __init__(
        self,
        init_state,
        init_state_covariance,
        # controls
        acceleration_sampler=make_normal_distribution_fn(0, 0.0001),
        angular_acceleration_sampler=make_normal_distribution_fn(0, np.pi / 64),
        num_particles=1000,
    ):
        """

        Args:
            init_state: 4d state vector [x, y, speed, heading]
            init_state_covariance:
            acceleration_mean:
            acceleration_std:
            angular_acceleration_mean:
            angular_acceleration_std:
            num_particles:
        """
        if init_state.shape == (4,):
            init_state = np.expand_dims(init_state, axis=0)  # make it a 1x4 array
        assert init_state.shape == (
            1,
            4,
        ), f"state must be a 1x4 array, got {init_state.shape}"
        self.init_state = init_state
        self.init_state_covariance = init_state_covariance

        self.num_particles = num_particles

        self.particles = np.repeat(self.init_state, self.num_particles, axis=0)
        self.expected_state = self.init_state  # np.mean(self.particles, axis=0)

        self.acceleration_sampler = acceleration_sampler
        self.angular_acceleration_sampler = angular_acceleration_sampler

    def reinit_state(self, init_state):
        self.init_state = init_state
        self.particles = np.repeat(self.init_state, self.num_particles, axis=0)
        self.expected_state = self.init_state  # np.mean(self.particles, axis=0)

    def propagate(self, delta_time):
        """
        Samples control inputs [accleration, angular acceleration] for each particle, and applies that control input
        respectively
        Args:
            delta_time:

        """
        # propagate particles
        accelerations = self.acceleration_sampler(self.num_particles)
        angular_accelerations = self.angular_acceleration_sampler(self.num_particles)

        xs, ys, speeds, headings = self.particles.T
        new_xs = xs + np.cos(headings) * (
            speeds * delta_time + 0.5 * accelerations * delta_time ** 2
        )
        new_ys = ys + np.sin(headings) * (
            speeds * delta_time + 0.5 * accelerations * delta_time ** 2
        )
        new_speeds = speeds + accelerations * delta_time
        new_headings = headings + angular_accelerations * delta_time

        self.particles = np.column_stack((new_xs, new_ys, new_speeds, new_headings))
        self.expected_state = np.mean(self.particles, axis=0)

    def reset_state(
        self,
        init_state=None,
        init_state_covariance=None,
        # controls
        acceleration_mean=None,
        acceleration_std=None,
        angular_acceleration_mean=None,
        angular_acceleration_std=None,
        num_particles=None,
    ):
        self.init_state = init_state if init_state is not None else self.init_state
        self.init_state_covariance = (
            init_state_covariance
            if init_state_covariance is not None
            else self.init_state_covariance
        )
        self.acceleration_mean = (
            acceleration_mean
            if acceleration_mean is not None
            else self.acceleration_mean
        )
        self.acceleration_std = (
            acceleration_std if acceleration_std is not None else self.acceleration_std
        )
        self.angular_acceleration_mean = (
            angular_acceleration_mean
            if angular_acceleration_mean is not None
            else self.angular_acceleration_mean
        )
        self.angular_acceleration_std = (
            angular_acceleration_std
            if angular_acceleration_std is not None
            else self.angular_acceleration_std
        )
        self.num_particles = (
            num_particles if num_particles is not None else self.num_particles
        )

if __name__ == "__main__":
    init_state = np.array([0, 0, 0.5, 0])

    pf = ParticleFilter(
        init_state=init_state,
        init_state_covariance=None,
    )
    pf.propagate(1)
