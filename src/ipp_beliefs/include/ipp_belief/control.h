//
// Created by andrew on 12/17/21.
//

#ifndef ipp_belief_CONTROL_H
#define ipp_belief_CONTROL_H

#include "state.h"

namespace tracking {
/**
 * Abstract Base Class of a Controller. Applies control input to a state of
 * type StateSpace
 * @tparam StateSpace the state class
 */

class LinearAngularController {
protected:

    std::normal_distribution<double> linear_acceleration_distribution;
    std::normal_distribution<double> angular_velocity_distribution;

    double linear_acc_mean, linear_acc_std, angular_velocity_mean, angular_velocity_std;

public:
    // https://stackoverflow.com/a/38245134
    std::random_device rd;  // random device class instance, source of 'true' randomness
    // for initializing random seed
    std::mt19937 gen;  // Mersenne twister PRNG, initialized with seed from previous
    // random device instance

    // public:
    /**
     * @brief Construct a new Linear Angular Acceleration Controller object
     *
     * @param linear_acceleration_mean_ linear acceleration
     * @param angular_velocity_mean angular velocity
     * @param linear_acceleration_std_ standard deviation in linear acceleration
     * @param angular_velocity_std standard deviation in angular acceleration
     */
    explicit LinearAngularController(
            double linear_acceleration_mean_ = 0,
            double angular_velocity_mean = 0,
            double linear_acceleration_std_ = 0.05,
            double angular_velocity_std = 0.05)
            :
            gen(std::mt19937(rd())),
            linear_acc_mean(linear_acceleration_mean_),
            angular_velocity_mean(angular_velocity_mean),
            linear_acc_std(linear_acceleration_std_),
            angular_velocity_std(angular_velocity_std) {
        linear_acceleration_distribution =
                std::normal_distribution<double>(linear_acc_mean, linear_acc_std);
        angular_velocity_distribution =
                std::normal_distribution<double>(angular_velocity_mean, angular_velocity_std);
    }

    // copy constructor
    LinearAngularController(const LinearAngularController &other) {
        linear_acceleration_distribution = other.linear_acceleration_distribution;
        angular_velocity_distribution = other.angular_velocity_distribution;
        linear_acc_mean = other.linear_acc_mean;
        linear_acc_std = other.linear_acc_std;
        angular_velocity_mean = other.angular_velocity_mean;
        angular_velocity_std = other.angular_velocity_std;
        gen = other.gen;
    }


    /** Samples a linear acceleration and an angular acceleration from the
     * distributions. Applies these accelerations to the current state and modifies the
     * state in place.
     * @param state the state to apply the control to
     * @param delta_time the time step
     */
    void apply_control_origin(TargetState &state, double delta_time) {
        // printf("Applying control\n");
        // linear
        /*
        vx = math.cos(self.heading) * self.linear_speed
        vy = math.sin(self.heading) * self.linear_speed
        self.x += vx * del_t
        self.y += vy * del_t
        self.heading += self.angular_speed * del_t
        */
        // NOTE AJ 2022.09.05: disabled random control becasue differing number of sampled control propagation steps build up differences. 
        // less steps would always underestimate the variance growth, leading to bias to not go deep. we can figure out how 
        // to normalize this later after ICRA. or maybe random control won't be needed at all
        double linear_acc = linear_acceleration_distribution(gen); 
        // double linear_acc = 0; 
        state.x += std::cos(state.heading) * (state.linear_velocity * delta_time + 0.5 *  linear_acc * delta_time * delta_time);
        state.y += std::sin(state.heading) * (state.linear_velocity * delta_time + 0.5 * linear_acc * delta_time * delta_time);
        // state.linear_velocity += linear_acc * delta_time;
        // angular
        // NOTE AJ 2022.09.05: disabled random control becasue differing number of sampled control propagation steps build up differences. 
        // less steps would always underestimate the variance growth, leading to bias to not go deep. we can figure out how 
        // to normalize this later after ICRA. or maybe random control won't be needed at all
        double new_angular_velocity = angular_velocity_distribution(gen); 
        // // take a weighted average of the angulars. Prefer the new angular
        double wt_avg_ang_vel = (2 * new_angular_velocity + state.angular_velocity) / 3;
        state.heading += wt_avg_ang_vel * delta_time;

        // Note AJ 2022.09.05: disabled angular velocity becuase our intersection code is linear
        state.heading += state.angular_velocity * delta_time;

        // printf("Sampled linear acc=%f, angular_acc=%f\n", linear_acc, angular_acc);
        // printf("x=%f, y=%f", state.x, state.y);
    }

    void apply_control(TargetState& state, double delta_time)
    {
        double vx = state.linear_velocity * std::cos(state.heading);
        double vy = state.linear_velocity * std::sin(state.heading);
        double x_acc = linear_acceleration_distribution(gen) / 1.414;
        double y_acc = linear_acceleration_distribution(gen) / 1.414;
        state.x += vx * delta_time + 0.5 * x_acc * delta_time * delta_time;
        state.y += vy * delta_time + 0.5 * y_acc * delta_time * delta_time;
        double new_vx = vx + x_acc * delta_time;
        double new_vy = vy + y_acc * delta_time;
        // state.linear_velocity = std::sqrt(new_vx * new_vx + new_vy + new_vy);

        double new_angular_velocity = angular_velocity_distribution(gen);
        double wt_avg_ang_vel = (2 * new_angular_velocity + state.angular_velocity) / 3;
        state.heading += wt_avg_ang_vel * delta_time;
        // state.angular_velocity = new_angular_velocity;
    }

    double sample_linear_acceleration() {
        return linear_acceleration_distribution(gen);
    }

    double sample_angular_velocity() {
        return angular_velocity_distribution(gen);
    }

    double get_linear_acc_mean() const { return linear_acc_mean; }

    double get_angular_vel_mean() const { return angular_velocity_mean; }

    double get_linear_acc_std() const { return linear_acc_std; }

    double get_angular_vel_std() const { return angular_velocity_std; }

    // LinearAngularController *clone() const override {
    //     return new LinearAngularController(this->linear_acc_mean,
    //                                                       this->angular_velocity_mean,
    //                                                       this->linear_acc_std,
    //                                                       this->angular_velocity_std);
    // }

    LinearAngularController& operator=(const LinearAngularController& other) {
        linear_acc_mean = other.linear_acc_mean;
        angular_velocity_mean = other.angular_velocity_mean;
        linear_acc_std = other.linear_acc_std;
        angular_velocity_std = other.angular_velocity_std;
        linear_acceleration_distribution = other.linear_acceleration_distribution;
        angular_velocity_distribution = other.angular_velocity_distribution;
        return *this;
    }

    // outstream operator
    friend std::ostream &operator<<(std::ostream &os, const LinearAngularController &controller) {
        os << "LinearAngularController("
           << "linear_acc_mean=" << controller.linear_acc_mean
           << ", angular_velocity_mean=" << controller.angular_velocity_mean
           << ", linear_acc_std=" << controller.linear_acc_std
           << ", angular_velocity_std=" << controller.angular_velocity_std
           << ")";
        return os;
    }
};
} // namespace tracking

#endif  // ipp_belief_CONTROL_H
