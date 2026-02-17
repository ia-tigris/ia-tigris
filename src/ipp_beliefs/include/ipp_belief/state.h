/**
 * @file state.hpp
 * @author Andrew Jong (ajong@andrew.cmu.edu)
 * @brief representations of object state and state distributions
 * @version 0.1
 * @date 2022-02-13
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef ipp_belief_STATE_H
#define ipp_belief_STATE_H

#define _USE_MATH_DEFINES

#include <cmath>
#include <memory>
#include <random>
#include <iostream>

#include "sampling_distributions.h"

namespace tracking
{
    /**
     * Represents Identifier, X,Y,Z (3d coordinates), heading, linear velocity, angular velocity, and probability
     *
     */
    class TargetState //: public XYState
    {
    protected:
        unsigned int id; // ID of the object
        double x, y, z, heading, linear_velocity, angular_velocity, log_probability;

        // give the LinearAngularController access to directly modify private
        // members
        friend class LinearAngularController;

    public:
        /**
         * @brief Construct a new TargetState object
         *
         * @param id target id that this state represents. several instances may represent the same
         * thing with the same id (e.g. in particle filter)
         * @param x
         * @param y
         * @param z
         * @param heading
         * @param linear_velocity
         * @param angular_velocity
         * @param probability
         */
        explicit TargetState(unsigned int id = 0, double x = 0, double y = 0,
                                double z = 0, double heading = 0,
                                double linear_velocity = 0, double angular_velocity = 0,
                                double probability = 1.0)
            : id(id),
              x(x),
              y(y),
              z(z),
              heading(heading),
              linear_velocity(linear_velocity),
              angular_velocity(angular_velocity),
              log_probability(std::log(probability))
        {
            //   printf("New TargetState id is %d\n", id);
        }

        unsigned int get_id() const { return id; }

        double get_x() const { return x; }

        double get_y() const { return y; }

        double get_z() const { return z; }

        double get_heading() const { return heading; }

        double get_speed() const { return linear_velocity; }

        double get_angular_velocity() const { return angular_velocity; }

        void set_angular_velocity(double angularVelocity) { angular_velocity = angularVelocity; }

        double get_log_probability() const { return log_probability; }

        double get_probability() const { return std::exp(log_probability); }

        void set_probability(double probability) { this->log_probability = std::log(probability); }

        void set_id(unsigned int id) { this->id = id; }

        /**
         * @brief Set the heading in radians
         *
         * @param heading
         */
        void set_heading(double heading) { this->heading = heading; }

        void set_speed(double speed) { this->linear_velocity = speed; }

        void set_x(double x) { this->x = x; }

        void set_y(double y) { this->y = y; }

        void set_z(double z) { this->z = z; }

        /**
         * Euclidean distance between the spatial coordinates, i.e. xyz
         *
         * @param other
         * @return double
         */
        double euclidean_distance(const TargetState &other) const
        {
            return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2) +
                             std::pow(z - other.z, 2));
        }
        friend std::ostream &operator<<(std::ostream &os, const TargetState &s)
        {
            os << "TargetState(id=" << s.id << ", x=" << s.x << ", y=" << s.y << ", z=" << s.z << ", h="
               << s.heading << ", lin=" << s.linear_velocity << ", ang=" << s.angular_velocity << ", p="
               << s.get_probability() << ")";
            return os;
        }
    };

    /* ===== CONCRETE STATE DISTRIBUTION CLASSES ====== */

    class TargetGaussian
    {
    public:
        unsigned int id;
        TargetState means;
        Eigen::VectorXd mean_vec;
        Eigen::Matrix<double, 5, 5> xyhla_covariance_mat;
        MultivariateGaussianSampler xyhla_sampler;

        // numerical stability, see https://juanitorduz.github.io/multivariate_normal/
        double epsilon = 0; // 0.0000001;

        explicit TargetGaussian(): id(-1), mean_vec(5), xyhla_covariance_mat(Eigen::Matrix<double, 5,5>::Identity()){
            xyhla_sampler = MultivariateGaussianSampler{mean_vec, xyhla_covariance_mat};
        }

        /**
         * @brief Construct a new TargetGaussian object
         *
         * @param means
         * @param xyhla_cov size 25 state vector for covariance between X, Y, Heading,
         * Linear velocity, and Angular velocity
         */
        explicit TargetGaussian(TargetState means,
                                   std::vector<double> xyhla_cov = {}): means(means)
        {
            if (xyhla_cov.empty())
            {
                xyhla_covariance_mat = Eigen::Matrix<double, 5, 5>::Identity();
                xyhla_covariance_mat(0, 0) = 0.2 * 0.2;
                xyhla_covariance_mat(1, 1) = 0.2 * 0.2;
                xyhla_covariance_mat(2, 2) = 0.05 * 0.05;
                xyhla_covariance_mat(3, 3) = 0.05 * 0.05;
                xyhla_covariance_mat(4, 4) = 0.05 * 0.05;
            }
            else
            {
                xyhla_covariance_mat =
                    Eigen::Map<Eigen::Matrix<double, 5, 5>>(xyhla_cov.data());
            }
            xyhla_covariance_mat += epsilon * Eigen::Matrix<double, 5, 5>::Identity();
            this->id = means.get_id();

            mean_vec = convertXyzhLaToEigenVector(means);
            xyhla_sampler = MultivariateGaussianSampler{mean_vec, xyhla_covariance_mat};
        }

        TargetGaussian(TargetState means,
                                   Eigen::Matrix<double, 5, 5> xyhla_cov)
        {
            xyhla_covariance_mat =
                xyhla_cov + epsilon * Eigen::Matrix<double, 5, 5>::Identity();
            this->id = means.get_id();

            mean_vec = convertXyzhLaToEigenVector(means);
            xyhla_sampler = MultivariateGaussianSampler{mean_vec, xyhla_covariance_mat};
        }

        Eigen::VectorXd convertXyzhLaToEigenVector(const TargetState &means) const
        {
            Eigen::VectorXd mean_vec(5);
            mean_vec << means.get_x(), means.get_y(), means.get_heading(),
                means.get_speed(), means.get_angular_velocity();
            return mean_vec;
        }

        TargetState sample_state()
        {
            auto xyhsw_vector = xyhla_sampler();
            auto x = xyhsw_vector(0);
            auto y = xyhsw_vector(1);
            auto z = 0.0;
            auto heading = xyhsw_vector(2);
            heading = std::fmod(heading, 2 * M_PI); // angle wrapping
            auto linear = xyhsw_vector(3);
            auto angular = xyhsw_vector(4) * 0.1; // TODO: this is a hack!! we had too much estimated angular velocity so we're turning it down

            // TODO: warning, I have no idea how to determine the actual sampling
            // probability. this is a placeholder
            double probability = means.get_probability();

            TargetState state(this->id, x, y, z, heading, linear, angular, probability);
            return state;
        }

        friend std::ostream &operator<<(std::ostream &os, const TargetGaussian &cov)
        {
            os << "TargetGaussian - Means: " << cov.mean_vec << "\nCovariance:\n"
               << cov.xyhla_covariance_mat;
            return os;
        }
        unsigned int get_id() const { return id; }
    };

} // namespace tracking
#endif // ipp_belief_STATE_H
