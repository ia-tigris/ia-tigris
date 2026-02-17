//
// Created by andrew on 2/24/22.
//

#ifndef ipp_belief_DISTRIBUTIONS_H
#define ipp_belief_DISTRIBUTIONS_H

#include <Eigen/Dense>
#include <random>
#include <utility>


// from https://stackoverflow.com/a/40245513/5118517
struct MultivariateGaussianSampler
{
    Eigen::VectorXd mean;
    Eigen::MatrixXd transform;
    
    MultivariateGaussianSampler() = default;
    explicit MultivariateGaussianSampler(const Eigen::MatrixXd &covar)
            : MultivariateGaussianSampler(Eigen::VectorXd::Zero(covar.rows()), covar)
    {}

    MultivariateGaussianSampler(const Eigen::VectorXd  &mean, const Eigen::MatrixXd &covar)
            : mean(mean)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
        auto eigenValues = eigenSolver.eigenvalues();
        if (eigenValues.minCoeff() < 0)
        {
            printf("ERROR ERROR ERROR: Covariance matrix is not positive semi-definite\n");
            std::stringstream ss;
            ss << "Covariance matrix is not positive semi-definite:\n" << covar << std::endl;
            ss << "Eigen values are supposed to be all non-negative, but some are not:\n" << eigenValues << std::endl;
            throw std::runtime_error(ss.str());
        }
        transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }


    Eigen::VectorXd operator()() const
    {
        static std::mt19937 gen{ std::random_device{}() };
        static std::normal_distribution<> dist;

        const Eigen::VectorXd &xd =
                mean + transform * Eigen::VectorXd{mean.size()}.unaryExpr([&](auto x) { return dist(gen); });
        return xd;
    }
};


#endif //ipp_belief_DISTRIBUTIONS_H
