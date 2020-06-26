#ifndef QY_PARTICLE
#define QY_PARTICLE
#pragma once
#include <vector>
#include <Eigen/Dense>

namespace qy {
namespace filter {
class ParticleFilter {
public:
    ParticleFilter(
        int numParticles, int dimension,
        const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& propagate,
        const std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)>& evaluate,
        const std::vector<double>& lower_bound,
        const std::vector<double>& upper_bound
    );

    void init(const Eigen::VectorXd& x0);
    void update(const Eigen::VectorXd& z);
    int sir();
    Eigen::VectorXd state() const {
        Eigen::VectorXd ans;
        ans.resize(dimension_);
        ans.setZero();
        for (auto each : particles_)
            ans += each;
        ans /= static_cast<double>(particles_.size());
        return ans;
    }

private:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> particles_;
    std::vector<double> importance_;
    int size_, dimension_;
    std::vector<double> cdf_;

    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> propagate_;
    std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)> evaluate_;

};
}
}

#endif // QY_PARTICLE
