#include <random>
#include "inc/particle.h"

namespace qy {
namespace filter {
ParticleFilter::ParticleFilter(
    int numParticles, int dimension,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& propagate,
    const std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)>& evaluate,
    const std::vector<double>& lower_bound,
    const std::vector<double>& upper_bound
) : size_(numParticles), dimension_(dimension), 
    propagate_(propagate), evaluate_(evaluate)
{
    particles_.resize(numParticles);
    for (auto particle : particles_)
        particle.resize(dimension);
    importance_.resize(numParticles, 1.0 / numParticles);
    cdf_.resize(numParticles + 1);

    std::random_device rd;
    std::mt19937 mt(rd());
    std::vector<std::uniform_real_distribution<double>> urds;
    for (int i = 0; i < dimension; ++i)
        urds.push_back(std::uniform_real_distribution<double>(lower_bound[i], upper_bound[i]));

    for (int i = 0; i < numParticles; ++i) {
        particles_[i].resize(dimension);
        for (int j = 0; j < dimension; ++j)
            particles_[i][j] = urds[j](mt);
        cdf_[i] = i * 1.0 / numParticles;
    }
    cdf_[numParticles] = 1.0;
}

int ParticleFilter::sir() {
    
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> urd(0.0, 1.0);

    double rn = urd(mt);
    int left = 0;
    int right = size_;

    while (left + 1 < right) {
        int mid = (left + right) / 2;
        if (rn < cdf_[mid])
            right = mid;
        else 
            left = mid;
    }
    return left;

}

void ParticleFilter::update(const Eigen::VectorXd& z) {
    int p_idx;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> p_tmp(size_);
    double importance_sum = 0;

    for (int i = 0; i < size_; i++) {
	    p_idx = sir();
        p_tmp[i] = propagate_(particles_[p_idx]);
        importance_[i] = evaluate_(p_tmp[i], z);
        importance_sum += importance_[i];
    }

    if (importance_sum == 0.0) {
        throw std::runtime_error("none of the particles is valid.\n");
    }

    cdf_[0] = 0;
    for (int i = 0; i < size_; i++) {
	    importance_[i] /= importance_sum;
	    cdf_[i + 1] = cdf_[i] + importance_[i];
    }
    particles_ = p_tmp;
}

}
}