#include "inc/particle.h"
#include <cmath>
#include <fstream>
#include <random>
using namespace qy::filter;

int main() {

    int num_particles = 500;
    int pos_error = 5;
    int vel_error = 5;
    int m_stdev = 5;

    double max_velocity = 30;

    std::vector<double> lower{ 280, 80, -max_velocity, -max_velocity };
    std::vector<double> upper{ 320, 120, max_velocity, max_velocity };

    std::random_device rd;
    std::mt19937 mt(rd());
    std::normal_distribution<double> pos_nd(0, pos_error);
    std::normal_distribution<double> vel_nd(0, vel_error);

    auto propogate = [&](const Eigen::VectorXd& a) -> Eigen::VectorXd {
        Eigen::VectorXd ans(a.rows());
        ans(0) = a(0) + a(2) + pos_nd(mt);
	    ans(1) = a(1) + a(3) + pos_nd(mt);
	    ans(2) = a(2) + vel_nd(mt);
	    ans(3) = a(3) + vel_nd(mt);
        return ans;
    };

    auto evaluate = [&](const Eigen::VectorXd& a, const Eigen::VectorXd& z) -> double {
        static double alpha = 1.0 / std::sqrt(2 * M_PI * m_stdev * m_stdev);
        static double beta = 2 * m_stdev * m_stdev;
        return alpha * std::exp(-(a(0) - z(0)) * (a(0) - z(0)) / beta) *
            alpha * std::exp(-(a(1) - z(1)) * (a(1) - z(1)) / beta);
    };

    ParticleFilter pf(num_particles, 4, propogate, evaluate, lower, upper);
    
    std::ifstream fs("circle.noise.txt");
    std::string s;
    while (std::getline(fs, s)) {

        int space = 0;
        for (; space < s.size(); ++space)
            if (s[space] == ' ')
                break;
        double x = std::stod(s.substr(0, space));
        double y = std::stod(s.substr(space + 1));
        Eigen::VectorXd observation;
        observation.resize(2);
        observation << x, y;
        pf.update(observation);

    }
    return 0;
}
