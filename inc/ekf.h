#ifndef QY_EKF
#define QT_EKF
#pragma once
#include <functional>
#include <Eigen/Dense>

namespace qy {
namespace filter {

class ExtendedKalmanFilter {
public:

    ExtendedKalmanFilter(
        const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& f,
        const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& h,
        const Eigen::MatrixXd& F,
        const Eigen::MatrixXd& H,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R
    );

    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);
    void update(const Eigen::VectorXd& z);
    void predict();
    
    Eigen::VectorXd state() const { return x_; }
    Eigen::MatrixXd state_cov() const { return P_; }


private:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Eigen::MatrixXd F_, H_, Q_, R_;
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> f_, h_;

    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;

};
}
}

#endif // QY_EKF