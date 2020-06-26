#ifndef QY_KALMAN
#define QY_KALMAN
#pragma once
#include <Eigen/Dense>

namespace qy {
namespace filter {
class KalmanFilter {
public:
    KalmanFilter() = default;
    
    KalmanFilter(
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& C,
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

    Eigen::MatrixXd A_, C_, Q_, R_;
    int m_, n_;

    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
};
}
}

#endif // QY_KALMAN
