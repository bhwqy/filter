#include "inc/kalman.h"

namespace qy {
namespace filter {
KalmanFilter::KalmanFilter(
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& C,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R
) : A_(A), C_(C), Q_(Q), R_(R),
    m_(C.rows()), n_(A.rows()) {}

void KalmanFilter::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) {
    x_ = x0;
    P_ = P0;
}

void KalmanFilter::predict() {
    x_ = A_ * x_;
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd& z) {

    Eigen::MatrixXd K_ = P_ * C_.transpose() * (C_ * P_ * C_.transpose() + R_).inverse();
    x_ += K_ * (z - C_ * x_);
    P_ = (Eigen::MatrixXd::Identity(n_, n_) - K_ * C_) * P_;

}

}
}
