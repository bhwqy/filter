#include "inc/ekf.h"

namespace qy {
namespace filter {

ExtendedKalmanFilter::ExtendedKalmanFilter(
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& f,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& h,
    const Eigen::MatrixXd& F,
    const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R
) : f_(f), h_(h), F_(F), H_(H), Q_(Q), R_(R) {}

void ExtendedKalmanFilter::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) {
    x_ = x0;
    P_ = P0;
}

void ExtendedKalmanFilter::predict() {
    x_ = f_(x_);
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void ExtendedKalmanFilter::update(const Eigen::VectorXd& z) {
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + K * (z - h_(x_));
    P_ = (Eigen::MatrixXd::Identity(F_.rows(), F_.rows()) - K * H_) * P_;
}

}
}