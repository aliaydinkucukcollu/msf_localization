#include <iostream>

#include "filters/filter.hpp"
#include "filters/lkf.hpp"

namespace msf_localization_core {

  /**
   * @brief Constructor of Linear Kalman Filter.
   *
   * @param x0 initial state
   * @param P0 initial covariance matrix
   * @param delta_t sampling period
   */
LinearKalmanFilter::LinearKalmanFilter(const double& delta_t, const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0)
    : delta_t_(delta_t), x_(x0), P_(P0),
      A_(state_size_, state_size_), B_(state_size_, input_size_),
      Q_(state_size_, state_size_),K_(state_size_, measurement_size_), 
      C_(measurement_size_, state_size_), R_(measurement_size_, measurement_size_) {

  std::cout << "\n\nLinear Kalman Filter Initialized!\n\n";

  std::cout << "\nInitial state: \n";
  std::cout << x_ << '\n';

  std::cout << "\nInitial covariance: \n";
  std::cout << P_ << '\n';

  A_.setZero();
  B_.setZero();
  Q_.setZero();
  K_.setZero();
  C_.setZero();
  R_.setZero();

}

/**
 * @brief Prediction step for Linear Kalman Filter.
 *
 *   x_ = A_ * x_ +  B * u
 *
 *   P_ = A_ * P_ * A_^T + Q
 */
void LinearKalmanFilter::predict(const Eigen::VectorXd &u) {

  A_ << 1, 0, 0, 0, 0, 0, delta_t_, 0, 0,
        0, 1, 0, 0, 0, 0, 0, delta_t_, 0,
        0, 0, 1, 0, 0, 0, 0, 0, delta_t_,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;
  
  B_ << 0.5 * delta_t_ * delta_t_, 0, 0, 0, 0, 0,
        0, 0.5 * delta_t_ * delta_t_, 0, 0, 0, 0,
        0, 0, 0.5 * delta_t_ * delta_t_, 0, 0, 0,
        0, 0, 0, delta_t_, 0, 0,
        0, 0, 0, 0, delta_t_, 0,
        0, 0, 0, 0, 0, delta_t_,
        delta_t_, 0, 0, 0, 0, 0,
        0, delta_t_, 0, 0, 0, 0,
        0, 0, delta_t_, 0, 0, 0;

  Q_ << 0.01, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0.01, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0.01, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0.005, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0.005, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0.005, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0.1;

  x_ = A_ * x_ + B_ * u;

  P_ = (A_ * P_ * A_.transpose()) + Q_;
}

/**
 * @brief Update step for Linear Kalman Filter.
 *
 *  K_ = P_ * C_^T * (C_ * P_ * C_^T + R_)^-1
 *
 *  x_ = x_ + K_ * (y_ - C_ * x_)
 *
 *  P_ = (1 - K_ * C_) * P_
 */
void LinearKalmanFilter::update(const Eigen::VectorXd &y) {

  C_ << 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0;

  R_ << 2.0, 0, 0,
        0, 2.0, 0,
        0, 0, 4.0;
  
  K_ = P_ * C_.transpose() * ((C_ * P_ * C_.transpose()) + R_).inverse();

  x_ = x_ + (K_ * (y - C_ * x_));

  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_size_, state_size_);

  P_ = (I - (K_ * C_)) * P_;
}

Eigen::VectorXd LinearKalmanFilter::get_state() const {
  return x_;
}

Eigen::MatrixXd LinearKalmanFilter::get_covariance() const {
  return P_;
}

} // namespace msf_localization_core