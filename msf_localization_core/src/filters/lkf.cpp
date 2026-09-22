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
LinearKalmanFilter::LinearKalmanFilter(const double& delta_t, const StateVector& x0, const StateTransitionMatrix& P0)
    : delta_t_(delta_t), x_(x0), P_(P0),
      A_(StateTransitionMatrix::Zero()), B_(ControlMatrix::Zero()),
      Q_(ProcessNoiseCovarianceMatrix::Zero()), K_(KalmanGainMatrix::Zero()), 
      C_(MeasurementMatrix::Zero()), R_(MeasurementCovarianceMatrix::Zero()) {

  std::cout << "\n\nLinear Kalman Filter Initialized!\n\n";
}

/**
 * @brief Prediction step for Linear Kalman Filter.
 *
 *   x_ = A_ * x_ +  B * u
 *
 *   P_ = A_ * P_ * A_^T + Q
 */
void LinearKalmanFilter::predict(const ControlVector &u) {

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
void LinearKalmanFilter::update(const MeasurementVector &y) {

  C_ << 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0;

  R_ << 2.0, 0, 0,
        0, 2.0, 0,
        0, 0, 4.0;
  
  K_ = P_ * C_.transpose() * ((C_ * P_ * C_.transpose()) + R_).inverse();

  x_ = x_ + (K_ * (y - C_ * x_));

  IdentityMatrix I = IdentityMatrix::Identity();

  P_ = (I - (K_ * C_)) * P_;
}

Filter::StateVector LinearKalmanFilter::get_state() const {
  return x_;
}

Filter::StateCovarianceMatrix LinearKalmanFilter::get_covariance() const {
  return P_;
}

} // namespace msf_localization_core