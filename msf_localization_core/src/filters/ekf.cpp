#include <iostream>

#include "filters/ekf.hpp"
#include "filters/filter.hpp"

namespace msf_localization_core 
{

  /**
   * @brief Constructor of Extended Kalman Filter.
   *
   * @param x0 initial state
   * @param P0 initial covariance matrix
   * @param delta_t sampling period
   */
ExtendedKalmanFilter::ExtendedKalmanFilter(const double& delta_t, const StateVector& x0, const StateTransitionMatrix& P0)
    : delta_t_(delta_t), x_(x0), P_(P0),
    f_(StateTransitionFunction::Zero()),
      F_(StateTransitionMatrix::Zero()),
      Q_(ProcessNoiseCovarianceMatrix::Zero()),K_(KalmanGainMatrix::Zero()), 
      g_(MeasurementFunction::Zero()), G_(MeasurementJacobian::Zero()),
      R_(MeasurementCovarianceMatrix::Zero()) {

  std::cout << "\n\nExtended Kalman Filter Initialized!\n\n";

}

/**
 * @brief Prediction step for Extended Kalman Filter.
 *
 *   x_ = f(x_,u)
 *
 *   P_ = F_ * P_ * F_^T + Q
 */
void ExtendedKalmanFilter::predict(const ControlVector &u)
{
    // define F matrix based on the current state and delta_t
    F_ << 1, 0, 0, 0, 0, 0, delta_t_, 0, 0,
          0, 1, 0, 0, 0, 0, 0, delta_t_, 0,
          0, 0, 1, 0, 0, 0, 0, 0, delta_t_,
          0, 0, 0, 1, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 1, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 1, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    // State transition model: x_ = f(x_, u)
    // For this example, we assume a simple linear state transition model
    x_ = f_ * x_;
    
    // Update the state covariance matrix
    P_ = F_ * P_ * F_.transpose() + Q_;
}


/**
 * @brief Correction step for Extended Kalman Filter.
 *
 *  K_ = P_ * C_^T * (C_ * P_ * C_^T + R_)^-1
 *
 *  x_ = x_ + K_ * (y_ - C_ * x_)
 *
 *  P_ = (1 - K_ * C_) * P_
 */
void ExtendedKalmanFilter::update(const MeasurementVector &y)
{
  G_ << 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 0;

  R_ << 2.0, 0, 0,
        0, 2.0, 0,
        0, 0, 4.0;

  // Compute the Kalman Gain
  K_ = P_ * G_.transpose() * (G_ * P_ * G_.transpose() + R_).inverse();
  
  // Update the state estimate
  x_ = x_ + K_ * (y - g_);
  
  IdentityMatrix I = IdentityMatrix::Identity();

  // Update the state covariance matrix
  P_ = (I - K_ * G_) * P_;
}

Filter::StateVector ExtendedKalmanFilter::get_state() const {
  return x_;
}

Filter::StateCovarianceMatrix ExtendedKalmanFilter::get_covariance() const {
  return P_;
}

}// namespace msf_localization_core