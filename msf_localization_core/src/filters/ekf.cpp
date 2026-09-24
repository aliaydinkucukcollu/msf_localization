#include <cmath>
#include <iostream>
#include <math.h>

#include "filters/ekf.hpp"
#include "filters/filter.hpp"

namespace msf_localization_core {

/**
 * @brief Constructor of Extended Kalman Filter.
 *
 * @param x0 initial state
 * @param P0 initial covariance matrix
 * @param delta_t sampling period
 */
ExtendedKalmanFilter::ExtendedKalmanFilter(const double &delta_t,
                                           const StateVector &x0,
                                           const StateTransitionMatrix &P0)
    : delta_t_(delta_t), x_(x0), P_(P0),
      f_(StateTransitionFunction::Identity()),
      F_(StateTransitionMatrix::Identity()),
      Q_(ProcessNoiseCovarianceMatrix::Zero()),
      K_(KalmanGainMatrix::Zero()),
      g_(MeasurementFunction::Zero()),
      G_(MeasurementJacobian::Zero()),
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
void ExtendedKalmanFilter::predict(const ControlVector &u) {
  // get current state variables
  double roll = x_(3);  // rad
  double pitch = x_(4); // rad
  double yaw = x_(5);   // rad
  double vx = x_(6);
  double vy = x_(7);
  double vz = x_(8);

  // get control variables
  // linear acceleration in m/s^2
  double ax = u(0);
  double ay = u(1);
  double az = u(2);
  // angular velocity should be in rad/sec
  double wx = u(3);
  double wy = u(4);
  double wz = u(5);

  const double sr = std::sin(roll);
  const double cr = std::cos(roll);

  const double sp = std::sin(pitch);
  const double cp = std::cos(pitch);
  const double tp = std::tan(pitch);

  const double sy = std::sin(yaw);
  const double cy = std::cos(yaw);

  f_(0, 6) = cy * cp * delta_t_;
  f_(0, 7) = (sr * sp * cy - sy * cr) * delta_t_;
  f_(0, 8) = (sr * sy + sp * cr * cy) * delta_t_;
  f_(1, 6) = sy * cp * delta_t_;
  f_(1, 7) = (sr * sp * sy + cr * cy) * delta_t_;
  f_(1, 8) = (-sr * cy + sy * sp * cr) * delta_t_;
  f_(2, 6) = -sp * delta_t_;
  f_(2, 7) = sr * cp * delta_t_;
  f_(2, 8) = cr * cp * delta_t_;

  F_(0, 3) = (vy * (sr * sy + sp * cr * cy) + vz * (-sr * sp * cy + sy * cp)) * delta_t_ +
             (ay * (sr * sy + sp * cr * cy) + az * (- sr * sp * cy + sy * cr)) * delta_t_ * delta_t_ * 0.5;
  F_(0, 4) = (-vx * sp * cy + vy * sr * cy * cp + vz * cr * cy * cp) * delta_t_ +
             (-ax * sp * cy + ay * sr * cy * cp + az * cr * cy * cp) * delta_t_ * delta_t_ * 0.5;
  F_(0, 5) = (-vx * sy * cp + vy * (-sr*sy*sp-cr*cy) + vz*(sr * cy - sy * sp * cr)) * delta_t_ +
             (-ax * sy * cp + ay * (-sr * sy * sp - cr * cy) + az * (sr * cy - sy * sp * cr)) * delta_t_ * delta_t_ * 0.5;
  F_(0, 6) = cy * cp * delta_t_;
  F_(0, 7) = (sr * sp * cy - sy * cr) * delta_t_;
  F_(0, 8) = (sr * sy + sp * cr * cy) * delta_t_;


  F_(1, 3) = (vy * (-sr * cy + sy * sp * cr) + vz * (-sr * sy * sp - cr * cy)) * delta_t_ + 
             (ay * (- sr * cy + sy * sp * cr) + az * (- sr * sy * sp - cr * cy)) * delta_t_ * delta_t_ * 0.5;
  F_(1, 4) = (-vx * sy * sp + vy * sr * sy * cp + vz * sy * cr * cp) * delta_t_ + 
             (-ax * sy * sp + ay * sr * sy * cp + az * sy * cr * cp) * delta_t_ * delta_t_ * 0.5;
  F_(1, 5) = (vx * cy * cp + vy * (sr * sp * cy - sy * cr) + vz * (sr * sy + sp * cr * cy)) * delta_t_ + 
             (ax * cy * cp + ay * (sr * sp * cy - sy * cr) + az * (sr * sy + sp * sr * cy)) * delta_t_ * delta_t_ * 0.5;
  F_(1, 6) = sy * cp * delta_t_;
  F_(1, 7) = (sr * sy * sp + cr * cy) * delta_t_;
  F_(1, 8) = (-sr * cy + sy * sp * cr) * delta_t_;


  F_(2, 3) = (vy * cr * cp - vz * sr * cp) * delta_t_ +
             (ay * cr * cp - az * sr * cp) * delta_t_ * delta_t_ * 0.5;
  F_(2, 4) = (-vx * cp - vy * sr * sp - vz * sp * cr) * delta_t_ +
             (-ax * cp - ay * sr * sp - az * sp * cr) * delta_t_ * delta_t_ * 0.5;
  F_(2, 6) = -sp * delta_t_;
  F_(2, 7) = sr * cp * delta_t_;
  F_(2, 8) = cr * cp * delta_t_;

  F_(3, 3) = (wy * cr * tp - wz * sr * tp) * delta_t_ + 1;
  F_(3, 4) = (wy * (std::pow(tp, 2) + 1) * sr + wz * (std::pow(tp, 2) + 1) * cr) * delta_t_;


  F_(4, 3) = (-wy * sr - wz * cr) * delta_t_;

  F_(5, 3) = (wy * (cr / cp) - wz * (sr / cp)) * delta_t_;
  F_(5, 4) = (wy * sr * sp + wz * sp * cr / std::pow(cp, 2)) * delta_t_;

  Q_ << 0.01, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0.01, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0.01, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0.005, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0.005, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0.005, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0.1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0.1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0.1;

  // calculate state
  x_ = f_ * x_; // x_ = f(x_, u)

  // Update the state covariance matrix
  P_ = F_ * P_ * F_.transpose() + Q_;
}

/**
 * @brief Correction step for Extended Kalman Filter.
 *
 *  K_ = P_ * G_^T * (G_ * P_ * G_^T + R_)^-1
 *
 *  x_ = x_ + K_ * (y_ - g(x_))
 *
 *  P_ = (1 - K_ * G_) * P_
 */
void ExtendedKalmanFilter::update(const MeasurementVector &y) {
  G_ << 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0;

  R_ << 2.0, 0, 0,
        0, 2.0, 0,
        0, 0, 4.0;

  // Compute the Kalman Gain
  K_ = P_ * G_.transpose() * (G_ * P_ * G_.transpose() + R_).inverse();

  // Update the state estimate
  x_ = x_ + K_ * (y - G_ * x_);

  IdentityMatrix I = IdentityMatrix::Identity();

  // Update the state covariance matrix
  P_ = (I - K_ * G_) * P_;
}

Filter::StateVector ExtendedKalmanFilter::get_state() const { return x_; }

Filter::StateCovarianceMatrix ExtendedKalmanFilter::get_covariance() const {
  return P_;
}

} // namespace msf_localization_core