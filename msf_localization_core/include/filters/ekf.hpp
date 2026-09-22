#pragma once

#include "filter.hpp"

namespace msf_localization_core {

/**
 * @brief Extended Kalman Filter for State Estimation
 *
 * This class estimates current state via using IMU measurements and GNSS
 * measurements with nonlinear modeling.
 */
class ExtendedKalmanFilter : public Filter {
public:
  /**
   * @brief Constructor of Extended Kalman Filter.
   *
   * @param x0 initial state
   * @param P0 initial covariance matrix
   * @param delta_t sampling period
   */
  explicit ExtendedKalmanFilter(const double& delta_t, const StateVector& x0, const StateCovarianceMatrix& P0);


  /**
   * @brief Virtual destructor.
   */
  ~ExtendedKalmanFilter() override = default;

  /**
   * @brief Performs prediction step for Extended Kalman Filter.
   *
   * @param u control input
   */
  void predict(const ControlVector &u) override;

  /**
   * @brief Performs update step for Extended Kalman Filter.
   *
   * @param y measurement input
   */
  void update(const MeasurementVector &y) override;

  /**
   * @brief Returns current state vector.
   */
  Filter::StateVector get_state() const override;

  /**
   * @brief Returns current state covariance matrix.
   */
  Filter::StateCovarianceMatrix get_covariance() const override;

private:

  /// delta_t
  const double delta_t_;

  /// Current estimated state 9-DoF [x, y, z, roll, pitch, yaw, vx, vy, vz]^T
  StateVector x_;

  /// State covariance matrix
  StateCovarianceMatrix P_;

  /// Nonlinear State Transition Function
  StateTransitionFunction f_;

  /// ∂f/∂x
  StateTransitionJacobian F_;

  /// Process noise covariance
  ProcessNoiseCovarianceMatrix Q_;

  /// Kalman Gain
  KalmanGainMatrix K_;

  /// Measurement Function
  MeasurementFunction g_;

  /// Measurement Jacobian
  MeasurementJacobian G_;

  /// Measurement covariance matrix
  MeasurementCovarianceMatrix R_;

};

} // namespace msf_localization_core