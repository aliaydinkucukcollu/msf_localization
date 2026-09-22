#pragma once

#include "filter.hpp"

namespace msf_localization_core {

/**
 * @brief Kalman Filter (Linear) for state estimation.
 *
 * This class estimates current state using IMU measurements for prediction and
 * GNSS measurements for prediction via assuming the model as linear.
 */
class LinearKalmanFilter : public Filter {
public:
  /**
   * @brief Constructor of Linear Kalman Filter.
   *
   * @param x0 initial state
   * @param P0 initial covariance matrix
   * @param delta_t sampling period
   */
  explicit LinearKalmanFilter(const double& delta_t, const StateVector& x0, const StateCovarianceMatrix& P0);

  /**
   * @brief Virtual destructor.
   */
  ~LinearKalmanFilter() override = default;

  /**
   * @brief Performs prediction step for Kalman Filters.
   *
   * @param imu IMU measurement 6-DoF [ax, ay, az, wx, wy, wz]^T
   */
  void predict(const ControlVector &u) override;

  /**
   * @brief Performs update step for Bayes Filters.
   *
   * @param gnss GNSS position measurement 3-DoF [px, py, pz]^T
   */
  void update(const MeasurementVector &y) override;

  /**
   * @brief Returns current state vector.
   */
  StateVector get_state() const override;

  /**
   * @brief Returns current state covariance matrix.
   */
  StateCovarianceMatrix get_covariance() const override;

private:
  /// delta_t
  const double delta_t_;

  /// Current estimated state 9-DoF [x, y, z, roll, pitch, yaw, vx, vy, vz]^T
  StateVector x_;

  /// State covariance matrix
  StateCovarianceMatrix P_;

  /// State transition matrix
  StateTransitionMatrix A_;

  /// Control matrix
  ControlMatrix B_;

  /// Process noise covariance
  ProcessNoiseCovarianceMatrix Q_;

  /// Kalman Gain
  KalmanGainMatrix K_;

  /// Measurement matrix
  MeasurementMatrix C_;

  /// Measurement covariance matrix
  MeasurementCovarianceMatrix R_;
};
} // namespace msf_localization_core