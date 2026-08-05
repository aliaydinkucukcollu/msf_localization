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
  explicit LinearKalmanFilter(const double& delta_t, const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);

  /**
   * @brief Virtual destructor.
   */
  ~LinearKalmanFilter() override = default;

  /**
   * @brief Performs prediction step for Kalman Filters.
   *
   * @param imu IMU measurement.
   */
  void predict(const Eigen::VectorXd &u) override;

  /**
   * @brief Performs update step for Bayes Filters.
   *
   * @param gnss GNSS position measurement.
   */
  void update(const Eigen::VectorXd &y) override;

  /**
   * @brief Returns current state vector.
   */
  Eigen::VectorXd get_state() const override;

  /**
   * @brief Returns current state covariance matrix.
   */
  Eigen::MatrixXd get_covariance() const override;

private:
  /// delta_t
  double delta_t_;

  /// Current estimated state
  Eigen::VectorXd x_;

  /// State covariance matrix
  Eigen::MatrixXd P_;

  /// State transition matrix
  Eigen::MatrixXd A_;

  /// Control matrix
  Eigen::MatrixXd B_;

  /// Process noise covariance
  Eigen::MatrixXd Q_;

  /// Kalman Gain
  Eigen::MatrixXd K_;

  /// Measurement matrix
  Eigen::MatrixXd C_;

  /// Measurement covariance matrix
  Eigen::MatrixXd R_;
};
} // namespace msf_localization_core