#pragma once

#include <Eigen/Dense>

namespace msf_localization_core {

/**
 * @brief Abstract class for Kalman Filters used for state estimation.
 *
 *
 * @note This class is independent of ROS2 and depends on Eigen for matrix
 * operations and GeographicLib for Geodetic to Geocentric conversions vice
 * versa.
 */
class Filter {
public:
  static constexpr int state_size_ = 9;
  static constexpr int control_size_ = 6;
  static constexpr int measurement_size_ = 3;

  using StateVector = Eigen::Matrix<double, state_size_, 1>;
  using ControlVector = Eigen::Matrix<double, control_size_, 1>;
  using MeasurementVector = Eigen::Matrix<double, measurement_size_, 1>;

  using StateTransitionMatrix = Eigen::Matrix<double, state_size_, state_size_>;
  using ControlMatrix = Eigen::Matrix<double, state_size_, control_size_>;
  using StateCovarianceMatrix = Eigen::Matrix<double, state_size_, state_size_>;
  using ProcessNoiseCovarianceMatrix = Eigen::Matrix<double, state_size_, state_size_>;

  using StateTransitionFunction = Eigen::Matrix<double, state_size_, state_size_>;
  using StateTransitionJacobian = Eigen::Matrix<double, state_size_, state_size_>;

  using MeasurementFunction = Eigen::Matrix<double, measurement_size_, state_size_>;
  using MeasurementJacobian = Eigen::Matrix<double, measurement_size_, state_size_>;

  using KalmanGainMatrix = Eigen::Matrix<double, state_size_, measurement_size_>;

  using MeasurementMatrix = Eigen::Matrix<double, measurement_size_, state_size_>;
  using MeasurementCovarianceMatrix = Eigen::Matrix<double, measurement_size_, measurement_size_>;

  using IdentityMatrix = Eigen::Matrix<double, state_size_, state_size_>;

  /**
   * @brief Virtual destructor.
   */
  virtual ~Filter() = default;

  /**
   * @brief Performs prediction step for Kalman Filters.
   *
   * @param imu IMU measurement.
   */
  virtual void predict(const ControlVector &u) = 0;

  /**
   * @brief Performs update step for Bayes Filters.
   *
   * @param gnss GNSS position measurement.
   */
  virtual void update(const MeasurementVector &y) = 0;

  /**
   * @brief Returns current state vector.
   */
  virtual StateVector get_state() const = 0;

  /**
   * @brief Returns current state covariance matrix.
   */
  virtual StateCovarianceMatrix get_covariance() const = 0;
};
} // namespace msf_localization_core