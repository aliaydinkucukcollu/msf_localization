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

  /**
  * @brief Virtual destructor.
  */
  virtual ~Filter() = default;

  /**
   * @brief Performs prediction step for Kalman Filters.
   *
   * @param imu IMU measurement.
   */
  virtual void predict(const Eigen::VectorXd &u) = 0;

  /**
   * @brief Performs update step for Bayes Filters.
   *
   * @param gnss GNSS position measurement.
   */
  virtual void update(const Eigen::VectorXd &y) = 0;

  /**
   * @brief Returns current state vector.
   */
  virtual Eigen::VectorXd get_state() const = 0;

  /**
   * @brief Returns current state covariance matrix.
   */
  virtual Eigen::MatrixXd get_covariance() const = 0;

  static constexpr std::size_t state_size_ = 9;
  static constexpr std::size_t input_size_ = 6;
  static constexpr std::size_t measurement_size_ = 3;
};
} // namespace msf_localization_core