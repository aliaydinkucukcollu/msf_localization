#pragma once

#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"

#include "filters/filter.hpp"
#include "filters/lkf.hpp"

namespace msf_localization {

enum class FilterType {
  LKF, // Linear Kalman Filter
  NONE // For NOT FilterType
};

/**
 * @brief ROS2 API for State Estimation.
 *
 * Subscribes GNSS and IMU topics and calls related kalman filter step
 * as prediction or update.
 */
class MSFLocalization : public rclcpp::Node {
public:
  explicit MSFLocalization(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  /**
   * @brief Declare package parameters
   */
  void declare_parameters();

  /**
   * @brief Load package parameters
   */
  void load_parameters();

  /**
   * @brief Print package parameters
   */
  void print_parameters() const;

  /**
   * @brief IMU subscriber callback.
   *
   * @param msg IMU sensor data
   */
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /**
   * @brief GNSS subscriber callback.
   *
   * @param msg GNSS sensor data
   */
  void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  /**
   * @brief Timer for publishers.
   */
  void timer_callback();

  /**
   * @brief std::string to FilterType conversion
   *
   * @param str Kalman Filter Type
   */
  FilterType string_to_filtertype(std::string str);

  /// Kalman filter type
  FilterType filter_type_;

  /// Filter type parameter
  std::string filter_name_;

  /// GNSS topic to subscribe
  std::string gnss_topic_;

  /// IMU topic to subscribe
  std::string imu_topic_;

  /// Odometry topic to publish
  std::string odom_topic_;

  /// Trajectory topic to publish
  std::string trajectory_topic_;

  /// GNSS/INS topic to publish
  std::string fused_gnss_topic_;

  /// World frame
  std::string world_frame_;

  /// Map frame
  std::string map_frame_;

  /// Body frame
  std::string body_frame_;

  /// Datum is used or not
  bool use_datum_;

  //// Datum information
  std::vector<double> datum_;

  /// Origin latitude
  double latitude_;

  /// Origin longitude
  double longitude_;

  /// Origin altitude
  double altitude_;

  /// Global trajectory
  nav_msgs::msg::Path global_path_;

  /// Static transform publisher for earth to map (earth -> map)
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> origin_stb_;

  /// Dynamic transform publisher for map to base_link ( map -> base_link)
  std::unique_ptr<tf2_ros::TransformBroadcaster> body_tb_;

  /// GNSS subscriber
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;

  /// IMU subscriber
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  /// Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  /// Path publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  /// GNSS / INS publisher
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fused_gnss_pub_;

  /// Publisher Timer
  rclcpp::TimerBase::SharedPtr pub_timer_;

  /// GeographicLib ENU frame
  std::optional<GeographicLib::LocalCartesian> enu_frame_;

  /// Kalman filter
  std::unique_ptr<msf_localization_core::Filter> filter_;

  // initial state for bayes filter
  Eigen::VectorXd initial_state_{9};

  // initial covariance for bayes filter
  Eigen::MatrixXd initial_covariance_{9,9};
};
} // namespace msf_localization