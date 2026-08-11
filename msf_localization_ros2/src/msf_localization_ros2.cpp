#include <chrono>
#include <memory>

#include "msf_localization_ros2/msf_localization_ros2.hpp"

#include "filters/filter.hpp"
#include "filters/lkf.hpp"

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

namespace msf_localization {
MSFLocalization::MSFLocalization(const rclcpp::NodeOptions &options)
    : rclcpp::Node("msf_localization_ros2_node", options) {

  RCLCPP_WARN(get_logger(),
              "====> [Multi Sensor Fusion Localization] Node Started! ====");

  declare_parameters();
  load_parameters();
  print_parameters();

  initial_state_.setZero();

  initial_covariance_ << 1e-9, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 1e-9, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 1e-9, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 1e-9, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 1e-9, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 1e-9, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 1e-9, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 
                         0, 0, 0, 0, 0, 0, 0, 0, 1e-9;

  const double dt{0.1};

  /// Create kalman filter
  filter_ = std::make_unique<msf_localization_core::LinearKalmanFilter>(dt, initial_state_, initial_covariance_);

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MSFLocalization::imu_callback, this, std::placeholders::_1));

  gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      gnss_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MSFLocalization::gnss_callback, this, std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(30));

  path_pub_ = create_publisher<nav_msgs::msg::Path>(trajectory_topic_, rclcpp::QoS(30));

  fused_gnss_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(fused_gnss_topic_, rclcpp::QoS(10));

  pub_timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&MSFLocalization::timer_callback, this));

  origin_stb_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  body_tb_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void MSFLocalization::declare_parameters() {
  declare_parameter<std::string>("filter_type", "LKF");

  declare_parameter<std::string>("gnss_topic", "/imu/nav_sat_fix");
  declare_parameter<std::string>("imu_topic", "/imu/topic");

  declare_parameter<std::string>("odometry_topic", "/odometry/topic");
  declare_parameter<std::string>("trajectory_topic", "/trajectory/topic");
  declare_parameter<std::string>("fused_gnss_topic", "/fused/trajectory/topic");

  declare_parameter<std::string>("world_frame", "earth");
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("body_frame", "base_link");

  declare_parameter<bool>("use_datum", false);
  declare_parameter<std::vector<double>>("datum", {0.0, 0.0, 0.0});
}

void MSFLocalization::load_parameters() {
  filter_name_ = get_parameter("filter_type").as_string();
  filter_type_ = string_to_filtertype(filter_name_);

  gnss_topic_ = get_parameter("gnss_topic").as_string();
  imu_topic_ = get_parameter("imu_topic").as_string();

  odom_topic_ = get_parameter("odometry_topic").as_string();
  trajectory_topic_ = get_parameter("trajectory_topic").as_string();
  fused_gnss_topic_ = get_parameter("fused_gnss_topic").as_string();

  world_frame_ = get_parameter("world_frame").as_string();
  map_frame_ = get_parameter("map_frame").as_string();
  body_frame_ = get_parameter("body_frame").as_string();

  use_datum_ = get_parameter("use_datum").as_bool();
  datum_ = get_parameter("datum").as_double_array();
}

void MSFLocalization::print_parameters() const {
  RCLCPP_INFO(get_logger(), "================= Filter Type =================");
  RCLCPP_INFO_STREAM(get_logger(), "----> Filter Type       : " << filter_name_);

  RCLCPP_INFO(get_logger(), "================= Topic Names =================");
  RCLCPP_INFO_STREAM(get_logger(), "----> GNSS Topic        : " << gnss_topic_);
  RCLCPP_INFO_STREAM(get_logger(), "----> IMU Topic         : " << imu_topic_);
  RCLCPP_INFO_STREAM(get_logger(), "----> Odometry Topic    : " << odom_topic_);
  RCLCPP_INFO_STREAM(get_logger(), "----> Trajectory Topic  : " << trajectory_topic_);
  RCLCPP_INFO_STREAM(get_logger(), "----> Fused GNSS Topic  : " << fused_gnss_topic_);

  RCLCPP_INFO(get_logger(), "================= Frame Names =================");
  RCLCPP_INFO_STREAM(get_logger(), "----> World Frame       : " << world_frame_);
  RCLCPP_INFO_STREAM(get_logger(), "----> Map Frame         : " << map_frame_);
  RCLCPP_INFO_STREAM(get_logger(), "----> Body Frame        : " << body_frame_);

  RCLCPP_INFO(get_logger(), "================= Datum =================");
  RCLCPP_INFO_STREAM(get_logger(), "----> Use Datum         : " << std::boolalpha << use_datum_);
  RCLCPP_INFO_STREAM(get_logger(), "----> Datum             : " << "[ " << datum_[0] << ", " << datum_[1] << ", " << datum_[2] << " ]");
}

void MSFLocalization::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {

  Eigen::VectorXd input(6);
  input(0) = msg->linear_acceleration.x;
  input(1) = msg->linear_acceleration.y;
  input(2) = msg->linear_acceleration.z - 9.81; // gravity compensation
  input(3) = msg->angular_velocity.x;
  input(4) = msg->angular_velocity.y;
  input(5) = msg->angular_velocity.z;

  // check if the klaman filter is initialized or not
  if (enu_frame_.has_value()) {
    filter_->predict(input);
  }
}

void MSFLocalization::gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {

  // Check first message: Is initialized the kalman filter?
  if (!enu_frame_.has_value()) {
    
    // Is datum used?
    // YES: use config datum
    if (use_datum_) {
      latitude_ = datum_[0];
      longitude_ = datum_[1];
      altitude_ = datum_[2];
    }
    // NO: use first GNSS message
    else {
      latitude_ = msg->latitude;
      longitude_ = msg->longitude;
      altitude_ = msg->altitude;
    }

    enu_frame_.emplace(latitude_, longitude_, altitude_, GeographicLib::Geocentric::WGS84());

    RCLCPP_WARN_STREAM(get_logger(), "=== Origin set ===> lat: " << latitude_ 
                                                     << " lon: " << longitude_
                                                     << " alt: " << altitude_);

    const GeographicLib::Geocentric &earth = GeographicLib::Geocentric::WGS84();
    double ecef_x, ecef_y, ecef_z;
    earth.Forward(latitude_, longitude_, altitude_, ecef_x, ecef_y, ecef_z);

    // publish first GNSS fix as map frame
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = now();
    t.header.frame_id = world_frame_;
    t.child_frame_id = map_frame_;

    t.transform.translation.x = ecef_x;
    t.transform.translation.y = ecef_y;
    t.transform.translation.z = ecef_z;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    origin_stb_->sendTransform(t);

    return;
  }

  // convert new messages to ENU
  double east, north, up;
  enu_frame_->Forward(msg->latitude, msg->longitude, msg->altitude, east, north, up);

  Eigen::VectorXd measurement(3);
  measurement << east, north, up;

  filter_->update(measurement);
}

void MSFLocalization::timer_callback() {

  Eigen::VectorXd current_state_ = filter_->get_state();
  Eigen::MatrixXd current_covariance_ = filter_->get_covariance();

  tf2::Quaternion q;
  q.setRPY(current_state_(3), current_state_(4), current_state_(5));

  double qx = q.x();
  double qy = q.y();
  double qz = q.z();
  double qw = q.w();

  // ---- map -> base_link transform ----
  geometry_msgs::msg::TransformStamped current_t;
  current_t.header.stamp = now();
  current_t.header.frame_id = map_frame_;
  current_t.child_frame_id = body_frame_;

  current_t.transform.translation.x = current_state_(0);
  current_t.transform.translation.y = current_state_(1);
  current_t.transform.translation.z = current_state_(2);
  current_t.transform.rotation.x = qx;
  current_t.transform.rotation.y = qy;
  current_t.transform.rotation.z = qz;
  current_t.transform.rotation.w = qw;

  body_tb_->sendTransform(current_t);

  // ---- Publish Odometry ----
  auto odom = nav_msgs::msg::Odometry();
  odom.header.stamp = now();
  odom.header.frame_id = map_frame_;
  odom.child_frame_id = body_frame_;
  odom.pose.pose.position.x = current_state_(0);
  odom.pose.pose.position.y = current_state_(1);
  odom.pose.pose.position.z = current_state_(2);

  odom.pose.pose.orientation.x = qx;
  odom.pose.pose.orientation.y = qy;
  odom.pose.pose.orientation.z = qz;
  odom.pose.pose.orientation.w = qw;

  odom.pose.covariance[0] = current_covariance_(0, 0);
  odom.pose.covariance[7] = current_covariance_(1, 1);
  odom.pose.covariance[14] = current_covariance_(2, 2);
  odom.pose.covariance[21] = current_covariance_(3, 3);
  odom.pose.covariance[28] = current_covariance_(4, 4);
  odom.pose.covariance[35] = current_covariance_(5, 5);

  odom.twist.twist.linear.x = current_state_(6);
  odom.twist.twist.linear.y = current_state_(7);
  odom.twist.twist.linear.z = current_state_(8);

  odom.twist.covariance[0] = current_covariance_(6, 6);
  odom.twist.covariance[7] = current_covariance_(7, 7);
  odom.twist.covariance[14] = current_covariance_(8, 8);

  odom_pub_->publish(odom);

  // ---- Global path publish ----
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = map_frame_;
  pose.pose.position.x = current_state_(0);
  pose.pose.position.y = current_state_(1);
  pose.pose.position.z = current_state_(2);

  pose.pose.orientation.x = qx;
  pose.pose.orientation.y = qy;
  pose.pose.orientation.z = qz;
  pose.pose.orientation.w = qw;

  global_path_.header.stamp = now();
  global_path_.header.frame_id = map_frame_;
  global_path_.poses.push_back(pose);
  path_pub_->publish(global_path_);

  // ---- Fused GNSS / INS publish ----
  auto fused_gnss = sensor_msgs::msg::NavSatFix();
  enu_frame_->Reverse(current_state_(0), current_state_(1), current_state_(2),
                      fused_gnss.latitude, fused_gnss.longitude,
                      fused_gnss.altitude);
  fused_gnss_pub_->publish(fused_gnss);
}

FilterType MSFLocalization::string_to_filtertype(std::string str) {

  std::transform(str.begin(), str.end(), str.begin(), ::toupper);

  if (str == "LKF") {
    return FilterType::LKF;
  } else {
    return FilterType::NONE;
    throw std::runtime_error("Unknown filter type: " + str);
  }
}

} // namespace msf_localization