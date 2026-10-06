#include "uart_to_stm32/uart_to_stm32.hpp"
#include <iostream>

#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>

namespace uart_to_stm32
{

using namespace std::chrono_literals;

namespace
{
bool isSupportedRouteChoice(uint8_t route_id)
{
  return route_id == 1 || route_id == 2 || route_id == 3;
}

void writeInt16LittleEndian(
  std::vector<uint8_t> & data, std::size_t offset, int16_t value)
{
  const uint16_t raw = static_cast<uint16_t>(value);
  data[offset] = static_cast<uint8_t>(raw & 0xFFU);
  data[offset + 1] = static_cast<uint8_t>((raw >> 8U) & 0xFFU);
}
}  // namespace

UartToStm32::UartToStm32(rclcpp::Node::SharedPtr node)
: node_(std::move(node)),
  update_rate_(0.0),
  current_yaw_(0.0),
  yaw_valid_(false),
  route_task_active_(false),
  has_st_ready_pub_(false),
  command_timeout_latched_(false),
  last_target_velocity_time_(std::chrono::steady_clock::now()),
  has_last_lidar_pose_(false),
  has_filtered_lidar_velocity_(false),
  last_lidar_pose_stamp_ns_(0),
  last_lidar_x_m_(0.0),
  last_lidar_y_m_(0.0),
  filtered_lidar_vx_cmps_(0.0),
  filtered_lidar_vy_cmps_(0.0)
{
  RCLCPP_INFO(node_->get_logger(), "UartToStm32 created");
}

UartToStm32::~UartToStm32()
{
  if (timer_) {
    timer_->cancel();
  }
  if (serial_comm_) {
    if (serial_comm_->is_open()) {
      sendZeroTargetVelocity();
      sendMissionControlToSerial(false);
    }
    serial_comm_->stop_protocol_receive();
    serial_comm_->close();
  }
}

bool UartToStm32::initialize(double update_rate, const std::string & source_frame, const std::string & target_frame)
{
  try {
    update_rate_ = update_rate;
    source_frame_ = source_frame;
    target_frame_ = target_frame;
    serial_port_ = node_->declare_parameter<std::string>("serial_port", "/dev/ttyS6");
    baud_rate_ = node_->declare_parameter<int>("baud_rate", 921600);
    target_velocity_frame_ =
      node_->declare_parameter<std::string>("target_velocity_frame", "map");
    command_timeout_sec_ = node_->declare_parameter<double>("command_timeout_sec", 0.30);
    lidar_velocity_enabled_ =
      node_->declare_parameter<bool>("lidar_velocity_enabled", true);
    lidar_velocity_frame_ =
      node_->declare_parameter<std::string>("lidar_velocity_frame", "odom");
    lidar_velocity_filter_alpha_ =
      node_->declare_parameter<double>("lidar_velocity_filter_alpha", 0.35);
    lidar_velocity_min_dt_sec_ =
      node_->declare_parameter<double>("lidar_velocity_min_dt_sec", 0.02);
    lidar_velocity_max_dt_sec_ =
      node_->declare_parameter<double>("lidar_velocity_max_dt_sec", 0.30);
    lidar_velocity_pose_timeout_sec_ =
      node_->declare_parameter<double>("lidar_velocity_pose_timeout_sec", 0.25);
    lidar_velocity_max_cmps_ =
      node_->declare_parameter<double>("lidar_velocity_max_cmps", 55.0);

    if (update_rate_ <= 0.0) {
      RCLCPP_ERROR(node_->get_logger(), "update_rate must be positive");
      return false;
    }
    if (target_velocity_frame_ != "map" && target_velocity_frame_ != "body") {
      RCLCPP_ERROR(
        node_->get_logger(),
        "target_velocity_frame must be 'map' or 'body', got '%s'",
        target_velocity_frame_.c_str());
      return false;
    }
    if (command_timeout_sec_ <= 0.0) {
      RCLCPP_ERROR(node_->get_logger(), "command_timeout_sec must be positive");
      return false;
    }
    if (lidar_velocity_frame_.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "lidar_velocity_frame must not be empty");
      return false;
    }
    if (lidar_velocity_filter_alpha_ <= 0.0 || lidar_velocity_filter_alpha_ > 1.0) {
      RCLCPP_ERROR(node_->get_logger(), "lidar_velocity_filter_alpha must be in (0, 1]");
      return false;
    }
    if (lidar_velocity_min_dt_sec_ <= 0.0 ||
      lidar_velocity_max_dt_sec_ <= lidar_velocity_min_dt_sec_)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "lidar velocity dt limits must satisfy 0 < min_dt < max_dt");
      return false;
    }
    if (lidar_velocity_pose_timeout_sec_ <= 0.0 || lidar_velocity_max_cmps_ <= 0.0 ||
      lidar_velocity_max_cmps_ >= 60.0)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "lidar velocity timeout/max must be positive and max must be below 60 cm/s");
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "UartToStm32 initialized with update rate: %.1f Hz", update_rate_);
    RCLCPP_INFO(
      node_->get_logger(), "Looking for transform from '%s' to '%s'",
      source_frame_.c_str(), target_frame_.c_str());

    serial_comm_ = std::make_unique<serial_comm::SerialComm>();
    if (!serial_comm_->initialize(serial_port_, static_cast<unsigned int>(baud_rate_))) {
      RCLCPP_ERROR(
        node_->get_logger(), "Failed to initialize serial port %s at %d baudrate",
        serial_port_.c_str(), baud_rate_);
      RCLCPP_ERROR(node_->get_logger(), "Serial error: %s", serial_comm_->get_last_error().c_str());
      return false;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "Serial port %s initialized at %d baudrate; target velocity frame=%s timeout=%.2fs",
      serial_port_.c_str(), baud_rate_, target_velocity_frame_.c_str(), command_timeout_sec_);
    RCLCPP_INFO(
      node_->get_logger(),
      "Lidar measured velocity %s: %s->%s alpha=%.2f dt=[%.3f, %.3f]s timeout=%.2fs max=%.1fcm/s",
      lidar_velocity_enabled_ ? "enabled" : "disabled",
      lidar_velocity_frame_.c_str(), target_frame_.c_str(),
      lidar_velocity_filter_alpha_, lidar_velocity_min_dt_sec_,
      lidar_velocity_max_dt_sec_, lidar_velocity_pose_timeout_sec_,
      lidar_velocity_max_cmps_);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto period = std::chrono::duration<double>(1.0 / update_rate_);
    timer_ = node_->create_wall_timer(
      period,
      std::bind(&UartToStm32::lookupTransform, this));

    route_choice_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "/route_choice", rclcpp::QoS(1).reliable(),
      std::bind(&UartToStm32::routeChoiceCallback, this, std::placeholders::_1));

    target_velocity_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/target_velocity", 10,
      std::bind(&UartToStm32::targetVelocityCallback, this, std::placeholders::_1));

    mission_complete_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/mission_complete", rclcpp::QoS(10),
      std::bind(&UartToStm32::missionCompleteCallback, this, std::placeholders::_1));

    height_pub_ = node_->create_publisher<std_msgs::msg::Int16>("/height", 10);
    is_st_ready_pub_ =
      node_->create_publisher<std_msgs::msg::UInt8>("/is_st_ready", rclcpp::QoS(10).transient_local());
    mission_step_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/mission_step", 10);
    lidar_velocity_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/lidar_velocity_body", 10);

    has_st_ready_pub_ = false;

    serial_comm_->start_protocol_receive(
      [this](uint8_t id, const std::vector<uint8_t> & data) { protocolDataHandler(id, data); },
      [this](const std::string & err) {
        RCLCPP_WARN(node_->get_logger(), "Serial protocol error: %s", err.c_str());
      });

    RCLCPP_INFO(node_->get_logger(), "UartToStm32 initialized successfully");
    RCLCPP_INFO(
      node_->get_logger(),
      "Subscribed to /route_choice, /target_velocity, and /mission_complete topics");
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize topic subscriber: %s", e.what());
    return false;
  }
}

void UartToStm32::lookupTransform()
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      source_frame_, target_frame_, tf2::TimePointZero);
    processTfTransform(transform);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(node_->get_logger(), "Transform lookup failed: %s", ex.what());
  }
  if (lidar_velocity_enabled_) {
    updateLidarVelocity();
  }
  checkTargetVelocityTimeout();
}

void UartToStm32::processTfTransform(const geometry_msgs::msg::TransformStamped & transform)
{
  const double x = transform.transform.translation.x;
  const double y = transform.transform.translation.y;
  const double z = transform.transform.translation.z;

  const double qx = transform.transform.rotation.x;
  const double qy = transform.transform.rotation.y;
  const double qz = transform.transform.rotation.z;
  const double qw = transform.transform.rotation.w;

  tf2::Quaternion q(qx, qy, qz, qw);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  current_yaw_ = yaw;
  yaw_valid_ = true;

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Transform %s -> %s: pos(%.3f, %.3f, %.3f) rot(%.3f, %.3f, %.3f)",
    source_frame_.c_str(), target_frame_.c_str(), x, y, z, roll, pitch, yaw);

}

void UartToStm32::updateLidarVelocity()
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      lidar_velocity_frame_, target_frame_, tf2::TimePointZero);
    processLidarVelocityTransform(transform);
  } catch (const tf2::TransformException & ex) {
    resetLidarVelocityEstimator();
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Lidar velocity transform %s->%s unavailable: %s",
      lidar_velocity_frame_.c_str(), target_frame_.c_str(), ex.what());
  }
}

void UartToStm32::resetLidarVelocityEstimator()
{
  has_last_lidar_pose_ = false;
  has_filtered_lidar_velocity_ = false;
  last_lidar_pose_stamp_ns_ = 0;
  filtered_lidar_vx_cmps_ = 0.0;
  filtered_lidar_vy_cmps_ = 0.0;
}

void UartToStm32::processLidarVelocityTransform(
  const geometry_msgs::msg::TransformStamped & transform)
{
  const int64_t stamp_ns =
    static_cast<int64_t>(transform.header.stamp.sec) * 1000000000LL +
    static_cast<int64_t>(transform.header.stamp.nanosec);
  if (stamp_ns <= 0) {
    resetLidarVelocityEstimator();
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Lidar velocity transform has a zero timestamp; measurement suppressed");
    return;
  }

  const double pose_age_sec =
    static_cast<double>(node_->now().nanoseconds() - stamp_ns) / 1e9;
  if (pose_age_sec > lidar_velocity_pose_timeout_sec_ || pose_age_sec < -0.1) {
    resetLidarVelocityEstimator();
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Lidar velocity pose is stale/invalid: age=%.3fs", pose_age_sec);
    return;
  }

  if (has_last_lidar_pose_ && stamp_ns <= last_lidar_pose_stamp_ns_) {
    return;
  }

  const double x_m = transform.transform.translation.x;
  const double y_m = transform.transform.translation.y;
  if (!std::isfinite(x_m) || !std::isfinite(y_m)) {
    resetLidarVelocityEstimator();
    return;
  }

  if (!has_last_lidar_pose_) {
    has_last_lidar_pose_ = true;
    last_lidar_pose_stamp_ns_ = stamp_ns;
    last_lidar_x_m_ = x_m;
    last_lidar_y_m_ = y_m;
    return;
  }

  const double dt_sec = static_cast<double>(stamp_ns - last_lidar_pose_stamp_ns_) / 1e9;
  if (dt_sec < lidar_velocity_min_dt_sec_) {
    return;
  }

  if (dt_sec > lidar_velocity_max_dt_sec_) {
    has_filtered_lidar_velocity_ = false;
    last_lidar_pose_stamp_ns_ = stamp_ns;
    last_lidar_x_m_ = x_m;
    last_lidar_y_m_ = y_m;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Lidar velocity sample gap %.3fs exceeds %.3fs; estimator reset",
      dt_sec, lidar_velocity_max_dt_sec_);
    return;
  }

  const double raw_vx_cmps = (x_m - last_lidar_x_m_) * 100.0 / dt_sec;
  const double raw_vy_cmps = (y_m - last_lidar_y_m_) * 100.0 / dt_sec;
  last_lidar_pose_stamp_ns_ = stamp_ns;
  last_lidar_x_m_ = x_m;
  last_lidar_y_m_ = y_m;

  const double raw_speed_cmps = std::hypot(raw_vx_cmps, raw_vy_cmps);
  if (!std::isfinite(raw_speed_cmps) || raw_speed_cmps > lidar_velocity_max_cmps_) {
    has_filtered_lidar_velocity_ = false;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Rejecting lidar velocity outlier: vx=%.1f vy=%.1f speed=%.1fcm/s",
      raw_vx_cmps, raw_vy_cmps, raw_speed_cmps);
    return;
  }

  if (!has_filtered_lidar_velocity_) {
    filtered_lidar_vx_cmps_ = raw_vx_cmps;
    filtered_lidar_vy_cmps_ = raw_vy_cmps;
    has_filtered_lidar_velocity_ = true;
  } else {
    filtered_lidar_vx_cmps_ =
      lidar_velocity_filter_alpha_ * raw_vx_cmps +
      (1.0 - lidar_velocity_filter_alpha_) * filtered_lidar_vx_cmps_;
    filtered_lidar_vy_cmps_ =
      lidar_velocity_filter_alpha_ * raw_vy_cmps +
      (1.0 - lidar_velocity_filter_alpha_) * filtered_lidar_vy_cmps_;
  }

  const double qx = transform.transform.rotation.x;
  const double qy = transform.transform.rotation.y;
  const double qz = transform.transform.rotation.z;
  const double qw = transform.transform.rotation.w;
  tf2::Quaternion q(qx, qy, qz, qw);
  tf2::Matrix3x3 rotation(q);
  double roll, pitch, yaw;
  rotation.getRPY(roll, pitch, yaw);

  const Eigen::Vector3d local_velocity(
    filtered_lidar_vx_cmps_, filtered_lidar_vy_cmps_, 0.0);
  const Eigen::Vector3d body_velocity = transformVelocity(local_velocity, yaw);
  const float body_vx_cmps = static_cast<float>(body_velocity.x());
  const float body_vy_cmps = static_cast<float>(body_velocity.y());

  std_msgs::msg::Float32MultiArray velocity_msg;
  velocity_msg.data = {body_vx_cmps, body_vy_cmps, 0.0F};
  lidar_velocity_pub_->publish(velocity_msg);

  if (sendLidarVelocityToSerial(body_vx_cmps, body_vy_cmps)) {
    RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Lidar measured velocity: body(%.1f, %.1f)cm/s raw-local(%.1f, %.1f)cm/s",
      body_vx_cmps, body_vy_cmps, raw_vx_cmps, raw_vy_cmps);
  }
}

void UartToStm32::routeChoiceCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  const uint8_t route_id = msg->data;
  if (!isSupportedRouteChoice(route_id)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Ignoring unsupported /route_choice=%u. Target velocity forwarding stays %s.",
      static_cast<unsigned>(route_id),
      route_task_active_ ? "enabled" : "disabled");
    return;
  }

  if (!route_task_active_) {
    bool enable_sent = false;
    for (int i = 0; i < 3; ++i) {
      enable_sent = sendMissionControlToSerial(true) || enable_sent;
      std::this_thread::sleep_for(20ms);
    }
    if (!enable_sent) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Cannot enable route task because mission-control frame was not sent");
      return;
    }
    route_task_active_ = true;
    command_timeout_latched_ = false;
    last_target_velocity_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      node_->get_logger(),
      "Received /route_choice=%u. Target velocity forwarding to STM32 is now enabled for the active task.",
      static_cast<unsigned>(route_id));
    return;
  }

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Received /route_choice=%u while a route task is already active. Target velocity forwarding remains enabled.",
    static_cast<unsigned>(route_id));
}

Eigen::Vector3d UartToStm32::transformVelocity(const Eigen::Vector3d & linear, double yaw)
{
  Eigen::Matrix3d Rz;
  Rz << std::cos(yaw), std::sin(yaw), 0.0,
    -std::sin(yaw), std::cos(yaw), 0.0,
    0.0, 0.0, 1.0;

  const Eigen::Vector3d transformed = Rz * linear;

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Velocity transform: yaw=%.3f deg, original(%.3f,%.3f,%.3f) -> transformed(%.3f,%.3f,%.3f)",
    yaw * 180.0 / M_PI,
    linear.x(), linear.y(), linear.z(),
    transformed.x(), transformed.y(), transformed.z());

  return transformed;
}

void UartToStm32::targetVelocityCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (!route_task_active_) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Dropping /target_velocity because there is no active route task yet.");
    return;
  }

  if (msg->data.size() < 4) {
    RCLCPP_WARN(node_->get_logger(),
      "Target velocity message should contain 4 float values [vx_cm/s, vy_cm/s, vz_cm/s, vyaw_deg/s]");
    return;
  }

  float vx_cm_per_s = msg->data[0];
  float vy_cm_per_s = msg->data[1];
  const float vz_cm_per_s = msg->data[2];
  const float vyaw_deg_per_s = msg->data[3];

  if (target_velocity_frame_ == "map") {
    if (!yaw_valid_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Dropping map-frame target velocity because map->body yaw is unavailable");
      return;
    }
    const Eigen::Vector3d map_velocity(vx_cm_per_s, vy_cm_per_s, vz_cm_per_s);
    const Eigen::Vector3d body_velocity = transformVelocity(map_velocity, current_yaw_);
    vx_cm_per_s = static_cast<float>(body_velocity.x());
    vy_cm_per_s = static_cast<float>(body_velocity.y());
  }

  last_target_velocity_time_ = std::chrono::steady_clock::now();
  command_timeout_latched_ = false;

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "Target Velocity: linear(%.1f, %.1f, %.1f)cm/s angular(%.1f)deg/s",
    vx_cm_per_s, vy_cm_per_s, vz_cm_per_s, vyaw_deg_per_s);

  sendTargetVelocityToSerial(vx_cm_per_s, vy_cm_per_s, vz_cm_per_s, vyaw_deg_per_s);
}

void UartToStm32::checkTargetVelocityTimeout()
{
  if (!route_task_active_ || command_timeout_latched_) {
    return;
  }

  const double age_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_target_velocity_time_).count();
  if (age_sec <= command_timeout_sec_) {
    return;
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "Target velocity timeout (%.3fs > %.3fs): sending zero command and disabling forwarding",
    age_sec, command_timeout_sec_);
  for (int i = 0; i < 3; ++i) {
    sendZeroTargetVelocity();
  }
  for (int i = 0; i < 3; ++i) {
    sendMissionControlToSerial(false);
  }
  command_timeout_latched_ = true;
  route_task_active_ = false;
}

void UartToStm32::sendZeroTargetVelocity()
{
  sendTargetVelocityToSerial(0.0F, 0.0F, 0.0F, 0.0F);
}

void UartToStm32::sendTargetVelocityToSerial(
  float vx_cm_per_s, float vy_cm_per_s, float vz_cm_per_s, float vyaw_deg_per_s)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send target velocity data");
    return;
  }

  try {
    const int16_t vel_x = static_cast<int16_t>(std::lround(vx_cm_per_s));
    const int16_t vel_y = static_cast<int16_t>(std::lround(vy_cm_per_s));
    const int16_t vel_z = static_cast<int16_t>(std::lround(vz_cm_per_s));
    const int16_t vel_yaw = static_cast<int16_t>(std::lround(vyaw_deg_per_s));

    std::vector<uint8_t> data(8);
    writeInt16LittleEndian(data, 0, vel_x);
    writeInt16LittleEndian(data, 2, vel_y);
    writeInt16LittleEndian(data, 4, vel_z);
    writeInt16LittleEndian(data, 6, vel_yaw);

    if (serial_comm_->send_protocol_data(TARGET_VELOCITY_FRAME_ID, static_cast<uint8_t>(data.size()), data)) {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Sent target velocity data: x=%d, y=%d, z=%d, yaw=%d",
        vel_x, vel_y, vel_z, vel_yaw);
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Failed to send target velocity data: %s", serial_comm_->get_last_error().c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception in sendTargetVelocityToSerial: %s", e.what());
  }
}

bool UartToStm32::sendLidarVelocityToSerial(
  float vx_cm_per_s, float vy_cm_per_s)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send lidar measured velocity");
    return false;
  }

  const int16_t vel_x = static_cast<int16_t>(std::lround(vx_cm_per_s));
  const int16_t vel_y = static_cast<int16_t>(std::lround(vy_cm_per_s));
  std::vector<uint8_t> data(12, 0U);

  // Legacy 0x32 layout: bytes 0..5 are unused tacc fields; bytes 6..11
  // carry the body-frame measured velocity consumed by Set_m_speed_now().
  writeInt16LittleEndian(data, 6, vel_x);
  writeInt16LittleEndian(data, 8, vel_y);
  writeInt16LittleEndian(data, 10, 0);

  if (!serial_comm_->send_protocol_data(
      LIDAR_VELOCITY_FRAME_ID, static_cast<uint8_t>(data.size()), data))
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Failed to send lidar measured velocity: %s",
      serial_comm_->get_last_error().c_str());
    return false;
  }
  return true;
}

void UartToStm32::protocolDataHandler(uint8_t id, const std::vector<uint8_t> & data)   
{
  switch (id) {
    case ST_READY_QUERY_ID: {
      if (data.size() < 9) {
        RCLCPP_WARN(node_->get_logger(), "protocolDataHandler: ID 0xF1 data too short, len=%zu", data.size());
        break;
      }
      const uint8_t first = data[0];
      if (mission_step_pub_) {
        std_msgs::msg::UInt8 msg;
        msg.data = first;
        mission_step_pub_->publish(msg);
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
          "Published /mission_step: %u (from 0xF1 frame)", static_cast<unsigned>(first));
      }
      if (has_st_ready_pub_) {
        break;
      }
      const uint8_t second = data[1];
      if (second == 1) {
        if (is_st_ready_pub_) {
          std_msgs::msg::UInt8 msg;
          msg.data = 1;
          is_st_ready_pub_->publish(msg);
          RCLCPP_INFO(node_->get_logger(), "Published /is_st_ready: 1 (from 0xF1 frame)");
        }
        has_st_ready_pub_ = true;
      } else {
        RCLCPP_DEBUG(node_->get_logger(), "0xF1 frame second byte != 1 (%u), ignoring", static_cast<unsigned>(second));
      }
      break;
    }
    case 0x05: {
      if (data.size() < 2) {
        RCLCPP_WARN(node_->get_logger(), "protocolDataHandler: ID 0x05 data too short");
        break;
      }
      const int16_t value = static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8));
      std_msgs::msg::Int16 msg;
      msg.data = value;
      if (height_pub_) {
        height_pub_->publish(msg);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
          "Current height from flight controller: %d", value);
      } else {
        RCLCPP_WARN(node_->get_logger(), "Height publisher not initialized");
      }
      break;
    }
    case 0xB1: {
      if (data.size() < 8) {
        RCLCPP_WARN(node_->get_logger(),
                    "protocolDataHandler: ID 0xB1 data too short (expected 8, got %zu)", data.size());
        break;
      }

      int16_t vel_x = static_cast<int16_t>(data[0] | (data[1] << 8));
      int16_t vel_y = static_cast<int16_t>(data[2] | (data[3] << 8));
      int16_t vel_z = static_cast<int16_t>(data[4] | (data[5] << 8));
      int16_t yaw = static_cast<int16_t>(data[6] | (data[7] << 8));

      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "[0xB1] Target Speed -> X:%d, Y:%d, Z:%d, Yaw:%d",
        vel_x, vel_y, vel_z, yaw);

      break;
    }
    default: {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
        "Unhandled protocol ID: 0x%02X, len=%zu", id, data.size());
      break;
    }
  }
}

void UartToStm32::sendMissionCompleteToSerial()
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send mission complete data");
    return;
  }

  std::vector<uint8_t> data(1, MISSION_COMPLETE_VALUE);
  if (serial_comm_->send_protocol_data(MISSION_COMPLETE_FRAME_ID, static_cast<uint8_t>(data.size()), data)) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Sent mission complete frame: id=0x%02X value=0x%02X",
      static_cast<unsigned>(MISSION_COMPLETE_FRAME_ID),
      static_cast<unsigned>(MISSION_COMPLETE_VALUE));
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "Failed to send mission complete frame: %s",
      serial_comm_->get_last_error().c_str());
  }
}

bool UartToStm32::sendMissionControlToSerial(bool enable)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send mission-control data");
    return false;
  }

  const uint8_t value = enable ? MISSION_ENABLE_VALUE : MISSION_DISABLE_VALUE;
  std::vector<uint8_t> data(1, value);
  if (!serial_comm_->send_protocol_data(
      MISSION_CONTROL_FRAME_ID, static_cast<uint8_t>(data.size()), data))
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Failed to send mission-control frame: %s",
      serial_comm_->get_last_error().c_str());
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Sent mission-control frame: id=0x%02X value=0x%02X (%s)",
    static_cast<unsigned>(MISSION_CONTROL_FRAME_ID),
    static_cast<unsigned>(value),
    enable ? "enable" : "disable");
  return true;
}

void UartToStm32::missionCompleteCallback(const std_msgs::msg::Empty::SharedPtr)
{
  RCLCPP_INFO(
    node_->get_logger(),
    "Received mission complete event. Sending frame 0x%02X three times.",
    static_cast<unsigned>(MISSION_COMPLETE_FRAME_ID));

  for (int i = 0; i < 3; ++i) {
    sendZeroTargetVelocity();
    std::this_thread::sleep_for(20ms);
  }
  for (int i = 0; i < 3; ++i) {
    sendMissionCompleteToSerial();
    std::this_thread::sleep_for(100ms);
  }
  for (int i = 0; i < 3; ++i) {
    sendMissionControlToSerial(false);
    std::this_thread::sleep_for(20ms);
  }

  route_task_active_ = false;
  RCLCPP_INFO(
    node_->get_logger(),
    "Mission complete sent. Target velocity forwarding is now disabled until the next valid /route_choice.");
}

}  // 命名空间 uart_to_stm32
