#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
using PositionCommand = quadrotor_msgs::msg::PositionCommand;
using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;

struct DesiredState
{
  Vector3 position{Vector3::Zero()};
  Vector3 velocity{Vector3::Zero()};
  Vector3 acceleration{Vector3::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};
};

struct ControlOutput
{
  Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
  double thrust{0.5};
  double yaw_rate{0.0};
};

enum class ControlState
{
  waiting_for_command,
  takeoff,
  hover,
  tracking,
  safety_fault
};

bool finite(const Vector3 & value)
{
  return value.allFinite();
}

class Px4Ctrl final : public rclcpp::Node
{
public:
  Px4Ctrl()
  : Node("px4ctrl")
  {
    declare_parameter("planner_frame", "lidar");
    declare_parameter("command_topic", "/drone_0_planning/pos_cmd");
    declare_parameter("odom_topic", "/fastlio2/lio_odom");
    declare_parameter("px4_status_topic", "/fmu/out/vehicle_status_v4");
    declare_parameter("px4_attitude_topic", "/fmu/out/vehicle_attitude");
    declare_parameter("px4_local_position_topic", "/fmu/out/vehicle_local_position_v1");
    declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode");
    // PX4 1.x exposes the versioned uORB input for this message.  Publishing
    // to the unversioned legacy name leaves no PX4 subscriber on current SITL.
    declare_parameter("attitude_setpoint_topic", "/fmu/in/vehicle_attitude_setpoint_v1");
    declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command");
    declare_parameter("status_topic", "/px4ctrl/status");
    declare_parameter("manual_yaw_topic", "/px4ctrl/manual_yaw");
    declare_parameter("clear_manual_yaw_topic", "/px4ctrl/clear_manual_yaw");
    declare_parameter("publish_rate_hz", 100.0);
    declare_parameter("state_timeout_sec", 0.30);
    declare_parameter("command_timeout_sec", 0.50);
    declare_parameter("prestream_seconds", 1.0);
    declare_parameter("start_offboard_on_first_command", true);
    declare_parameter("arm_on_first_command", true);
    declare_parameter("gravity_mps2", 9.81);
    declare_parameter("hover_thrust", 0.864);
    declare_parameter("min_thrust", 0.08);
    declare_parameter("max_thrust", 0.95);
    declare_parameter("max_tilt_deg", 30.0);
    declare_parameter("max_acceleration_mps2", 4.0);
    declare_parameter("local_position_timeout_sec", 0.50);
    declare_parameter("local_position_fault_delay_sec", 1.00);
    declare_parameter("max_ekf_reset_delta_m", 1.0);
    declare_parameter("max_ekf_reset_heading_rad", 0.35);
    declare_parameter("require_local_position_valid", true);
    declare_parameter("max_local_position_m", 100.0);
    declare_parameter("max_local_velocity_mps", 15.0);
    declare_parameter("align_to_px4_local_origin", true);
    declare_parameter("frame_alignment_velocity_tolerance_mps", 0.20);
    declare_parameter("land_on_safety_fault", true);
    declare_parameter("auto_takeoff_altitude_m", 0.50);
    declare_parameter("minimum_takeoff_altitude_m", 0.25);
    declare_parameter("takeoff_duration_sec", 3.0);
    declare_parameter("takeoff_tolerance_m", 0.08);
    declare_parameter("hover_settle_sec", 0.80);
    declare_parameter("hover_velocity_tolerance_mps", 0.15);
    declare_parameter("tracking_handoff_sec", 0.75);
    declare_parameter("max_yaw_rate_radps", 1.5);
    declare_parameter("kp", std::vector<double>{1.5, 1.5, 1.8});
    declare_parameter("kv", std::vector<double>{1.2, 1.2, 1.4});
    declare_parameter("transform_rotation_row_major", std::vector<double>{
      0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0});

    planner_frame_ = get_parameter("planner_frame").as_string();
    state_timeout_sec_ = get_parameter("state_timeout_sec").as_double();
    command_timeout_sec_ = get_parameter("command_timeout_sec").as_double();
    prestream_seconds_ = get_parameter("prestream_seconds").as_double();
    start_offboard_ = get_parameter("start_offboard_on_first_command").as_bool();
    arm_on_first_ = get_parameter("arm_on_first_command").as_bool();
    gravity_ = get_parameter("gravity_mps2").as_double();
    hover_thrust_ = get_parameter("hover_thrust").as_double();
    min_thrust_ = get_parameter("min_thrust").as_double();
    max_thrust_ = get_parameter("max_thrust").as_double();
    max_tilt_rad_ = get_parameter("max_tilt_deg").as_double() * M_PI / 180.0;
    max_acceleration_ = get_parameter("max_acceleration_mps2").as_double();
    local_position_timeout_sec_ = get_parameter("local_position_timeout_sec").as_double();
    local_position_fault_delay_sec_ = get_parameter("local_position_fault_delay_sec").as_double();
    max_ekf_reset_delta_m_ = get_parameter("max_ekf_reset_delta_m").as_double();
    max_ekf_reset_heading_rad_ = get_parameter("max_ekf_reset_heading_rad").as_double();
    require_local_position_valid_ = get_parameter("require_local_position_valid").as_bool();
    max_local_position_m_ = get_parameter("max_local_position_m").as_double();
    max_local_velocity_mps_ = get_parameter("max_local_velocity_mps").as_double();
    align_to_px4_local_origin_ = get_parameter("align_to_px4_local_origin").as_bool();
    frame_alignment_velocity_tolerance_mps_ =
      get_parameter("frame_alignment_velocity_tolerance_mps").as_double();
    land_on_safety_fault_ = get_parameter("land_on_safety_fault").as_bool();
    auto_takeoff_altitude_m_ = get_parameter("auto_takeoff_altitude_m").as_double();
    minimum_takeoff_altitude_m_ = get_parameter("minimum_takeoff_altitude_m").as_double();
    takeoff_duration_sec_ = get_parameter("takeoff_duration_sec").as_double();
    takeoff_tolerance_m_ = get_parameter("takeoff_tolerance_m").as_double();
    hover_settle_sec_ = get_parameter("hover_settle_sec").as_double();
    hover_velocity_tolerance_mps_ = get_parameter("hover_velocity_tolerance_mps").as_double();
    tracking_handoff_sec_ = get_parameter("tracking_handoff_sec").as_double();
    max_yaw_rate_radps_ = get_parameter("max_yaw_rate_radps").as_double();
    const auto kp = get_parameter("kp").as_double_array();
    const auto kv = get_parameter("kv").as_double_array();
    const auto rotation = get_parameter("transform_rotation_row_major").as_double_array();
    if (kp.size() != 3 || kv.size() != 3 || rotation.size() != 9 ||
        !std::isfinite(state_timeout_sec_) || state_timeout_sec_ <= 0.0 ||
        !std::isfinite(command_timeout_sec_) || command_timeout_sec_ <= 0.0 ||
        !std::isfinite(prestream_seconds_) || prestream_seconds_ < 0.0 ||
        !std::isfinite(gravity_) || gravity_ <= 0.0 || !std::isfinite(hover_thrust_) ||
        hover_thrust_ <= 0.0 || hover_thrust_ >= 1.0 || !std::isfinite(min_thrust_) ||
        !std::isfinite(max_thrust_) || min_thrust_ < 0.0 || max_thrust_ > 1.0 ||
        min_thrust_ >= max_thrust_ || !std::isfinite(max_tilt_rad_) || max_tilt_rad_ <= 0.0 ||
        max_tilt_rad_ >= M_PI_2 || !std::isfinite(max_acceleration_) || max_acceleration_ <= 0.0 ||
        !std::isfinite(local_position_timeout_sec_) || local_position_timeout_sec_ <= 0.0 ||
        !std::isfinite(local_position_fault_delay_sec_) || local_position_fault_delay_sec_ <= 0.0 ||
        !std::isfinite(max_ekf_reset_delta_m_) || max_ekf_reset_delta_m_ <= 0.0 ||
        !std::isfinite(max_ekf_reset_heading_rad_) || max_ekf_reset_heading_rad_ <= 0.0 ||
        !std::isfinite(max_local_position_m_) || max_local_position_m_ <= 0.0 ||
        !std::isfinite(max_local_velocity_mps_) || max_local_velocity_mps_ <= 0.0 ||
        !std::isfinite(frame_alignment_velocity_tolerance_mps_) ||
        frame_alignment_velocity_tolerance_mps_ <= 0.0 ||
        !std::isfinite(auto_takeoff_altitude_m_) || auto_takeoff_altitude_m_ < 0.0 ||
        !std::isfinite(minimum_takeoff_altitude_m_) || minimum_takeoff_altitude_m_ < 0.0 ||
        minimum_takeoff_altitude_m_ > auto_takeoff_altitude_m_ ||
        !std::isfinite(takeoff_duration_sec_) || takeoff_duration_sec_ <= 0.0 ||
        !std::isfinite(takeoff_tolerance_m_) || takeoff_tolerance_m_ <= 0.0 ||
        !std::isfinite(hover_settle_sec_) || hover_settle_sec_ <= 0.0 ||
        !std::isfinite(hover_velocity_tolerance_mps_) || hover_velocity_tolerance_mps_ <= 0.0 ||
        !std::isfinite(tracking_handoff_sec_) || tracking_handoff_sec_ < 0.0 ||
        !std::isfinite(max_yaw_rate_radps_) || max_yaw_rate_radps_ <= 0.0)
    {
      throw std::runtime_error("invalid px4ctrl parameters");
    }
    for (size_t i = 0; i < 3; ++i) {
      kp_(static_cast<Eigen::Index>(i)) = kp[i];
      kv_(static_cast<Eigen::Index>(i)) = kv[i];
      if (!std::isfinite(kp[i]) || !std::isfinite(kv[i]) || kp[i] < 0.0 || kv[i] < 0.0) {
        throw std::runtime_error("kp and kv must be finite and non-negative");
      }
    }
    for (size_t r = 0; r < 3; ++r) for (size_t c = 0; c < 3; ++c) {
      rotation_(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = rotation[r * 3 + c];
    }
    if (!rotation_.isApprox(rotation_.transpose().inverse(), 1e-4) ||
        std::abs(rotation_.determinant() - 1.0) > 1e-4) {
      throw std::runtime_error("transform_rotation_row_major must be a proper rotation");
    }

    const auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    command_sub_ = create_subscription<PositionCommand>(
      get_parameter("command_topic").as_string(), rclcpp::QoS(20),
      std::bind(&Px4Ctrl::command_callback, this, std::placeholders::_1));
    // FAST-LIO is a sensor stream.  Use best-effort reception so the
    // controller remains compatible with both the current RELIABLE publisher
    // and deployments where the LIO node uses sensor-data QoS.
    const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(20)).best_effort().durability_volatile();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), sensor_qos,
      std::bind(&Px4Ctrl::odom_callback, this, std::placeholders::_1));
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      get_parameter("px4_status_topic").as_string(), px4_qos,
      std::bind(&Px4Ctrl::vehicle_status_callback, this, std::placeholders::_1));
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      get_parameter("px4_attitude_topic").as_string(), px4_qos,
      std::bind(&Px4Ctrl::attitude_callback, this, std::placeholders::_1));
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      get_parameter("px4_local_position_topic").as_string(), px4_qos,
      std::bind(&Px4Ctrl::local_position_callback, this, std::placeholders::_1));
    emergency_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/planning/emergency_stop", rclcpp::QoS(10),
      [this](const std_msgs::msg::Empty::SharedPtr) { latch_safety_fault("external_emergency_stop"); });
    manual_yaw_sub_ = create_subscription<std_msgs::msg::Float64>(
      get_parameter("manual_yaw_topic").as_string(), rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        if (std::isfinite(msg->data)) manual_yaw_ = msg->data;
      });
    clear_manual_yaw_sub_ = create_subscription<std_msgs::msg::Empty>(
      get_parameter("clear_manual_yaw_topic").as_string(), rclcpp::QoS(10),
      [this](const std_msgs::msg::Empty::SharedPtr) { manual_yaw_.reset(); });

    mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      get_parameter("offboard_control_mode_topic").as_string(), px4_qos);
    setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
      get_parameter("attitude_setpoint_topic").as_string(), px4_qos);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      get_parameter("vehicle_command_topic").as_string(), px4_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("status_topic").as_string(), rclcpp::QoS(10));

    const double rate = get_parameter("publish_rate_hz").as_double();
    if (!std::isfinite(rate) || rate < 10.0) throw std::runtime_error("publish_rate_hz must be >= 10");
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), std::bind(&Px4Ctrl::timer_callback, this));
    publish_status("waiting_for_fastlio_odom_and_planner_command");
  }

private:
  void command_callback(const PositionCommand::SharedPtr msg)
  {
    if (!msg || msg->header.frame_id != planner_frame_ ||
        msg->trajectory_flag != PositionCommand::TRAJECTORY_STATUS_READY) return;
    DesiredState candidate;
    candidate.position = {msg->position.x, msg->position.y, msg->position.z};
    candidate.velocity = {msg->velocity.x, msg->velocity.y, msg->velocity.z};
    candidate.acceleration = {msg->acceleration.x, msg->acceleration.y, msg->acceleration.z};
    candidate.yaw = msg->yaw;
    candidate.yaw_rate = msg->yaw_dot;
    if (!finite(candidate.position) || !finite(candidate.velocity) ||
        !finite(candidate.acceleration) || !std::isfinite(candidate.yaw) ||
        !std::isfinite(candidate.yaw_rate)) return;
    desired_ = candidate;
    pending_desired_ = candidate;
    desired_command_time_ = std::chrono::steady_clock::now();
    command_seen_ = true;
    if (!first_command_time_) first_command_time_ = desired_command_time_;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg) return;
    if (msg->header.frame_id != planner_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Rejecting odometry frame '%s'; expected '%s'",
        msg->header.frame_id.c_str(), planner_frame_.c_str());
      return;
    }
    const Vector3 p(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    const Vector3 body_v(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    if (!finite(p) || !finite(body_v) || !std::isfinite(q.norm()) || q.norm() < 1e-6) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Rejecting non-finite FAST-LIO odometry");
      return;
    }
    q.normalize();
    const Vector3 v = q * body_v;
    if (!finite(v)) return;
    odom_position_ = p;
    odom_velocity_ = v;
    odom_orientation_ = q;
    odom_time_ = std::chrono::steady_clock::now();
    odom_received_ = true;
  }

  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    vehicle_status_ = *msg;
  }

  void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    if (msg->timestamp > 0) px4_timestamp_ = msg->timestamp;
  }

  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    if (!msg) return;
    const bool local_frame_reset = local_position_ &&
      (msg->xy_reset_counter != local_position_->xy_reset_counter ||
      msg->z_reset_counter != local_position_->z_reset_counter ||
      msg->heading_reset_counter != local_position_->heading_reset_counter);
    if (local_frame_reset && frame_aligned_) {
      const bool xy_reset = msg->xy_reset_counter != local_position_->xy_reset_counter;
      const bool z_reset = msg->z_reset_counter != local_position_->z_reset_counter;
      const bool heading_reset =
        msg->heading_reset_counter != local_position_->heading_reset_counter;
      const Vector3 reset_delta(
        xy_reset ? static_cast<double>(msg->delta_xy[0]) : 0.0,
        xy_reset ? static_cast<double>(msg->delta_xy[1]) : 0.0,
        z_reset ? static_cast<double>(msg->delta_z) : 0.0);
      const double heading_delta = heading_reset ?
        static_cast<double>(msg->delta_heading) : 0.0;
      if (!finite(reset_delta) || !std::isfinite(heading_delta) ||
        reset_delta.norm() > max_ekf_reset_delta_m_ ||
        std::abs(heading_delta) > max_ekf_reset_heading_rad_)
      {
        latch_safety_fault("px4_local_position_reset_too_large");
      } else if (frame_aligned_) {
        // PX4 position-control tasks compensate EKF resets by adding delta_xy
        // and delta_z to their existing setpoints. Apply the same correction to
        // every local-NED target used by this direct attitude controller.
        translation_ += reset_delta;
        hover_position_ += reset_delta;
        takeoff_target_ += reset_delta;
        tracking_handoff_position_ += reset_delta;
        // PX4's position controller keeps its local-frame setpoints valid
        // across a heading reset by adding delta_heading to the yaw setpoint.
        // Keep the same offset for the direct attitude path instead of
        // treating a normal estimator yaw reset as a flight failure.
        heading_reset_offset_ = std::remainder(
          heading_reset_offset_ + heading_delta, 2.0 * M_PI);
        RCLCPP_WARN(get_logger(),
          "Compensating PX4 local-position reset: delta [%.3f %.3f %.3f] m",
          reset_delta.x(), reset_delta.y(), reset_delta.z());
      }
    }
    local_position_ = *msg;
    if (msg->timestamp > 0) px4_timestamp_ = msg->timestamp;
    local_position_time_ = std::chrono::steady_clock::now();
    local_position_received_ = true;
  }

  bool local_position_healthy() const
  {
    if (!require_local_position_valid_) return true;
    if (!local_position_received_ || !local_position_time_) return false;
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - *local_position_time_).count() >
      local_position_timeout_sec_) return false;
    const auto & p = *local_position_;
    if (p.timestamp == 0 || !p.xy_valid || !p.z_valid || !p.v_xy_valid || !p.v_z_valid || p.dead_reckoning) return false;
    const double position_norm = std::hypot(std::hypot(static_cast<double>(p.x), static_cast<double>(p.y)),
      static_cast<double>(p.z));
    const double velocity_norm = std::hypot(std::hypot(static_cast<double>(p.vx), static_cast<double>(p.vy)),
      static_cast<double>(p.vz));
    return std::isfinite(position_norm) && std::isfinite(velocity_norm) &&
      position_norm <= max_local_position_m_ && velocity_norm <= max_local_velocity_mps_;
  }

  void latch_safety_fault(const std::string & reason)
  {
    if (safety_latched_) return;
    safety_latched_ = true;
    state_ = ControlState::safety_fault;
    publish_status("safety_fault_" + reason);
    RCLCPP_ERROR(get_logger(), "PX4 control latched safety fault: %s", reason.c_str());
    if (land_on_safety_fault_ && vehicle_status_ &&
      vehicle_status_->arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
      publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0F, 0.0F);
    }
  }

  bool state_fresh() const
  {
    return odom_position_ && odom_velocity_ && odom_time_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - *odom_time_).count() <=
      state_timeout_sec_;
  }

  bool command_fresh() const
  {
    return pending_desired_ && desired_command_time_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - *desired_command_time_).count() <=
      command_timeout_sec_;
  }

  bool align_frames_if_stationary()
  {
    if (frame_aligned_) return true;
    if (!align_to_px4_local_origin_) {
      frame_aligned_ = true;
      return true;
    }
    if (!odom_position_ || !odom_velocity_ || !local_position_ || !local_position_healthy()) {
      return false;
    }
    const Vector3 local_velocity(
      static_cast<double>(local_position_->vx), static_cast<double>(local_position_->vy),
      static_cast<double>(local_position_->vz));
    if (odom_velocity_->norm() > frame_alignment_velocity_tolerance_mps_ ||
      local_velocity.norm() > frame_alignment_velocity_tolerance_mps_)
    {
      return false;
    }
    const Vector3 local_position(
      static_cast<double>(local_position_->x), static_cast<double>(local_position_->y),
      static_cast<double>(local_position_->z));
    translation_ = local_position - rotation_ * *odom_position_;
    frame_aligned_ = true;
    RCLCPP_INFO(get_logger(), "Aligned planner frame to PX4 local NED: translation [%.3f %.3f %.3f] m",
      translation_.x(), translation_.y(), translation_.z());
    publish_status("planner_frame_aligned");
    return true;
  }

  ControlOutput calculate_output() const
  {
    const auto & lpos = *local_position_;
    const Vector3 p(static_cast<double>(lpos.x), static_cast<double>(lpos.y), static_cast<double>(lpos.z));
    const Vector3 v(static_cast<double>(lpos.vx), static_cast<double>(lpos.vy), static_cast<double>(lpos.vz));
    DesiredState d = desired_.value_or(DesiredState{});
    d.position = rotation_ * d.position + translation_;
    d.velocity = rotation_ * d.velocity;
    d.acceleration = rotation_ * d.acceleration;
    const Vector3 heading_in_ned = rotation_ * Vector3(std::cos(d.yaw), std::sin(d.yaw), 0.0);
    const Vector3 heading_rate_in_ned = rotation_ * Vector3(
      -std::sin(d.yaw) * d.yaw_rate, std::cos(d.yaw) * d.yaw_rate, 0.0);
    d.yaw = std::atan2(heading_in_ned.y(), heading_in_ned.x());
    d.yaw = std::remainder(d.yaw + heading_reset_offset_, 2.0 * M_PI);
    const double heading_norm_sq = heading_in_ned.head<2>().squaredNorm();
    d.yaw_rate = heading_norm_sq > 1e-8 ?
      (heading_in_ned.x() * heading_rate_in_ned.y() -
      heading_in_ned.y() * heading_rate_in_ned.x()) / heading_norm_sq : 0.0;
    if (!command_fresh()) {
      d.position = p;
      d.velocity.setZero();
      d.acceleration.setZero();
      d.yaw_rate = 0.0;
    } else if (state_ == ControlState::tracking && tracking_handoff_start_time_ &&
      tracking_handoff_sec_ > 1e-6)
    {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - *tracking_handoff_start_time_).count();
      const double u = std::clamp(elapsed / tracking_handoff_sec_, 0.0, 1.0);
      const double blend = u * u * (3.0 - 2.0 * u);
      d.position = (1.0 - blend) * tracking_handoff_position_ + blend * d.position;
      d.velocity = (1.0 - blend) * tracking_handoff_velocity_ + blend * d.velocity;
      d.acceleration *= blend;
      d.yaw = tracking_handoff_yaw_ + blend * std::remainder(
        d.yaw - tracking_handoff_yaw_, 2.0 * M_PI);
      d.yaw_rate *= blend;
    }
    Vector3 a = d.acceleration + kp_.cwiseProduct(d.position - p) + kv_.cwiseProduct(d.velocity - v);
    if (!a.allFinite()) a.setZero();
    const double a_norm = a.norm();
    if (a_norm > max_acceleration_) a *= max_acceleration_ / a_norm;

    Vector3 force = a - Vector3(0.0, 0.0, gravity_);
    if (force.norm() < 1e-6) force = Vector3(0.0, 0.0, -gravity_);
    const double vertical_force = std::max(0.1, -force.z());
    const double max_horizontal = vertical_force * std::tan(max_tilt_rad_);
    const Vector3 horizontal(force.x(), force.y(), 0.0);
    if (horizontal.norm() > max_horizontal) {
      force.x() = horizontal.x() * max_horizontal / horizontal.norm();
      force.y() = horizontal.y() * max_horizontal / horizontal.norm();
    }

    Vector3 heading = manual_yaw_ ? rotation_ * Vector3(
      std::cos(*manual_yaw_), std::sin(*manual_yaw_), 0.0) :
      Vector3(std::cos(d.yaw), std::sin(d.yaw), 0.0);
    heading.z() = 0.0;
    if (heading.norm() < 1e-6) heading = Vector3(1.0, 0.0, 0.0);
    heading.normalize();
    const Vector3 b3 = -force.normalized();
    Vector3 b2 = b3.cross(heading);
    if (b2.norm() < 1e-6) b2 = Vector3(0.0, 1.0, 0.0);
    b2.normalize();
    const Vector3 b1 = b2.cross(b3).normalized();
    Eigen::Matrix3d rotation;
    rotation.col(0) = b1;
    rotation.col(1) = b2;
    rotation.col(2) = b3;

    ControlOutput out;
    out.attitude = Eigen::Quaterniond(rotation).normalized();
    out.thrust = std::clamp(force.norm() / gravity_ * hover_thrust_, min_thrust_, max_thrust_);
    out.yaw_rate = std::clamp(manual_yaw_ ? 0.0 : d.yaw_rate,
      -max_yaw_rate_radps_, max_yaw_rate_radps_);
    return out;
  }

  void begin_takeoff()
  {
    if (!local_position_) return;
    const auto & p = *local_position_;
    hover_position_ = {static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z)};
    hold_yaw_ = std::isfinite(p.heading) ? static_cast<double>(p.heading) : 0.0;
    takeoff_target_ = hover_position_;
    // PX4 local z is Down, therefore a positive altitude is a negative z target.
    takeoff_target_.z() -= auto_takeoff_altitude_m_;
    takeoff_start_time_ = std::chrono::steady_clock::now();
    hover_settle_start_time_.reset();
    state_ = auto_takeoff_altitude_m_ > 1e-3 ? ControlState::takeoff : ControlState::hover;
    publish_status(state_ == ControlState::takeoff ? "state_takeoff" : "state_hover");
  }

  void update_state_machine()
  {
    if (!local_position_) return;
    const auto & p = *local_position_;
    const Vector3 current{static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z)};
    const Vector3 velocity{static_cast<double>(p.vx), static_cast<double>(p.vy), static_cast<double>(p.vz)};
    if (state_ == ControlState::waiting_for_command && command_seen_) begin_takeoff();

    if (state_ == ControlState::takeoff) {
      const double climb_m = hover_position_.z() - current.z();
      // The payload's calibrated hover point can be below the nominal target.
      // Release the takeoff gate once it has either reached the requested
      // height or demonstrated a safe minimum climb above the launch plane.
      const bool at_height = (current - takeoff_target_).norm() <= takeoff_tolerance_m_;
      const bool safely_airborne = climb_m >= minimum_takeoff_altitude_m_;
      const bool settled = velocity.norm() <= hover_velocity_tolerance_mps_;
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - *takeoff_start_time_).count();
      if ((at_height || safely_airborne) && settled && elapsed >= takeoff_duration_sec_) {
        state_ = ControlState::hover;
        hover_position_ = current;
        hover_settle_start_time_ = std::chrono::steady_clock::now();
        publish_status(safely_airborne && !at_height ? "state_hover_minimum_altitude" : "state_hover");
      }
    }
    if (state_ == ControlState::hover) {
      if (!hover_settle_start_time_) hover_settle_start_time_ = std::chrono::steady_clock::now();
      if (velocity.norm() > hover_velocity_tolerance_mps_) {
        hover_settle_start_time_ = std::chrono::steady_clock::now();
      }
      const double settled_for = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - *hover_settle_start_time_).count();
      if (command_fresh() && pending_desired_ && settled_for >= hover_settle_sec_) {
        state_ = ControlState::tracking;
        desired_ = pending_desired_;
        tracking_handoff_position_ = current;
        tracking_handoff_velocity_ = velocity;
        tracking_handoff_yaw_ = std::isfinite(p.heading) ? static_cast<double>(p.heading) : hold_yaw_;
        tracking_handoff_start_time_ = std::chrono::steady_clock::now();
        publish_status("state_tracking");
      }
    }
    if (state_ == ControlState::tracking && !command_fresh()) {
      hover_position_ = current;
      hover_settle_start_time_ = std::chrono::steady_clock::now();
      state_ = ControlState::hover;
      publish_status("state_hover_command_timeout");
    }
  }

  void timer_callback()
  {
    px4_msgs::msg::OffboardControlMode mode;
    mode.timestamp = px4_timestamp_;
    mode.attitude = !safety_latched_;
    mode_pub_->publish(mode);
    if (safety_latched_) {
      publish_status("safety_latched_no_setpoint");
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!state_fresh()) {
      if (vehicle_status_ && vehicle_status_->arming_state !=
        px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED)
      {
        if (!unhealthy_since_) unhealthy_since_ = now;
        if (std::chrono::duration<double>(now - *unhealthy_since_).count() >=
          local_position_fault_delay_sec_)
        {
          latch_safety_fault("fastlio_odom_timeout");
        }
      } else {
        publish_status("waiting_for_fresh_fastlio_odom");
      }
      return;
    }
    if (!local_position_healthy()) {
      publish_status("waiting_for_healthy_px4_local_position");
      if (vehicle_status_ && vehicle_status_->arming_state !=
        px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
        if (!unhealthy_since_) unhealthy_since_ = now;
        if (std::chrono::duration<double>(now - *unhealthy_since_).count() >=
          local_position_fault_delay_sec_)
        {
          latch_safety_fault("px4_local_position_unhealthy");
        }
      }
      return;
    }
    unhealthy_since_.reset();

    if (!align_frames_if_stationary()) {
      publish_status("waiting_for_stationary_frame_alignment");
      return;
    }

    update_state_machine();
    if (state_ == ControlState::waiting_for_command) {
      publish_status("waiting_for_planner_command");
      return;
    }
    if (state_ == ControlState::hover || state_ == ControlState::takeoff) {
      DesiredState hold;
      const Vector3 target = state_ == ControlState::takeoff ? takeoff_target_ : hover_position_;
      // calculate_output applies planner->PX4 rotation to all desired states.
      hold.position = rotation_.transpose() * (target - translation_);
      hold.velocity.setZero();
      hold.acceleration.setZero();
      const Vector3 px4_heading{std::cos(hold_yaw_), std::sin(hold_yaw_), 0.0};
      const Vector3 planner_heading = rotation_.transpose() * px4_heading;
      hold.yaw = manual_yaw_.value_or(std::atan2(planner_heading.y(), planner_heading.x()));
      desired_ = hold;
    }
    if (state_ == ControlState::tracking && pending_desired_) {
      desired_ = pending_desired_;
    }
    const ControlOutput output = calculate_output();
    px4_msgs::msg::VehicleAttitudeSetpoint setpoint;
    setpoint.timestamp = px4_timestamp_;
    setpoint.q_d = {static_cast<float>(output.attitude.w()), static_cast<float>(output.attitude.x()),
                    static_cast<float>(output.attitude.y()), static_cast<float>(output.attitude.z())};
    setpoint.thrust_body = {0.0F, 0.0F, static_cast<float>(-output.thrust)};
    setpoint.yaw_sp_move_rate = static_cast<float>(output.yaw_rate);
    setpoint_pub_->publish(setpoint);
    if (!command_seen_ || !start_offboard_ || !vehicle_status_) return;
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - *first_command_time_).count();
    if (elapsed < prestream_seconds_) return;
    if (vehicle_status_->nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      if (retry_due()) {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
        last_command_time_ = std::chrono::steady_clock::now();
        publish_status("attitude_offboard_requested");
      }
      return;
    }
    if (arm_on_first_ && vehicle_status_->arming_state ==
        px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED &&
        vehicle_status_->pre_flight_checks_pass && retry_due()) {
      publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F, 0.0F);
      last_command_time_ = std::chrono::steady_clock::now();
      publish_status("attitude_offboard_arm_requested");
    }
    publish_status(command_fresh() ? "attitude_thrust_tracking" : "command_stale_hovering");
  }

  bool retry_due() const
  {
    return !last_command_time_ ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() - *last_command_time_).count() >= 1.0;
  }

  void publish_vehicle_command(uint16_t command, float p1, float p2)
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = px4_timestamp_;
    msg.param1 = p1;
    msg.param2 = p2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  std::string planner_frame_;
  double state_timeout_sec_{0.3};
  double command_timeout_sec_{0.5};
  double prestream_seconds_{1.0};
  bool start_offboard_{true};
  bool arm_on_first_{true};
  double gravity_{9.81};
  double hover_thrust_{0.864};
  double min_thrust_{0.08};
  double max_thrust_{0.95};
  double max_tilt_rad_{0.52};
  double max_acceleration_{4.0};
  double local_position_timeout_sec_{0.5};
  double local_position_fault_delay_sec_{1.0};
  double max_ekf_reset_delta_m_{1.0};
  double max_ekf_reset_heading_rad_{0.35};
  bool require_local_position_valid_{true};
  double max_local_position_m_{100.0};
  double max_local_velocity_mps_{15.0};
  bool align_to_px4_local_origin_{true};
  double frame_alignment_velocity_tolerance_mps_{0.20};
  bool land_on_safety_fault_{true};
  double auto_takeoff_altitude_m_{0.5};
  double minimum_takeoff_altitude_m_{0.25};
  double takeoff_duration_sec_{3.0};
  double takeoff_tolerance_m_{0.08};
  double hover_settle_sec_{0.8};
  double hover_velocity_tolerance_mps_{0.15};
  double tracking_handoff_sec_{0.75};
  double max_yaw_rate_radps_{1.5};
  Vector3 kp_{1.5, 1.5, 1.8};
  Vector3 kv_{1.2, 1.2, 1.4};
  Matrix3 rotation_{Matrix3::Identity()};
  Vector3 translation_{Vector3::Zero()};
  bool frame_aligned_{false};
  std::optional<Vector3> odom_position_;
  std::optional<Vector3> odom_velocity_;
  std::optional<Eigen::Quaterniond> odom_orientation_;
  std::optional<std::chrono::steady_clock::time_point> odom_time_;
  bool odom_received_{false};
  std::optional<DesiredState> desired_;
  std::optional<DesiredState> pending_desired_;
  std::optional<std::chrono::steady_clock::time_point> desired_command_time_;
  std::optional<std::chrono::steady_clock::time_point> first_command_time_;
  std::optional<std::chrono::steady_clock::time_point> last_command_time_;
  std::optional<double> manual_yaw_;
  std::optional<px4_msgs::msg::VehicleStatus> vehicle_status_;
  std::optional<px4_msgs::msg::VehicleLocalPosition> local_position_;
  std::optional<std::chrono::steady_clock::time_point> local_position_time_;
  std::optional<std::chrono::steady_clock::time_point> unhealthy_since_;
  bool local_position_received_{false};
  bool safety_latched_{false};
  bool command_seen_{false};
  ControlState state_{ControlState::waiting_for_command};
  Vector3 hover_position_{Vector3::Zero()};
  Vector3 takeoff_target_{Vector3::Zero()};
  double hold_yaw_{0.0};
  double heading_reset_offset_{0.0};
  std::optional<std::chrono::steady_clock::time_point> takeoff_start_time_;
  std::optional<std::chrono::steady_clock::time_point> hover_settle_start_time_;
  Vector3 tracking_handoff_position_{Vector3::Zero()};
  Vector3 tracking_handoff_velocity_{Vector3::Zero()};
  double tracking_handoff_yaw_{0.0};
  std::optional<std::chrono::steady_clock::time_point> tracking_handoff_start_time_;
  uint64_t px4_timestamp_{0};
  rclcpp::Subscription<PositionCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr manual_yaw_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_manual_yaw_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Px4Ctrl>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("px4ctrl"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
