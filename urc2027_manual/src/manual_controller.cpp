#include "trc2026_manual/manual_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace trc2026_manual
{

ManualController::ManualController(const rclcpp::NodeOptions & options)
: BaseManualController("manual_controller_node", options)
{
  launcher_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/launcher_controller/commands", 1);
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  arm_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/arm_velocity_controller/commands", 1);
  arm_4_only_ = this->declare_parameter("arm_4_only", false);
  arm_position_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    arm_4_only_ ? "/arm_4_position_controller/joint_trajectory" :
    "/arm_position_controller/position_targets", 1);
  recovery_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
    "/arm_position_controller/recovery_ready", rclcpp::QoS(1).transient_local(),
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      if (!msg->data || !recovery_ready_) {
        resync_position_targets_.fill(true);
        arm_position_targets_initialized_ = false;
        arm_4_axis_command_ = 0.0;
        arm_123_axis_command_ = 0.0;
        selected_arm_123_joint_ = -1;
      }
      recovery_ready_ = msg->data;
    });
  joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&ManualController::joint_state_callback, this, std::placeholders::_1));
  health_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/robstride/health", 10,
    std::bind(&ManualController::health_callback, this, std::placeholders::_1));

  this->declare_parameter("axis_linear_x", 4);
  this->declare_parameter("axis_linear_y", 3);
  this->declare_parameter("axis_angular_z", 0);
  this->declare_parameter("scale_linear_x", 2.0);
  this->declare_parameter("scale_linear_y", 2.0);
  this->declare_parameter("scale_angular_z", 3.0);
  this->declare_parameter("arm_y_axis", 6);
  this->declare_parameter("arm_4_positive_axis", 2);
  this->declare_parameter("arm_4_negative_axis", 5);
  this->declare_parameter("arm_123_axis", 7);
  this->declare_parameter("arm_1_button", 1);
  this->declare_parameter("arm_2_button", 2);
  this->declare_parameter("arm_3_button", 3);
  this->declare_parameter("gripper_positive_button", 5);
  this->declare_parameter("gripper_negative_button", 4);
  this->declare_parameter("arm_y_velocity_scale", 20000.0);
  this->declare_parameter("arm_4_position_rate", 10.0);
  this->declare_parameter("arm_123_position_rates", std::vector<double>{8.0, 0.2, 0.5});
  this->declare_parameter("arm_position_trajectory_duration", 0.1);
  this->declare_parameter("arm_123_max_tracking_error", 0.1);
  this->declare_parameter("arm_2_max_tracking_error", 0.2);
  this->declare_parameter("gripper_velocity_scale", 5000.0);
  this->declare_parameter("deadzone", 0.01);
  this->declare_parameter("yaw_deadzone", 0.1);
  this->declare_parameter("joy_timeout", 0.5);

  this->get_parameter("axis_linear_x", axis_linear_x_);
  this->get_parameter("axis_linear_y", axis_linear_y_);
  this->get_parameter("axis_angular_z", axis_angular_z_);
  this->get_parameter("scale_linear_x", scale_linear_x_);
  this->get_parameter("scale_linear_y", scale_linear_y_);
  this->get_parameter("scale_angular_z", scale_angular_z_);
  this->get_parameter("arm_y_axis", arm_y_axis_);
  this->get_parameter("arm_4_positive_axis", arm_4_positive_axis_);
  this->get_parameter("arm_4_negative_axis", arm_4_negative_axis_);
  this->get_parameter("arm_123_axis", arm_123_axis_);
  this->get_parameter("arm_1_button", arm_1_button_);
  this->get_parameter("arm_2_button", arm_2_button_);
  this->get_parameter("arm_3_button", arm_3_button_);
  this->get_parameter("gripper_positive_button", gripper_positive_button_);
  this->get_parameter("gripper_negative_button", gripper_negative_button_);
  this->get_parameter("arm_y_velocity_scale", arm_y_velocity_scale_);
  this->get_parameter("arm_4_position_rate", arm_4_position_rate_);
  this->get_parameter("arm_123_position_rates", arm_123_position_rates_);
  if (arm_123_position_rates_.size() != 3 ||
    std::any_of(arm_123_position_rates_.begin(), arm_123_position_rates_.end(),
      [](double rate) {return !std::isfinite(rate) || rate <= 0.0;}))
  {
    throw std::invalid_argument("arm_123_position_rates must contain three finite positive values");
  }
  this->get_parameter("arm_position_trajectory_duration", arm_position_trajectory_duration_);
  this->get_parameter("arm_123_max_tracking_error", arm_123_max_tracking_error_);
  if (!std::isfinite(arm_123_max_tracking_error_) || arm_123_max_tracking_error_ <= 0.0) {
    throw std::invalid_argument("arm_123_max_tracking_error must be finite and positive");
  }
  this->get_parameter("arm_2_max_tracking_error", arm_2_max_tracking_error_);
  if (!std::isfinite(arm_2_max_tracking_error_) || arm_2_max_tracking_error_ <= 0.0) {
    throw std::invalid_argument("arm_2_max_tracking_error must be finite and positive");
  }
  if (!std::isfinite(arm_position_trajectory_duration_) || arm_position_trajectory_duration_ <= 0.0) {
    throw std::invalid_argument("arm_position_trajectory_duration must be finite and positive");
  }
  this->get_parameter("gripper_velocity_scale", gripper_velocity_scale_);
  this->get_parameter("deadzone", deadzone_);
  this->get_parameter("yaw_deadzone", yaw_deadzone_);
  if (!std::isfinite(yaw_deadzone_) || yaw_deadzone_ < 0.0 || yaw_deadzone_ >= 1.0) {
    throw std::invalid_argument("yaw_deadzone must be finite and in [0, 1)");
  }
  this->get_parameter("joy_timeout", joy_timeout_sec_);

  last_joy_time_ = this->now();
  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&ManualController::publish_timer_callback, this));
}

void ManualController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  const auto launcher_target = launcher_trigger_.update(
    msg->buttons.size() > 8, msg->buttons.size() > 8 && msg->buttons[8] != 0,
    launcher_publisher_->get_subscription_count() > 0, launcher_clock_.now().seconds());
  if (launcher_target) {
    std_msgs::msg::Float64MultiArray command;
    command.data = {*launcher_target};
    launcher_publisher_->publish(command);
  }
  last_joy_time_ = this->now();
  const auto axis_value = [&msg](int index) {
    return index >= 0 && static_cast<size_t>(index) < msg->axes.size() ? msg->axes[index] : 0.0;
  };
  const auto deadzoned = [this](double value) {
    return std::abs(value) > deadzone_ ? value : 0.0;
  };

  last_cmd_vel_.linear.x = axis_value(axis_linear_x_) * scale_linear_x_;
  last_cmd_vel_.linear.y = axis_value(axis_linear_y_) * scale_linear_y_;
  const double yaw = axis_value(axis_angular_z_);
  last_cmd_vel_.angular.z = std::abs(yaw) > yaw_deadzone_ ? yaw * scale_angular_z_ : 0.0;
  arm_y_velocity_command_ = deadzoned(axis_value(arm_y_axis_)) * arm_y_velocity_scale_ * -1.0;
  const auto positive_trigger =
    std::clamp((1.0 - axis_value(arm_4_positive_axis_)) / 2.0, 0.0, 1.0);
  const auto negative_trigger =
    std::clamp((1.0 - axis_value(arm_4_negative_axis_)) / 2.0, 0.0, 1.0);
  arm_4_axis_command_ = deadzoned(positive_trigger - negative_trigger);

  const bool arm_1_selected = arm_1_button_ >= 0 &&
    static_cast<size_t>(arm_1_button_) < msg->buttons.size() && msg->buttons[arm_1_button_];
  const bool arm_2_selected = arm_2_button_ >= 0 &&
    static_cast<size_t>(arm_2_button_) < msg->buttons.size() && msg->buttons[arm_2_button_];
  const bool arm_3_selected = arm_3_button_ >= 0 &&
    static_cast<size_t>(arm_3_button_) < msg->buttons.size() && msg->buttons[arm_3_button_];
  const int selected_count = static_cast<int>(arm_1_selected) +
    static_cast<int>(arm_2_selected) + static_cast<int>(arm_3_selected);
  selected_arm_123_joint_ = selected_count == 1 ?
    (arm_1_selected ? 0 : (arm_2_selected ? 1 : 2)) : -1;
  arm_123_axis_command_ = selected_arm_123_joint_ >= 0 ?
    deadzoned(axis_value(arm_123_axis_)) * (arm_1_selected ? -1.0 : 1.0) : 0.0;

  const bool positive_button = gripper_positive_button_ >= 0 &&
    static_cast<size_t>(gripper_positive_button_) < msg->buttons.size() &&
    msg->buttons[gripper_positive_button_];
  const bool negative_button = gripper_negative_button_ >= 0 &&
    static_cast<size_t>(gripper_negative_button_) < msg->buttons.size() &&
    msg->buttons[gripper_negative_button_];
  gripper_velocity_command_ = positive_button == negative_button ? 0.0 :
    (positive_button ? gripper_velocity_scale_ : -gripper_velocity_scale_);
}

void ManualController::publish_timer_callback()
{
  constexpr double period_sec = 0.05;
  const bool command_is_fresh = (this->now() - last_joy_time_).seconds() <= joy_timeout_sec_;
  auto velocity_command = std_msgs::msg::Float64MultiArray();
  velocity_command.data.resize(2, 0.0);
  auto position_command = trajectory_msgs::msg::JointTrajectory();
  position_command.joint_names = {"arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint"};
  position_command.points.resize(1);
  auto & position_point = position_command.points.front();
  position_point.positions.resize(4, 0.0);
  position_point.time_from_start = rclcpp::Duration::from_seconds(arm_position_trajectory_duration_);

  if ((this->now() - last_health_time_).seconds() > 0.5) {
    position_hardware_healthy_.fill(false);
    resync_position_targets_.fill(true);
  }

  if (command_is_fresh) {
    velocity_command.data[0] = arm_y_velocity_command_;
    velocity_command.data[1] = gripper_velocity_command_;
    if ((arm_4_only_ || recovery_ready_) && arm_position_targets_initialized_) {
      arm_4_position_target_ += arm_4_axis_command_ * arm_4_position_rate_ * period_sec;
    }
    if (recovery_ready_ && selected_arm_123_joint_ >= 0 &&
      position_hardware_healthy_[selected_arm_123_joint_] &&
      !resync_position_targets_[selected_arm_123_joint_] &&
      position_sync_hold_[selected_arm_123_joint_] == 0) {
      const auto index = static_cast<size_t>(selected_arm_123_joint_);
      const auto feedback_age =
        (this->now() - arm_123_feedback_times_[index]).seconds();
      if (feedback_age >= 0.0 && feedback_age <= 0.5) {
        auto & target = arm_123_position_targets_[index];
        const double actual = arm_123_actual_positions_[index];
        const double max_tracking_error =
          index == 1 ? arm_2_max_tracking_error_ : arm_123_max_tracking_error_;
        const double step = arm_123_axis_command_ * arm_123_position_rates_[index] * period_sec;
        if (step > 0.0) {
          const double upper = actual + max_tracking_error;
          target += std::min(step, std::max(0.0, upper - target));
        } else if (step < 0.0) {
          const double lower = actual - max_tracking_error;
          target += std::max(step, std::min(0.0, lower - target));
        }
      }
    }
    cmd_vel_publisher_->publish(last_cmd_vel_);
  } else {
    last_cmd_vel_ = geometry_msgs::msg::Twist();
    arm_y_velocity_command_ = 0.0;
    arm_4_axis_command_ = 0.0;
    arm_123_axis_command_ = 0.0;
    selected_arm_123_joint_ = -1;
    gripper_velocity_command_ = 0.0;
    cmd_vel_publisher_->publish(last_cmd_vel_);
  }

  arm_velocity_publisher_->publish(velocity_command);
  for (size_t index = 0; index < 3; ++index) {
    // Keep finite targets for all trajectory joints. Hardware gates unavailable motors.
    position_point.positions[index] = arm_123_position_targets_[index];
    if (position_sync_hold_[index] > 0) {--position_sync_hold_[index];}
  }
  position_point.positions[3] = arm_4_position_target_;
  bool synchronized = arm_position_targets_initialized_;
  for (size_t index = 0; index < 3; ++index) {
    if (position_hardware_healthy_[index] && resync_position_targets_[index]) {
      synchronized = false;
    }
  }
  if (arm_4_only_ && arm_position_targets_initialized_) {
    position_command.joint_names = {"arm_4_joint"};
    position_point.positions = {arm_4_position_target_};
    position_command.header.stamp = this->now();
    arm_position_publisher_->publish(position_command);
  } else if (!arm_4_only_ && recovery_ready_ && synchronized) {
    position_command.header.stamp = this->now();
    arm_position_publisher_->publish(position_command);
  }
}

void ManualController::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  const auto find_position = [&msg](const std::string & name, double & position) {
      const auto it = std::find(msg->name.begin(), msg->name.end(), name);
      if (it == msg->name.end()) {return false;}
      const auto index = static_cast<size_t>(std::distance(msg->name.begin(), it));
      if (index >= msg->position.size() || !std::isfinite(msg->position[index])) {return false;}
      position = msg->position[index];
      return true;
    };
  double position = 0.0;
  if (find_position("launcher_joint", position)) {
    launcher_trigger_.feedback(position, launcher_clock_.now().seconds());
  }
  if (!arm_position_targets_initialized_ && find_position("arm_4_joint", position)) {
    arm_4_position_target_ = position;
    arm_position_targets_initialized_ = true;
  }
  for (size_t index = 0; index < 3; ++index) {
    if (!find_position("arm_" + std::to_string(index + 1) + "_joint", position)) {
      continue;
    }
    arm_123_actual_positions_[index] = position;
    arm_123_feedback_times_[index] = this->now();
    if (position_hardware_healthy_[index] && resync_position_targets_[index]) {
      arm_123_position_targets_[index] = position;
      resync_position_targets_[index] = false;
      position_sync_hold_[index] = 2;
    }
  }
}

void ManualController::health_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // RobStride order: arm_1, arm_2, arm_3; each has online/ready/fault.
  last_health_time_ = this->now();
  for (size_t index = 0; index < 3; ++index) {
    const bool healthy = msg->data.size() == 9 && msg->data[index * 3] > 0.5 &&
      msg->data[index * 3 + 1] > 0.5 && msg->data[index * 3 + 2] < 0.5;
    if (!healthy || !position_hardware_healthy_[index]) {
      resync_position_targets_[index] = true;
    }
    position_hardware_healthy_[index] = healthy;
  }
}

}  // namespace trc2026_manual

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_manual::ManualController)
