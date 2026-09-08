#include "spr_decision/behaviors/align_chassis.hpp"

#include <cmath>

using namespace BT;
namespace spr_decision
{
namespace
{
double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double shortestAngularError(double from_rad, double to_rad)
{
  return normalizeAngle(to_rad - from_rad);
}
}  // namespace

  AlignChassis::AlignChassis(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<tf2_ros::Buffer> tf_buffer, std::shared_ptr<tf2_ros::TransformListener> tf_listener)
  : StatefulActionNode(name, config), node_(node), tf_buffer_(tf_buffer), tf_listener_(tf_listener)
  {
    node_->get_parameter_or<std::string>("align_chassis_frame", chassis_frame_, "chassis");
    node_->get_parameter_or<double>("align_chassis_angular_sign", angular_sign_, 1.0);
    double heading_offset_deg = 0.0;
    node_->get_parameter_or<double>("align_chassis_heading_offset_deg", heading_offset_deg, 0.0);
    heading_offset_rad_ = heading_offset_deg * M_PI / 180.0;
    node_->get_parameter_or<std::string>(
      "align_chassis_cmd_vel_topic", cmd_vel_input_topic_, "cmd_vel_nav2_result");
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_input_topic_, 10);
  }

  void AlignChassis::publishCmdVel(double angular_z)
  {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.angular.z = angular_z;
    cmd_vel_pub_->publish(cmd_vel);
  }

  NodeStatus AlignChassis::onStart()
  {
    auto target_angle = getInput<double>("target_angle");
    if (!target_angle)
    {
      RCLCPP_ERROR(node_->get_logger(), "target_angle is not set");
      return NodeStatus::FAILURE;
    }
    else
    {
      target_angle_ = target_angle.value()/180*M_PI;
    }
    auto min_angle_diff = getInput<double>("min_angle_diff");
    if (!min_angle_diff)
    {
      RCLCPP_ERROR(node_->get_logger(), "min_angle_diff is not set");
      return NodeStatus::FAILURE;
    }
    else
    {
      min_angle_diff_ = min_angle_diff.value()/180*M_PI;
    }
    auto angular_speed = getInput<double>("angular_speed");
    if (!angular_speed)
    {
      RCLCPP_ERROR(node_->get_logger(), "angular_speed is not set");
      return NodeStatus::FAILURE;
    }
    else
    {
      angular_speed_ = angular_speed.value();
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "AlignChassis started: target=%.1f deg, frame=%s, cmd_vel_topic=%s (via fake_vel_transform)",
      target_angle.value(),
      chassis_frame_.c_str(),
      cmd_vel_input_topic_.c_str());
    return NodeStatus::RUNNING;
  }

  NodeStatus AlignChassis::onRunning()
  {
    try
    {
      geometry_msgs::msg::TransformStamped map_chassis =
        tf_buffer_->lookupTransform("map", chassis_frame_, tf2::TimePointZero);
      angle_diff_map_chassis_ = tf2::getYaw(map_chassis.transform.rotation);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Can't get transform map -> %s: %s",
        chassis_frame_.c_str(),
        ex.what());
      return NodeStatus::FAILURE;
    }

    const double current_yaw = normalizeAngle(angle_diff_map_chassis_ + heading_offset_rad_);
    const double err = shortestAngularError(current_yaw, target_angle_);

    // REP-103: angular.z>0 为绕 map Z 逆时针；err>0 时沿最短路径增大 yaw
    const double cmd_omega =
      angular_sign_ * std::copysign(std::min(std::abs(err), angular_speed_), err);

    RCLCPP_INFO(
      node_->get_logger(),
      "AlignChassis: current=%.1f deg, target=%.1f deg, err=%.1f deg, cmd_wz=%.4f",
      current_yaw * 180.0 / M_PI,
      target_angle_ * 180.0 / M_PI,
      err * 180.0 / M_PI,
      cmd_omega);
    if (std::abs(err) < min_angle_diff_)
    {
      publishCmdVel(0.0);
      RCLCPP_INFO(node_->get_logger(), "AlignChassis succeeded");
      return NodeStatus::SUCCESS;
    }

    publishCmdVel(cmd_omega);
    return NodeStatus::RUNNING;
  }

  void AlignChassis::onHalted()
  {
    publishCmdVel(0.0);
    RCLCPP_INFO(node_->get_logger(), "AlignChassis halted");
  }

  PortsList AlignChassis::providedPorts()
  {
    return {InputPort<double>("target_angle"),
            InputPort<double>("min_angle_diff"),
            InputPort<double>("angular_speed")};
  }

} // end namespace spr_decision