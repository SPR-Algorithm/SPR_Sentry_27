#include "serial_node.hpp"

namespace serial_node {

GimbalYawPublisher::GimbalYawPublisher(rclcpp::Node* node, std::shared_ptr<DataBuffer> buffer)
: node_(node), buffer_(std::move(buffer)) {
  pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("serial/gimbal_joint_state", 10);
}

void GimbalYawPublisher::update() {
  if (!buffer_ || !buffer_->hasData()) return;
  const auto rx = buffer_->get();

  // RCLCPP_INFO_THROTTLE(
  //   node_->get_logger(),
  //   *node_->get_clock(),
  //   100,
  //   "---- 相对角 ---- gimbal_yaw: %f",
  //   rx.gimbal_yaw
  // );
  
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->get_clock()->now();
  msg.name.push_back("gimbal_yaw_joint");
  msg.position.push_back(-rx.gimbal_yaw);
  pub_->publish(msg);
}

}  // namespace serial_node

