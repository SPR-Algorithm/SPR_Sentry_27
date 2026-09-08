#include "spr_decision/behaviors/spin_or_not.hpp"

namespace spr_decision
{
  SpinOrNot::SpinOrNot(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
      : SyncActionNode(name, config), node_(std::move(node))
  {
    std::string spin_or_not_topic_name;
    node_->get_parameter_or<std::string>("spin_or_not_topic_name", spin_or_not_topic_name, "spin_or_not");
    spin_or_not_pub_ = node_->create_publisher<std_msgs::msg::UInt8>(
      spin_or_not_topic_name,
      rclcpp::QoS(1).transient_local());
  }

  NodeStatus SpinOrNot::tick()
  {
    // XML 属性是字符串；用 int 做 BT port 更稳，然后归一化到 0/1 再发到串口
    auto v = getInput<int>("spin_or_not");
    if (!v)
    {
      RCLCPP_WARN(rclcpp::get_logger("SPIN_OR_NOT"), "error reading port [spin_or_not], fallback to 0");
      v = 0;
    }

    const uint8_t out = (v.value() != 0) ? static_cast<uint8_t>(1) : static_cast<uint8_t>(0);
    if (last_sent_.has_value() && last_sent_.value() == out)
    {
      return NodeStatus::SUCCESS;
    }
    std_msgs::msg::UInt8 msg;
    msg.data = out;
    spin_or_not_pub_->publish(msg);
    last_sent_ = out;
    return NodeStatus::SUCCESS;
  }

  PortsList SpinOrNot::providedPorts()
  {
    const char *description = "spin_or_not (0 or 1).";
    return {InputPort<int>("spin_or_not", description)};
  }
} // namespace spr_decision