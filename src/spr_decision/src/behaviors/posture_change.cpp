#include "spr_decision/behaviors/posture_change.hpp"

namespace spr_decision
{
  PostureChange::PostureChange(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
      : SyncActionNode(name, config), node_(std::move(node))
  {
    posture_change_pub_ = node_->create_publisher<std_msgs::msg::UInt8>(
      "posture",
      rclcpp::QoS(1).transient_local());
  }

  NodeStatus PostureChange::tick()
  {
    auto v = getInput<int>("posture_change");
    if (!v)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("POSTURE_CHANGE"),
        "error reading port [posture_change], fallback to moving (3)");
      v = 3;
    }

    // 与 protocol.hpp TxData::posture 一致：1进攻 2防御 3移动
    uint8_t out = 3;
    const int raw = v.value();
    if (raw >= 1 && raw <= 3) {
      out = static_cast<uint8_t>(raw);
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("POSTURE_CHANGE"),
        "invalid posture_change=%d, use 1/2/3; fallback to moving (3)", raw);
    }

    std_msgs::msg::UInt8 msg;
    msg.data = out;
    posture_change_pub_->publish(msg);
    return NodeStatus::SUCCESS;
  }

  PortsList PostureChange::providedPorts()
  {
    const char *description = "posture_change: 1 attack, 2 protect, 3 moving.";
    return {InputPort<int>("posture_change", description)};
  }
} // namespace spr_decision