#ifndef SPR_DECISION_SPIN_OR_NOT_HPP_
#define SPR_DECISION_SPIN_OR_NOT_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include <std_msgs/msg/u_int8.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <optional>

using namespace BT;
namespace spr_decision
{
  class SpinOrNot : public SyncActionNode
  {
  public:
    SpinOrNot(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    ~SpinOrNot() override = default;

    NodeStatus tick() override;
    static PortsList providedPorts();

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr spin_or_not_pub_;
    std::optional<uint8_t> last_sent_;
  };
} //end namespace spr_decision
#endif