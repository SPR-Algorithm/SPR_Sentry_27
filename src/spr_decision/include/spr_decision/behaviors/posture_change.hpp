#ifndef SPR_DECISION_POSTURE_CHANGE_HPP_
#define SPR_DECISION_POSTURE_CHANGE_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include <std_msgs/msg/u_int8.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <optional>

using namespace BT;
namespace spr_decision
{
  class PostureChange : public SyncActionNode
  {
  public:
    PostureChange(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    ~PostureChange() override = default;

    NodeStatus tick() override;
    static PortsList providedPorts();

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr posture_change_pub_;
  };
} //end namespace spr_decision
#endif