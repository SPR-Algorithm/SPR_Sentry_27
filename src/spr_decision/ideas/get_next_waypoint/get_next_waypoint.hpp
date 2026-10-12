#ifndef SPR_DECISION__IDEAS__GET_NEXT_WAYPOINT_HPP_
#define SPR_DECISION__IDEAS__GET_NEXT_WAYPOINT_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace spr_decision
{

// Experimental reference implementation. It is not part of the package build.
class GetNextWaypoint : public BT::SyncActionNode
{
public:
  GetNextWaypoint(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<rclcpp::Node> node);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  struct WaypointRecord
  {
    std::string id;
    geometry_msgs::msg::PoseStamped pose;
    std::string command;
  };

  bool loadCsvIfNeeded(const std::string & csv_file);

  std::shared_ptr<rclcpp::Node> node_;
  std::string loaded_csv_file_;
  std::vector<WaypointRecord> waypoints_;
  std::size_t cursor_{0U};
};

}  // namespace spr_decision

#endif  // SPR_DECISION__IDEAS__GET_NEXT_WAYPOINT_HPP_
