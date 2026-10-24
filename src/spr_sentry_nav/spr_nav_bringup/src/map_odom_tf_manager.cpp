// Sole dynamic map->odom authority. Holding this edge does not freeze odometry
// and is not a guarantee of localization accuracy or permission to keep moving.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace
{
using Transform = geometry_msgs::msg::TransformStamped;
using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
using Status = diagnostic_msgs::msg::DiagnosticStatus;

tf2::Transform validatedTransform(
  double x, double y, double z, const geometry_msgs::msg::Quaternion & orientation)
{
  for (const auto v : {x, y, z, orientation.x, orientation.y, orientation.z, orientation.w}) {
    if (!std::isfinite(v)) {
      throw std::invalid_argument("non-finite transform");
    }
  }
  tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
  const double norm = q.length();
  if (!std::isfinite(norm) || norm < 1e-9 || std::abs(norm - 1.0) > 0.01) {
    throw std::invalid_argument("invalid quaternion");
  }
  q.normalize();
  return tf2::Transform(q, tf2::Vector3(x, y, z));
}

tf2::Transform fromTransform(const geometry_msgs::msg::Transform & value)
{
  return validatedTransform(
    value.translation.x, value.translation.y, value.translation.z, value.rotation);
}

geometry_msgs::msg::Transform toTransform(const tf2::Transform & value)
{
  geometry_msgs::msg::Transform result;
  result.translation.x = value.getOrigin().x();
  result.translation.y = value.getOrigin().y();
  result.translation.z = value.getOrigin().z();
  const auto q = value.getRotation().normalized();
  result.rotation.x = q.x();
  result.rotation.y = q.y();
  result.rotation.z = q.z();
  result.rotation.w = q.w();
  return result;
}
}  // namespace

class MapOdomTfManager : public rclcpp::Node
{
public:
  MapOdomTfManager() : Node("map_odom_tf_manager")
  {
    mode_ = declare_parameter<std::string>("source_mode", "static");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    timeout_ = declare_parameter<double>("source_timeout", 2.0);
    const auto period = declare_parameter<double>("publish_period", 0.05);
    future_tolerance_ = declare_parameter<double>("future_tolerance", 0.2);
    tolerance_ = declare_parameter<double>("transform_tolerance", 0.1);
    const auto initial = declare_parameter<std::vector<double>>(
      "initial_pose", std::vector<double>{0.0, 0.0, 0.0});  // x,y,yaw
    if (mode_ != "gicp" && mode_ != "slam" && mode_ != "static") {
      throw std::invalid_argument("source_mode must be gicp/slam/static");
    }
    for (const auto & frame : {map_frame_, odom_frame_, base_frame_}) {
      if (frame.empty() || frame.front() == '/') {
        throw std::invalid_argument("TF frame names must be nonempty and relative");
      }
    }
    if (map_frame_ == odom_frame_ || map_frame_ == base_frame_ || odom_frame_ == base_frame_) {
      throw std::invalid_argument("map/odom/base frames must differ");
    }
    for (const auto value : {timeout_, period, future_tolerance_, tolerance_}) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("timing parameters must be finite");
      }
    }
    if (timeout_ <= 0 || period < 1e-9 || future_tolerance_ < 0 || tolerance_ < 0) {
      throw std::invalid_argument("timeout/period must be positive; tolerances nonnegative");
    }
    if (initial.size() != 3 || !std::all_of(initial.begin(), initial.end(), [](double v) {
        return std::isfinite(v);
      }))
    {
      throw std::invalid_argument("initial_pose must contain finite [x,y,yaw]");
    }
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, initial[2]);
    current_ = tf2::Transform(rotation, tf2::Vector3(initial[0], initial[1], 0.0));
    last_now_ns_ = now().nanoseconds();
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // Attach the listener to this node so namespace and /tf remappings are inherited.
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, this, false);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("tf_manager/status", 10);
    if (mode_ == "gicp") {
      gicp_sub_ = create_subscription<Transform>(
        "localization/gicp_transform", 10,
        std::bind(&MapOdomTfManager::onGicp, this, std::placeholders::_1));
    } else if (mode_ == "slam") {
      slam_sub_ = create_subscription<Pose>(
        "localization/slam_pose", 10,
        std::bind(&MapOdomTfManager::onSlam, this, std::placeholders::_1));
    }
    // Schedule with steady time: a system/ROS clock rewind must not stall the
    // sole TF publisher until the old deadline is reached. Keep ROS timestamps;
    // the callbacks below suppress publication while the replay clock is paused.
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&MapOdomTfManager::tick, this));
    status_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MapOdomTfManager::publishStatus, this));
  }

private:
  int64_t nowNs()
  {
    const auto time = now().nanoseconds();
    if (time < last_now_ns_) {
      // A replay seek invalidates freshness, never resets the held numeric value.
      has_source_stamp_ = false;
      pending_pose_.reset();
      buffer_->clear();
      RCLCPP_WARN(get_logger(), "ROS time moved backwards; keeping the last map->odom value");
    }
    last_now_ns_ = time;
    return time;
  }

  bool validStamp(int64_t stamp)
  {
    const double age = (nowNs() - stamp) / 1e9;
    return stamp > 0 && age >= -future_tolerance_ && age <= timeout_ &&
           (!has_source_stamp_ || stamp > last_source_ns_);
  }

  void accept(const tf2::Transform & value, int64_t stamp)
  {
    if (validStamp(stamp)) {
      // Validate the composed SLAM result too (overflow must not poison /tf).
      current_ = fromTransform(toTransform(value));
      last_source_ns_ = stamp;
      has_source_stamp_ = true;
      has_result_ = true;
      rejection_.clear();
    }
  }

  void onGicp(const Transform::ConstSharedPtr msg)
  {
    if (msg->header.frame_id != map_frame_ || msg->child_frame_id != odom_frame_) {
      rejection_ = "GICP frame mismatch";
      return;
    }
    try {
      accept(fromTransform(msg->transform), rclcpp::Time(msg->header.stamp).nanoseconds());
    } catch (const std::exception & error) {
      rejection_ = error.what();
    }
  }

  void onSlam(const Pose::ConstSharedPtr msg)
  {
    if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000U) {
      rejection_ = "invalid SLAM timestamp";
      return;
    }
    const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
    if (msg->header.frame_id != map_frame_ || !validStamp(stamp)) {
      rejection_ = "SLAM frame or timestamp mismatch";
      return;
    }
    if (pending_pose_ && stamp <= rclcpp::Time(pending_pose_->header.stamp).nanoseconds()) {
      return;
    }
    pending_pose_ = msg;
    trySlam();
  }

  void trySlam()
  {
    const auto msg = pending_pose_;
    if (!msg) {
      return;
    }
    const rclcpp::Time stamp(msg->header.stamp);
    if (!validStamp(stamp.nanoseconds())) {
      pending_pose_.reset();
      return;
    }
    try {
      // Never use latest TF here: the corrected pose belongs to this scan timestamp.
      // Use BufferCore's immediate lookup, not the ROS timeout overload: callbacks
      // share one executor and missing transforms are retried by our own timer.
      const auto odom_base = buffer_->lookupTransform(
        odom_frame_, base_frame_, tf2::TimePoint(std::chrono::nanoseconds(stamp.nanoseconds())));
      const auto & pose = msg->pose.pose;
      const auto map_base = validatedTransform(
        pose.position.x, pose.position.y, pose.position.z, pose.orientation);
      accept(map_base * fromTransform(odom_base.transform).inverse(), stamp.nanoseconds());
      pending_pose_.reset();
    } catch (const tf2::TransformException &) {
      // Nonblocking retry on the next tick while the input is still fresh.
      rejection_ = "waiting for odom->base at SLAM pose timestamp";
    } catch (const std::exception & error) {
      rejection_ = error.what();
      pending_pose_.reset();
    }
  }

  void tick()
  {
    const auto time = nowNs();
    if (time == 0 || time == last_publish_ns_) {
      return;  // No /clock yet, or bag playback is paused.
    }
    trySlam();
    Transform msg;
    msg.header.stamp = rclcpp::Time(time, get_clock()->get_clock_type()) +
      rclcpp::Duration::from_seconds(tolerance_);
    msg.header.frame_id = map_frame_;
    msg.child_frame_id = odom_frame_;
    msg.transform = toTransform(current_);
    broadcaster_->sendTransform(msg);
    last_publish_ns_ = time;
  }

  void publishStatus()
  {
    const auto time = nowNs();
    if (time == 0 || time == last_status_ns_) {
      return;
    }
    last_status_ns_ = time;
    const double age = has_source_stamp_ ? std::max(0.0, (time - last_source_ns_) / 1e9) : -1.0;
    Status status;
    status.name = get_fully_qualified_name();
    status.hardware_id = "map_to_odom";
    status.level = Status::WARN;
    if (mode_ == "static") {
      status.message = "FIXED";
      status.level = Status::OK;
    } else if (!has_result_) {
      status.message = "UNINITIALIZED";
    } else if (!has_source_stamp_ || age > timeout_) {
      status.message = "HOLDING_LAST";
    } else {
      status.message = "TRACKING";
      status.level = Status::OK;
    }
    if (status.message != state_) {
      state_ = status.message;
      if (status.level == Status::OK) {
        RCLCPP_INFO(get_logger(), "map->odom: %s (source=%s)", state_.c_str(), mode_.c_str());
      } else {
        RCLCPP_WARN(
          get_logger(), "map->odom: %s (source=%s); holding TF is not localization health",
          state_.c_str(), mode_.c_str());
      }
    }
    auto field = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue entry;
        entry.key = key;
        entry.value = value;
        status.values.push_back(entry);
      };
    field("source", mode_);
    field("last_result_age", has_source_stamp_ ? std::to_string(age) : "None");
    field("input_note", rejection_);
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = now();
    msg.status.push_back(status);
    diagnostics_->publish(msg);
  }

  std::string mode_, map_frame_, odom_frame_, base_frame_, state_, rejection_;
  double timeout_, future_tolerance_, tolerance_;
  int64_t last_now_ns_{0}, last_source_ns_{0};
  int64_t last_publish_ns_{-1}, last_status_ns_{-1};
  bool has_source_stamp_{false}, has_result_{false};
  tf2::Transform current_;
  Pose::ConstSharedPtr pending_pose_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<Transform>::SharedPtr gicp_sub_;
  rclcpp::Subscription<Pose>::SharedPtr slam_sub_;
  rclcpp::TimerBase::SharedPtr timer_, status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<MapOdomTfManager>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("map_odom_tf_manager"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
