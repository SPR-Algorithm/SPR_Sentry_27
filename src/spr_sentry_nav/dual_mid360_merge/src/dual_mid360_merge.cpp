#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>

class DualMid360Merge : public rclcpp::Node
{
public:
  DualMid360Merge()
  : Node("merge_cloud_node")
  {
    lid_topic_1_ = this->declare_parameter<std::string>("lid_topic_1", "livox/lidar_192_168_1_194");
    lid_topic_2_ = this->declare_parameter<std::string>("lid_topic_2", "livox/lidar_192_168_1_104");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "livox/lidar_merged");
    output_frame_id_ = this->declare_parameter<std::string>("output_frame_id", "front_mid360");
    sync_queue_size_ = this->declare_parameter<int>("sync_queue_size", 30);
    max_sync_slop_sec_ = this->declare_parameter<double>("max_sync_slop_sec", 0.05);
    max_point_offset_ns_ = this->declare_parameter<int64_t>("max_point_offset_ns", 150000000LL);
    output_queue_depth_ = this->declare_parameter<int>("output_queue_depth", 30);

    std::vector<double> rpy_default = {0.0, 0.0, 0.0};
    std::vector<double> xyz_default = {0.0, 0.0, 0.0};
    auto rpy2 = this->declare_parameter<std::vector<double>>("extrinsic_cloud2_rpy", rpy_default);
    auto xyz2 = this->declare_parameter<std::vector<double>>("extrinsic_cloud2_xyz", xyz_default);
    if (rpy2.size() == 3U && xyz2.size() == 3U) {
      roll2_ = static_cast<float>(rpy2[0]);
      pitch2_ = static_cast<float>(rpy2[1]);
      yaw2_ = static_cast<float>(rpy2[2]);
      tx2_ = static_cast<float>(xyz2[0]);
      ty2_ = static_cast<float>(xyz2[1]);
      tz2_ = static_cast<float>(xyz2[2]);
    }
    setTransformFromExtrinsic();

    auto qos = rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(std::max(1, sync_queue_size_)));

    sub1_.subscribe(this, lid_topic_1_, qos.get_rmw_qos_profile());
    sub2_.subscribe(this, lid_topic_2_, qos.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(static_cast<uint32_t>(std::max(1, sync_queue_size_))), sub1_, sub2_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_sync_slop_sec_));
    sync_->registerCallback(
      std::bind(&DualMid360Merge::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

    merged_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(
      output_topic_,
      rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(std::max(1, output_queue_depth_))));

    RCLCPP_INFO(
      this->get_logger(),
      "dual_mid360_merge(CustomMsg): %s + %s -> %s | frame_id=%s | extrinsic 雷达2→雷达1(194) slop=%.3fs",
      lid_topic_1_.c_str(), lid_topic_2_.c_str(), output_topic_.c_str(), output_frame_id_.c_str(),
      max_sync_slop_sec_);

    diag_timer_ = this->create_wall_timer(
      std::chrono::seconds(5), std::bind(&DualMid360Merge::diagTimerCallback, this));
  }

private:
  void diagTimerCallback()
  {
    if (merged_publish_count_ > 0U) {
      diag_timer_->cancel();
      return;
    }
    RCLCPP_WARN(
      this->get_logger(),
      "尚未发布融合 CustomMsg：请确认 xfer_format:=1、multi_topic:=1，且两路 livox/lidar_* 有数据并在 "
      "max_sync_slop_sec 内可对齐；Point-LIO 请使用 lid_topic:=输出话题、imu_topic:=livox/imu_192_168_1_194");
    diag_timer_->cancel();
  }

  static void setTransformMatrix(
    Eigen::Matrix4f & transform, float roll, float pitch, float yaw, float tx, float ty, float tz)
  {
    Eigen::AngleAxisf roll_angle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch_angle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw_angle(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Quaternion<float> q = yaw_angle * pitch_angle * roll_angle;
    transform.setIdentity();
    transform.block<3, 3>(0, 0) = q.matrix();
    transform(0, 3) = tx;
    transform(1, 3) = ty;
    transform(2, 3) = tz;
  }

  void setTransformFromExtrinsic()
  {
    Eigen::Matrix4f T_2_to_1 = Eigen::Matrix4f::Identity();
    setTransformMatrix(T_2_to_1, roll2_, pitch2_, yaw2_, tx2_, ty2_, tz2_);
    R_2_to_1_ = T_2_to_1.block<3, 3>(0, 0);
    t_2_to_1_ = T_2_to_1.block<3, 1>(0, 3);
  }

  static bool computeRelativeOffset(
    const int64_t base_ns, const uint32_t point_offset_ns, const int64_t out_base_ns, uint32_t & out_rel_ns)
  {
    const int64_t abs_ns = base_ns + static_cast<int64_t>(point_offset_ns);
    const int64_t rel_ns = abs_ns - out_base_ns;
    if (rel_ns < 0LL || rel_ns > static_cast<int64_t>(std::numeric_limits<uint32_t>::max())) {
      return false;
    }
    out_rel_ns = static_cast<uint32_t>(rel_ns);
    return true;
  }

  void transformPoint(
    const livox_ros_driver2::msg::CustomPoint & in,
    livox_ros_driver2::msg::CustomPoint & out)
  {
    const float x = in.x;
    const float y = in.y;
    const float z = in.z;
    out.x = R_2_to_1_(0, 0) * x + R_2_to_1_(0, 1) * y + R_2_to_1_(0, 2) * z + t_2_to_1_(0);
    out.y = R_2_to_1_(1, 0) * x + R_2_to_1_(1, 1) * y + R_2_to_1_(1, 2) * z + t_2_to_1_(1);
    out.z = R_2_to_1_(2, 0) * x + R_2_to_1_(2, 1) * y + R_2_to_1_(2, 2) * z + t_2_to_1_(2);
    out.reflectivity = in.reflectivity;
    out.tag = in.tag;
    out.line = in.line;
  }

  void syncCallback(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg1,
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg2)
  {
    const int64_t base1 = static_cast<int64_t>(msg1->timebase);
    const int64_t base2 = static_cast<int64_t>(msg2->timebase);
    const int64_t out_base = std::min(base1, base2);

    if (msg1->points.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "参考雷达 livox/lidar_192_168_1_194 点云为空，跳过融合");
      return;
    }

    out_buffer_.header.frame_id = output_frame_id_;
    out_buffer_.timebase = static_cast<uint64_t>(out_base);
    out_buffer_.lidar_id = msg1->lidar_id;
    out_buffer_.rsvd[0] = 0;
    out_buffer_.rsvd[1] = 0;
    out_buffer_.rsvd[2] = 0;
    out_buffer_.points.clear();

    const size_t n1 = msg1->points.size();
    const size_t n2 = msg2->points.size();
    out_buffer_.points.reserve(n1 + n2);
    size_t skipped_invalid_offset = 0U;
    size_t skipped_over_max = 0U;

    for (size_t i = 0; i < n1; ++i) {
      if (base1 == out_base) {
        out_buffer_.points.push_back(msg1->points[i]);
        continue;
      }
      livox_ros_driver2::msg::CustomPoint p = msg1->points[i];
      uint32_t rel_ns = 0U;
      if (!computeRelativeOffset(base1, msg1->points[i].offset_time, out_base, rel_ns)) {
        ++skipped_invalid_offset;
        continue;
      }
      if (static_cast<int64_t>(rel_ns) > max_point_offset_ns_) {
        ++skipped_over_max;
        continue;
      }
      p.offset_time = rel_ns;
      out_buffer_.points.push_back(p);
    }

    for (size_t i = 0; i < n2; ++i) {
      livox_ros_driver2::msg::CustomPoint p_out{};
      transformPoint(msg2->points[i], p_out);
      uint32_t rel_ns = 0U;
      if (!computeRelativeOffset(base2, msg2->points[i].offset_time, out_base, rel_ns)) {
        ++skipped_invalid_offset;
        continue;
      }
      if (static_cast<int64_t>(rel_ns) > max_point_offset_ns_) {
        ++skipped_over_max;
        continue;
      }
      p_out.offset_time = rel_ns;
      out_buffer_.points.push_back(p_out);
    }

    if (skipped_invalid_offset > 0U || skipped_over_max > 0U) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "融合点过滤统计: invalid_offset=%zu, over_max=%zu (max_point_offset_ns=%lld)",
        skipped_invalid_offset, skipped_over_max, static_cast<long long>(max_point_offset_ns_));
    }

    out_buffer_.point_num = static_cast<uint32_t>(out_buffer_.points.size());
    if (out_buffer_.points.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "融合后无有效点（可能两雷达时间基差过大或外参导致全部过滤），跳过发布");
      return;
    }

    // header.stamp 保持参考雷达 ROS 时间轴（与 livox/imu_192_168_1_194 一致）；timebase/offset_time 用 out_base 对齐
    rclcpp::Time t_ros(msg1->header.stamp);
    if (last_out_valid_) {
      if (t_ros <= last_out_stamp_) {
        t_ros = last_out_stamp_ + rclcpp::Duration(0, 1);
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "融合输出 header.stamp 非单调，已 +1ns（避免 Point-LIO lidar loop back）");
      }
    }
    last_out_stamp_ = t_ros;
    last_out_valid_ = true;
    out_buffer_.header.stamp = t_ros;

    merged_pub_->publish(out_buffer_);
    merged_publish_count_++;
    if (merged_publish_count_ == 1U) {
      if (diag_timer_) {
        diag_timer_->cancel();
      }
      RCLCPP_INFO(
        this->get_logger(), "首次发布融合 CustomMsg: point_num=%u -> %s", out_buffer_.point_num,
        output_topic_.c_str());
    }
  }

  typedef message_filters::sync_policies::ApproximateTime<
    livox_ros_driver2::msg::CustomMsg, livox_ros_driver2::msg::CustomMsg>
    SyncPolicy;

  message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> sub1_;
  message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> sub2_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr merged_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  uint64_t merged_publish_count_{0};
  rclcpp::Time last_out_stamp_{0, 0, RCL_ROS_TIME};
  bool last_out_valid_{false};

  std::string lid_topic_1_;
  std::string lid_topic_2_;
  std::string output_topic_;
  std::string output_frame_id_;
  int sync_queue_size_{10};
  double max_sync_slop_sec_{0.05};
  int64_t max_point_offset_ns_{150000000LL};
  int output_queue_depth_{30};

  float roll2_{0.0F};
  float pitch2_{0.0F};
  float yaw2_{0.0F};
  float tx2_{0.0F};
  float ty2_{0.0F};
  float tz2_{0.0F};
  Eigen::Matrix3f R_2_to_1_{Eigen::Matrix3f::Identity()};
  Eigen::Vector3f t_2_to_1_{Eigen::Vector3f::Zero()};
  livox_ros_driver2::msg::CustomMsg out_buffer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualMid360Merge>());
  rclcpp::shutdown();
  return 0;
}
