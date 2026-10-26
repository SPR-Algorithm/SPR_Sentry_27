// Copyright 2026 SPR Algorithm
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <iostream>
#include <limits>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <unistd.h>
#include <random>

#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "small_gicp_relocalization/height_filter.hpp"
#include "small_gicp_relocalization/small_gicp_relocalization.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"

namespace
{

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using CovCloud = pcl::PointCloud<pcl::PointCovariance>;
using small_gicp_relocalization::filterByMapHeight;

TEST(HeightFilter, InclusiveBoundaryAndNonFinitePoints)
{
  Cloud cloud;
  cloud.emplace_back(1.0f, 2.0f, 2.0f);
  cloud.emplace_back(1.0f, 2.0f, 2.001f);
  cloud.emplace_back(1.0f, 2.0f, -3.0f);
  cloud.emplace_back(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f);
  cloud.emplace_back(0.0f, std::numeric_limits<float>::infinity(), 0.0f);
  cloud.emplace_back(0.0f, 0.0f, std::numeric_limits<float>::infinity());
  const auto filtered = filterByMapHeight(cloud, Eigen::Isometry3d::Identity(), 2.0);
  ASSERT_EQ(filtered->size(), 2U);
  EXPECT_FLOAT_EQ(filtered->at(0).z, 2.0f);
  EXPECT_FLOAT_EQ(filtered->at(1).z, -3.0f);
  EXPECT_EQ(cloud.size(), 6U);
  EXPECT_EQ(filterByMapHeight(cloud, Eigen::Isometry3d::Identity(), 0.0)->size(), 3U);
  EXPECT_TRUE(filterByMapHeight(Cloud(), Eigen::Isometry3d::Identity(), 2.0)->empty());
}

TEST(HeightFilter, UsesMapHeightButPreservesInputCoordinates)
{
  Eigen::Isometry3d map_from_odom = Eigen::Isometry3d::Identity();
  map_from_odom.linear() = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()).toRotationMatrix();
  map_from_odom.translation().z() = 0.75;
  Cloud source;
  source.emplace_back(4.0f, 1.0f, 2.5f);   // raw z > 2, map z < 2
  source.emplace_back(-4.0f, 1.0f, 1.0f);  // raw z < 2, map z > 2
  Cloud target;
  pcl::transformPointCloud(source, target, map_from_odom.matrix());
  const auto kept_source = filterByMapHeight(source, map_from_odom, 2.0);
  const auto kept_target = filterByMapHeight(target, Eigen::Isometry3d::Identity(), 2.0);
  ASSERT_EQ(kept_source->size(), 1U);
  ASSERT_EQ(kept_target->size(), 1U);
  EXPECT_FLOAT_EQ(kept_source->front().z, 2.5f);
  const Eigen::Vector3d transformed =
    map_from_odom * kept_source->front().getVector3fMap().cast<double>();
  EXPECT_LT((transformed - kept_target->front().getVector3fMap().cast<double>()).norm(), 1e-5);
}

// Uneven floor and perpendicular, finite walls retain 3D constraints after z clipping.
Cloud makeRoom()
{
  Cloud cloud;
  std::mt19937 generator(42);
  std::uniform_real_distribution<float> unit(0.0f, 1.0f);
  for (int i = 0; i < 6000; ++i) {
    const float a = unit(generator);
    const float b = unit(generator);
    cloud.emplace_back(8.0f * a - 3.0f, 6.0f * b - 2.0f, 0.04f * std::sin(8.0f * a));
    cloud.emplace_back(-3.0f, 6.0f * a - 2.0f, 4.0f * b);
    cloud.emplace_back(8.0f * a - 3.0f, -2.0f, 4.0f * b);
    if (a > 0.5f) {
      cloud.emplace_back(1.1f, 2.0f * a - 0.5f, 1.4f * b);
    }
  }
  return cloud;
}

TEST(HeightFilter, GicpRecoversKnownPoseAfterConsistentClipping)
{
  const Cloud raw_target = makeRoom();
  Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
  truth.linear() = (
    Eigen::AngleAxisd(0.16, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.04, Eigen::Vector3d::UnitY())).toRotationMatrix();
  truth.translation() << 0.35, -0.25, 0.15;
  Cloud raw_source;
  pcl::transformPointCloud(raw_target, raw_source, truth.inverse().matrix());

  // The height test uses an imperfect estimate; points remain in odom for GICP.
  Eigen::Isometry3d initial = truth;
  initial.translation() += Eigen::Vector3d(0.45, -0.35, 0.05);
  initial.linear() = Eigen::AngleAxisd(0.14, Eigen::Vector3d::UnitZ()) * truth.linear();
  const auto clipped_target = filterByMapHeight(
    raw_target, Eigen::Isometry3d::Identity(), 2.0);
  const auto clipped_source = filterByMapHeight(raw_source, initial, 2.0);
  ASSERT_LT(clipped_target->size(), raw_target.size());
  ASSERT_LT(clipped_source->size(), raw_source.size());
  const auto target = small_gicp::voxelgrid_sampling_omp<Cloud, CovCloud>(*clipped_target, 0.1, 2);
  const auto source = small_gicp::voxelgrid_sampling_omp<Cloud, CovCloud>(*clipped_source, 0.05, 2);
  small_gicp::estimate_covariances_omp(*target, 20, 2);
  small_gicp::estimate_covariances_omp(*source, 20, 2);
  small_gicp::KdTree<CovCloud> tree(target, small_gicp::KdTreeBuilderOMP(2));
  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
  registration.reduction.num_threads = 2;
  registration.rejector.max_dist_sq = 3.0;
  registration.optimizer.max_iterations = 30;
  const auto result = registration.align(*target, *source, tree, initial);
  std::cout << "clipped room: converged=" << result.converged <<
    " iterations=" << result.iterations + 1 <<
    " inliers=" << result.num_inliers << "/" << source->size() << std::endl;
  EXPECT_TRUE(result.converged);
  EXPECT_GT(static_cast<double>(result.num_inliers) / source->size(), 0.9);
  EXPECT_LT((result.T_target_source.translation() - truth.translation()).norm(), 0.03);
  EXPECT_LT(Eigen::AngleAxisd(result.T_target_source.linear().transpose() * truth.linear()).angle(),
    0.01);
}

// Exercise actual target initialization, ROS parameter loading, source filtering
// and output gating. This test starts no drivers, controllers or robot processes.
TEST(HeightFilter, NodeRejectsEmptyFilteredCloudAndPublishesValidMatch)
{
  ASSERT_EQ(setenv("ROS_LOCALHOST_ONLY", "1", 1), 0);
  rclcpp::InitOptions init_options;
  init_options.set_domain_id(100 + static_cast<size_t>(getpid()) % 100);
  rclcpp::init(0, nullptr, init_options);
  struct Shutdown
  {
    ~Shutdown() {rclcpp::shutdown();}
  } shutdown;
  char temporary_path[] = "/tmp/gicp_height_test_XXXXXX";
  const int fd = mkstemp(temporary_path);
  ASSERT_GE(fd, 0);
  close(fd);
  struct RemoveFile
  {
    const char * path;
    ~RemoveFile() {std::remove(path);}
  } remove_file{temporary_path};

  const Cloud target = makeRoom();
  Eigen::Isometry3d map_from_pcd = Eigen::Isometry3d::Identity();
  map_from_pcd.linear() = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()).toRotationMatrix();
  map_from_pcd.translation() << 0.0, 0.0, 0.3;
  Cloud raw_map;
  pcl::transformPointCloud(target, raw_map, map_from_pcd.inverse().matrix());
  ASSERT_EQ(pcl::io::savePCDFileBinary(temporary_path, raw_map), 0);

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"prior_pcd_file", std::string(temporary_path)},
    {"base_frame", "height_test_base"}, {"lidar_frame", "height_test_lidar"},
    {"odom_frame", "height_test_odom"}, {"map_frame", "height_test_map"},
    {"input_cloud_topic", "height_test_scan"},
    {"publish_tf", false}, {"height_filter.max_z", 2.0},
    {"global_leaf_size", 0.1}, {"registered_leaf_size", 0.05}, {"num_threads", 2},
  });
  auto node = std::make_shared<small_gicp_relocalization::SmallGicpRelocalizationNode>(options);
  auto peer = std::make_shared<rclcpp::Node>("height_filter_test_peer");
  auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(peer);
  auto extrinsic = tf2::eigenToTransform(map_from_pcd);
  extrinsic.header.frame_id = "height_test_base";
  extrinsic.child_frame_id = "height_test_lidar";
  extrinsic.header.stamp = peer->now();
  broadcaster->sendTransform(extrinsic);
  auto publisher = peer->create_publisher<sensor_msgs::msg::PointCloud2>("height_test_scan", 10);
  size_t received = 0;
  geometry_msgs::msg::TransformStamped correction;
  auto subscription = peer->create_subscription<geometry_msgs::msg::TransformStamped>(
    "localization/gicp_transform", 10,
    [&](const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
      correction = *msg;
      ++received;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(peer);
  const auto spin_for = [&](double seconds) {
      const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
      while (std::chrono::steady_clock::now() < end) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    };
  spin_for(1.0);
  const auto publish = [&](const Cloud & cloud, const std::string & frame) {
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(cloud, msg);
      msg.header.frame_id = frame;
      msg.header.stamp = peer->now();
      publisher->publish(msg);
    };

  Cloud overhead;
  for (int i = 0; i < 200; ++i) {
    overhead.emplace_back(0.01f * i, 0.3f, 5.0f);
  }
  publish(overhead, "height_test_odom");
  spin_for(0.8);
  EXPECT_EQ(received, 0U);
  publish(target, "incorrect_sensor_frame");
  spin_for(0.8);
  EXPECT_EQ(received, 0U);

  Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
  truth.linear() = Eigen::AngleAxisd(0.06, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  truth.translation() << 0.2, -0.1, 0.05;
  Cloud source;
  pcl::transformPointCloud(target, source, truth.inverse().matrix());
  publish(source, "height_test_odom");
  spin_for(1.0);
  ASSERT_GT(received, 0U);
  EXPECT_EQ(correction.header.frame_id, "height_test_map");
  EXPECT_EQ(correction.child_frame_id, "height_test_odom");
  const auto actual = tf2::transformToEigen(correction);
  EXPECT_LT((actual.translation() - truth.translation()).norm(), 0.03);
  EXPECT_LT(Eigen::AngleAxisd(actual.linear().transpose() * truth.linear()).angle(), 0.01);

  const size_t previous_received = received;
  Cloud no_overlap;
  pcl::transformPointCloud(
    source, no_overlap, Eigen::Translation3f(100.0f, 100.0f, 0.0f) *
    Eigen::Quaternionf::Identity());
  publish(no_overlap, "height_test_odom");
  spin_for(0.8);
  EXPECT_EQ(received, previous_received);
  publish(overhead, "height_test_odom");
  spin_for(0.8);
  EXPECT_EQ(received, previous_received);
}

// Optional real-map geometry check. The source is synthesized from a map
// crop with a known pose: this is not a recorded lidar scan or a field test.
TEST(HeightFilter, ExistingMapCropKnownPose)
{
  const char * map_path = std::getenv("GICP_TEST_PCD");
  if (!map_path) {
    GTEST_SKIP() << "Set GICP_TEST_PCD to run the optional existing-map geometry check";
  }
  Cloud raw_map;
  ASSERT_EQ(pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, raw_map), 0);
  Eigen::Isometry3d map_from_pcd = Eigen::Isometry3d::Identity();
  map_from_pcd.linear() = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()).toRotationMatrix();
  map_from_pcd.translation().z() = 0.3;
  Cloud map;
  pcl::transformPointCloud(raw_map, map, map_from_pcd.matrix());
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  for (const auto & point : map) {
    center += point.getVector3fMap() / static_cast<float>(map.size());
  }
  Cloud crop;
  for (const auto & point : map) {
    if ((point.getVector3fMap().head<2>() - center.head<2>()).norm() < 7.0) {
      crop.push_back(point);
    }
  }
  ASSERT_GT(crop.size(), 100U);
  Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
  truth.linear() = Eigen::AngleAxisd(0.12, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  truth.translation() << 0.3, -0.2, 0.1;
  Cloud raw_source;
  pcl::transformPointCloud(crop, raw_source, truth.inverse().matrix());
  Eigen::Isometry3d initial = truth;
  initial.translation() += Eigen::Vector3d(0.5, -0.4, 0.05);
  initial.linear() = Eigen::AngleAxisd(0.12, Eigen::Vector3d::UnitZ()) * truth.linear();
  for (double max_z : {0.0, 2.0}) {
    const auto clipped_target = filterByMapHeight(map, Eigen::Isometry3d::Identity(), max_z);
    const auto clipped_source = filterByMapHeight(raw_source, initial, max_z);
    const auto target = small_gicp::voxelgrid_sampling_omp<Cloud, CovCloud>(
      *clipped_target, 0.1, 4);
    const auto source = small_gicp::voxelgrid_sampling_omp<Cloud, CovCloud>(
      *clipped_source, 0.05, 4);
    ASSERT_GE(target->size(), 20U);
    ASSERT_GE(source->size(), 20U);
    small_gicp::estimate_covariances_omp(*target, 20, 4);
    small_gicp::estimate_covariances_omp(*source, 20, 4);
    small_gicp::KdTree<CovCloud> tree(target, small_gicp::KdTreeBuilderOMP(4));
    for (int budget : {10, 30}) {
      small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> reg;
      reg.reduction.num_threads = 4;
      reg.rejector.max_dist_sq = 3.0;
      reg.optimizer.max_iterations = budget;
      const auto start = std::chrono::steady_clock::now();
      const auto result = reg.align(*target, *source, tree, initial);
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
      const double translation_error =
        (result.T_target_source.translation() - truth.translation()).norm();
      const double rotation_error = Eigen::AngleAxisd(
        result.T_target_source.linear().transpose() * truth.linear()).angle();
      std::cout << map_path << " max_z=" << max_z << " budget=" << budget <<
        " target=" << target->size() << " source=" << source->size() <<
        " converged=" << result.converged << " iterations=" << result.iterations + 1 <<
        " inliers=" << result.num_inliers << " align_ms=" << elapsed_ms <<
        " translation_error_m=" << translation_error <<
        " rotation_error_rad=" << rotation_error << std::endl;
      if (budget == 30) {
        EXPECT_TRUE(result.converged);
        EXPECT_LT(translation_error, 0.05);
        EXPECT_LT(rotation_error, 0.01);
      }
    }
  }
}

}  // namespace
