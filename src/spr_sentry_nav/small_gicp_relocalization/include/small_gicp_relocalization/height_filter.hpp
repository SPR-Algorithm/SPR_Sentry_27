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

#ifndef SMALL_GICP_RELOCALIZATION__HEIGHT_FILTER_HPP_
#define SMALL_GICP_RELOCALIZATION__HEIGHT_FILTER_HPP_

#include <cmath>
#include <memory>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace small_gicp_relocalization
{

// Test height in map, but keep coordinates in the input frame for registration.
// Applying the same numerical z threshold in lidar/odom and map is incorrect
// when their origins or roll/pitch differ.
inline pcl::PointCloud<pcl::PointXYZ>::Ptr filterByMapHeight(
  const pcl::PointCloud<pcl::PointXYZ> & input, const Eigen::Isometry3d & map_from_input,
  double max_z)
{
  auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  output->header = input.header;
  output->sensor_origin_ = input.sensor_origin_;
  output->sensor_orientation_ = input.sensor_orientation_;
  output->reserve(input.size());
  for (const auto & point : input) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    // Zero disables height clipping; non-finite measurements are still rejected.
    if (max_z != 0.0) {
      const double map_z = map_from_input.linear().row(2).dot(
        point.getVector3fMap().cast<double>()) + map_from_input.translation().z();
      if (!std::isfinite(map_z) || map_z > max_z) {
        continue;
      }
    }
    output->push_back(point);
  }
  output->is_dense = true;
  return output;
}

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__HEIGHT_FILTER_HPP_
