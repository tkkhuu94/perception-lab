#ifndef LIDAR_SLAM_MAP_MAP_H_
#define LIDAR_SLAM_MAP_MAP_H_

#include "absl/status/statusor.h"
#include "pcl/point_types.h"

#include "lidar/slam/down_sample/down_sample.h"

using lidar::slam::down_sample::IDownSample;

namespace lidar {
namespace slam {
namespace map {

class Map {
public:
  static absl::StatusOr<std::unique_ptr<Map>>
  Create(std::unique_ptr<IDownSample<pcl::PointXYZI>> down_sampler);

private:
  Map() = default;

  std::unique_ptr<IDownSample<pcl::PointXYZI>> down_sampler_;
};

} // namespace map
} // namespace slam
} // namespace lidar

#endif
