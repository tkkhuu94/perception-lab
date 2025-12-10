#include "lidar/slam/map/map.h"

namespace lidar {
namespace slam {
namespace map {

absl::StatusOr<std::unique_ptr<Map>>
Map::Create(std::unique_ptr<IDownSample<pcl::PointXYZI>> down_sampler) {
  if (down_sampler == nullptr) {
    return absl::InvalidArgumentError("down_sampler is null");
  }

  std::unique_ptr<Map> map(new Map());

  map->down_sampler_ = std::move(down_sampler);

  return map;
}

} // namespace map
} // namespace slam
} // namespace lidar
