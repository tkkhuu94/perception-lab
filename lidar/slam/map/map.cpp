#include "lidar/slam/map/map.h"

namespace lidar {
namespace slam {
namespace map {

absl::StatusOr<std::unique_ptr<Map>>
Map::Create(std::unique_ptr<IDownSample<pcl::PointXYZI>> down_sampler, std::unique_ptr<IFeatureExtractor<pcl::PointXYZI>> feature_extractor) {
  if (down_sampler == nullptr) {
    return absl::InvalidArgumentError("down_sampler is null");
  }

  if (feature_extractor == nullptr) {
    return absl::InvalidArgumentError("feature_extractor is null");
  }

  std::unique_ptr<Map> map(new Map());

  map->down_sampler_ = std::move(down_sampler);
  map->feature_extractor_ = std::move(feature_extractor);

  return map;
}

} // namespace map
} // namespace slam
} // namespace lidar
