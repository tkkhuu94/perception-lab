#include "lidar/slam/map/map.h"

namespace lidar {
namespace slam {
namespace map {

absl::StatusOr<std::unique_ptr<Map>> Map::Create(
    std::unique_ptr<IDownSample<pcl::PointXYZI>> down_sampler,
    std::unique_ptr<IFeatureExtractor<pcl::PointXYZI>> feature_extractor,
    std::unique_ptr<IMatcher<pcl::PointXYZI>> matcher) {
  if (down_sampler == nullptr) {
    return absl::InvalidArgumentError("down_sampler is null");
  }

  if (feature_extractor == nullptr) {
    return absl::InvalidArgumentError("feature_extractor is null");
  }

  if (matcher == nullptr) {
    return absl::InvalidArgumentError("matcher is null");
  }

  return std::unique_ptr<Map>(new Map(std::move(down_sampler),
                                      std::move(feature_extractor),
                                      std::move(matcher)));
}

absl::StatusOr<Eigen::Matrix4f>
Map::UpdateMap(pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud) {
  if (raw_cloud == nullptr) {
    return absl::InvalidArgumentError("raw_cloud is null");
  }

  auto processed_cloud = down_sampler_->Apply(*raw_cloud);
  auto feature_cloud = feature_extractor_->Extract(processed_cloud);

  if (!feature_cloud.ok()) {
    return feature_cloud.status();
  }

  if (prev_feature_cloud_ != nullptr) {
    auto transform = matcher_->CalculateTransform(prev_feature_cloud_,
                                                  feature_cloud->makeShared());
    if (!transform.ok()) {
      return transform.status();
    }

    global_pose_ = global_pose_ * transform.value();
  }

  prev_feature_cloud_ = feature_cloud->makeShared();
  return global_pose_;
}

Map::Map(std::unique_ptr<IDownSample<pcl::PointXYZI>> down_sampler,
         std::unique_ptr<IFeatureExtractor<pcl::PointXYZI>> feature_extractor,
         std::unique_ptr<IMatcher<pcl::PointXYZI>> matcher)
    : down_sampler_(std::move(down_sampler)),
      feature_extractor_(std::move(feature_extractor)),
      matcher_(std::move(matcher)),
      map_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()),
      global_pose_(Eigen::Matrix4f::Identity()) {}

} // namespace map
} // namespace slam
} // namespace lidar
