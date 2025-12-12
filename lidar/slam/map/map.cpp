#include "lidar/slam/map/map.h"

#include "pcl/common/transforms.h"

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

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  outlier_filter_.setInputCloud(processed_cloud.makeShared());
  outlier_filter_.filter(*filtered_cloud);

  auto feature_cloud = feature_extractor_->Extract(*filtered_cloud);

  if (!feature_cloud.ok()) {
    return feature_cloud.status();
  }

  if (prev_feature_cloud_ != nullptr) {
    auto transform = matcher_->CalculateTransform(feature_cloud->makeShared(),
                                                  prev_feature_cloud_);
    if (!transform.ok()) {
      return transform.status();
    }

    global_pose_ = global_pose_ * transform.value();

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*filtered_cloud, *transformed_cloud, global_pose_);

    *map_ += *transformed_cloud;
  } else {
    map_ = feature_cloud->makeShared();
  }

  prev_feature_cloud_ = feature_cloud->makeShared();
  return global_pose_;
}

const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &
Map::PointCloudMap() const {
  return map_;
}

Map::Map(std::unique_ptr<IDownSample<pcl::PointXYZI>> down_sampler,
         std::unique_ptr<IFeatureExtractor<pcl::PointXYZI>> feature_extractor,
         std::unique_ptr<IMatcher<pcl::PointXYZI>> matcher)
    : down_sampler_(std::move(down_sampler)),
      feature_extractor_(std::move(feature_extractor)),
      matcher_(std::move(matcher)),
      map_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()),
      global_pose_(Eigen::Matrix4f::Identity()) {
  outlier_filter_.setMeanK(50);
  outlier_filter_.setStddevMulThresh(0.8);
}

} // namespace map
} // namespace slam
} // namespace lidar
