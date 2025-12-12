#ifndef LIDAR_SLAM_MAP_MAP_H_
#define LIDAR_SLAM_MAP_MAP_H_

#include "absl/status/statusor.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "lidar/slam/down_sample/down_sample.h"
#include "lidar/slam/feature_extractor/feature_extractor.h"
#include "lidar/slam/matcher/matcher.h"

using lidar::slam::down_sample::IDownSample;
using lidar::slam::feature_extractor::IFeatureExtractor;
using lidar::slam::matcher::IMatcher;

namespace lidar {
namespace slam {
namespace map {

class Map {
public:
  static absl::StatusOr<std::unique_ptr<Map>>
  Create(std::unique_ptr<IDownSample<pcl::PointXYZI>> down_sampler,
         std::unique_ptr<IFeatureExtractor<pcl::PointXYZI>> feature_extractor,
         std::unique_ptr<IMatcher<pcl::PointXYZI>> matcher);

  absl::StatusOr<Eigen::Matrix4f>
  UpdateMap(pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud);

private:
  Map(std::unique_ptr<IDownSample<pcl::PointXYZI>> down_sampler,
      std::unique_ptr<IFeatureExtractor<pcl::PointXYZI>> feature_extractor,
      std::unique_ptr<IMatcher<pcl::PointXYZI>> matcher);

  std::unique_ptr<IDownSample<pcl::PointXYZI>> down_sampler_;
  std::unique_ptr<IFeatureExtractor<pcl::PointXYZI>> feature_extractor_;
  std::unique_ptr<IMatcher<pcl::PointXYZI>> matcher_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> map_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> prev_feature_cloud_;

  Eigen::Matrix4f global_pose_;
};

} // namespace map
} // namespace slam
} // namespace lidar

#endif
