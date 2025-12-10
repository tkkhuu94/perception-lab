#ifndef LIDAR_SLAM_FEATURE_EXTRACTOR_H_
#define LIDAR_SLAM_FEATURE_EXTRACTOR_H_

#include <string>

#include "absl/status/statusor.h"
#include "pcl/point_cloud.h"

namespace lidar {
namespace slam {
namespace feature_extractor {

template <typename PointT> class IFeatureExtractor {
public:
  using FeatureCloud = pcl::PointCloud<PointT>;

  virtual ~IFeatureExtractor() = default;

  virtual std::string Type() const { return "FeatureExtractor"; }

  virtual absl::StatusOr<FeatureCloud>
  Extract(const pcl::PointCloud<PointT> &input_cloud) = 0;
};

} // namespace feature_extractor
} // namespace slam
} // namespace lidar

#endif
