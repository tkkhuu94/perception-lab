#ifndef LIDAR_SLAM_FEATURE_EXTRACTOR_FEATURE_EXTRACTOR_PARAMS_H_
#define LIDAR_SLAM_FEATURE_EXTRACTOR_FEATURE_EXTRACTOR_PARAMS_H_

#include <string>

#include "absl/status/statusor.h"
#include "pcl/point_cloud.h"

namespace lidar {
namespace slam {
namespace feature_extractor {

struct IFeatureExtractorParams {
  virtual ~IFeatureExtractorParams() = default;
};

} // namespace feature_extractor
} // namespace slam
} // namespace lidar

#endif
