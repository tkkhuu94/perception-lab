#ifndef LIDAR_SLAM_FEATURE_EXTRACTOR_FACTORY_H_
#define LIDAR_SLAM_FEATURE_EXTRACTOR_FACTORY_H_

#include <memory>

#include "lidar/slam/feature_extractor/curvature_calculator.h"
#include "lidar/slam/feature_extractor/feature_extractor.h"
#include "lidar/slam/feature_extractor/feature_extractor_params.h"

namespace lidar {
namespace slam {
namespace feature_extractor {

enum class FeatureType {
  kUnknown = 0,
  kCurvature,
};

class FeatureExtractorFactory {
public:
  template <typename PointT>
  static std::unique_ptr<IFeatureExtractor<PointT>>
  Create(const FeatureType &type, const IFeatureExtractorParams &params);
};

template <typename PointT>
std::unique_ptr<IFeatureExtractor<PointT>>
FeatureExtractorFactory::Create(const FeatureType &type,
                                const IFeatureExtractorParams &params) {
  switch (type) {

  case FeatureType::kCurvature:
    return CurvatureCalculator<PointT>::Create(params);

  default:
    return nullptr;
  }
}

} // namespace feature_extractor
} // namespace slam
} // namespace lidar

#endif
