#ifndef LIDAR_SLAM_FEATURE_EXTRACTOR_CURVATURE_CALCULATOR_PARAMS_H_
#define LIDAR_SLAM_FEATURE_EXTRACTOR_CURVATURE_CALCULATOR_PARAMS_H_

#include "lidar/slam/feature_extractor/feature_extractor_params.h"

namespace lidar {
namespace slam {
namespace feature_extractor {

struct CurvatureCalculatorParams : public IFeatureExtractorParams {
  float curvature_threshold;
  float search_radius;
};

} // namespace feature_extractor
} // namespace slam
} // namespace lidar

#endif
