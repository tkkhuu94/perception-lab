#include "lidar/slam/feature_extractor/curvature_calculator_params.h"

namespace lidar {
namespace slam {
namespace feature_extractor {

CurvatureCalculatorParams::CurvatureCalculatorParams(float curvature_threshold,
                                                     float search_radius)
    : curvature_threshold_(curvature_threshold), search_radius_(search_radius) {
}

float CurvatureCalculatorParams::CurvatureThreshold() const {
  return curvature_threshold_;
}

float CurvatureCalculatorParams::SearchRadius() const {
  return search_radius_;
};

} // namespace feature_extractor
} // namespace slam
} // namespace lidar
