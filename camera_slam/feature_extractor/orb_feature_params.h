#ifndef CAMERA_SLAM_FEATURE_EXTRACTOR_ORB_FEATURE_PARAMS_H_
#define CAMERA_SLAM_FEATURE_EXTRACTOR_ORB_FEATURE_PARAMS_H_

#include "opencv2/features2d.hpp"

#include "camera_slam/feature_extractor/feature_extractor_params.h"

namespace camera_slam {
namespace feature_extractor {

struct OrbFeatureParams : public IFeatureExtractorParams {
  OrbFeatureParams(int n_features_, int n_levels_, int edge_threshold_,
                   int first_level_, int wta_k_, int patch_size_,
                   int fast_threshold_, float scale_factor_,
                   cv::ORB::ScoreType score_type_);

  std::string Type() const override;

  int n_features;
  int n_levels;
  int edge_threshold;
  int first_level;
  int wta_k;
  int patch_size;
  int fast_threshold;

  float scale_factor;

  cv::ORB::ScoreType score_type;
};

} // namespace feature_extractor
} // namespace camera_slam

#endif
