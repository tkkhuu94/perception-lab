#include "camera_slam/feature_extractor/orb_feature_params.h"

namespace camera_slam {
namespace feature_extractor {

OrbFeatureParams::OrbFeatureParams(int n_features_, int n_levels_,
                                   int edge_threshold_, int first_level_,
                                   int wta_k_, int patch_size_,
                                   int fast_threshold_, float scale_factor_,
                                   cv::ORB::ScoreType score_type_)
    : n_features(n_features_), n_levels(n_levels_),
      edge_threshold(edge_threshold_), first_level(first_level_), wta_k(wta_k_),
      patch_size(patch_size_), fast_threshold(fast_threshold_),
      scale_factor(scale_factor_), score_type(score_type_) {}

std::string OrbFeatureParams::Type() const { return "OrbFeatureParams"; }

} // namespace feature_extractor
} // namespace camera_slam
