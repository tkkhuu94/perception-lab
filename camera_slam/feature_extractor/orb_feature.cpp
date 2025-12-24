#include "camera_slam/feature_extractor/orb_feature.h"

namespace camera_slam {
namespace feature_extractor {

std::string OrbFeature::Type() const { return "OrbFeature"; }

std::unique_ptr<OrbFeature> OrbFeature::Create(const OrbFeatureParams &params) {
  std::unique_ptr<OrbFeature> orb_feature =
      std::unique_ptr<OrbFeature>(new OrbFeature());

  orb_feature->orb_ = cv::ORB::create(
      params.n_features, params.scale_factor, params.n_levels,
      params.edge_threshold, params.first_level, params.wta_k,
      params.score_type, params.patch_size, params.fast_threshold);

  return orb_feature;
}

} // namespace feature_extractor
} // namespace camera_slam
