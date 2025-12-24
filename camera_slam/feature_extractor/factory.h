#ifndef CAMERA_SLAM_FEATURE_EXTRACTOR_FACTORY_H_
#define CAMERA_SLAM_FEATURE_EXTRACTOR_FACTORY_H_

#include "camera_slam/feature_extractor/feature_extractor.h"
#include "camera_slam/feature_extractor/feature_extractor_params.h"
#include "camera_slam/feature_extractor/orb_feature.h"
#include "camera_slam/feature_extractor/orb_feature_params.h"

namespace camera_slam {
namespace feature_extractor {

enum class ExtractorType { kUnknown = 0, kOrb };

class Factory {
public:
  static std::unique_ptr<IFeatureExtractor>
  Create(ExtractorType extractor_type, const IFeatureExtractorParams &params);
};

} // namespace feature_extractor
} // namespace camera_slam

#endif
