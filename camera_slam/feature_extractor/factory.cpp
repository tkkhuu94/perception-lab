#include "camera_slam/feature_extractor/factory.h"

namespace camera_slam {
namespace feature_extractor {

std::unique_ptr<IFeatureExtractor>
Factory::Create(ExtractorType extractor_type,
                const IFeatureExtractorParams &params) {
  switch (extractor_type) {
  case ExtractorType::kUnknown:
    return nullptr;

  case ExtractorType::kOrb:
    return OrbFeature::Create(dynamic_cast<const OrbFeatureParams &>(params));

  default:
    return nullptr;
  }
}

} // namespace feature_extractor
} // namespace camera_slam
