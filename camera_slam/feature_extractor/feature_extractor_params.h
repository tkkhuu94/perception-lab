#ifndef CAMERA_SLAM_FEATURE_EXTRACTOR_FEATURE_EXTRACTOR_PARAMS_H_
#define CAMERA_SLAM_FEATURE_EXTRACTOR_FEATURE_EXTRACTOR_PARAMS_H_

#include <string>

namespace camera_slam {
namespace feature_extractor {

class IFeatureExtractorParams {
public:
  virtual ~IFeatureExtractorParams() = default;

  virtual std::string Type() const { return "IFeatureExtractorParams"; };
};

} // namespace feature_extractor
} // namespace camera_slam

#endif
