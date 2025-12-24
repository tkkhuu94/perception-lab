#ifndef CAMERA_SLAM_FEATURE_EXTRACTOR_FEATURE_EXTRACTOR_H_
#define CAMERA_SLAM_FEATURE_EXTRACTOR_FEATURE_EXTRACTOR_H_

#include <string>

namespace camera_slam {
namespace feature_extractor {

class IFeatureExtractor {
public:
  virtual ~IFeatureExtractor() = default;

  virtual std::string Type() const { return "IFeatureExtractor"; };
};

} // namespace feature_extractor
} // namespace camera_slam

#endif
