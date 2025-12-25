#ifndef CAMERA_SLAM_FEATURE_EXTRACTOR_FEATURE_EXTRACTOR_H_
#define CAMERA_SLAM_FEATURE_EXTRACTOR_FEATURE_EXTRACTOR_H_

#include <string>

#include "absl/status/statusor.h"
#include "opencv2/core.hpp"

namespace camera_slam {
namespace feature_extractor {

struct Features {
  cv::Mat descriptors;
  std::vector<cv::KeyPoint> keypoints;
  std::vector<cv::Point3f> points;
};

class IFeatureExtractor {
public:
  virtual ~IFeatureExtractor() = default;

  virtual std::string Type() const { return "IFeatureExtractor"; };

  virtual absl::StatusOr<Features> Extract(const cv::Mat &image) = 0;
};

} // namespace feature_extractor
} // namespace camera_slam

#endif
