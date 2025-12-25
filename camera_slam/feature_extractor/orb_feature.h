#ifndef CAMERA_SLAM_FEATURE_EXTRACTOR_ORB_FEATURE_H_
#define CAMERA_SLAM_FEATURE_EXTRACTOR_ORB_FEATURE_H_

#include <string>

#include "opencv2/features2d.hpp"

#include "camera_slam/feature_extractor/feature_extractor.h"
#include "camera_slam/feature_extractor/orb_feature_params.h"

namespace camera_slam {
namespace feature_extractor {

class OrbFeature : public IFeatureExtractor {
public:
  static std::unique_ptr<OrbFeature> Create(const OrbFeatureParams &params);

  std::string Type() const override;

  absl::StatusOr<Features> Extract(const cv::Mat &image) override;

private:
  cv::Ptr<cv::ORB> orb_;
};

} // namespace feature_extractor
} // namespace camera_slam

#endif
