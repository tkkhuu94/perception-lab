#ifndef CAMERA_SLAM_VISUAL_ODOMETRY_VISUAL_ODOMETRY_H_
#define CAMERA_SLAM_VISUAL_ODOMETRY_VISUAL_ODOMETRY_H_

#include <memory>

#include "absl/status/statusor.h"
#include "camera_slam/camera/stereo_camera.h"
#include "camera_slam/feature_extractor/factory.h"
#include "camera_slam/feature_extractor/feature_extractor_params.h"

using namespace camera_slam;

namespace camera_slam {
namespace visual_odometry {
namespace {
struct FeaturesWithDepth {
  feature_extractor::Features features;
  std::vector<cv::Point3f> points;
};
} // namespace

class VisualOdometry {
public:
  static absl::StatusOr<std::unique_ptr<VisualOdometry>>
  Create(const feature_extractor::IFeatureExtractorParams
             &feature_extractor_params,
         const feature_extractor::ExtractorType &feature_extractor_type);

  absl::Status Update(const cv::Mat &left_image, const cv::Mat &right_image);

private:
  absl::StatusOr<std::vector<cv::Point3f>>
  ComputeStereoDepth(const feature_extractor::Features &left_features,
                     const feature_extractor::Features &right_features) const;

  std::unique_ptr<camera::StereoCamera> stereo_camera_;
  std::unique_ptr<feature_extractor::IFeatureExtractor> extractor_;

  std::unique_ptr<FeaturesWithDepth> previous_features_;

  cv::Mat rotation_ = cv::Mat::eye(3, 3, CV_64F);   // Final Rotation (World)
  cv::Mat translation_ = cv::Mat::zeros(3, 1, CV_64F); // Final Position (World)
};

} // namespace visual_odometry
} // namespace camera_slam

#endif
