#include "camera_slam/visual_odometry/visual_odometry.h"

#include "opencv2/calib3d.hpp"

namespace camera_slam {
namespace visual_odometry {

absl::StatusOr<std::unique_ptr<VisualOdometry>> VisualOdometry::Create(
    const feature_extractor::IFeatureExtractorParams &feature_extractor_params,
    const feature_extractor::ExtractorType &feature_extractor_type) {
  auto vo = std::unique_ptr<VisualOdometry>();

  auto extractor = feature_extractor::Factory::Create(feature_extractor_type,
                                                      feature_extractor_params);
  if (extractor == nullptr) {
    return absl::InternalError("Failed to create feature extractor");
  }
  vo->extractor_ = std::move(extractor);

  return vo;
}

absl::StatusOr<std::vector<cv::Point3f>> VisualOdometry::ComputeStereoDepth(
    const feature_extractor::Features &left_features,
    const feature_extractor::Features &right_features) const {

  if (left_features.descriptors.empty() || right_features.descriptors.empty()) {
    return absl::InternalError("No features detected");
  }

  cv::BFMatcher matcher(cv::NORM_HAMMING, true);
  std::vector<cv::DMatch> matches;
  matcher.match(left_features.descriptors, right_features.descriptors, matches);

  if (matches.empty()) {
    return absl::InternalError("No matches found");
  }

  auto left_fx =
      stereo_camera_->LeftCamera()->IntrinsicMatrix().at<double>(0, 0);
  auto left_fy =
      stereo_camera_->LeftCamera()->IntrinsicMatrix().at<double>(1, 1);
  auto left_cx =
      stereo_camera_->LeftCamera()->IntrinsicMatrix().at<double>(0, 2);
  auto left_cy =
      stereo_camera_->LeftCamera()->IntrinsicMatrix().at<double>(1, 2);

  std::vector<cv::Point3f> points;

  for (const auto &match : matches) {
    auto p_left = left_features.keypoints[match.queryIdx].pt;
    auto p_right = right_features.keypoints[match.trainIdx].pt;

    // Since the images are rectified, we can work under the assumption
    // that a feature point on row n in the left image will also be on
    // row n in the right image (Epipolar line constraint).
    // Thus left_keypoint.y = right_keypoint.y.
    // We will give this constraint a little tolerance
    const float kEpipolarLineConstraintTolerance = 2.0;
    if (std::abs(p_left.y - p_right.y) > kEpipolarLineConstraintTolerance) {
      continue;
    }

    // Disparity check! Due to the geometric shape, naturally
    // disparity should be positive: p_left.x > p_right.y
    // If this is not the case, it is likely a bad match
    if (p_left.x < p_right.y) {
      continue;
    }

    // If disparity is too small, that means the point is too far
    // away to be reliable.
    const float kMinDisparity = 3.0;
    float disparity = p_left.x - p_right.y;
    if (disparity < kMinDisparity) {
      continue;
    }

    float z = (left_fx * stereo_camera_->BaseLine()) / disparity;

    const float kMaxReliableDepth = 50.0;
    if (z > kMaxReliableDepth) {
      continue;
    }

    double x = (p_left.x - left_cx) * z / left_fx;
    double y = (p_left.y - left_cy) * z / left_fy;

    points.emplace_back(cv::Point3f(x, y, z));
  }

  return points;
}

absl::Status VisualOdometry::Update(const cv::Mat &left_image,
                                    const cv::Mat &right_image) {

  auto left_features = extractor_->Extract(left_image);
  auto right_features = extractor_->Extract(right_image);

  if (!left_features.ok()) {
    return left_features.status();
  }

  if (!right_features.ok()) {
    return right_features.status();
  }

  if (left_features->descriptors.empty()) {
    return absl::InternalError("No features found in the left image");
  }

  if (right_features->descriptors.empty()) {
    return absl::InternalError("No features found in the right image");
  }

  auto points = ComputeStereoDepth(*left_features, *right_features);
  if (!points.ok()) {
    return points.status();
  }

  if (previous_features_ == nullptr) {
    previous_features_ = std::unique_ptr<FeaturesWithDepth>(
        new FeaturesWithDepth{*left_features, *points});
    return absl::OkStatus();
  }

  // Motion estimation by matching features from the current frame
  // with features in the previous frame in the left camera
  cv::BFMatcher matcher(cv::NORM_HAMMING, true);
  std::vector<cv::DMatch> matches;
  matcher.match(previous_features_->features.descriptors,
                left_features->descriptors, matches);

  // --- STEP D: PREPARE PnP INPUTS ---
  std::vector<cv::Point3f> object_points; // 3D Points (from Prev Frame)
  std::vector<cv::Point2f> image_points;  // 2D Points (in Curr Frame)

  for (auto &match : matches) {
    cv::Point3f point = previous_features_->points[match.queryIdx];
    // Filter: Only use matches that had a valid Stereo Depth (Z > 0)
    if (point.z < 0) {
      continue;
    }

    object_points.push_back(point);
    image_points.push_back(left_features->keypoints[match.trainIdx].pt);
  }

  if (object_points.size() < 10) {
    return absl::InternalError(
        "Not enough feature points to estimate for motion");
  }
  cv::Mat rvec, tvec, R;
  // Solve PnP: Finds Pose of 'object_points' relative to 'curr_camera'
  // We use RANSAC to reject outliers (bad matches)
  cv::solvePnPRansac(object_points, image_points,
                     stereo_camera_->LeftCamera()->IntrinsicMatrix(), cv::noArray(),
                     rvec, tvec);
  cv::Rodrigues(rvec, R);

  // Invert logic: We want Camera Motion in World Frame
  // T_world_curr = T_world_prev * T_prev_curr
  // The result of PnP is T_curr_prev (Pose of Prev in Curr)
  // So: t_f = t_f + (R_f * (-R.t() * tvec))

  cv::Mat t_local = -R.t() * tvec;
  translation_ = translation_ + rotation_ * t_local;
  rotation_ = rotation_ * R.t();

  return absl::OkStatus();
}

} // namespace visual_odometry
} // namespace camera_slam

int main(int argc, char **argv) {

  auto extractor_params = feature_extractor::OrbFeatureParams(
      500 /* n_features */, 8 /* n_levels */, 31 /* edge_threshold */,
      0 /* first_level */, 2 /* wta_k */, 31 /* patch_size */,
      20 /* fast_threshold */, 1.2f /* scale_factor */,
      cv::ORB::HARRIS_SCORE /* score_type */);

  auto extractor = feature_extractor::Factory::Create(
      feature_extractor::ExtractorType::kOrb, extractor_params);

  return 0;
}
