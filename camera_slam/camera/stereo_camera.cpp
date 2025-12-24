#include "camera_slam/camera/stereo_camera.h"

namespace camera_slam {
namespace camera {

absl::StatusOr<std::unique_ptr<StereoCamera>>
StereoCamera::Create(const cv::Mat &left_camera_intrinsic_matrix,
                     const cv::Mat &left_camera_extrinsic_matrix,
                     const cv::Mat &left_camera_projection_matrix,
                     const cv::Mat &right_camera_intrinsic_matrix,
                     const cv::Mat &right_camera_extrinsic_matrix,
                     const cv::Mat &right_camera_projection_matrix,
                     float base_line) {

  if (base_line < 0) {
    return absl::InvalidArgumentError("Base line must be positive");
  }

  auto left_camera = PinholeCamera::Create(left_camera_intrinsic_matrix,
                                           left_camera_extrinsic_matrix,
                                           left_camera_projection_matrix);
  if (!left_camera.ok()) {
    return left_camera.status();
  }

  auto right_camera = PinholeCamera::Create(right_camera_intrinsic_matrix,
                                            right_camera_extrinsic_matrix,
                                            right_camera_projection_matrix);
  if (!right_camera.ok()) {
    return right_camera.status();
  }

  auto stereo_camera =
      std::unique_ptr<StereoCamera>(new StereoCamera(base_line));
  stereo_camera->left_camera_ = std::move(*left_camera);
  stereo_camera->right_camera_ = std::move(*right_camera);

  return stereo_camera;
}

const std::unique_ptr<PinholeCamera> &StereoCamera::LeftCamera() const {
  return left_camera_;
}

const std::unique_ptr<PinholeCamera> &StereoCamera::RightCamera() const {
  return right_camera_;
}

float StereoCamera::BaseLine() const { return base_line_; }

StereoCamera::StereoCamera(float base_line) : base_line_(base_line) {}

} // namespace camera

} // namespace camera_slam
