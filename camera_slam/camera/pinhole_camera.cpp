#include "camera_slam/camera/pinhole_camera.h"

namespace camera_slam {
namespace camera {

absl::StatusOr<std::unique_ptr<PinholeCamera>>
PinholeCamera::Create(const cv::Mat &intrinsic_matrix,
                      const cv::Mat &extrinsic_matrix,
                      const cv::Mat &projection_matrix) {

  if (intrinsic_matrix.rows != 3 || intrinsic_matrix.cols != 3) {
    return absl::InvalidArgumentError(
        "Invalid intrinsic matrix size, expecting a 3x3 matrix");
  }

  // if (extrinsic_matrix.rows != 3 || extrinsic_matrix.cols != 4) {
  //   return absl::InvalidArgumentError(
  //       "Invalid extrinsic matrix size, expecting a 3x4 matrix");
  // }

  if (projection_matrix.rows != 3 || projection_matrix.cols != 4) {
    return absl::InvalidArgumentError(
        "Invalid projection matrix size, expecting a 3x4 matrix");
  }

  return std::unique_ptr<PinholeCamera>(
      new PinholeCamera(intrinsic_matrix, extrinsic_matrix, projection_matrix));
}

const cv::Mat &PinholeCamera::IntrinsicMatrix() const {
  return intrinsic_matrix_;
}

const cv::Mat &PinholeCamera::ExtrinsicMatrix() const {
  return extrinsic_matrix_;
}

const cv::Mat &PinholeCamera::ProjectionMatrix() const {
  return projection_matrix_;
}

PinholeCamera::PinholeCamera(const cv::Mat &intrinsic_matrix,
                             const cv::Mat &extrinsic_matrix,
                             const cv::Mat &projection_matrix)
    : intrinsic_matrix_(intrinsic_matrix), extrinsic_matrix_(extrinsic_matrix),
      projection_matrix_(projection_matrix) {}

} // namespace camera
} // namespace camera_slam
