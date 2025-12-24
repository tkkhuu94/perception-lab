#ifndef CAMERA_SLAM_CAMERA_STEREO_CAMERA_H_
#define CAMERA_SLAM_CAMERA_STEREO_CAMERA_H_

#include "absl/status/statusor.h"
#include "opencv2/core.hpp"

#include "camera_slam/camera/pinhole_camera.h"

namespace camera_slam {
namespace camera {

class StereoCamera {

public:
  static absl::StatusOr<std::unique_ptr<StereoCamera>>
  Create(const cv::Mat &left_camera_intrinsic_matrix,
         const cv::Mat &left_camera_extrinsic_matrix,
         const cv::Mat &left_camera_projection_matrix,
         const cv::Mat &right_camera_intrinsic_matrix,
         const cv::Mat &right_camera_extrinsic_matrix,
         const cv::Mat &right_camera_projection_matrix, float base_line);

  [[nodiscard]] const std::unique_ptr<PinholeCamera> &LeftCamera() const;
  [[nodiscard]] const std::unique_ptr<PinholeCamera> &RightCamera() const;
  float BaseLine() const;

private:
  StereoCamera(float base_line);

  std::unique_ptr<PinholeCamera> left_camera_;
  std::unique_ptr<PinholeCamera> right_camera_;

  float base_line_;
};

} // namespace camera
} // namespace camera_slam

#endif
