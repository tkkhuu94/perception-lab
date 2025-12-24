#ifndef CAMERA_SLAM_CAMERA_PINHOLE_CAMERA_H_
#define CAMERA_SLAM_CAMERA_PINHOLE_CAMERA_H_

#include "absl/status/statusor.h"
#include "opencv2/core.hpp"

namespace camera_slam {
namespace camera {

class PinholeCamera {
public:
  static absl::StatusOr<std::unique_ptr<PinholeCamera>>
  Create(const cv::Mat &intrinsic_matrix, const cv::Mat &extrinsic_matrix,
         const cv::Mat &projection_matrix);

  [[nodiscard]] const cv::Mat &IntrinsicMatrix() const;
  [[nodiscard]] const cv::Mat &ExtrinsicMatrix() const;
  [[nodiscard]] const cv::Mat &ProjectionMatrix() const;

private:
  PinholeCamera(const cv::Mat &intrinsic_matrix,
                const cv::Mat &extrinsic_matrix,
                const cv::Mat &projection_matrix);

  cv::Mat intrinsic_matrix_;
  cv::Mat extrinsic_matrix_; // [R | t]
  cv::Mat projection_matrix_;
};

} // namespace camera
} // namespace camera_slam

#endif
