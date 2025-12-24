#include "absl/log/log.h"
#include "absl/strings/str_format.h"

#include "camera_slam/camera/stereo_camera.h"
#include "data_loader/kitti/data_loader.h"

absl::StatusOr<std::string> GetKITTIDataPath(const std::string &sequence_name) {
  const std::string kHomePath = std::getenv("HOME");
  if (kHomePath.empty()) {
    return absl::InternalError("$HOME is empty");
  }

  const std::string kKittiRoot =
      absl::StrFormat("%s/repositories/perception-lab/data/KITTI", kHomePath);
  if (!std::filesystem::exists(kKittiRoot)) {
    return absl::NotFoundError(
        absl::StrFormat("Cannot find KITTI root at %s", kKittiRoot));
  }

  const std::string data_path =
      absl::StrFormat("%s/%s", kKittiRoot, sequence_name);
  if (!std::filesystem::exists(data_path)) {
    return absl::NotFoundError(
        absl::StrFormat("Cannot find KITTI data at %s", data_path));
  }

  return data_path;
}

float CalculateBaseLine(const cv::Mat &left_camera_projection_matrix,
                        const cv::Mat &right_camera_projection_matrix) {
  /**
   The projection matrix takes the form
   P = K * [R | t]

   R is simply the identity since we will be using the rectified images.

   P = [fx 0  cx]   [1 0 0 tx]
       [0  fy cy] . [0 1 0 ty]
       [0  0  1 ]   [0 0 1 tz]

   P[0][3] = (fx * tx) + (0 * ty) + (cx * tz)

   Since we will be using rectified images, we will assume that
   there are only displacement in the X direction between the
   2 cameras. P[0][3] can then be simplified to:

   P[0][3] = (fx * tx) + (0 * 0) + (cx * 0)
           = fx * tx

   => tx = P[0][3] / fx
  */

  const float left_tx = left_camera_projection_matrix.at<double>(0, 3) /
                        left_camera_projection_matrix.at<double>(0, 0);
  const float right_tx = right_camera_projection_matrix.at<double>(0, 3) /
                         right_camera_projection_matrix.at<double>(0, 0);
    LOG(INFO) << "Left tx: " << left_tx;
    LOG(INFO) << "Right tx: " << right_tx;

  return std::fabs(left_tx - right_tx);
}

int main(int argc, char **argv) {

  const std::string kKittiRawSet = "2011_09_26/2011_09_26_drive_0096_sync";

  auto kitti_dataset = GetKITTIDataPath(kKittiRawSet);
  if (!kitti_dataset.ok()) {
    LOG(ERROR) << kitti_dataset.status();
    return 1;
  }

  auto camera_calibration = data_loader::kitti::LoadCameraCalibration(
      absl::StrFormat("%s/%s", kitti_dataset.value(), "calib_cam_to_cam.txt"));

  if (!camera_calibration.ok()) {
    LOG(ERROR) << camera_calibration.status();
    return 1;
  }

  cv::Mat left_camera_extrinsic;
  cv::Mat right_camera_extrinsic;

  cv::hconcat(camera_calibration->at("R_rect_02"),
              camera_calibration->at("T_02"), left_camera_extrinsic);
  cv::hconcat(camera_calibration->at("R_rect_03"),
              camera_calibration->at("T_03"), right_camera_extrinsic);

  auto stereo_camera = camera_slam::camera::StereoCamera::Create(
      camera_calibration->at("K_02"), left_camera_extrinsic,
      camera_calibration->at("P_rect_02"), camera_calibration->at("K_03"),
      cv::Mat::eye(4, 4, CV_32F), camera_calibration->at("P_rect_03"),
      CalculateBaseLine(camera_calibration->at("P_rect_02"),
                        camera_calibration->at("P_rect_03")));
  if (!stereo_camera.ok()) {
    LOG(ERROR) << stereo_camera.status();
    return 1;
  }

  LOG(INFO) << "Stereo Camera initialized!";
  LOG(INFO) << "Left camera projection matrix: \n"
            << (*stereo_camera)->LeftCamera()->ProjectionMatrix();
  LOG(INFO) << "Right camera projection matrix: \n"
            << (*stereo_camera)->RightCamera()->ProjectionMatrix();
  LOG(INFO) << "Stereo Camera Baseline: " << (*stereo_camera)->BaseLine();

  return 0;
}
