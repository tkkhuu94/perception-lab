#include "absl/log/log.h"
#include "absl/strings/str_format.h"
#include "opencv2/imgcodecs.hpp"

#include "camera_slam/camera/stereo_camera.h"
#include "camera_slam/feature_extractor/factory.h"
#include "camera_slam/visual_odometry/visual_odometry.h"
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

using camera_slam::visual_odometry::VisualOdometry;

int main(int argc, char **argv) {

  const std::string kKittiRawSet = "2011_09_26/2011_09_26_drive_0096_sync";

  auto kitti_dataset = GetKITTIDataPath(kKittiRawSet);
  if (!kitti_dataset.ok()) {
    LOG(ERROR) << kitti_dataset.status();
    return 1;
  }

  LOG(INFO) << "Dataset: " << *kitti_dataset;

  auto camera_calibration = data_loader::kitti::LoadCameraCalibration(
      absl::StrFormat("%s/%s", kitti_dataset.value(), "calib_cam_to_cam.txt"));

  if (!camera_calibration.ok()) {
    LOG(ERROR) << camera_calibration.status();
    return 1;
  }

  auto left_png_files = data_loader::kitti::LoadImagePaths(
      absl::StrFormat("%s/%s", kitti_dataset.value(), "image_02/data"));
  auto right_png_files = data_loader::kitti::LoadImagePaths(
      absl::StrFormat("%s/%s", kitti_dataset.value(), "image_03/data"));
  if (!left_png_files.ok()) {
    LOG(ERROR) << left_png_files.status();
    return 1;
  }
  if (!right_png_files.ok()) {
    LOG(ERROR) << right_png_files.status();
    return 1;
  }
  if (left_png_files->size() != right_png_files->size()) {
    LOG(ERROR) << "Number of left and right images do not match";
    return 1;
  }

  LOG(INFO) << absl::StrFormat(
      "Loaded %d files for left images and %d files for right images",
      left_png_files->size(), right_png_files->size());

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

  auto extractor_params = feature_extractor::OrbFeatureParams(
      500 /* n_features */, 8 /* n_levels */, 31 /* edge_threshold */,
      0 /* first_level */, 2 /* wta_k */, 31 /* patch_size */,
      20 /* fast_threshold */, 1.2f /* scale_factor */,
      cv::ORB::HARRIS_SCORE /* score_type */);

  auto vo = VisualOdometry::Create(std::move(*stereo_camera), extractor_params,
                                   feature_extractor::ExtractorType::kOrb);
  if (!vo.ok()) {
    LOG(ERROR) << vo.status();
    return 1;
  }

  LOG(INFO) << "Visual Odometry initialized!";

  for (size_t i = 0; i < left_png_files->size(); ++i) {
    cv::Mat left_image = cv::imread(left_png_files->at(i), cv::IMREAD_GRAYSCALE);
    cv::Mat right_image = cv::imread(right_png_files->at(i), cv::IMREAD_GRAYSCALE);

    auto update_status =(*vo)->Update(left_image, right_image);
    if (!update_status.ok()) {
      LOG(ERROR) << update_status;
      return 1;
    }

    LOG(INFO) << "Rotation\n" << (*vo)->Rotation();
    LOG(INFO) << "Translation\n" << (*vo)->Translation();
    LOG(INFO) << "===========";
  }

  return 0;
}
