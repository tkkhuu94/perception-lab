#include "absl/log/log.h"
#include "absl/strings/str_format.h"
#include "data_loader/kitti/data_loader.h"

absl::StatusOr<std::string> GetKITTIDataPath(const std::string &raw_data_set) {
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
      absl::StrFormat("%s/%s", kKittiRoot, raw_data_set);
  if (!std::filesystem::exists(data_path)) {
    return absl::NotFoundError(
        absl::StrFormat("Cannot find KITTI data at %s", data_path));
  }

  return data_path;
}

int main(int argc, char **argv) {

  const std::string kKittiRawSet = "2011_09_26/2011_09_26_drive_0096_sync";

  auto kitti_dataset = GetKITTIDataPath(kKittiRawSet);
  if (!kitti_dataset.ok()) {
    LOG(ERROR) << kitti_dataset.status();
    return 1;
  }

  auto kitti_camera_calibration = data_loader::kitti::LoadCameraCalibration(
      absl::StrFormat("%s/%s", kitti_dataset.value(), "calib_cam_to_cam.txt"));

  if (!kitti_camera_calibration.ok()) {
    LOG(ERROR) << kitti_camera_calibration.status();
    return 1;
  }

  for (auto it = kitti_camera_calibration->begin();
       it != kitti_camera_calibration->end(); ++it) {
    LOG(INFO) << it->first << ": (" << it->second.rows << ", "
              << it->second.cols << ")";
    LOG(INFO) << it->second;
  }

  return 0;
}