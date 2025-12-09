#include "absl/log/log.h"
#include "data_loader/waymo/utility.h"
#include "data_loader/waymo/waymo_open_dataset_loader.h"
#include "pcl/io/pcd_io.h"

const std::string file_path =
    "/Users/tri/repositories/perception-lab/data/waymo_open_dataset/"
    "individual_files_training_segment-10023947602400723454_1120_000_1140_000_"
    "with_camera_labels.tfrecord";

struct Point3d {
  float x;
  float y;
  float z;
  float reflectance;
};

int main(int argc, char **argv) {

  data_loader::waymo::WaymoOpenDatasetLoader waymo_loader;

  auto read_status = waymo_loader.ReadTfRecord(file_path);
  if (!read_status.ok()) {
    LOG(ERROR) << read_status;
    return 1;
  }

  LOG(INFO) << "Loaded TfRecord " << file_path;

  // int frame_count = 0;
  auto frame = waymo_loader.NextFrame();
   frame = waymo_loader.NextFrame();

   frame = waymo_loader.NextFrame();

  // while (next_frame.ok()) {
  //   ++frame_count;
  //   next_frame = waymo_loader.NextFrame();
  // }

  const auto kLaserName = waymo::open_dataset::LaserName::TOP;

  bool got_laser = false;
  bool got_calibration = false;
  waymo::open_dataset::Laser lidar;
  waymo::open_dataset::LaserCalibration calibration;

  for (const auto &laser : frame->lasers()) {
    if (laser.name() == kLaserName) {
      lidar = laser;
      got_laser = true;
      break;
    }
  }

  for (const auto &calib : frame->context().laser_calibrations()) {
    if (calib.name() == kLaserName) {
      calibration = calib;
      got_calibration = true;
      break;
    }
  }

  if (!got_laser) {
    LOG(ERROR) << "Could not find laser with name " << kLaserName;
    return 1;
  }

  if (!got_calibration) {
    LOG(ERROR) << "Could not find calibration with name " << kLaserName;
    return 1;
  }

  LOG(INFO) << "Calibration " << calibration.ShortDebugString() << std::endl;

  auto point_cloud = data_loader::waymo::ToPointCloud(lidar, calibration);
  if (!point_cloud.ok()) {
    LOG(ERROR) << "Failed to convert to point cloud " << point_cloud.status();
    return 1;
  } 

  


  pcl::io::savePCDFileASCII("/Users/tri/Downloads/waymo_data.pcd", *(point_cloud.value()));

  // LOG(INFO) << "Number of frames " << frame_count;

  return 0;
}
