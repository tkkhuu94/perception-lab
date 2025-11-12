#include "absl/log/log.h"
#include "data_loader/point_cloud_loader.hpp"
const std::string file_path =
    "/home/tri/repositories/perception-lab/data/KITTI/"
    "2011_09_26_drive_0001_sync/2011_09_26/"
    "2011_09_26_drive_0001_sync/velodyne_points/data/0000000000.bin";

struct Point3d {
  float x;
  float y;
  float z;
  float reflectance;
};

int main(int argc, char **argv) {
  auto cloud =
      data_loader::point_cloud_loader::LoadFromFile<Point3d>(file_path);
  if (cloud.ok()) {
    for (const auto &point : *cloud) {
      LOG(INFO) << "Point: " << point.x << ", " << point.y << ", " << point.z
                << ", " << point.reflectance;
    }
  }

  return 0;
}
