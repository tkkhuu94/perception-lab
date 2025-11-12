#include <filesystem>

#include "absl/strings/str_format.h"
#include "data_loader/point_cloud_loader.h"
#include "lidar/slam/map.h"

const std::string data_set = "2011_09_26_drive_0009_sync";
const std::string home = std::getenv("HOME");
const std::string data_root =
    absl::StrFormat("%s/repositories/perception-lab/data/KITTI/%s/2011_09_26/"
                    "%s/velodyne_points/data",
                    home, data_set, data_set);

int main(int argc, char **argv) {

  std::vector<std::string> bin_files;
  try {
    // Create an iterator for the given directory
    for (const auto &entry : std::filesystem::directory_iterator(data_root)) {
      if (!entry.is_regular_file()) {
        continue;
      }

      if (entry.path().extension() != ".bin") {
        continue;
      }

      bin_files.emplace_back(entry.path().string());
    }
  } catch (const std::filesystem::filesystem_error &e) {
    LOG(ERROR) << e.what();
  }

  std::sort(bin_files.begin(), bin_files.end());

  lidar::slam::Map map;

  for (const auto &bin_file : bin_files) {

    auto points =
        data_loader::point_cloud_loader::LoadFromFile<pcl::PointXYZI>(bin_file);
    if (!points.ok()) {
      LOG(ERROR) << points.status();
      return 1;
    }

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud->points.resize(points->size());
    for (const auto &p : *points) {
      cloud->emplace_back(p);
    }

    cloud->width = points->size();
    cloud->height = 1;
    cloud->is_dense = true;
    map.UpdateMap(cloud);

    LOG(INFO) << "Processed " << bin_file;
  }

  map.SaveMap();

  return 0;
}