#ifndef DATA_LOADER_POINT_CLOUD_LOADER_H_
#define DATA_LOADER_POINT_CLOUD_LOADER_H_

#include <fstream>
#include <string>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "pcl/point_cloud.h"

namespace data_loader {
namespace point_cloud_loader {

template <typename PointT>
absl::StatusOr<std::shared_ptr<pcl::PointCloud<PointT>>>
LoadFromFile(const std::string &file_path) {
  std::ifstream file(file_path, std::ios::binary);

  if (!file.is_open()) {
    LOG(ERROR) << "File not found";
    return absl::NotFoundError(absl::StrCat(file_path, "not found"));
  }

  // --- 1. Get the file size ---
  // Seek to the end of the file
  file.seekg(0, std::ios::end);
  // Get the current position (which is the file size in bytes)
  std::streamsize file_size = file.tellg();
  // Seek back to the beginning
  file.seekg(0, std::ios::beg);

  // --- 2. Calculate number of points ---
  // Each point is 4 floats (x, y, z, intensity)
  const int num_floats_per_point = 4;
  const int bytes_per_float = sizeof(float);
  const int bytes_per_point = num_floats_per_point * bytes_per_float;

  // Total points = (total bytes) / (bytes per point)
  int num_points = file_size / bytes_per_point;

  // --- 3. Resize cloud and read data ---

  // Read all data at once into a buffer
  // This is much faster than reading point by point
  std::vector<float> buffer(num_floats_per_point * num_points);
  file.read(reinterpret_cast<char *>(buffer.data()), file_size);
  file.close();

  // --- 4. Copy data from buffer to PCL cloud ---
  auto cloud = std::make_shared<pcl::PointCloud<PointT>>();
  cloud->points.resize(num_points);

  for (int i = 0; i < num_points; ++i) {
    PointT p;
    p.x = buffer[i * num_floats_per_point + 0];
    p.y = buffer[i * num_floats_per_point + 1];
    p.z = buffer[i * num_floats_per_point + 2];
    p.intensity = buffer[i * num_floats_per_point + 3];
    cloud->points.push_back(p);
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;

  return cloud;
}

} // namespace point_cloud_loader
} // namespace data_loader

#endif
