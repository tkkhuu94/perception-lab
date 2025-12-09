#ifndef DATA_LOADER_WAYMO_UTILITY_H_
#define DATA_LOADER_WAYMO_UTILITY_H_

#include "absl/status/statusor.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "waymo_open_dataset/dataset.pb.h"

namespace data_loader {
namespace waymo {

absl::StatusOr<std::string> Decompress(const std::string &compressed);

absl::StatusOr<pcl::PointCloud<pcl::PointXYZI>::Ptr>
ToPointCloud(const ::waymo::open_dataset::Laser &laser,
             const ::waymo::open_dataset::LaserCalibration &calibration);

} // namespace waymo_open_dataset
} // namespace data_loader

#endif
