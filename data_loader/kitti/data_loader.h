#ifndef DATA_LOADER_KITTI_DATA_LOADER_H_
#define DATA_LOADER_KITTI_DATA_LOADER_H_

#include <string>
#include <unordered_map>

#include "absl/status/statusor.h"
#include "opencv2/core.hpp"

namespace data_loader {
namespace kitti {

absl::StatusOr<std::unordered_map<std::string, cv::Mat>>
LoadCameraCalibration(const std::string &file_path);

} // namespace kitti
} // namespace data_loader

#endif
