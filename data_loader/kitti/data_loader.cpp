#include "data_loader/kitti/data_loader.h"

#include <filesystem>
#include <fstream>
#include <sstream>

#include "absl/strings/str_format.h"

namespace data_loader {
namespace kitti {

absl::StatusOr<std::unordered_map<std::string, cv::Mat>>
LoadCameraCalibration(const std::string &file_path) {
  if (!std::filesystem::exists(file_path)) {
    return absl::NotFoundError(
        absl::StrFormat("File does not exist %s", file_path));
  }

  std::ifstream calib_file(file_path);
  if (!calib_file.is_open()) {
    return absl::NotFoundError(
        absl::StrFormat("Failed to open calibration file: %s", file_path));
  }

  std::unordered_map<std::string, cv::Mat> matrices;
  std::string line;

  while (std::getline(calib_file, line)) {
    if (line.empty())
      continue;

    std::stringstream ss(line);
    std::string key;
    ss >> key;

    // Clean up the key (remove trailing colon common in KITTI files, e.g.,
    // "P0:")
    if (!key.empty() && key.back() == ':') {
      key.pop_back();
    }

    // Parse the float values
    std::vector<double> values;
    double val;
    while (ss >> val) {
      values.push_back(val);
    }

    cv::Mat mat;
    if (values.size() == 12) {
      // Standard Projection Matrix (P0, P1, P2, P3) -> 3x4
      // cv::Mat(std::vector) creates a column vector (12x1).
      // reshape(channels, rows) -> reshape(1, 3) makes it 3x4.
      mat = cv::Mat(values, true).reshape(1, 3);
    } else if (values.size() == 9) {
      // Rotation Matrix (R_rect) -> 3x3
      mat = cv::Mat(values, true).reshape(1, 3);
    } else if (values.size() == 3) {
      mat = cv::Mat(values, true).reshape(1, 1).t();
    } else {
      // Skip unrecognized line formats or store as flat vector if preferred
      continue;
    }

    mat.convertTo(mat, CV_64F);
    matrices[key] = mat;
  }

  return matrices;
}

absl::StatusOr<std::vector<std::string>>
LoadImagePaths(const std::string &directory) {
  if (!std::filesystem::is_directory(directory)) {
    return absl::NotFoundError(
        absl::StrFormat("Directory does not exist: %s", directory));
  }

  std::vector<std::string> png_files;

  try {

    for (const auto &entry : std::filesystem::directory_iterator(directory)) {
      if (!entry.is_regular_file()) {
        continue;
      }

      if (entry.path().extension() != ".png") {
        continue;
      }

      png_files.emplace_back(entry.path().string());
    }
  } catch (const std::filesystem::filesystem_error &e) {
    return absl::InternalError(
        absl::StrFormat("Failed to iterate through .png files: %s", e.what()));
  }

  std::sort(png_files.begin(), png_files.end());

  return png_files;
}

} // namespace kitti
} // namespace data_loader
