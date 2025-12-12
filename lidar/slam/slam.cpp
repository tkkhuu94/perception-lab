#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/initialize.h"
#include "absl/log/log.h"
#include "pcl/io/pcd_io.h"

#include "data_loader/point_cloud_loader.h"
#include "lidar/slam/down_sample/factory.h"
#include "lidar/slam/down_sample/voxel_grid.h"
#include "lidar/slam/down_sample/voxel_grid_params.h"
#include "lidar/slam/feature_extractor/curvature_calculator.h"
#include "lidar/slam/feature_extractor/curvature_calculator_params.h"
#include "lidar/slam/feature_extractor/factory.h"
#include "lidar/slam/map/map.h"
#include "lidar/slam/matcher/factory.h"

using namespace lidar::slam;

absl::StatusOr<std::vector<std::string>>
LoadKITTIBinFiles(const std::string &data_root) {
  if (!std::filesystem::is_directory(data_root)) {
    return absl::NotFoundError(
        absl::StrFormat("Data root directory does not exist: %s", data_root));
  }

  std::vector<std::string> bin_files;

  try {

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
    return absl::InternalError(
        absl::StrFormat("Failed to iterate through .bin files: %s", e.what()));
  }

  std::sort(bin_files.begin(), bin_files.end());

  return bin_files;
}

ABSL_FLAG(std::string, velodyne_data_root, "",
          "Path to the velodyne data containing the .bin files.");
ABSL_FLAG(size_t, max_frames, 20, "Max number of frames to process.");

int main(int argc, char **argv) {
  absl::ParseCommandLine(argc, argv);

  LOG(INFO) << "Max number of frames: " << absl::GetFlag(FLAGS_max_frames);

  if (absl::GetFlag(FLAGS_velodyne_data_root).empty()) {
    LOG(ERROR) << "velodyne_data_root is required.";
    return 1;
  }

  auto down_sampler = down_sample::DownSampleFactory::Create<pcl::PointXYZI>(
      down_sample::DownSampleType::kVoxelGrid,
      down_sample::VoxelGridParams(0.1, 0.1, 0.1));

  auto feature_extractor =
      feature_extractor::FeatureExtractorFactory::Create<pcl::PointXYZI>(
          feature_extractor::FeatureType::kCurvature,
          feature_extractor::CurvatureCalculatorParams(0.08f, 0.2f));

  auto matcher = matcher::MatcherFactory::Create<pcl::PointXYZI>(
      matcher::MatcherType::kIcp, matcher::IcpParams(0.5, 50, 1e-8));

  auto map = map::Map::Create(std::move(down_sampler),
                              std::move(feature_extractor), std::move(matcher));
  if (!map.ok()) {
    LOG(ERROR) << map.status();
    return 1;
  }

  LOG(INFO) << "Lidar Map created successfully";

  auto bin_files = LoadKITTIBinFiles(absl::GetFlag(FLAGS_velodyne_data_root));
  if (!bin_files.ok()) {
    LOG(ERROR) << bin_files.status();
    return 1;
  }

  LOG(INFO) << absl::StrFormat("Parsed %d bin files", bin_files->size());

  size_t frames_processed = 0;
  for (const auto &bin_file : *bin_files) {
    LOG(INFO) << "Processing file: " << bin_file;
    auto point_cloud =
        data_loader::point_cloud_loader::LoadFromFile<pcl::PointXYZI>(bin_file);

    if (!point_cloud.ok()) {
      LOG(ERROR) << point_cloud.status();
      return 1;
    }

    auto current_pose = (*map)->UpdateMap(*point_cloud);
    if (!current_pose.ok()) {
      LOG(ERROR) << current_pose.status();
      return 1;
    }

    LOG(INFO) << "Current pose\n" << current_pose.value();
    frames_processed++;
    if (frames_processed >= absl::GetFlag(FLAGS_max_frames)) {
      break;
    }
  }

  auto point_cloud_map = (*map)->PointCloudMap();
  pcl::io::savePCDFileASCII("/Users/tri/Downloads/my_test_cloud.pcd",
                            *point_cloud_map);

  return 0;
}
