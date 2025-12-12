#include "absl/log/log.h"

#include "lidar/slam/down_sample/factory.h"
#include "lidar/slam/down_sample/voxel_grid.h"
#include "lidar/slam/down_sample/voxel_grid_params.h"
#include "lidar/slam/feature_extractor/curvature_calculator.h"
#include "lidar/slam/feature_extractor/curvature_calculator_params.h"
#include "lidar/slam/feature_extractor/factory.h"
#include "lidar/slam/map/map.h"
#include "lidar/slam/matcher/factory.h"

using namespace lidar::slam;

int main(int argc, char **argv) {

  auto down_sampler = down_sample::DownSampleFactory::Create<pcl::PointXYZI>(
      down_sample::DownSampleType::kVoxelGrid,
      down_sample::VoxelGridParams(0.2, 0.2, 0.2));

  auto feature_extractor =
      feature_extractor::FeatureExtractorFactory::Create<pcl::PointXYZI>(
          feature_extractor::FeatureType::kCurvature,
          feature_extractor::CurvatureCalculatorParams());

  auto matcher = matcher::MatcherFactory::Create<pcl::PointXYZI>(
      matcher::MatcherType::kIcp, matcher::IcpParams(1.0, 50, 1e-8));

  auto map = map::Map::Create(std::move(down_sampler),
                              std::move(feature_extractor), std::move(matcher));
  if (!map.ok()) {
    LOG(ERROR) << map.status();
    return 1;
  }

  LOG(INFO) << "Lidar Map created successfully";

  std::vector<std::string> bin_files;

  for (const auto &bin_file : bin_files) {
    
  }

  return 0;
}
