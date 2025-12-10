#include "absl/log/log.h"

#include "lidar/slam/down_sample/factory.h"
#include "lidar/slam/down_sample/voxel_grid.h"
#include "lidar/slam/down_sample/voxel_grid_params.h"
#include "lidar/slam/map/map.h"

using namespace lidar::slam;

int main(int argc, char **argv) {

  auto down_sampler = down_sample::DownSampleFactory::Create<pcl::PointXYZI>(
      down_sample::DownSampleType::kVoxelGrid,
      down_sample::VoxelGridParams(0.2, 0.2, 0.2));

  auto map = map::Map::Create(std::move(down_sampler));
  if (!map.ok()) {
    LOG(ERROR) << map.status();
    return 1;
  }

  return 0;
}
