#ifndef LIDAR_SLAM_DOWN_SAMPLE_FACTORY_H_
#define LIDAR_SLAM_DOWN_SAMPLE_FACTORY_H_

#include <memory>

#include "lidar/slam/down_sample/down_sample.h"
#include "lidar/slam/down_sample/down_sample_params.h"
#include "lidar/slam/down_sample/voxel_grid.h"

namespace lidar {
namespace slam {
namespace down_sample {

enum class DownSampleType {
  kUnknown = 0,
  kVoxelGrid,
};

class DownSampleFactory {
public:
  template <typename PointT>
  std::unique_ptr<IDownSample<PointT>> Create(const DownSampleType &type,
                                              const IDownSampleParams &params);
};

template <typename PointT>
std::unique_ptr<IDownSample<PointT>>
DownSampleFactory::Create(const DownSampleType &type,
                          const IDownSampleParams &params) {
  switch (type) {

  case DownSampleType::kVoxelGrid:
    return VoxelGrid<PointT>::Create(params);

  default:
    return nullptr;
  }
}

} // namespace down_sample
} // namespace slam
} // namespace lidar

#endif
