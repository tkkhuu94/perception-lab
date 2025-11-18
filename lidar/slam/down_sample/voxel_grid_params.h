#ifndef LIDAR_SLAM_DOWN_SAMPLE_VOXEL_GRID_PARAMS_H_
#define LIDAR_SLAM_DOWN_SAMPLE_VOXEL_GRID_PARAMS_H_

#include "lidar/slam/down_sample/down_sample_params.h"

namespace lidar {
namespace slam {
namespace down_sample {

struct VoxelGridParams : public IDownSampleParams {
  float leaf_size_x;
  float leaf_size_y;
  float leaf_size_z;
};

} // namespace down_sample
} // namespace slam
} // namespace lidar

#endif
