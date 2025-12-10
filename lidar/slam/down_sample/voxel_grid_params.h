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

  VoxelGridParams(float lsize_x, float lsize_y, float lsize_z)
      : leaf_size_x(lsize_x), leaf_size_y(lsize_y), leaf_size_z(lsize_z) {}
};

} // namespace down_sample
} // namespace slam
} // namespace lidar

#endif
