#include "gtest/gtest.h"

#include "lidar/slam/down_sample/voxel_grid.h"
#include "lidar/slam/down_sample/voxel_grid_params.h"
#include "pcl/point_types.h"

namespace lidar {
namespace slam {
namespace down_sample {

namespace {

class VoxelGridTest : public ::testing::Test {};

TEST_F(VoxelGridTest, TestApplyDownSample) {
  pcl::PointCloud<pcl::PointXYZ> input_cloud;
  input_cloud.points.emplace_back(0.0f, 0.0f, 0.0f);
  input_cloud.points.emplace_back(0.05f, 0.05f, 0.05f);
  input_cloud.points.emplace_back(0.09f, 0.09f, 0.09f);
  input_cloud.width = input_cloud.points.size();
  input_cloud.height = 1;
  input_cloud.is_dense = true;

  VoxelGridParams params;
  params.leaf_size_x = 0.1f;
  params.leaf_size_y = 0.1f;
  params.leaf_size_z = 0.1f;

  auto voxel_grid = VoxelGrid<pcl::PointXYZ>::Create(params);

  auto filtered_cloud = voxel_grid->Apply(input_cloud);

  // Since all points in the input cloud are within the 0.1x0.1x0.1 leaf size,
  // they should be downsampled to a single point.
  EXPECT_EQ(filtered_cloud.size(), 1);
}

} // namespace
} // namespace down_sample
} // namespace slam
} // namespace lidar
