#ifndef LIDAR_SLAM_DOWN_SAMPLE_VOXEL_GRID_H_
#define LIDAR_SLAM_DOWN_SAMPLE_VOXEL_GRID_H_

#include "lidar/slam/down_sample/down_sample.h"
#include "lidar/slam/down_sample/down_sample_params.h"
#include "lidar/slam/down_sample/voxel_grid_params.h"
#include "pcl/filters/voxel_grid.h"

namespace lidar {
namespace slam {
namespace down_sample {

template <typename PointT> class VoxelGrid : public IDownSample<PointT> {
public:
  static std::unique_ptr<IDownSample<PointT>>
  Create(const IDownSampleParams &params);

  pcl::PointCloud<PointT>
  Apply(const pcl::PointCloud<PointT> &input_cloud) override;

private:
  VoxelGrid(const VoxelGridParams &params);

  VoxelGridParams params_;
  pcl::VoxelGrid<PointT> voxel_grid_;
};

template <typename PointT>
VoxelGrid<PointT>::VoxelGrid(const VoxelGridParams &params) : params_(params) {}

template <typename PointT>
std::unique_ptr<IDownSample<PointT>>
VoxelGrid<PointT>::Create(const IDownSampleParams &params) {
  const auto &voxel_params = dynamic_cast<const VoxelGridParams &>(params);

  auto ptr =
      std::unique_ptr<VoxelGrid<PointT>>(new VoxelGrid<PointT>(voxel_params));
  ptr->voxel_grid_.setLeafSize(ptr->params_.leaf_size_x,
                               ptr->params_.leaf_size_y,
                               ptr->params_.leaf_size_z);

  return ptr;
}

template <typename PointT>
pcl::PointCloud<PointT>
VoxelGrid<PointT>::Apply(const pcl::PointCloud<PointT> &input_cloud) {
  pcl::PointCloud<PointT> filtered_cloud;

  voxel_grid_.setInputCloud(input_cloud.makeShared());
  voxel_grid_.filter(filtered_cloud);
  return filtered_cloud;
}

} // namespace down_sample
} // namespace slam
} // namespace lidar

#endif
