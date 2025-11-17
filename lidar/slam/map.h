#ifndef LIDAR_SLAM_MAP_H_
#define LIDAR_SLAM_MAP_H_

#include "pcl/features/normal_3d.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "pcl/registration/icp.h"

namespace lidar {
namespace slam {
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
class Map {
public:
  Map();
  void UpdateMap(const PointCloudXYZI::Ptr &raw_cloud);

  void SaveMap();

private:
  PointCloudXYZI::Ptr DownSample(const PointCloudXYZI::Ptr &in_cloud);
  PointCloudXYZI::Ptr RemoveOutliers(const PointCloudXYZI::Ptr &in_cloud);
  PointCloudXYZI::Ptr FindFeatureCloud(const PointCloudXYZI::Ptr &in_cloud);

  PointCloudXYZI::Ptr map_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> outlier_filter_;

  pcl::search::KdTree<pcl::PointXYZI>::Ptr kd_tree_;
  pcl::NormalEstimation<pcl::PointXYZI, pcl::PointNormal> normal_estimation_;

  PointCloudXYZI::Ptr prev_feature_cloud_;

  Eigen::Matrix4f global_pose_;
};
} // namespace slam
} // namespace lidar

#endif
