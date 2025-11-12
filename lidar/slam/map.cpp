#include "lidar/slam/map.h"
#include "absl/log/log.h"
#include "pcl/io/pcd_io.h"

namespace lidar {
namespace slam {

Map::Map() {
  voxel_grid_filter_.setLeafSize(0.2, 0.2, 0.2);

  outlier_filter_.setMeanK(50);
  outlier_filter_.setStddevMulThresh(1.0);

  global_pose_ = Eigen::Matrix4f::Identity();

  map_ = std::make_shared<PointCloudXYZI>();
}

void Map::SaveMap() {
  pcl::io::savePCDFileASCII("/home/tri/Downloads/my_test_cloud.pcd", *map_);
}

void Map::UpdateMap(const PointCloudXYZI::Ptr &raw_cloud) {

  auto processed_cloud = DownSample(raw_cloud);

  processed_cloud = RemoveOutliers(processed_cloud);

  auto feature_cloud = FindFeatureCloud(processed_cloud);

  if (prev_feature_cloud_ != nullptr) {
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputSource(feature_cloud);
    icp.setInputTarget(prev_feature_cloud_);

    icp.setMaxCorrespondenceDistance(1.0);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-8);

    pcl::PointCloud<pcl::PointXYZI> aligned_cloud;
    icp.align(aligned_cloud);

    if (icp.hasConverged()) {
      Eigen::Matrix4f transformation = icp.getFinalTransformation();

      global_pose_ = global_pose_ * transformation;

      PointCloudXYZI::Ptr transformed_cloud(new PointCloudXYZI);
      pcl::transformPointCloud(*raw_cloud, *transformed_cloud, global_pose_);

      *map_ += *transformed_cloud;

      PointCloudXYZI::Ptr downsampled_map(new PointCloudXYZI);
      voxel_grid_filter_.setInputCloud(map_);
      voxel_grid_filter_.filter(*downsampled_map);
      map_ = downsampled_map;
    }
  } else {
    *map_ += *raw_cloud;
  }

  prev_feature_cloud_ = feature_cloud;
}

PointCloudXYZI::Ptr Map::DownSample(const PointCloudXYZI::Ptr &dense_cloud) {
  PointCloudXYZI::Ptr filtered_cloud = std::make_shared<PointCloudXYZI>();
  voxel_grid_filter_.setInputCloud(dense_cloud);
  voxel_grid_filter_.filter(*filtered_cloud);
  return filtered_cloud;
}

PointCloudXYZI::Ptr Map::RemoveOutliers(const PointCloudXYZI::Ptr &in_cloud) {
  PointCloudXYZI::Ptr filtered_cloud = std::make_shared<PointCloudXYZI>();
  outlier_filter_.setInputCloud(in_cloud);
  outlier_filter_.filter(*filtered_cloud);
  return filtered_cloud;
}

PointCloudXYZI::Ptr Map::FindFeatureCloud(const PointCloudXYZI::Ptr &in_cloud) {
  if (kd_tree_ == nullptr) {
    kd_tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr feature_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  kd_tree_->setInputCloud(in_cloud);
  normal_estimation_.setInputCloud(in_cloud);
  normal_estimation_.setSearchMethod(kd_tree_);
  normal_estimation_.setRadiusSearch(0.2);

  normal_estimation_.compute(*cloud_with_normals);

  const float CURVATURE_THRESHOLD = 0.08f;

  if (in_cloud->points.size() != cloud_with_normals->points.size()) {
    std::cerr << "Error: Input and Normal clouds have different sizes!"
              << std::endl;
    return feature_cloud; // Return empty cloud
  }

  for (size_t i = 0; i < cloud_with_normals->points.size(); ++i) {

    //   // Get the point with curvature data
    const auto &point_with_normal = cloud_with_normals->points[i];

    // Check if the point is valid and curvature is high
    if (point_with_normal.curvature > CURVATURE_THRESHOLD) {
      feature_cloud->points.push_back(in_cloud->points[i]);
    }
  }

  feature_cloud->width = feature_cloud->points.size();
  feature_cloud->height = 1;
  feature_cloud->is_dense = true;

  return feature_cloud;
}

} // namespace slam
} // namespace lidar
