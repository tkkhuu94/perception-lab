#ifndef LIDAR_SLAM_FEATURE_EXTRACTOR_CURVATURE_CALCULATOR_H_
#define LIDAR_SLAM_FEATURE_EXTRACTOR_CURVATURE_CALCULATOR_H_

#include <memory>
#include <string>

#include "absl/status/statusor.h"
#include "lidar/slam/feature_extractor/curvature_calculator_params.h"
#include "lidar/slam/feature_extractor/feature_extractor.h"
#include "pcl/features/normal_3d.h"
#include "pcl/point_cloud.h"
#include "pcl/search/kdtree.h"

namespace lidar {
namespace slam {
namespace feature_extractor {

template <typename PointT>
class CurvatureCalculator : public IFeatureExtractor<PointT> {
public:
  static std::unique_ptr<CurvatureCalculator<PointT>>
  Create(const IFeatureExtractorParams &params);

  std::string Type() const override;

  absl::StatusOr<pcl::PointCloud<PointT>>
  Extract(const pcl::PointCloud<PointT> &input_cloud) override;

private:
  using NormalEstimation = pcl::NormalEstimation<PointT, pcl::PointNormal>;
  using KdTree = pcl::search::KdTree<PointT>;

  CurvatureCalculatorParams params_;
  std::shared_ptr<KdTree> kd_tree_;
  std::shared_ptr<NormalEstimation> normal_estimation_;
};

template <typename PointT>
std::unique_ptr<CurvatureCalculator<PointT>>
CurvatureCalculator<PointT>::Create(const IFeatureExtractorParams &params) {

  auto calculator = std::unique_ptr<CurvatureCalculator<PointT>>(
      new CurvatureCalculator<PointT>());
  calculator->params_ = dynamic_cast<const CurvatureCalculatorParams &>(params);
  calculator->kd_tree_ = std::make_unique<KdTree>();
  calculator->normal_estimation_ = std::make_unique<NormalEstimation>();
  return calculator;
}

template <typename PointT>
std::string CurvatureCalculator<PointT>::Type() const {
  return "CurvatureCalculator";
}

template <typename PointT>
absl::StatusOr<pcl::PointCloud<PointT>> CurvatureCalculator<PointT>::Extract(
    const pcl::PointCloud<PointT> &input_cloud) {

  if (kd_tree_ == nullptr) {
    return absl::InternalError("KdTree is not initialized");
  }

  if (normal_estimation_ == nullptr) {
    return absl::InternalError("normal estimation is not initialized");
  }

  pcl::PointCloud<pcl::PointNormal> cloud_with_normals;
  auto input_cloud_ptr = input_cloud.makeShared();
  kd_tree_->setInputCloud(input_cloud_ptr);
  normal_estimation_->setInputCloud(input_cloud_ptr);
  normal_estimation_->setSearchMethod(kd_tree_);
  normal_estimation_->setRadiusSearch(params_.search_radius);
  normal_estimation_->compute(cloud_with_normals);

  if (input_cloud.points.size() != cloud_with_normals.points.size()) {
    return absl::InternalError("Failed to calculate curvature for all points");
  }

  pcl::PointCloud<PointT> feature_cloud;

  for (size_t i = 0; i < cloud_with_normals.points.size(); ++i) {
    const auto &point_with_normal = cloud_with_normals.points[i];

    if (point_with_normal.curvature > params_.curvature_threshold) {
      feature_cloud.points.push_back(input_cloud.points[i]);
    }
  }

  feature_cloud.width = feature_cloud.points.size();
  feature_cloud.height = 1;
  feature_cloud.is_dense = true;

  return feature_cloud;
}

} // namespace feature_extractor
} // namespace slam
} // namespace lidar

#endif
