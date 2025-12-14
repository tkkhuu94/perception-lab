#ifndef LIDAR_SLAM_NOISE_REMOVER_OUTLIER_REMOVER_H_
#define LIDAR_SLAM_NOISE_REMOVER_OUTLIER_REMOVER_H_

#include <memory>

#include "lidar/slam/noise_remover/noise_remover.h"
#include "lidar/slam/noise_remover/statistical_outlier_remover_params.h"

#include "pcl/filters/statistical_outlier_removal.h"

namespace lidar {
namespace slam {
namespace noise_remover {

template <typename PointT>
class StatisticalOutlierRemover : public INoiseRemover<PointT> {
public:
  static std::unique_ptr<StatisticalOutlierRemover<PointT>>
  Create(const StatisticalOutlierRemoverParams &params);

  std::string Type() const override;

  std::shared_ptr<pcl::PointCloud<PointT>>
  Apply(const std::shared_ptr<pcl::PointCloud<PointT>> &input_cloud) override;

private:
  StatisticalOutlierRemover(const StatisticalOutlierRemoverParams &params);

  pcl::StatisticalOutlierRemoval<PointT> statistical_outlier_removal_;
};

template <typename PointT>
std::unique_ptr<StatisticalOutlierRemover<PointT>>
StatisticalOutlierRemover<PointT>::Create(
    const StatisticalOutlierRemoverParams &params) {

  auto ptr = std::unique_ptr<StatisticalOutlierRemover<PointT>>(
      new StatisticalOutlierRemover<PointT>(params));
  return ptr;
}

template <typename PointT>
std::string StatisticalOutlierRemover<PointT>::Type() const {
  return "StatisticalOutlierRemover";
}

template <typename PointT>
std::shared_ptr<pcl::PointCloud<PointT>>
StatisticalOutlierRemover<PointT>::Apply(
    const std::shared_ptr<pcl::PointCloud<PointT>> &input_cloud) {

  std::shared_ptr<pcl::PointCloud<PointT>> filtered_cloud(
      new pcl::PointCloud<PointT>());

  statistical_outlier_removal_.setInputCloud(input_cloud);
  statistical_outlier_removal_.filter(*filtered_cloud);
  return filtered_cloud;
}

template <typename PointT>
StatisticalOutlierRemover<PointT>::StatisticalOutlierRemover(
    const StatisticalOutlierRemoverParams &params) {
  statistical_outlier_removal_.setMeanK(params.MeanK());
  statistical_outlier_removal_.setStddevMulThresh(params.StdDevMulThresh());
}

} // namespace noise_remover
} // namespace slam
} // namespace lidar

#endif
