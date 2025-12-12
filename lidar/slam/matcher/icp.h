#ifndef LIDAR_SLAM_ICP_H_
#define LIDAR_SLAM_ICP_H_

#include "pcl/registration/icp.h"

#include "lidar/slam/matcher/icp.h"
#include "lidar/slam/matcher/icp_params.h"
#include "lidar/slam/matcher/matcher.h"
#include "lidar/slam/matcher/matcher_params.h"

namespace lidar {
namespace slam {
namespace matcher {

template <typename PointT> class Icp : public IMatcher<PointT> {

public:
  static std::unique_ptr<Icp<PointT>>
  Create(const IcpParams &params);

  std::string Type() const override { return "Icp"; }

  absl::StatusOr<Eigen::Matrix4f> CalculateTransform(
      const std::shared_ptr<pcl::PointCloud<PointT>> &source,
      const std::shared_ptr<pcl::PointCloud<PointT>> &target) override;

private:
  Icp() = default;

  std::unique_ptr<pcl::IterativeClosestPoint<PointT, PointT>> icp_;
};

template <typename PointT>
std::unique_ptr<Icp<PointT>>
Icp<PointT>::Create(const IcpParams &params) {
  std::unique_ptr<Icp<PointT>> ptr(new Icp<PointT>());

  ptr->icp_ = std::make_unique<pcl::IterativeClosestPoint<PointT, PointT>>();
  ptr->icp_->setMaxCorrespondenceDistance(params.MaxCorrespondenceDistance());
  ptr->icp_->setMaximumIterations(params.MaximumIterations());
  ptr->icp_->setTransformationEpsilon(params.TransformationEpsilon());

  return ptr;
}

template <typename PointT>
absl::StatusOr<Eigen::Matrix4f> Icp<PointT>::CalculateTransform(
    const std::shared_ptr<pcl::PointCloud<PointT>> &source,
    const std::shared_ptr<pcl::PointCloud<PointT>> &target) {
  if (source == nullptr) {
    return absl::InvalidArgumentError("Source cloud is null");
  }

  if (target == nullptr) {
    return absl::InvalidArgumentError("Target cloud is null");
  }

  icp_->setInputSource(source);
  icp_->setInputTarget(target);

  pcl::PointCloud<PointT> aligned_cloud;
  icp_->align(aligned_cloud);

  if (!icp_->hasConverged()) {
    return absl::InternalError("ICP failed to converge");
  }

  return icp_->getFinalTransformation();
}

} // namespace matcher
} // namespace slam
} // namespace lidar

#endif // LIDAR_SLAM_ICP_H_
