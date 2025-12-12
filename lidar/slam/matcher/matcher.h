#ifndef LIDAR_SLAM_MATCHER_H_
#define LIDAR_SLAM_MATCHER_H_

#include <string>

#include "Eigen/Dense"
#include "absl/status/statusor.h"
#include "pcl/point_cloud.h"

namespace lidar {
namespace slam {
namespace matcher {

template <typename PointT> class IMatcher {
public:
  virtual ~IMatcher() = default;

  virtual std::string Type() const { return "Matcher"; }

  virtual absl::StatusOr<Eigen::Matrix4f>
  CalculateTransform(const std::shared_ptr<pcl::PointCloud<PointT>> &source,
                     const std::shared_ptr<pcl::PointCloud<PointT>> &target) = 0;
};

} // namespace matcher
} // namespace slam
} // namespace lidar

#endif
