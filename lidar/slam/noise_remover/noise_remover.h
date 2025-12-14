#ifndef LIDAR_SLAM_NOISE_REMOVER_NOISE_REMOVER_H_
#define LIDAR_SLAM_NOISE_REMOVER_NOISE_REMOVER_H_

#include <string>

#include "pcl/point_cloud.h"

namespace lidar {
namespace slam {
namespace noise_remover {

template <typename PointT> class INoiseRemover {

public:
  virtual ~INoiseRemover() = default;

  virtual std::string Type() const { return "INoiseRemover"; };

  virtual std::shared_ptr<pcl::PointCloud<PointT>>
  Apply(const std::shared_ptr<pcl::PointCloud<PointT>> &input_cloud) = 0;
};

} // namespace noise_remover
} // namespace slam
} // namespace lidar

#endif
