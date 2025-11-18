#ifndef LIDAR_SLAM_DOWN_SAMPLE_DOWN_SAMPLE_H_
#define LIDAR_SLAM_DOWN_SAMPLE_DOWN_SAMPLE_H_

#include <memory>

#include "lidar/slam/down_sample/down_sample_params.h"
#include "pcl/point_cloud.h"

namespace lidar {
namespace slam {
namespace down_sample {

template <typename PointT> class IDownSample {
public:
  virtual std::string Type() const { return "IDownSample"; };

  virtual pcl::PointCloud<PointT>
  Apply(const pcl::PointCloud<PointT> &input_cloud) = 0;
};

} // namespace down_sample
} // namespace slam
} // namespace lidar

#endif
