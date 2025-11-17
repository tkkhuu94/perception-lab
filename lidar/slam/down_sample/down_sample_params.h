#ifndef LIDAR_SLAM_DOWN_SAMPLE_H_
#define LIDAR_SLAM_DOWN_SAMPLE_H_

namespace lidar {
namespace slam {
namespace down_sample {

struct IDownSampleParams {
  virtual ~IDownSampleParams() = default;
};

} // namespace down_sample
} // namespace slam
} // namespace lidar

#endif
