#ifndef LIDAR_SLAM_MATCHER_MATCHER_PARAMS_H_
#define LIDAR_SLAM_MATCHER_MATCHER_PARAMS_H_

namespace lidar {
namespace slam {
namespace matcher {

struct IMatcherParams {
  virtual ~IMatcherParams() = default;
};

} // namespace matcher
} // namespace slam
} // namespace lidar

#endif
