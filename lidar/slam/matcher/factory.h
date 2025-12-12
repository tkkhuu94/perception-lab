#ifndef LIDAR_SLAM_MATCHER_FACTORY_H_
#define LIDAR_SLAM_MATCHER_FACTORY_H_

#include "lidar/slam/matcher/icp.h"
#include "lidar/slam/matcher/matcher.h"
#include "lidar/slam/matcher/matcher_params.h"

namespace lidar {
namespace slam {
namespace matcher {

enum class MatcherType {
  kUnknown = 0,
  kIcp,
};

class MatcherFactory {
public:
  template <typename PointT>
  static std::unique_ptr<IMatcher<PointT>> Create(const MatcherType &type,
                                                  const IMatcherParams &params);
};

template <typename PointT>
std::unique_ptr<IMatcher<PointT>>
MatcherFactory::Create(const MatcherType &type, const IMatcherParams &params) {
  switch (type) {

  case MatcherType::kIcp:
    return Icp<PointT>::Create(dynamic_cast<const IcpParams &>(params));

  default:
    return nullptr;
  }
}

} // namespace matcher
} // namespace slam
} // namespace lidar

#endif
