#include "lidar/slam/matcher/icp_params.h"

namespace lidar {
namespace slam {
namespace matcher {

IcpParams::IcpParams(float max_correspondence_distance, int maximum_iterations,
                     float transformation_epsilon)
    : max_correspondence_distance_(max_correspondence_distance),
      maximum_iterations_(maximum_iterations),
      transformation_epsilon_(transformation_epsilon) {}

float IcpParams::MaxCorrespondenceDistance() const {
  return max_correspondence_distance_;
}

float IcpParams::TransformationEpsilon() const {
  return transformation_epsilon_;
}

int IcpParams::MaximumIterations() const { return maximum_iterations_; }

} // namespace matcher
} // namespace slam
} // namespace lidar
