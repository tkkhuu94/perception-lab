#ifndef LIDAR_SLAM_MATCHER_ICP_PARAMS_H_
#define LIDAR_SLAM_MATCHER_ICP_PARAMS_H_

#include "lidar/slam/matcher/matcher_params.h"

namespace lidar {
namespace slam {
namespace matcher {

class IcpParams : public IMatcherParams {
public:
  IcpParams(float max_correspondence_distance, int maximum_iterations,
            float transformation_epsilon);

  float MaxCorrespondenceDistance() const;
  float TransformationEpsilon() const;
  int MaximumIterations() const;

private:
  float max_correspondence_distance_;
  int maximum_iterations_;
  float transformation_epsilon_;
};

} // namespace matcher
} // namespace slam
} // namespace lidar

#endif
