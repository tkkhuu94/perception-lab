#ifndef LIDAR_SLAM_NOISE_REMOVER_NOISE_REMOVER_PARAMS_H_
#define LIDAR_SLAM_NOISE_REMOVER_NOISE_REMOVER_PARAMS_H_

namespace lidar {
namespace slam {
namespace noise_remover {

class INoiseRemoverParams {
public:
  virtual ~INoiseRemoverParams() = default;
};

} // namespace noise_remover
} // namespace slam
} // namespace lidar

#endif
