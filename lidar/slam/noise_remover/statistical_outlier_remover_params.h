#ifndef LIDAR_SLAM_NOISE_REMOVER_STATISTICAL_OUTLIER_REMOVER_PARAMS_H_
#define LIDAR_SLAM_NOISE_REMOVER_STATISTICAL_OUTLIER_REMOVER_PARAMS_H_

#include "lidar/slam/noise_remover/noise_remover_params.h"

namespace lidar {
namespace slam {
namespace noise_remover {

class StatisticalOutlierRemoverParams : public INoiseRemoverParams {
public:
  StatisticalOutlierRemoverParams(int mean_k, double std_dev_mul_thresh);

  int MeanK() const;
  double StdDevMulThresh() const;

private:
  int mean_k_;
  double std_dev_mul_thresh_;
};

} // namespace noise_remover
} // namespace slam
} // namespace lidar

#endif
