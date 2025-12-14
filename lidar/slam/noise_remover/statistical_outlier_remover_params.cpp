#include "lidar/slam/noise_remover/statistical_outlier_remover_params.h"

namespace lidar {
namespace slam {
namespace noise_remover {

StatisticalOutlierRemoverParams::StatisticalOutlierRemoverParams(
    int mean_k, double std_dev_mul_thresh)
    : mean_k_(mean_k), std_dev_mul_thresh_(std_dev_mul_thresh) {}

int StatisticalOutlierRemoverParams::MeanK() const { return mean_k_; }

double StatisticalOutlierRemoverParams::StdDevMulThresh() const {
  return std_dev_mul_thresh_;
}

} // namespace noise_remover
} // namespace slam
} // namespace lidar
