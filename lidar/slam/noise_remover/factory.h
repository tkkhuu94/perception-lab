#ifndef LIDAR_SLAM_NOISE_REMOVER_FACTORY_H_
#define LIDAR_SLAM_NOISE_REMOVER_FACTORY_H_

#include "lidar/slam/noise_remover/noise_remover.h"
#include "lidar/slam/noise_remover/statistical_outlier_remover.h"

namespace lidar {
namespace slam {
namespace noise_remover {

enum class Type {
  kUnknown = 0,
  kStatisticalOutlierRemover,
};

class Factory {
public:
  template <typename PointT>
  static std::unique_ptr<INoiseRemover<PointT>>
  Create(const Type &type, const INoiseRemoverParams &params);
};

template <typename PointT>
std::unique_ptr<INoiseRemover<PointT>>
Factory::Create(const Type &type, const INoiseRemoverParams &params) {
  switch (type) {

  case Type::kStatisticalOutlierRemover:
    return StatisticalOutlierRemover<PointT>::Create(
        dynamic_cast<const StatisticalOutlierRemoverParams &>(params));

  default:
    return nullptr;
  }
}

} // namespace noise_remover
} // namespace slam
} // namespace lidar

#endif
