#ifndef CAMERA_VISUAL_ODOMETRY_MAP_MAP_H_
#define CAMERA_VISUAL_ODOMETRY_MAP_MAP_H_

#include "camera/visual_odometry/feature_extractor/feature_extractor.h"

namespace camera {
namespace visual_odometry {
namespace map {

class MapBuilder {
public:
private:
  std::unique_ptr<IFeatureExtractor> extractor_;
};

} // namespace map
} // namespace visual_odometry
} // namespace camera

#endif
