#ifndef LIDAR_POINT_CLOUD_REGISTRATION_ICP_H_
#define LIDAR_POINT_CLOUD_REGISTRATION_ICP_H_

#include <limits>
#include <vector>

namespace lidar {
namespace point_cloud_registration {
namespace icp {

struct Point3d {
  float x;
  float y;
  float z;
};

class IterativeClosestPoint {
public:
  using PointCloud3d = std::vector<Point3d>;

  Point3d FindClosestPoint(const Point3d &query,
                           const PointCloud3d &cloud) const;

private:
  static float SquaredDistance(const Point3d &p1, const Point3d &p2);
};

float IterativeClosestPoint::SquaredDistance(const Point3d &p1,
                                             const Point3d &p2) {
  auto dx = p1.x - p2.x;
  auto dy = p1.y - p2.y;
  auto dz = p1.z - p2.z;
  return dx * dx + dy * dy + dz * dz;
}

Point3d
IterativeClosestPoint::FindClosestPoint(const Point3d &query,
                                        const PointCloud3d &cloud) const {
  auto min_distance = std::numeric_limits<float>::max();
  Point3d closest_point = cloud.front();
  for (const auto &point : cloud) {
    auto squared_distance = SquaredDistance(query, point);
    if (squared_distance < min_distance) {
      closest_point = point;
      min_distance = squared_distance;
    }
  }

  return closest_point;
}

} // namespace icp
} // namespace point_cloud_registration
} // namespace lidar

#endif
