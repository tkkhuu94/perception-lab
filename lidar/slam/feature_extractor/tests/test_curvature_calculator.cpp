#include "gtest/gtest.h"

#include "lidar/slam/feature_extractor/curvature_calculator.h"
#include "lidar/slam/feature_extractor/curvature_calculator_params.h"
#include "pcl/point_types.h"

namespace lidar {
namespace slam {
namespace feature_extractor {

namespace {

class CurvatureCalculatorTest : public ::testing::Test {};

TEST_F(CurvatureCalculatorTest, TestExtractFeatures) {
  pcl::PointCloud<pcl::PointXYZRGB> input_cloud;

  // Create a synthetic point cloud with a distinct edge
  // Plane 1
  for (float x = -1.0f; x <= 0.0f; x += 0.1f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.1f) {
      input_cloud.points.emplace_back(x, y, 0.0f, 255, 0, 0);
    }
  }
  // Plane 2 (creates a 90 degree edge with Plane 1)
  for (float x = 0.05f; x <= 1.0f; x += 0.1f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.1f) {
      input_cloud.points.emplace_back(0, y, x, 0, 0, 255);
    }
  }

  input_cloud.width = input_cloud.points.size();
  input_cloud.height = 1;
  input_cloud.is_dense = true;

  CurvatureCalculatorParams params;
  params.search_radius = 0.2f;
  params.curvature_threshold = 0.04f;

  auto calculator = CurvatureCalculator<pcl::PointXYZRGB>::Create(params);
  ASSERT_EQ(calculator->Type(), "CurvatureCalculator");

  auto feature_cloud = calculator->Extract(input_cloud);
  ASSERT_TRUE(feature_cloud.ok());


  // Points along the edge (around x=0) should have high curvature and be
  // extracted.
  ASSERT_EQ(feature_cloud->size(), 40);
  EXPECT_LT(feature_cloud->size(), input_cloud.size());
}

} // namespace
} // namespace feature_extractor
} // namespace slam
} // namespace lidar
