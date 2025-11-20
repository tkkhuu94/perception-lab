#include "gtest/gtest.h"

#include "lidar/slam/feature_extractor/curvature_calculator_params.h"
#include "lidar/slam/feature_extractor/factory.h"
#include "pcl/point_types.h"

namespace lidar {
namespace slam {
namespace feature_extractor {
namespace {

class FeatureExtractorFactoryTest : public ::testing::Test {
protected:
  void SetUp() override {
    factory_ = std::make_unique<FeatureExtractorFactory>();
  }

  std::unique_ptr<FeatureExtractorFactory> factory_;
};

TEST_F(FeatureExtractorFactoryTest, CreateUnknownTypeReturnsNull) {
  IFeatureExtractorParams dummy_params;
  auto extractor =
      factory_->Create<pcl::PointXYZ>(FeatureType::kUnknown, dummy_params);
  EXPECT_EQ(extractor, nullptr);
}

TEST_F(FeatureExtractorFactoryTest, CreateCurvatureFeatureExtractorType) {
  CurvatureCalculatorParams params;

  auto extractor =
      factory_->Create<pcl::PointXYZ>(FeatureType::kCurvature, params);
  EXPECT_NE(extractor, nullptr);
  EXPECT_EQ(extractor->Type(), "CurvatureCalculator");
}

TEST_F(FeatureExtractorFactoryTest,
       CreateCurvatureFeatureExtractorWithWrongParamsThrowsException) {
  IFeatureExtractorParams incorrect_params;
  EXPECT_THROW(factory_->Create<pcl::PointXYZ>(FeatureType::kCurvature,
                                               incorrect_params),
               std::bad_cast);
}

} // namespace
} // namespace feature_extractor
} // namespace slam
} // namespace lidar
