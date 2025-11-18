#include "gtest/gtest.h"

#include "lidar/slam/down_sample/factory.h"
#include "lidar/slam/down_sample/voxel_grid_params.h"
#include "pcl/point_types.h"

namespace lidar {
namespace slam {
namespace down_sample {
namespace {

class DownSampleFactoryTest : public ::testing::Test {
protected:
  void SetUp() override { factory_ = std::make_unique<DownSampleFactory>(); }

  std::unique_ptr<DownSampleFactory> factory_;
};

TEST_F(DownSampleFactoryTest, CreateUnknownTypeReturnsNull) {
  IDownSampleParams dummy_params;
  auto down_sampler =
      factory_->Create<pcl::PointXYZ>(DownSampleType::kUnknown, dummy_params);
  EXPECT_EQ(down_sampler, nullptr);
}

TEST_F(DownSampleFactoryTest, CreateVoxelGridType) {
  VoxelGridParams params;
  params.leaf_size_x = 0.1f;
  params.leaf_size_y = 0.1f;
  params.leaf_size_z = 0.1f;

  auto down_sample =
      factory_->Create<pcl::PointXYZ>(DownSampleType::kVoxelGrid, params);
  EXPECT_NE(down_sample, nullptr);
  EXPECT_EQ(down_sample->Type(), "VoxelGrid");
}

TEST_F(DownSampleFactoryTest, CreateVoxelGridWithWrongParamsThrowsException) {
  IDownSampleParams incorrect_params;
  EXPECT_THROW(factory_->Create<pcl::PointXYZ>(DownSampleType::kVoxelGrid,
                                               incorrect_params),
               std::bad_cast);
}

} // namespace
} // namespace down_sample
} // namespace slam
} // namespace lidar
