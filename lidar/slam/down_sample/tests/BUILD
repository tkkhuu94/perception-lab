load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_test(
    name = "test_factory",
    srcs = ["test_factory.cpp"],
    deps = [
        "//lidar/slam/down_sample:lib_factory",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_voxel_grid",
    srcs = ["test_voxel_grid.cpp"],
    deps = [
        "//lidar/slam/down_sample:lib_voxel_grid",
        "@googletest//:gtest_main",
    ],
)
