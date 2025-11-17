load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_test(
    name = "test_voxel_grid",
    srcs = ["test_voxel_grid.cpp"],
    deps = [
        "//lidar/slam/down_sample:lib_voxel_grid",
        "@googletest//:gtest_main",
    ],
)
