load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "lib_down_sample_params",
    hdrs = ["down_sample_params.h"],
)

cc_library(
    name = "lib_down_sample",
    hdrs = ["down_sample.h"],
    deps = [
        ":lib_down_sample_params",
        "@pcl//:common",
    ],
)

cc_library(
    name = "lib_voxel_grid_params",
    hdrs = ["voxel_grid_params.h"],
)

cc_library(
    name = "lib_voxel_grid",
    hdrs = ["voxel_grid.h"],
    visibility = ["//lidar/slam/down_sample/tests:__subpackages__"],
    deps = [
        ":lib_down_sample",
        ":lib_voxel_grid_params",
        "@pcl//:filters",
    ],
)

cc_library(
    name = "lib_factory",
    hdrs = ["factory.h"],
    visibility = ["//lidar/slam/down_sample/tests:__subpackages__"],
    deps = [
        ":lib_down_sample",
        ":lib_down_sample_params",
        ":lib_voxel_grid",
    ],
)
