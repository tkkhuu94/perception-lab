load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "down_sample_params",
    hdrs = ["down_sample_params.h"],
)

cc_library(
    name = "down_sample",
    hdrs = ["down_sample.h"],
    visibility = ["//lidar/slam:__subpackages__"],
    deps = [
        ":down_sample_params",
        "@pcl//:common",
    ],
)

cc_library(
    name = "voxel_grid_params",
    hdrs = ["voxel_grid_params.h"],
    visibility = ["//lidar/slam:__subpackages__"],
)

cc_library(
    name = "voxel_grid",
    hdrs = ["voxel_grid.h"],
    visibility = ["//lidar/slam:__subpackages__"],
    deps = [
        ":down_sample",
        ":voxel_grid_params",
        "@pcl//:filters",
    ],
)

cc_library(
    name = "factory",
    hdrs = ["factory.h"],
    visibility = ["//lidar/slam:__subpackages__"],
    deps = [
        ":down_sample",
        ":down_sample_params",
        ":voxel_grid",
    ],
)
