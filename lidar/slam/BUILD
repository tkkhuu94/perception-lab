load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "lib_map",
    srcs = ["map.cpp"],
    hdrs = ["map.h"],
    visibility = ["//lidar:__subpackages__"],
    deps = [
        "@abseil-cpp//absl/log",
        "@pcl//:common",
        "@pcl//:features",
        "@pcl//:filters",
        "@pcl//:io",
        "@pcl//:registration",
    ],
)

cc_binary(
    name = "slam",
    srcs = ["slam.cpp"],
    defines = ["EIGEN_MAX_ALIGN_BYTES=32"],
    deps = [
        "@abseil-cpp//absl/log",
        "//lidar/slam/down_sample:factory",
        "//lidar/slam/down_sample:voxel_grid",
        "//lidar/slam/down_sample:voxel_grid_params",
        "//lidar/slam/map",
    ],
)
