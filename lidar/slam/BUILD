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
        "//data_loader:lib_point_cloud_loader",
        "//lidar/slam/down_sample:factory",
        "//lidar/slam/down_sample:voxel_grid",
        "//lidar/slam/down_sample:voxel_grid_params",
        "//lidar/slam/feature_extractor:curvature_calculator",
        "//lidar/slam/feature_extractor:curvature_calculator_params",
        "//lidar/slam/feature_extractor:factory",
        "//lidar/slam/map",
        "//lidar/slam/matcher:factory",
        "@abseil-cpp//absl/log",
        "@bazel_tools//tools/cpp/runfiles",
    ],
)
