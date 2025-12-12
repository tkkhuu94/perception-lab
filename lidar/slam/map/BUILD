load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "map",
    srcs = ["map.cpp"],
    hdrs = ["map.h"],
    defines = ["EIGEN_MAX_ALIGN_BYTES=32"],
    visibility = ["//lidar/slam:__subpackages__"],
    deps = [
        "//lidar/slam/down_sample",
        "//lidar/slam/feature_extractor",
        "//lidar/slam/matcher",
        "@abseil-cpp//absl/log",
        "@abseil-cpp//absl/status:statusor",
        "@pcl//:common",
        "@pcl//:filters",
    ],
)
