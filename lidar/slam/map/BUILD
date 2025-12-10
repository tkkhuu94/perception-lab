load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "map",
    srcs = ["map.cpp"],
    hdrs = ["map.h"],
    visibility = ["//lidar/slam:__subpackages__"],
    deps = [
        "//lidar/slam/down_sample",
        "@abseil-cpp//absl/log",
        "@abseil-cpp//absl/status:statusor",
        "@pcl//:common",
    ],
)
