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
