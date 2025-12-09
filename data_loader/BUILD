load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "lib_point_cloud_loader",
    hdrs = ["point_cloud_loader.h"],
    visibility = ["//lidar:__subpackages__"],
    deps = [
        "@abseil-cpp//absl/log",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
    ],
)

cc_binary(
    name = "test_load_point_cloud",
    srcs = ["main.cpp"],
    deps = [
        ":lib_point_cloud_loader",
        "//data_loader/waymo:utility",
        "//data_loader/waymo:waymo_open_dataset_loader",
        "@pcl//:io",
    ],
)
