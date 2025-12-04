load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "waymo_open_dataset_loader",
    srcs = ["waymo_open_dataset_loader.cpp"],
    hdrs = ["waymo_open_dataset_loader.h"],
    deps = [
        "@abseil-cpp//absl/status:statusor",
        "@waymo_open_dataset//:waymo_open_dataset_cc_protos",
    ],
)

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
    deps = [":lib_point_cloud_loader"],
)
