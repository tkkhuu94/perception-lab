load("@rules_cc//cc:cc_library.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "utility",
    srcs = ["utility.cpp"],
    hdrs = ["utility.h"],
    defines = ["EIGEN_MAX_ALIGN_BYTES=32"],
    deps = [
        "@abseil-cpp//absl/status:statusor",
        "@pcl//:common",
        "@waymo_open_dataset//:waymo_open_dataset_cc_protos",
    ],
)

cc_library(
    name = "waymo_open_dataset_loader",
    srcs = ["waymo_open_dataset_loader.cpp"],
    hdrs = ["waymo_open_dataset_loader.h"],
    defines = ["EIGEN_MAX_ALIGN_BYTES=32"],
    deps = [
        "@abseil-cpp//absl/status:statusor",
        "@waymo_open_dataset//:waymo_open_dataset_cc_protos",
    ],
)
