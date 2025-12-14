load("@rules_cc//cc:cc_library.bzl", "cc_library")

package(default_visibility = ["//lidar/slam:__subpackages__"])

cc_library(
    name = "noise_remover_params",
    hdrs = ["noise_remover_params.h"],
)

cc_library(
    name = "statistical_outlier_remover_params",
    srcs = ["statistical_outlier_remover_params.cpp"],
    hdrs = ["statistical_outlier_remover_params.h"],
    deps = [":noise_remover_params"],
)

cc_library(
    name = "noise_remover",
    hdrs = ["noise_remover.h"],
    deps = ["@pcl//:common"],
)

cc_library(
    name = "statistical_outlier_remover",
    hdrs = ["statistical_outlier_remover.h"],
    deps = [
        ":noise_remover",
        ":statistical_outlier_remover_params",
        "@pcl//:common",
        "@pcl//:filters",
    ],
)

cc_library(
    name = "factory",
    hdrs = ["factory.h"],
    deps = [
        ":noise_remover",
        ":noise_remover_params",
        ":statistical_outlier_remover",
        ":statistical_outlier_remover_params",
    ],
)
