load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "lib_feature_extractor_params",
    hdrs = ["feature_extractor_params.h"],
)

cc_library(
    name = "lib_feature_extractor",
    hdrs = ["feature_extractor.h"],
    deps = [
        "@abseil-cpp//absl/status:statusor",
        "@pcl//:common",
    ],
)

cc_library(
    name = "lib_curvature_calculator_params",
    hdrs = ["curvature_calculator_params.h"],
    deps = [":lib_feature_extractor_params"],
)

cc_library(
    name = "lib_curvature_calculator",
    hdrs = ["curvature_calculator.h"],
    visibility = ["//lidar/slam/feature_extractor/tests:__subpackages__"],
    deps = [
        ":lib_curvature_calculator_params",
        ":lib_feature_extractor",
        "@abseil-cpp//absl/status:statusor",
        "@pcl//:common",
        "@pcl//:features",
    ],
)

cc_library(
    name = "lib_factory",
    hdrs = ["factory.h"],
    visibility = ["//lidar/slam/feature_extractor/tests:__subpackages__"],
    deps = [
        ":lib_curvature_calculator",
        ":lib_feature_extractor",
        ":lib_feature_extractor_params",
    ],
)
