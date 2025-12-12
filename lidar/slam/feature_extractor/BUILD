load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "feature_extractor_params",
    hdrs = ["feature_extractor_params.h"],
)

cc_library(
    name = "feature_extractor",
    hdrs = ["feature_extractor.h"],
    visibility = ["//lidar/slam:__subpackages__"],
    deps = [
        "@abseil-cpp//absl/status:statusor",
        "@pcl//:common",
    ],
)

cc_library(
    name = "curvature_calculator_params",
    srcs = ["curvature_calculator_params.cpp"],
    hdrs = ["curvature_calculator_params.h"],
    visibility = ["//lidar/slam:__subpackages__"],
    deps = [":feature_extractor_params"],
)

cc_library(
    name = "curvature_calculator",
    hdrs = ["curvature_calculator.h"],
    visibility = ["//lidar/slam:__subpackages__"],
    deps = [
        ":curvature_calculator_params",
        ":feature_extractor",
        "@abseil-cpp//absl/status:statusor",
        "@pcl//:common",
        "@pcl//:features",
    ],
)

cc_library(
    name = "factory",
    hdrs = ["factory.h"],
    visibility = ["//lidar/slam:__subpackages__"],
    deps = [
        ":curvature_calculator",
        ":feature_extractor",
        ":feature_extractor_params",
    ],
)
