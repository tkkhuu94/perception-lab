load("@rules_cc//cc:cc_library.bzl", "cc_library")

package(default_visibility = ["//camera_slam:__subpackages__"])

cc_library(
    name = "feature_extractor_params",
    hdrs = ["feature_extractor_params.h"],
)

cc_library(
    name = "feature_extractor",
    hdrs = ["feature_extractor.h"],
    deps = [
        "@abseil-cpp//absl/status:statusor",
        "@opencv",
    ],
)

cc_library(
    name = "orb_feature_params",
    srcs = ["orb_feature_params.cpp"],
    hdrs = ["orb_feature_params.h"],
    deps = [
        ":feature_extractor_params",
        "@opencv",
    ],
)

cc_library(
    name = "orb_feature",
    srcs = ["orb_feature.cpp"],
    hdrs = ["orb_feature.h"],
    deps = [
        ":feature_extractor",
        ":orb_feature_params",
        "@opencv",
    ],
)

cc_library(
    name = "factory",
    srcs = ["factory.cpp"],
    hdrs = ["factory.h"],
    deps = [
        ":feature_extractor",
        ":feature_extractor_params",
        ":orb_feature",
        ":orb_feature_params",
    ],
)
