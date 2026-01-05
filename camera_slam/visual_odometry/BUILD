load("@rules_cc//cc:cc_library.bzl", "cc_library")

package(default_visibility = ["//camera_slam:__subpackages__"])

cc_library(
    name = "visual_odometry",
    srcs = ["visual_odometry.cpp"],
    hdrs = ["visual_odometry.h"],
    deps = [
        "//camera_slam/feature_extractor:factory",
        "//camera_slam/camera:stereo_camera",
    ],
)
