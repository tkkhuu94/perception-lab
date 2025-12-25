load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "visual_odometry",
    srcs = ["visual_odometry.cpp"],
    hdrs = ["visual_odometry.h"],
    deps = [
        "//camera_slam/feature_extractor:factory",
        "//camera_slam/camera:stereo_camera",
    ],
)
