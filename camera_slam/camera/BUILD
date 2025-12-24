load("@rules_cc//cc:cc_library.bzl", "cc_library")

package(default_visibility = ["//camera_slam:__subpackages__"])

cc_library(
    name = "pinhole_camera",
    srcs = ["pinhole_camera.cpp"],
    hdrs = ["pinhole_camera.h"],
    deps = [
        "@abseil-cpp//absl/status:statusor",
        "@opencv",
    ],
)

cc_library(
    name = "stereo_camera",
    srcs = ["stereo_camera.cpp"],
    hdrs = ["stereo_camera.h"],
    deps = [
        ":pinhole_camera",
        "@abseil-cpp//absl/status:statusor",
    ],
)
