load("@rules_cc//cc:cc_binary.bzl", "cc_binary")

cc_binary(
    name = "camera_slam",
    srcs = ["camera_slam.cpp"],
    deps = [
        "//data_loader/kitti:data_loader",
        "@abseil-cpp//absl/log",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
    ],
)
