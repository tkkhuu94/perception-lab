load("@rules_cc//cc:cc_binary.bzl", "cc_binary")

cc_binary(
    name = "build_map",
    srcs = ["build_map.cpp"],
    deps = [
        "//data_loader:lib_point_cloud_loader",
        "//lidar/slam:lib_map",
    ],
)
