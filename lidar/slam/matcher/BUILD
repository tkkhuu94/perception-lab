load("@rules_cc//cc:cc_library.bzl", "cc_library")

package(default_visibility = ["//lidar/slam:__subpackages__"])

cc_library(
    name = "matcher_params",
    hdrs = ["matcher_params.h"],
)

cc_library(
    name = "icp_params",
    srcs = ["icp_params.cpp"],
    hdrs = ["icp_params.h"],
    deps = [":matcher_params"],
)

cc_library(
    name = "icp",
    hdrs = ["icp.h"],
    deps = [
        ":icp_params",
        ":matcher",
        "@pcl//:registration",
    ],
)

cc_library(
    name = "matcher",
    hdrs = ["matcher.h"],
    deps = [
        "@abseil-cpp//absl/status:statusor",
        "@eigen",
        "@pcl//:common",
    ],
)

cc_library(
    name = "factory",
    hdrs = ["factory.h"],
    deps = [
        ":icp",
        ":matcher",
        ":matcher_params",
    ],
)
