load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_test(
    name = "test_curvature_calculator",
    srcs = ["test_curvature_calculator.cpp"],
    deps = [
        "//lidar/slam/feature_extractor:lib_curvature_calculator",
        "@googletest//:gtest_main",
    ],
)
