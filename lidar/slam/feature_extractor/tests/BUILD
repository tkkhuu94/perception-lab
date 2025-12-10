load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_test(
    name = "test_curvature_calculator",
    srcs = ["test_curvature_calculator.cpp"],
    deps = [
        "//lidar/slam/feature_extractor:curvature_calculator",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_factory",
    srcs = ["test_factory.cpp"],
    deps = [
        "//lidar/slam/feature_extractor:factory",
        "@googletest//:gtest_main",
    ],
)
