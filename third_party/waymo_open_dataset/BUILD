load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_python//python:proto.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "waymo_open_dataset_protos",
    srcs = glob(["src/waymo_open_dataset/**/*.proto"]),
    strip_import_prefix = "src",
)

cc_proto_library(
    name = "waymo_open_dataset_cc_protos",
    deps = [":waymo_open_dataset_protos"],
)


