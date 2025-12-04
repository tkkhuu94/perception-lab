def _waymo_overlay_impl(ctx):
    # 1. Download the official Waymo Open Dataset source
    ctx.download_and_extract(
        url = "https://github.com/waymo-research/waymo-open-dataset/archive/refs/tags/v1.6.1.tar.gz",
        stripPrefix = "waymo-open-dataset-1.6.1",
    )

    # 2. FIX: Delete existing BUILD files that define subpackages.
    # This prevents Bazel from seeing 'src/waymo_open_dataset' as a separate package,
    # allowing your root overlay to control all source files.
    ctx.delete("src/waymo_open_dataset/BUILD")
    
    # We also delete the protos/BUILD file to prevent similar errors for nested protos
    # if they exist in this version.
    if ctx.path("src/waymo_open_dataset/protos/BUILD").exists:
        ctx.delete("src/waymo_open_dataset/protos/BUILD")

    # 3. Inject a MODULE.bazel file into the downloaded repo.
    ctx.file("MODULE.bazel", """
module(name = "waymo_open_dataset")

bazel_dep(name = "rules_proto", version = "6.0.0")
bazel_dep(name = "rules_cc", version = "0.0.9")
bazel_dep(name = "rules_python", version = "0.31.0")
bazel_dep(name = "protobuf", version = "27.1")
    """)

    # 4. Symlink the BUILD file from your third_party directory
    # Ensure this matches the actual location of your overlay file.
    ctx.symlink(Label("//third_party/waymo_open_dataset:BUILD"), "BUILD.bazel")

waymo_repository = repository_rule(
    implementation = _waymo_overlay_impl,
    local = False,
)
