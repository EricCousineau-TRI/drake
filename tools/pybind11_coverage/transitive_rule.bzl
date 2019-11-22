# -*- python -*-

def add_pybind_coverage_data():
    """Gathers necessary source files so that we can have access to them for
    coverage analysis (Bazel does not like inter-package globs). This should be
    added to each package where coverage is desired."""
    native.filegroup(
        name = "pybind_coverage_data",
        srcs = native.glob(["*.cc"]),
        visibility = ["//bindings/pydrake:__pkg__"],
    )

def _generate_pybind_coverage_impl(ctx):
    source_files = depset(
        transitive = [x.files for x in ctx.attr.pybind_coverage_data],
    )
    (xml_file,) = ctx.attr.xml_docstrings.files.to_list()
    args = ctx.actions.args()
    args.add("--file_coverage=" + ctx.outputs.file_coverage.path)
    args.add("--class_coverage=" + ctx.outputs.class_coverage.path)
    args.add("--xml_docstrings=" + xml_file.path)
    args.add_all(source_files)
    ctx.actions.run(
        outputs = [ctx.outputs.file_coverage, ctx.outputs.class_coverage],
        inputs = depset(transitive = [
            source_files,
            ctx.attr.xml_docstrings.files,
        ]),
        arguments = [args],
        executable = ctx.executable._script,
    )

generate_pybind_coverage = rule(
    implementation = _generate_pybind_coverage_impl,
    attrs = {
        "pybind_coverage_data": attr.label_list(allow_files = True),
        "_script": attr.label(
            default = Label("//tools/pybind11_coverage:get_pybind_coverage"),
            allow_files = True,
            executable = True,
            cfg = "host",
        ),
        "file_coverage": attr.output(mandatory = True),
        "class_coverage": attr.output(mandatory = True),
        "xml_docstrings": attr.label(allow_single_file = True),
    },
)
