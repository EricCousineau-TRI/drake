# -*- python -*-

cc_library(
    name = "lib",
    srcs = glob(
        [
            "googlemock/src/*.cc",
            "googletest/src/*.cc",
            "googletest/src/*.h",
        ],
        exclude = [
            "googlemock/src/gmock_main.cc",
            "googletest/src/gtest_main.cc",
            "googlemock/src/gmock-all.cc",
            "googletest/src/gtest-all.cc",
        ],
    ),
    hdrs = glob([
        "googlemock/include/**/*.h",
        "googletest/include/**/*.h",
    ]),
    copts = ["-Wno-unused-const-variable"],
    defines = [
        "GTEST_DONT_DEFINE_FAIL=1",
        "GTEST_DONT_DEFINE_SUCCEED=1",
        "GTEST_DONT_DEFINE_TEST=1",
    ],
    includes = [
        "googlemock",
        "googlemock/include",
        "googletest",
        "googletest/include",
    ],
    linkopts = select({
        "@//tools:linux": ["-pthread"],
        "@//conditions:default": [],
    }),
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "main",
    srcs = ["googlemock/src/gmock_main.cc"],
    deps = [
        ":lib",
        ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)
