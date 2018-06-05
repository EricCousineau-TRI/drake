# -*- mode: python -*-
# vi: set ft=python :

"""
Makes selected VTK headers and precompiled shared libraries available to be
used as a C++ dependency. On Ubuntu Xenial, a VTK archive is downloaded and
unpacked. On macOS, VTK must be installed from the robotlocomotion/director tap
(https://git.io/vN6ft) using Homebrew.

Archive naming convention:
    vtk-v<version>-qt-<qt version>-xenial-<arch>[-<rebuild>]

Build configuration:
    BUILD_TESTING=OFF
    BUILD_SHARED_LIBS=ON
    CMAKE_BUILD_TYPE=Release
    Module_vtkRenderingOSPRay=ON
    VTK_Group_Qt=ON
    VTK_LEGACY_REMOVE=ON
    VTK_QT_VERSION=5
    VTK_USE_SYSTEM_EXPAT=ON
    VTK_USE_SYSTEM_FREETYPE=ON
    VTK_USE_SYSTEM_HDF5=ON
    VTK_USE_SYSTEM_JPEG=ON
    VTK_USE_SYSTEM_JSONCPP=ON
    VTK_USE_SYSTEM_LIBXML2=ON
    VTK_USE_SYSTEM_LZ4=ON
    VTK_USE_SYSTEM_NETCDF=ON
    VTK_USE_SYSTEM_NETCDFCPP=ON
    VTK_USE_SYSTEM_OGGTHEORA=ON
    VTK_USE_SYSTEM_PNG=ON
    VTK_USE_SYSTEM_TIFF=ON
    VTK_USE_SYSTEM_ZLIB=ON
    VTK_WRAP_PYTHON=ON

Example:
    WORKSPACE:
        load("@drake//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")
        load("@drake//tools/workspace/vtk:repository.bzl", "vtk_repository")
        vtk_repository(name = "foo", mirrors = DEFAULT_MIRRORS)

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo//:vtkCommonCore"],
            srcs = ["bar.cc"],
        )

Argument:
    name: A unique name for this rule.
"""

load("@drake//tools/workspace:os.bzl", "determine_os")

VTK_MAJOR_MINOR_VERSION = "8.1"

def _vtk_cc_library(
        os_name,
        name,
        hdrs = None,
        visibility = None,
        deps = None,
        header_only = False,
        linkopts = []):
    hdr_paths = []

    if hdrs:
        includes = ["include/vtk-{}".format(VTK_MAJOR_MINOR_VERSION)]

        if not visibility:
            visibility = ["//visibility:public"]

        for hdr in hdrs:
            hdr_paths += ["{}/{}".format(includes[0], hdr)]
    else:
        includes = []

        if not visibility:
            visibility = ["//visibility:private"]

    if not deps:
        deps = []

    srcs = []

    if os_name == "mac os x":
        if not header_only:
            linkopts = linkopts + [
                "-L/usr/local/opt/vtk@{}/lib".format(VTK_MAJOR_MINOR_VERSION),
                "-l{}-{}".format(name, VTK_MAJOR_MINOR_VERSION),
            ]
    elif not header_only:
        srcs = ["lib/lib{}-{}.so.1".format(name, VTK_MAJOR_MINOR_VERSION)]

    content = """
cc_library(
    name = "{}",
    srcs = {},
    hdrs = {},
    includes = {},
    linkopts = {},
    visibility = {},
    deps = {},
)
    """.format(name, srcs, hdr_paths, includes, linkopts, visibility, deps)

    return content

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)
    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        repository_ctx.symlink("/usr/local/opt/vtk@{}/include".format(
            VTK_MAJOR_MINOR_VERSION,
        ), "include")
    elif os_result.is_ubuntu:
        if os_result.ubuntu_release == "16.04":
            archive = "vtk-v8.1.1-qt-5.5.1-xenial-x86_64-1.tar.gz"
            sha256 = "77c4bd6eb41fa23c2c7421eeca70e761fc699aaed6cc4804e3d3a73dff16a0fb"  # noqa
        elif os_result.ubuntu_release == "18.04":
            archive = "vtk-v8.1.1-qt-5.9.5-bionic-x86_64-1.tar.gz"
            sha256 = "7c05576eb918fcf85934659d920e4a97bf2c5f034eda296d941495f0008e22b3"  # noqa
        else:
            fail("Operating system is NOT supported", attr = os_result)

        urls = [
            x.format(archive = archive)
            for x in repository_ctx.attr.mirrors.get("vtk")
        ]
        root_path = repository_ctx.path("")

        repository_ctx.download_and_extract(urls, root_path, sha256 = sha256)

    else:
        fail("Operating system is NOT supported", attr = os_result)

    file_content = """# -*- python -*-

# DO NOT EDIT: generated by vtk_repository()

licenses([
    "notice",  # BSD-3-Clause AND MIT
    "reciprocal",  # GL2PS
    "unencumbered",  # Public-Domain
])
"""

    # Note that we only create library targets for enough of VTK to support
    # those used directly or indirectly by Drake.

    # TODO(jamiesnape): Create a script to help generate the targets.

    # To see what the VTK module dependencies are, you can inspect VTK's source
    # tree. For example, for vtkIOXML and vtkIOXMLParser:
    #   VTK/IO/XML/module.cmake
    #   VTK/IO/XMLParser/module.cmake

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonColor",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonComputationalGeometry",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonCore",
        hdrs = [
            "vtkABI.h",
            "vtkAbstractArray.h",
            "vtkAOSDataArrayTemplate.h",
            "vtkAOSDataArrayTemplate.txx",
            "vtkArrayIterator.h",
            "vtkArrayIteratorTemplate.h",
            "vtkArrayIteratorTemplate.txx",
            "vtkAtomic.h",
            "vtkAtomicTypeConcepts.h",
            "vtkAtomicTypes.h",
            "vtkAutoInit.h",
            "vtkBuffer.h",
            "vtkCollection.h",
            "vtkCommand.h",
            "vtkCommonCoreModule.h",
            "vtkConfigure.h",
            "vtkDataArray.h",
            "vtkDebugLeaksManager.h",
            "vtkGenericDataArray.h",
            "vtkGenericDataArray.txx",
            "vtkGenericDataArrayLookupHelper.h",
            "vtkIdList.h",
            "vtkIdTypeArray.h",
            "vtkIndent.h",
            "vtkIntArray.h",
            "vtkIOStream.h",
            "vtkMath.h",
            "vtkMathConfigure.h",
            "vtkNew.h",
            "vtkObject.h",
            "vtkObjectBase.h",
            "vtkObjectFactory.h",
            "vtkOStreamWrapper.h",
            "vtkOStrStreamWrapper.h",
            "vtkPoints.h",
            "vtkSetGet.h",
            "vtkSmartPointer.h",
            "vtkSmartPointerBase.h",
            "vtkStdString.h",
            "vtkSystemIncludes.h",
            "vtkTimeStamp.h",
            "vtkType.h",
            "vtkTypeTraits.h",
            "vtkUnicodeString.h",
            "vtkUnsignedCharArray.h",
            "vtkVariant.h",
            "vtkVariantCast.h",
            "vtkVariantInlineOperators.h",
            "vtkVersion.h",
            "vtkVersionMacros.h",
            "vtkWeakPointer.h",
            "vtkWeakPointerBase.h",
            "vtkWin32Header.h",
            "vtkWindow.h",
            "vtkWrappingHints.h",
        ],
        deps = [
            ":vtkkwiml",
            ":vtksys",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonDataModel",
        hdrs = [
            "vtkAbstractCellLinks.h",
            "vtkCell.h",
            "vtkCellArray.h",
            "vtkCellData.h",
            "vtkCellLinks.h",
            "vtkCellType.h",
            "vtkCellTypes.h",
            "vtkCommonDataModelModule.h",
            "vtkDataObject.h",
            "vtkDataSet.h",
            "vtkDataSetAttributes.h",
            "vtkFieldData.h",
            "vtkImageData.h",
            "vtkPointSet.h",
            "vtkPolyData.h",
            "vtkRect.h",
            "vtkStructuredData.h",
            "vtkVector.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonMath",
            ":vtkCommonMisc",
            ":vtkCommonSystem",
            ":vtkCommonTransforms",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonExecutionModel",
        hdrs = [
            "vtkAlgorithm.h",
            "vtkCommonExecutionModelModule.h",
            "vtkImageAlgorithm.h",
            "vtkPolyDataAlgorithm.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonMisc",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonMath",
        hdrs = [
            "vtkCommonMathModule.h",
            "vtkMatrix4x4.h",
            "vtkTuple.h",
        ],
        deps = [":vtkCommonCore"],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonMisc",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonMath",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonSystem",
        deps = [":vtkCommonCore"],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonTransforms",
        hdrs = [
            "vtkAbstractTransform.h",
            "vtkCommonTransformsModule.h",
            "vtkHomogeneousTransform.h",
            "vtkLinearTransform.h",
            "vtkTransform.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonMath",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkDICOMParser",
        deps = [":vtksys"],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkFiltersCore",
        hdrs = [
            "vtkCleanPolyData.h",
            "vtkFiltersCoreModule.h",
        ],
        visibility = ["//visibility:private"],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkCommonMisc",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkFiltersGeometry",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkFiltersGeneral",
        hdrs = [
            "vtkFiltersGeneralModule.h",
            "vtkTransformPolyDataFilter.h",
        ],
        deps = [
            ":vtkCommonComputationalGeometry",
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkCommonMisc",
            ":vtkFiltersCore",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkFiltersSources",
        hdrs = [
            "vtkCubeSource.h",
            "vtkCylinderSource.h",
            "vtkFiltersSourcesModule.h",
            "vtkPlaneSource.h",
            "vtkSphereSource.h",
        ],
        deps = [
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkFiltersCore",
            ":vtkFiltersGeneral",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkIOCore",
        hdrs = [
            "vtkAbstractPolyDataReader.h",
            "vtkIOCoreModule.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonExecutionModel",
            "@liblz4",
        ],
    )

    # See: VTK/IO/XMLParser/{*.h,module.cmake}
    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkIOXMLParser",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkIOCore",
            ":vtksys",
            "@expat",
        ],
    )

    # See: VTK/IO/XML/{*.h,module.cmake}
    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkIOXML",
        hdrs = [
            "vtkIOXMLModule.h",
            "vtkXMLDataReader.h",
            "vtkXMLPolyDataReader.h",
            "vtkXMLReader.h",
            "vtkXMLUnstructuredDataReader.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkIOCore",
            ":vtkIOXMLParser",
            ":vtksys",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkImagingCore",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkIOGeometry",
        hdrs = [
            "vtkIOGeometryModule.h",
            "vtkOBJReader.h",
        ],
        deps = [
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkIOCore",
            ":vtkIOLegacy",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkIOImage",
        hdrs = [
            "vtkImageExport.h",
            "vtkImageReader2.h",
            "vtkImageWriter.h",
            "vtkIOImageModule.h",
            "vtkJPEGReader.h",
            "vtkPNGReader.h",
            "vtkPNGWriter.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonExecutionModel",
            ":vtkDICOMParser",
            ":vtkmetaio",
            "@libpng",
            "@zlib",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkIOImport",
        hdrs = [
            "vtkImporter.h",
            "vtkIOImportModule.h",
            "vtkOBJImporter.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonExecutionModel",
            ":vtkCommonMisc",
            ":vtkRenderingCore",
            ":vtksys",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkIOLegacy",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkIOCore",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkRenderingCore",
        hdrs = [
            "vtkAbstractMapper.h",
            "vtkAbstractMapper3D.h",
            "vtkActor.h",
            "vtkActorCollection.h",
            "vtkCamera.h",
            "vtkLight.h",
            "vtkMapper.h",
            "vtkPolyDataMapper.h",
            "vtkProp.h",
            "vtkProp3D.h",
            "vtkPropCollection.h",
            "vtkProperty.h",
            "vtkRenderer.h",
            "vtkRendererCollection.h",
            "vtkRenderingCoreModule.h",
            "vtkRenderPass.h",
            "vtkRenderWindow.h",
            "vtkTexture.h",
            "vtkViewport.h",
            "vtkVolume.h",
            "vtkVolumeCollection.h",
            "vtkWindowToImageFilter.h",
        ],
        deps = [
            ":vtkCommonColor",
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkCommonMath",
            ":vtkFiltersCore",
            ":vtkFiltersGeometry",
        ],
    )

    # Segmentation faults with system versions of GLEW on Ubuntu 16.04.
    if repository_ctx.os.name == "linux":
        VTKGLEW = ":vtkglew"
    else:
        VTKGLEW = "@glew"

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkRenderingOpenGL2",
        visibility = ["//visibility:public"],
        hdrs = [
            "vtkOpenGLHelper.h",
            "vtkOpenGLPolyDataMapper.h",
            "vtkOpenGLRenderWindow.h",
            "vtkOpenGLTexture.h",
            "vtkRenderingOpenGL2Module.h",
            "vtkRenderingOpenGLConfigure.h",
            "vtkShader.h",
            "vtkShaderProgram.h",
            "vtkTextureObject.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkRenderingCore",
            VTKGLEW,
        ],
    )

    file_content += """
cc_library(
    name = "ospray",
    srcs =
        glob(["lib/libembree.so*"]) +
        glob(["lib/libospray*.so*"]),
    visibility = ["//visibility:private"],
)
"""

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkRenderingOSPRay",
        visibility = ["//visibility:public"],
        hdrs = [
            "vtkOSPRayLightNode.h",
            "vtkOSPRayMaterialLibrary.h",
            "vtkOSPRayPass.h",
            "vtkOSPRayRendererNode.h",
            "vtkRenderingOSPRayModule.h",
            "vtkRenderingVolumeModule.h",
        ],
        deps = [
            ":vtkCommonDataModel",
            ":vtkImagingCore",
            ":vtkIOXML",
            ":vtkRenderingOpenGL2",
            ":vtkRenderingCore",
            ":vtkRenderingSceneGraph",
            ":vtkRenderingVolume",
            ":ospray",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkRenderingSceneGraph",
        visibility = ["//visibility:public"],
        hdrs = [
            "vtkLightNode.h",
            "vtkRendererNode.h",
            "vtkRenderingSceneGraphModule.h",
            "vtkViewNode.h",
        ],
        deps = [
            ":vtkCommonCore",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkRenderingVolume",
         deps = [
            ":vtkCommonCore",
            ":vtkRenderingCore",
        ],
    )

    if repository_ctx.os.name == "linux":
        file_content += _vtk_cc_library(repository_ctx.os.name, "vtkglew")

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkkwiml",
        hdrs = [
            "vtk_kwiml.h",
            "vtkkwiml/abi.h",
            "vtkkwiml/int.h",
        ],
        visibility = ["//visibility:private"],
        header_only = True,
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkmetaio",
        deps = ["@zlib"],
    )

    file_content += _vtk_cc_library(repository_ctx.os.name, "vtksys")

    # Glob all files for the data dependency of //tools:drake_visualizer.
    file_content += """
filegroup(
    name = "vtk",
    srcs = glob(["**/*"], exclude=["BUILD.bazel", "WORKSPACE"]),
    visibility = ["//visibility:public"],
)
"""

    if repository_ctx.os.name == "mac os x":
        # Use Homebrew VTK.
        files_to_install = []
    else:
        # Install all files.
        files_to_install = [":vtk"]

    file_content += """
load("@drake//tools/install:install.bzl", "install_files")
install_files(
    name = "install",
    dest = ".",
    files = {},
    visibility = ["//visibility:public"],
)
""".format(files_to_install)

    repository_ctx.file(
        "BUILD.bazel",
        content = file_content,
        executable = False,
    )

vtk_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
