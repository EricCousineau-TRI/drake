# -*- mode: python -*-
# vi: set ft=python :

"""
Makes selected VTK headers and precompiled shared libraries available to be
used as a C++ dependency. On Ubuntu, a VTK archive, built by the project
maintainers from the Dockerfile and shell scripts in this directory, is
downloaded and unpacked. On macOS, VTK must be installed from the
robotlocomotion/director tap (https://git.io/vN6ft) using Homebrew.

Archive naming convention:
    vtk-<version>-embree-<embree version>-ospray-<ospray version>
        -python-<python version>-qt-<qt version>-<platform>-<arch>[-<rebuild>]

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

VTK_MAJOR_MINOR_VERSION = "8.2"

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
            lib_dir = "/usr/local/opt/vtk@{}/lib".format(
                VTK_MAJOR_MINOR_VERSION,
            )
            linkopts = linkopts + [
                "-L{}".format(lib_dir),
                "-l{}-{}".format(name, VTK_MAJOR_MINOR_VERSION),
                "-Wl,-rpath,{}".format(lib_dir),
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
        repository_ctx.symlink(
            "/usr/local/opt/ospray@1.8/include",
            "ospray_include",
        )
        repository_ctx.symlink("/usr/local/opt/vtk@{}/include".format(
            VTK_MAJOR_MINOR_VERSION,
        ), "include")
    elif os_result.is_ubuntu:
        if os_result.ubuntu_release == "18.04":
            archive = "vtk-8.2.0-embree-3.5.2-ospray-1.8.5-python-3.6.9-qt-5.9.5-bionic-x86_64-1.tar.gz"  # noqa
            sha256 = "f32b7b8c0894c31b8665bb28cdf9682683315a13b7779effdfb765a812453a67"  # noqa
        elif os_result.ubuntu_release == "20.04":
            archive = "vtk-8.2.0-embree-3.5.2-ospray-1.8.5-python-3.8.2-qt-5.12.8-focal-x86_64-1.tar.gz"  # noqa
            sha256 = "495a97a208eddb4ebfa4214041400d6d7c842ae9dd3b3157208dd859759654e4"  # noqa
        else:
            fail("Operating system is NOT supported", attr = os_result)

        urls = [
            x.format(archive = archive)
            for x in repository_ctx.attr.mirrors.get("vtk")
        ]
        root_path = repository_ctx.path("")

        repository_ctx.download_and_extract(
            urls,
            output = root_path,
            sha256 = sha256,
            type = "tar.gz",
        )

    else:
        fail("Operating system is NOT supported", attr = os_result)

    file_content = """# -*- python -*-

# DO NOT EDIT: generated by vtk_repository()

licenses([
    "notice",  # Apache-2.0 AND BSD-3-Clause AND MIT
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
            "vtkFloatArray.h",
            "vtkGenericDataArray.h",
            "vtkGenericDataArray.txx",
            "vtkGenericDataArrayLookupHelper.h",
            "vtkIdList.h",
            "vtkIdTypeArray.h",
            "vtkIndent.h",
            "vtkInformation.h",
            "vtkInformationVector.h",
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
            "vtkDataSetAttributesFieldList.h",
            "vtkFieldData.h",
            "vtkImageData.h",
            "vtkPointData.h",
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
            ":vtksys",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonExecutionModel",
        hdrs = [
            "vtkAlgorithm.h",
            "vtkCommonExecutionModelModule.h",
            "vtkDemandDrivenPipeline.h",
            "vtkExecutive.h",
            "vtkImageAlgorithm.h",
            "vtkPolyDataAlgorithm.h",
            "vtkStreamingDemandDrivenPipeline.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonMisc",
            ":vtkCommonSystem",
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
            ":vtksys",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkCommonSystem",
        deps = [
            ":vtkCommonCore",
            ":vtksys",
        ],
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

    # TODO(jamiesnape): Use system library instead.
    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkdoubleconversion",
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkFiltersAMR",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkCommonMath",
            ":vtkCommonSystem",
            ":vtkFiltersCore",
            ":vtkIOXML",
            ":vtkParallelCore",
        ],
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
            ":vtkCommonMath",
            ":vtkCommonMisc",
            ":vtkCommonSystem",
            ":vtkCommonTransforms",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkFiltersGeometry",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkFiltersCore",
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
            ":vtkCommonMath",
            ":vtkCommonMisc",
            ":vtkCommonSystem",
            ":vtkCommonTransforms",
            ":vtkFiltersCore",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkFiltersSources",
        hdrs = [
            # TODO(SeanCurtis-TRI): Remove vtkCubeSource when
            #  attic/.../RgbdRenderer goes away.
            "vtkCubeSource.h",
            "vtkCylinderSource.h",
            "vtkFiltersSourcesModule.h",
            "vtkPlaneSource.h",
            "vtkSphereSource.h",
            "vtkTexturedSphereSource.h",
        ],
        deps = [
            ":vtkCommonComputationalGeometry",
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkCommonTransforms",
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
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkCommonMisc",
            ":vtkdoubleconversion",
            ":vtklzma",
            ":vtksys",
            "@liblz4",
            "@zlib",
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
            ":vtkCommonMisc",
            ":vtkCommonSystem",
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
            ":vtkCommonMath",
            ":vtkCommonTransforms",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkImagingMath",
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
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkCommonMisc",
            ":vtkCommonSystem",
            ":vtkCommonTransforms",
            ":vtkIOCore",
            ":vtkIOImage",
            ":vtkIOLegacy",
            ":vtksys",
            "@zlib",
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
            "vtkTIFFReader.h",
            "vtkTIFFWriter.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkCommonMath",
            ":vtkCommonMisc",
            ":vtkCommonSystem",
            ":vtkCommonTransforms",
            ":vtkDICOMParser",
            ":vtkmetaio",
            "@libjpeg",
            "@libpng",
            "@libtiff",
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
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkCommonMisc",
            ":vtkCommonTransforms",
            ":vtkFiltersCore",
            ":vtkFiltersSources",
            ":vtkIOImage",
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
            ":vtkCommonMisc",
            ":vtkIOCore",
            ":vtksys",
        ],
    )

    # TODO(jamiesnape): Use system library instead.
    file_content += _vtk_cc_library(repository_ctx.os.name, "vtklzma")

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkParallelCore",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonSystem",
            ":vtkIOLegacy",
            ":vtksys",
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
            ":vtkCommonComputationalGeometry",
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkCommonMath",
            ":vtkCommonSystem",
            ":vtkCommonTransforms",
            ":vtkFiltersCore",
            ":vtkFiltersGeneral",
            ":vtkFiltersGeometry",
            ":vtkFiltersSources",
            ":vtksys",
        ],
    )

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
            "vtkStateStorage.h",
            "vtkTextureObject.h",
        ],
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkCommonMath",
            ":vtkCommonSystem",
            ":vtkCommonTransforms",
            ":vtkRenderingCore",
            ":vtksys",
            "@glew",
            "@opengl",
        ],
    )

    if repository_ctx.os.name == "mac os x":
        EMBREE_MAJOR_MINOR_VERSION = "3.5"
        embree_lib_dir = "/usr/local/opt/embree@{}/lib".format(
            EMBREE_MAJOR_MINOR_VERSION,
        )
        file_content += """
cc_library(
    name = "embree",
    linkopts = [
        "-L{0}",
        "-lembree3",
        "-Wl,-rpath,{0}",
    ],
    visibility = ["//visibility:private"],
)
""".format(embree_lib_dir)

        OSPRAY_MAJOR_MINOR_VERSION = "1.8"
        ospray_lib_dir = "/usr/local/opt/ospray@{}/lib".format(
            OSPRAY_MAJOR_MINOR_VERSION,
        )
        file_content += """
cc_library(
    name = "ospray",
    hdrs = [
        "ospray_include/ospray/OSPDataType.h",
        "ospray_include/ospray/OSPTexture.h",
        "ospray_include/ospray/ospray.h",
    ],
    includes = ["ospray_include"],
    linkopts = [
        "-L{0}",
        "-lospray",
        "-Wl,-rpath,{0}",
    ],
    visibility = ["//visibility:public"],
    deps = [":embree"],
)
""".format(ospray_lib_dir)

    else:
        file_content += """
cc_library(
    name = "embree",
    srcs = glob(["lib/libembree3.so*"]),
    visibility = ["//visibility:private"],
)

cc_library(
    name = "ospray",
    srcs = glob([
        "lib/libembree3.so*",
        "lib/libospray_common.so*",
        "lib/libospray_module_ispc.so*",
        "lib/libospray.so*",
    ]),
    hdrs = [
        "include/ospray/OSPDataType.h",
        "include/ospray/OSPTexture.h",
        "include/ospray/ospray.h",
    ],
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [":embree"],
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
            ":ospray",
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkImagingCore",
            ":vtkIOImage",
            ":vtkIOLegacy",
            ":vtkIOXML",
            ":vtkRenderingCore",
            ":vtkRenderingOpenGL2",
            ":vtkRenderingSceneGraph",
            ":vtkRenderingVolume",
            ":vtkRenderingVolumeAMR",
            ":vtksys",
            "@jsoncpp",
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
            ":vtkCommonDataModel",
            ":vtkCommonMath",
            ":vtkRenderingCore",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkRenderingVolume",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonMath",
            ":vtkCommonMisc",
            ":vtkCommonSystem",
            ":vtkCommonTransforms",
            ":vtkImagingCore",
            ":vtkIOXML",
            ":vtkRenderingCore",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkRenderingVolumeAMR",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonExecutionModel",
            ":vtkCommonMath",
            ":vtkCommonSystem",
            ":vtkFiltersAMR",
            ":vtkImagingCore",
            ":vtkRenderingCore",
            ":vtkRenderingVolume",
            ":vtkRenderingVolumeOpenGL2",
        ],
    )

    file_content += _vtk_cc_library(
        repository_ctx.os.name,
        "vtkRenderingVolumeOpenGL2",
        deps = [
            ":vtkCommonCore",
            ":vtkCommonDataModel",
            ":vtkCommonMath",
            ":vtkCommonSystem",
            ":vtkCommonTransforms",
            ":vtkFiltersCore",
            ":vtkFiltersGeneral",
            ":vtkFiltersSources",
            ":vtkImagingCore",
            ":vtkImagingMath",
            ":vtkRenderingCore",
            ":vtkRenderingOpenGL2",
            ":vtkRenderingVolume",
            ":vtksys",
            "@glew",
            "@opengl",
        ],
    )

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
