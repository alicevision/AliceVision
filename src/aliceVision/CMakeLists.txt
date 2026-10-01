# Common modules

add_subdirectory(image)
add_subdirectory(numeric)
add_subdirectory(system)
add_subdirectory(cmdline)
add_subdirectory(stl)
add_subdirectory(utils)
add_subdirectory(gpu)

# SfM modules

if (ALICEVISION_BUILD_SFM)
    add_subdirectory(camera)
    add_subdirectory(colorHarmonization)
    add_subdirectory(dataio)
    add_subdirectory(feature)
    add_subdirectory(featureEngine)
    add_subdirectory(geometry)
    add_subdirectory(graph)
    add_subdirectory(imageMatching)
    add_subdirectory(imageProcessing)
    add_subdirectory(linearProgramming)
    add_subdirectory(lensCorrectionProfile)
    add_subdirectory(matching)
    add_subdirectory(matchingImageCollection)
    add_subdirectory(multiview)
    add_subdirectory(robustEstimation)
    add_subdirectory(sensorDB)
    add_subdirectory(sfm)
    add_subdirectory(sfmData)
    add_subdirectory(sfmDataIO)
    add_subdirectory(track)
    add_subdirectory(voctree)
    add_subdirectory(calibration)
    if (ALICEVISION_HAVE_MAGMA)
        add_subdirectory(uncertainty)
    endif()
    if (ALICEVISION_HAVE_OPENCV)
        add_subdirectory(imageMasking)
        add_subdirectory(keyframe)
    endif()
endif()

if (ALICEVISION_BUILD_PANORAMA)
    add_subdirectory(panorama)
endif()

if (ALICEVISION_BUILD_HDR)
    add_subdirectory(hdr)
endif()

# # MVS modules

if (ALICEVISION_BUILD_MVS)
    add_subdirectory(mesh)
    add_subdirectory(mvsData)
    add_subdirectory(mvsUtils)
    add_subdirectory(fuseCut)

    if (ALICEVISION_DEPTHMAP_BACKEND_CUDA OR ALICEVISION_DEPTHMAP_BACKEND_SYCL)
        add_subdirectory(depthMapCommon)
    endif()
    if (ALICEVISION_DEPTHMAP_BACKEND_CUDA)
        add_subdirectory(depthMap)
    endif()
    if(ALICEVISION_DEPTHMAP_BACKEND_SYCL)
        add_subdirectory(depthMap_sycl)
    endif()
endif()

if (ALICEVISION_HAVE_ONNX)
    add_subdirectory(segmentation)
endif()

if (ALICEVISION_BUILD_SFM AND ALICEVISION_BUILD_MVS)
    add_subdirectory(sfmMvsUtils)
endif()

if (ALICEVISION_BUILD_PHOTOMETRICSTEREO)
    if (ALICEVISION_HAVE_OPENCV)
        add_subdirectory(photometricStereo)
        if (ALICEVISION_HAVE_ONNX)
            add_subdirectory(sphereDetection)
        endif()
        add_subdirectory(lightingEstimation)
    endif()
endif()

# Install rules

install(
    DIRECTORY .
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/aliceVision
    COMPONENT headers
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.tcc"
)

# Add files to an empty target to get them in IDE
set(global_files_headers
    alicevision_omp.hpp
    prettyprint.hpp
    types.hpp
    version.hpp
)

add_custom_target(aliceVision_global_headers_ide SOURCES ${global_files_headers})

set_property(TARGET aliceVision_global_headers_ide
    PROPERTY FOLDER AliceVision
)
