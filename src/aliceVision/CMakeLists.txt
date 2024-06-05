# Common modules

add_subdirectory(image)
add_subdirectory(numeric)
add_subdirectory(system)
add_subdirectory(cmdline)
add_subdirectory(stl)
add_subdirectory(utils)
add_subdirectory(gpu)

# SfM modules

if(ALICEVISION_BUILD_SFM)
  add_subdirectory(camera)
  add_subdirectory(colorHarmonization)
  add_subdirectory(dataio)
  add_subdirectory(feature)
  add_subdirectory(featureEngine)
  add_subdirectory(geometry)
  add_subdirectory(graph)
  add_subdirectory(imageMatching)
  add_subdirectory(linearProgramming)
  add_subdirectory(lensCorrectionProfile)
  add_subdirectory(localization)
  add_subdirectory(matching)
  add_subdirectory(matchingImageCollection)
  add_subdirectory(multiview)
  add_subdirectory(rig)
  add_subdirectory(robustEstimation)
  add_subdirectory(sensorDB)
  add_subdirectory(sfm)
  add_subdirectory(sfmData)
  add_subdirectory(sfmDataIO)
  add_subdirectory(track)
  add_subdirectory(voctree)
  add_subdirectory(calibration)
  if(ALICEVISION_HAVE_OPENCV)
    add_subdirectory(imageMasking)
    add_subdirectory(keyframe)
  endif()
endif()

if (ALICEVISION_BUILD_PANORAMA)
  add_subdirectory(panorama)
endif()

if(ALICEVISION_BUILD_HDR)
  add_subdirectory(hdr)
endif()

# MVS modules

if(ALICEVISION_BUILD_MVS)
  add_subdirectory(mesh)
  add_subdirectory(mvsData)
  add_subdirectory(mvsUtils)
  add_subdirectory(fuseCut)

  if(ALICEVISION_HAVE_CUDA)
    add_subdirectory(depthMap)
  endif()

  if(ALICEVISION_HAVE_ONNX)
    add_subdirectory(segmentation)
  endif()
endif()



if(ALICEVISION_BUILD_SFM AND ALICEVISION_BUILD_MVS)
  add_subdirectory(sfmMvsUtils)
endif()

if (ALICEVISION_BUILD_PHOTOMETRICSTEREO)
    if(ALICEVISION_HAVE_OPENCV)
        add_subdirectory(photometricStereo)
        if(ALICEVISION_HAVE_ONNX)
            add_subdirectory(sphereDetection)
        endif()
        add_subdirectory(lightingEstimation)
    endif()
endif()


if(ALICEVISION_BUILD_SWIG_BINDING)
    set(UseSWIG_TARGET_NAME_PREFERENCE STANDARD)
    set_property(SOURCE aliceVision.i PROPERTY CPLUSPLUS ON)
    set_property(SOURCE aliceVision.i PROPERTY SWIG_MODULE_NAME pyalicevision)

    swig_add_library(pyalicevision
        TYPE MODULE
        LANGUAGE python
        SOURCES aliceVision.i
    )

    set_property(
        TARGET pyalicevision
        PROPERTY SWIG_COMPILE_OPTIONS -doxygen
    )

    target_include_directories(pyalicevision
    PRIVATE
        ../include
        ${ALICEVISION_ROOT}/include
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
    )
    set_property(
        TARGET pyalicevision
        PROPERTY SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON
    )
    set_property(
        TARGET pyalicevision
        PROPERTY COMPILE_OPTIONS -std=c++17
    )

    target_link_libraries(pyalicevision
    PUBLIC
        aliceVision_numeric
    )
    install(
    TARGETS
        pyalicevision
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
    install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/pyalicevision.py
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
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
