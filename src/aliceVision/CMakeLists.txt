# Common modules

add_subdirectory(image)
add_subdirectory(numeric)
add_subdirectory(system)


# SfM modules

if(ALICEVISION_BUILD_SFM)
  add_subdirectory(camera)
  add_subdirectory(colorHarmonization)
  add_subdirectory(dataio)
  add_subdirectory(feature)
  add_subdirectory(geometry)
  add_subdirectory(graph)
  add_subdirectory(gpu)
  add_subdirectory(keyframe)
  add_subdirectory(linearProgramming)
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
  add_subdirectory(stl)
  add_subdirectory(track)
  add_subdirectory(voctree)

  if(ALICEVISION_HAVE_OPENCV)
    add_subdirectory(calibration)
    add_subdirectory(imageMasking)
  endif()
endif()

if(ALICEVISION_BUILD_HDR)
add_subdirectory(hdr)
endif()

# MVS modules

if(ALICEVISION_BUILD_MVS)
  add_subdirectory(lightingEstimation)
  add_subdirectory(mesh)
  add_subdirectory(mvsData)
  add_subdirectory(mvsUtils)
  add_subdirectory(fuseCut)

  if(ALICEVISION_HAVE_CUDA)
    add_subdirectory(depthMap)
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
