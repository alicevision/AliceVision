add_subdirectory(camera)
add_subdirectory(colorHarmonization)
add_subdirectory(dataio)
add_subdirectory(feature)
add_subdirectory(geometry)
add_subdirectory(graph)
add_subdirectory(image)
add_subdirectory(keyframe)
add_subdirectory(linearProgramming)
add_subdirectory(localization)
add_subdirectory(matching)
add_subdirectory(matchingImageCollection)
add_subdirectory(multiview)
add_subdirectory(numeric)
add_subdirectory(rig)
add_subdirectory(robustEstimation)
add_subdirectory(sensorDB)
add_subdirectory(sfm)
add_subdirectory(stl)
add_subdirectory(system)
add_subdirectory(track)
add_subdirectory(voctree)

if(ALICEVISION_HAVE_OPENCV)
  add_subdirectory(calibration)
endif()

# Install rules

install(
  DIRECTORY .
  DESTINATION include/aliceVision
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.tcc"
)

set(global_files_headers
  alicevision_omp.hpp
  prettyprint.hpp
  types.hpp
  version.hpp
)

add_custom_target(aliceVision_global_headers_ide SOURCES ${global_files_headers})
