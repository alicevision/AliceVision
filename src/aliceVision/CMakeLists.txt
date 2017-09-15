add_subdirectory(camera)
add_subdirectory(exif)
add_subdirectory(feature)
add_subdirectory(graph)
add_subdirectory(image)
add_subdirectory(linearProgramming)
add_subdirectory(geometry)
add_subdirectory(matching)
add_subdirectory(matching_image_collection)
add_subdirectory(multiview)
add_subdirectory(numeric)
add_subdirectory(robust_estimation)
add_subdirectory(tracks)
add_subdirectory(colorHarmonization)
add_subdirectory(system)
add_subdirectory(sfm)
add_subdirectory(stl)

if(ALICEVISION_HAVE_BOOST)

  add_subdirectory(dataio)
  add_subdirectory(voctree)
  add_subdirectory(localization)
  add_subdirectory(rig)

  if(ALICEVISION_HAVE_OPENCV)
    add_subdirectory(calibration)
  endif()

  if(ALICEVISION_HAVE_OIIO)
    add_subdirectory(keyframe)
  endif()

endif()

#Install rules
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
