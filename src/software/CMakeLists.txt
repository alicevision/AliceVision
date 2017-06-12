add_subdirectory(SfM)
add_subdirectory(localization)
add_subdirectory(colorHarmonize)
#add_subdirectory(opencv)

if(OpenMVG_USE_BOOST)
 add_subdirectory(calibration)
endif()

if(OpenMVG_BUILD_OPENGL_EXAMPLES)
  add_subdirectory(SfMViewer)
endif()


