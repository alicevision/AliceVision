add_subdirectory(SfM)
add_subdirectory(localization)
add_subdirectory(colorHarmonize)
#add_subdirectory(opencv)

if(OPENMVG_HAVE_BOOST)
 add_subdirectory(calibration)
endif()

if(OPENMVG_BUILD_OPENGL_EXAMPLES)
  add_subdirectory(SfMViewer)
endif()


