add_subdirectory(SfM)
add_subdirectory(localization)
add_subdirectory(colorHarmonize)
#add_subdirectory(opencv)

if(OPENMVG_HAVE_BOOST)
 add_subdirectory(calibration)
endif()
