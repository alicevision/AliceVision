add_subdirectory(SfM)
add_subdirectory(localization)
add_subdirectory(colorHarmonize)

if(OPENMVG_HAVE_BOOST)
 add_subdirectory(calibration)
endif()
