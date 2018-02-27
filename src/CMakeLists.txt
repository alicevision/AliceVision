# ==============================================================================
# Dependencies libraries:
# ==============================================================================
add_subdirectory(dependencies)

# ==============================================================================
# aliceVision modules
# ==============================================================================

# software(s) under patent or commercial licence
# Included for research purpose only
if(ALICEVISION_BUILD_SFM)
  add_subdirectory(nonFree)
endif()

# The aliceVision library itself
add_subdirectory(aliceVision)

# aliceVision tutorial examples
if(ALICEVISION_BUILD_EXAMPLES AND ALICEVISION_BUILD_SFM)
  add_subdirectory(samples)
endif()

# Complete software(s) build on aliceVision libraries
add_subdirectory(software)






