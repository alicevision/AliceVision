# Headers
set(colorHarmonization_files_headers
  CommonDataByPair.hpp
  CommonDataByPair_fullFrame.hpp
  CommonDataByPair_matchedPoints.hpp
  CommonDataByPair_vldSegment.hpp
  GainOffsetConstraintBuilder.hpp
)

# Sources
set(colorHarmonization_files_sources
  GainOffsetConstraintBuilder.cpp
)

alicevision_add_library(aliceVision_colorHarmonization
  SOURCES ${colorHarmonization_files_headers} ${colorHarmonization_files_sources}
  PUBLIC_LINKS
    aliceVision_feature
    aliceVision_image
    aliceVision_linearProgramming
    aliceVision_matching
)

# Unit tests
alicevision_add_test(gainOffsetConstraintBuilder_test.cpp NAME "colorHarmonization_gainOffsetConstraintBuilder" LINKS aliceVision_colorHarmonization)
