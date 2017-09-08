# Headers
set(color_harmonization_files_headers
  global_quantile_gain_offset_alignment.hpp
  selection_fullFrame.hpp
  selection_interface.hpp
  selection_matchedPoints.hpp
  selection_VLDSegment.hpp
)

# Sources
set(color_harmonization_files_test
    global_quantile_gain_offset_alignment_test.cpp
)

add_library(aliceVision_color_harmonization INTERFACE)

target_link_libraries(aliceVision_color_harmonization
  INTERFACE aliceVision_image
            ${CLP_LIBRARIES}
            ${COINUTILS_LIBRARY}
            ${OSI_LIBRARY}
            ${LOG_LIB}
)

UNIT_TEST(aliceVision global_quantile_gain_offset_alignment "aliceVision_color_harmonization")

add_custom_target(aliceVision_color_harmonization_ide SOURCES ${color_harmonization_files_headers} ${color_harmonization_files_test})
