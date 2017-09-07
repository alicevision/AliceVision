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

add_library(openMVG_color_harmonization INTERFACE)

target_link_libraries(openMVG_color_harmonization
  INTERFACE openMVG_image
            ${CLP_LIBRARIES}
            ${COINUTILS_LIBRARY}
            ${OSI_LIBRARY}
            ${LOG_LIB}
)

UNIT_TEST(openMVG global_quantile_gain_offset_alignment "openMVG_color_harmonization")

add_custom_target(openMVG_color_harmonization_ide SOURCES ${color_harmonization_files_headers} ${color_harmonization_files_test})
