add_library(openMVG_color_harmonization INTERFACE)

target_link_libraries(openMVG_color_harmonization
  INTERFACE openMVG_image
            ${CLP_LIBRARIES}
            ${COINUTILS_LIBRARY}
            ${OSI_LIBRARY}
            ${LOG_LIB}
)

UNIT_TEST(openMVG global_quantile_gain_offset_alignment "openMVG_color_harmonization")
