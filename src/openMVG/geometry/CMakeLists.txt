add_library(openMVG_geometry INTERFACE)

target_link_libraries(openMVG_geometry
  INTERFACE openMVG_numeric
            openMVG_multiview
            ${CLP_LIBRARIES}
            ${COINUTILS_LIBRARY}
            ${OSI_LIBRARY}
            ${LOG_LIB}
)

UNIT_TEST(openMVG rigid_transformation3D_srt "openMVG_geometry")
UNIT_TEST(openMVG half_space_intersection    "openMVG_geometry")
UNIT_TEST(openMVG frustum_intersection       "openMVG_geometry;openMVG_multiview_test_data")
