# Headers
set(geometry_files_headers
  frustum.hpp
  half_space_intersection.hpp
  pose3.hpp
  rigid_transformation3D_srt.hpp
  Similarity3.hpp
)

# Sources
set(geometry_files_test
    frustum_intersection_test.cpp
    half_space_intersection_test.cpp
    rigid_transformation3D_srt_test.cpp
)

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

add_custom_target(openMVG_geometry_ide SOURCES ${geometry_files_headers} ${geometry_files_test})
