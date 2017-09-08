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

add_library(aliceVision_geometry INTERFACE)

target_link_libraries(aliceVision_geometry
  INTERFACE aliceVision_numeric
            aliceVision_multiview
            ${CLP_LIBRARIES}
            ${COINUTILS_LIBRARY}
            ${OSI_LIBRARY}
            ${LOG_LIB}
)

UNIT_TEST(aliceVision rigid_transformation3D_srt "aliceVision_geometry")
UNIT_TEST(aliceVision half_space_intersection    "aliceVision_geometry")
UNIT_TEST(aliceVision frustum_intersection       "aliceVision_geometry;aliceVision_multiview_test_data")

add_custom_target(aliceVision_geometry_ide SOURCES ${geometry_files_headers} ${geometry_files_test})
