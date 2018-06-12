# Headers
set(geometry_files_headers
    Frustum.hpp
    HalfPlane.hpp
    Pose3.hpp
    rigidTransformation3D.hpp
    Similarity3.hpp
)

# Sources
set(geometry_files_sources
    rigidTransformation3D.cpp
)

# Tests
set(geometry_files_test
    frustumIntersection_test.cpp
    halfSpaceIntersection_test.cpp
    rigidTransformation3D_test.cpp
)

add_library(aliceVision_geometry
    ${geometry_files_headers}
    ${geometry_files_sources}
)

set_property(TARGET aliceVision_geometry
  PROPERTY FOLDER AliceVision
)

target_link_libraries(aliceVision_geometry
  PUBLIC
    aliceVision_numeric
    aliceVision_linearProgramming
  PRIVATE
    # TODO: fix cross-dependency
    # aliceVision_robustEstimation
)

install(TARGETS aliceVision_geometry
  DESTINATION lib/
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision rigidTransformation3D    "aliceVision_geometry")
UNIT_TEST(aliceVision halfSpaceIntersection    "aliceVision_geometry")
UNIT_TEST(aliceVision frustumIntersection      "aliceVision_geometry;aliceVision_multiview;aliceVision_multiview_test_data")

add_custom_target(aliceVision_geometry_ide SOURCES ${geometry_files_headers} ${geometry_files_test})

set_property(TARGET aliceVision_geometry_ide
  PROPERTY FOLDER AliceVision
)
