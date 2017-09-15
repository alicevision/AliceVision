# Headers
set(multiview_files_headers
  conditioning.hpp
  essential.hpp
  projection.hpp
  rotation_averaging_common.hpp
  rotation_averaging.hpp
  rotation_averaging_l1.hpp
  rotation_averaging_l2.hpp
  solver_affine.hpp
  solver_essential_five_point.hpp
  solver_essential_kernel.hpp
  solver_fundamental_kernel.hpp
  solver_homography_kernel.hpp
  solver_resection_kernel.hpp
  solver_resection_p3p.hpp
  solver_resection_p4pf.hpp
  solver_resection_p5pfr.hpp
  solver_translation_knownRotation_kernel.hpp
  translation_averaging_common.hpp
  translation_averaging_solver.hpp
  triangulation_nview.hpp
  triangulation.hpp
  two_view_kernel.hpp
)

# Sources
set(multiview_files_sources
  conditioning.cpp
  essential.cpp
  projection.cpp
  rotation_averaging_l1.cpp
  rotation_averaging_l2.cpp
  solver_affine.cpp
  solver_essential_five_point.cpp
  solver_essential_kernel.cpp
  solver_fundamental_kernel.cpp
  solver_homography_kernel.cpp
  solver_resection_kernel.cpp
  solver_resection_p3p.cpp
  solver_resection_p4pf.cpp
  solver_resection_p5pfr.cpp
  translation_averaging_solver_l2_chordal.cpp
  translation_averaging_solver_softl1.cpp
  triangulation_nview.cpp
  triangulation.cpp
)

# Test data
set(multiview_files_test_data
  test_data_sets.hpp
  test_data_sets.cpp
)

add_library(aliceVision_multiview
  ${multiview_files_headers}
  ${multiview_files_sources}
)

set_property(TARGET aliceVision_multiview
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_multiview
  DESTINATION lib
  EXPORT aliceVision-targets
)

target_link_libraries(aliceVision_multiview
  ${LEMON_LIBRARY}
  ${LOG_LIB}
)

# Make multiview library integrate numeric library
add_library(aliceVision_multiview_test_data
  ${multiview_files_test_data}
)

set_property(TARGET aliceVision_multiview_test_data
  PROPERTY FOLDER AliceVision/AliceVision
)

target_link_libraries(aliceVision_multiview
  aliceVision_numeric
  ${CERES_LIBRARIES}
)

set_target_properties(aliceVision_multiview
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)


# Triangulation routines
UNIT_TEST(aliceVision triangulation       "aliceVision_multiview;aliceVision_multiview_test_data")
UNIT_TEST(aliceVision triangulation_nview "aliceVision_multiview;aliceVision_multiview_test_data")

# Solvers
UNIT_TEST(aliceVision solver_affine "aliceVision_multiview")
UNIT_TEST(aliceVision solver_fundamental_kernel "aliceVision_multiview")
UNIT_TEST(aliceVision solver_essential_kernel "aliceVision_multiview;aliceVision_multiview_test_data")
UNIT_TEST(aliceVision solver_homography_kernel "aliceVision_multiview;aliceVision_multiview_test_data")
UNIT_TEST(aliceVision solver_essential_five_point "aliceVision_multiview;aliceVision_multiview_test_data")
UNIT_TEST(aliceVision solver_resection_kernel "aliceVision_multiview;aliceVision_multiview_test_data")
UNIT_TEST(aliceVision solver_resection_p4pf "aliceVision_multiview;aliceVision_multiview_test_data")
UNIT_TEST(aliceVision solver_resection_p5pfr "aliceVision_multiview;aliceVision_multiview_test_data")
UNIT_TEST(aliceVision solver_resection_LORansac "aliceVision_multiview;aliceVision_feature;aliceVision_sfm;aliceVision_multiview_test_data;${CERES_LIBRARIES}")
UNIT_TEST(aliceVision solver_translation_knownRotation_kernel "aliceVision_multiview;aliceVision_multiview_test_data")

# Averaging routines
UNIT_TEST(aliceVision rotation_averaging    "aliceVision_multiview;aliceVision_multiview_test_data;${CERES_LIBRARIES}")
UNIT_TEST(aliceVision translation_averaging "aliceVision_multiview;aliceVision_multiview_test_data;${CERES_LIBRARIES}")

