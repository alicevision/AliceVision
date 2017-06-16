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

add_library(openMVG_multiview
  ${multiview_files_headers}
  ${multiview_files_sources}
)

set_property(TARGET openMVG_multiview
  PROPERTY FOLDER OpenMVG
)

install(TARGETS openMVG_multiview
  DESTINATION lib
  EXPORT openMVG-targets
)

target_link_libraries(openMVG_multiview
  ${LEMON_LIBRARY}
  ${LOG_LIB}
)

# Make multiview library integrate numeric library
add_library(openMVG_multiview_test_data
  ${multiview_files_test_data}
)

set_property(TARGET openMVG_multiview_test_data
  PROPERTY FOLDER OpenMVG/OpenMVG
)

target_link_libraries(openMVG_multiview
  openMVG_numeric
  ${CERES_LIBRARIES}
)

set_target_properties(openMVG_multiview
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)


# Triangulation routines
UNIT_TEST(openMVG triangulation       "openMVG_multiview;openMVG_multiview_test_data")
UNIT_TEST(openMVG triangulation_nview "openMVG_multiview;openMVG_multiview_test_data")

# Solvers
UNIT_TEST(openMVG solver_affine "openMVG_multiview")
UNIT_TEST(openMVG solver_fundamental_kernel "openMVG_multiview")
UNIT_TEST(openMVG solver_essential_kernel "openMVG_multiview;openMVG_multiview_test_data")
UNIT_TEST(openMVG solver_homography_kernel "openMVG_multiview;openMVG_multiview_test_data")
UNIT_TEST(openMVG solver_essential_five_point "openMVG_multiview;openMVG_multiview_test_data")
UNIT_TEST(openMVG solver_resection_kernel "openMVG_multiview;openMVG_multiview_test_data")
UNIT_TEST(openMVG solver_resection_p4pf "openMVG_multiview;openMVG_multiview_test_data")
UNIT_TEST(openMVG solver_resection_p5pfr "openMVG_multiview;openMVG_multiview_test_data")
UNIT_TEST(openMVG solver_resection_LORansac "openMVG_multiview;openMVG_features;openMVG_sfm;openMVG_multiview_test_data;${CERES_LIBRARIES}")
UNIT_TEST(openMVG solver_translation_knownRotation_kernel "openMVG_multiview;openMVG_multiview_test_data")

# Averaging routines
UNIT_TEST(openMVG rotation_averaging    "openMVG_multiview;openMVG_multiview_test_data;${CERES_LIBRARIES}")
UNIT_TEST(openMVG translation_averaging "openMVG_multiview;openMVG_multiview_test_data;${CERES_LIBRARIES}")

