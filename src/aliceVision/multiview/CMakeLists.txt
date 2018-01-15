# Headers
set(multiview_files_headers
  affineSolver.hpp
  conditioning.hpp
  essential.hpp
  projection.hpp
  essentialFivePointSolver.hpp
  essentialKernelSolver.hpp
  fundamentalKernelSolver.hpp
  homographyKernelSolver.hpp
  knownRotationTranslationKernel.hpp
  twoViewKernel.hpp
  resection/P3PSolver.hpp
  resection/P4PfSolver.hpp
  resection/P5PfrSolver.hpp
  resection/ResectionKernel.hpp
  rotationAveraging/common.hpp
  rotationAveraging/rotationAveraging.hpp
  rotationAveraging/l1.hpp
  rotationAveraging/l2.hpp
  translationAveraging/common.hpp
  translationAveraging/solver.hpp
  triangulation/Triangulation.hpp
  triangulation/triangulationDLT.hpp
  triangulation/NViewsTriangulationLORansac.hpp
)

# Sources
set(multiview_files_sources
  affineSolver.cpp
  conditioning.cpp
  essential.cpp
  projection.cpp
  essentialFivePointSolver.cpp
  essentialKernelSolver.cpp
  fundamentalKernelSolver.cpp
  homographyKernelSolver.cpp
  resection/ResectionKernel.cpp
  resection/P3PSolver.cpp
  resection/P4PfSolver.cpp
  resection/P5PfrSolver.cpp
  rotationAveraging/l1.cpp
  rotationAveraging/l2.cpp
  translationAveraging/solverL2Chordal.cpp
  translationAveraging/solverL1Soft.cpp
  triangulation/triangulationDLT.cpp
  triangulation/Triangulation.cpp
)

# Test data
set(multiview_files_test_data
  NViewDataSet.hpp
  NViewDataSet.cpp
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

# Unit test

add_subdirectory(resection)
add_subdirectory(rotationAveraging)
add_subdirectory(translationAveraging)
add_subdirectory(triangulation)


UNIT_TEST(aliceVision affineSolver                    "aliceVision_multiview")
UNIT_TEST(aliceVision fundamentalKernelSolver         "aliceVision_multiview")
UNIT_TEST(aliceVision essentialFivePointSolver        "aliceVision_multiview;aliceVision_multiview_test_data")
UNIT_TEST(aliceVision essentialKernelSolver           "aliceVision_multiview;aliceVision_multiview_test_data")
UNIT_TEST(aliceVision homographyKernelSolver          "aliceVision_multiview;aliceVision_multiview_test_data")
UNIT_TEST(aliceVision knownRotationTranslationKernel  "aliceVision_multiview;aliceVision_multiview_test_data")
