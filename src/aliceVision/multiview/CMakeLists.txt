# Headers
set(multiview_files_headers
  affineSolver.hpp
  conditioning.hpp
  essential.hpp
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

# Test Data Sources
set(multiview_files_test_data
  NViewDataSet.hpp
  NViewDataSet.cpp
)

alicevision_add_library(aliceVision_multiview
  SOURCES ${multiview_files_headers} ${multiview_files_sources}
  PUBLIC_LINKS
    aliceVision_numeric
    aliceVision_robustEstimation
    ${CERES_LIBRARIES}
  PRIVATE_LINKS
    aliceVision_system
    ${LEMON_LIBRARY}
)

alicevision_add_library(aliceVision_multiview_test_data
  SOURCES ${multiview_files_test_data}
  PUBLIC_LINKS
    aliceVision_numeric
  PRIVATE_LINKS
    aliceVision_multiview
)

# Unit tests
add_subdirectory(resection)
add_subdirectory(rotationAveraging)
add_subdirectory(translationAveraging)
add_subdirectory(triangulation)

alicevision_add_test(affineSolver_test.cpp                   NAME "multiview_affineSolver"                    LINKS aliceVision_multiview)
alicevision_add_test(fundamentalKernelSolver_test.cpp        NAME "multiview_fundamentalKernelSolver"         LINKS aliceVision_multiview)
alicevision_add_test(essentialFivePointSolver_test.cpp       NAME "multiview_essentialFivePointSolver"        LINKS aliceVision_multiview aliceVision_multiview_test_data)
alicevision_add_test(essentialKernelSolver_test.cpp          NAME "multiview_essentialKernelSolver"           LINKS aliceVision_multiview aliceVision_multiview_test_data)
alicevision_add_test(homographyKernelSolver_test.cpp         NAME "multiview_homographyKernelSolver"          LINKS aliceVision_multiview aliceVision_multiview_test_data)
alicevision_add_test(knownRotationTranslationKernel_test.cpp NAME "multiview_knownRotationTranslationKernel"  LINKS aliceVision_multiview aliceVision_multiview_test_data)
