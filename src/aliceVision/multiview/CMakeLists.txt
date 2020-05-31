# Headers
set(multiview_files_headers
  affineSolver.hpp
  essential.hpp
  epipolarEquation.hpp
  knownRotationTranslationKernel.hpp
  Unnormalizer.hpp
  AngularRadianErrorKernel.hpp
  RelativePoseKernel.hpp
  ResectionKernel.hpp
  relativePose/Essential5PSolver.hpp
  relativePose/Essential8PSolver.hpp
  relativePose/EssentialKernel.hpp
  relativePose/Fundamental7PSolver.hpp
  relativePose/Fundamental8PSolver.hpp
  relativePose/Fundamental10PSolver.hpp
  relativePose/FundamentalError.hpp
  relativePose/FundamentalKernel.hpp
  relativePose/Homography4PSolver.hpp
  relativePose/HomographyError.hpp
  relativePose/HomographyKernel.hpp
  relativePose/Rotation3PSolver.hpp
  relativePose/ISolverErrorRelativePose.hpp
  resection/EPnPSolver.hpp
  resection/EPnPKernel.hpp
  resection/P3PSolver.hpp
  resection/P4PfSolver.hpp
  resection/P5PfrSolver.hpp
  resection/ProjectionDistanceError.hpp
  resection/Resection6PSolver.hpp   
  resection/ResectionKernel.hpp
  resection/ISolverErrorResection.hpp
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
  essential.cpp
  Unnormalizer.cpp
  relativePose/Essential5PSolver.cpp
  relativePose/Essential8PSolver.cpp
  relativePose/Fundamental7PSolver.cpp
  relativePose/Fundamental8PSolver.cpp
  relativePose/Fundamental10PSolver.cpp
  relativePose/Homography4PSolver.cpp
  relativePose/Rotation3PSolver.cpp
  resection/EPnPSolver.cpp
  resection/P3PSolver.cpp
  resection/P4PfSolver.cpp
  resection/P5PfrSolver.cpp
  resection/Resection6PSolver.cpp
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
add_subdirectory(relativePose)
add_subdirectory(resection)
add_subdirectory(rotationAveraging)
add_subdirectory(translationAveraging)
add_subdirectory(triangulation)

alicevision_add_test(affineSolver_test.cpp                   NAME "multiview_affineSolver"                    LINKS aliceVision_multiview)
alicevision_add_test(knownRotationTranslationKernel_test.cpp NAME "multiview_knownRotationTranslationKernel"  LINKS aliceVision_multiview aliceVision_multiview_test_data)
