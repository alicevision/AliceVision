# Headers
set(lInftycomputervision_files_headers
  lInfinityCV.hpp
  resection.hpp
  resection_kernel.hpp
  triangulation.hpp
  global_translations_fromTij.hpp
  global_translations_fromTriplets.hpp
  tijsAndXis_From_xi_Ri.hpp
  tijsAndXis_From_xi_Ri_noise.hpp
  triplet_tijsAndXis_kernel.hpp
)

# Sources
set(lInftycomputervision_files_sources
  resection_kernel.cpp
)

# Disable some warnings on MSVC.
if (MSVC)
  # Disable Warning	C4267: conversion from 'size_t' to 'int', possible loss of data
  add_definitions("/wd4267")
endif()

alicevision_add_library(aliceVision_lInftyComputerVision
  SOURCES ${lInftycomputervision_files_headers} ${lInftycomputervision_files_sources}
  PUBLIC_LINKS
    aliceVision_linearProgramming
)


if(MOSEK_FOUND)  
    target_link_libraries(aliceVision_lInftyComputerVision PUBLIC ${MOSEK_LIB})
endif()

# Unit tests
# Linfinity Triangulation
alicevision_add_test(lInftyCV_triangulation_test.cpp
  NAME "lInftyCV_triangulation"
  LINKS aliceVision_multiview_test_data
        aliceVision_multiview
        aliceVision_lInftyComputerVision
)

# Linfinity Pose/Resection estimation
alicevision_add_test(lInftyCV_resection_test.cpp
  NAME "lInftyCV_resection"
  LINKS aliceVision_multiview_test_data
        aliceVision_multiview
        aliceVision_lInftyComputerVision
)
alicevision_add_test(lInftyCV_resection_robust_test.cpp
  NAME "lInftyCV_resectionRobust"
  LINKS aliceVision_multiview_test_data
        aliceVision_multiview
        aliceVision_lInftyComputerVision
)
# Linfinity Global translations from triplets of heading directions
alicevision_add_test(lInftyCV_global_translations_fromTriplets_test.cpp
  NAME "lInftyCV_globalTranslationsFromTriplets"
  LINKS aliceVision_multiview_test_data
        aliceVision_multiview
        aliceVision_lInftyComputerVision
)

# Linfinity Global translations from heading directions
alicevision_add_test(lInftyCV_global_translations_fromTij_test.cpp
  NAME "lInftyCV_globalTranslationsFromTij"
  LINKS aliceVision_multiview_test_data
        aliceVision_multiview
        aliceVision_lInftyComputerVision
)
# Linfinity Translation and Structure 
alicevision_add_test(lInftyCV_tijsAndXis_test.cpp
  NAME "lInftyCV_tijsAndXis"
  LINKS aliceVision_multiview_test_data
        aliceVision_multiview
        aliceVision_lInftyComputerVision
)

# Linfinity Translation and Structure with noise handling
alicevision_add_test(lInftyCV_tijsAndXis_outlier_test.cpp
  NAME "lInftyCV_tijsAndXisOutlier"
  LINKS aliceVision_multiview_test_data
        aliceVision_multiview
        aliceVision_lInftyComputerVision
)
