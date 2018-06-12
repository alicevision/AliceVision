# Headers
set(lInftycomputervision_headers
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
set(lInftycomputervision_sources
  resection_kernel.cpp
)

# Disable some warnings on MSVC.
if (MSVC)
  # Disable Warning	C4267: conversion from 'size_t' to 'int', possible loss of data
  add_definitions("/wd4267")
endif()

add_library(aliceVision_lInftyComputerVision
  ${lInftycomputervision_headers}
  ${lInftycomputervision_sources}
)

set_property(TARGET aliceVision_lInftyComputerVision
  PROPERTY FOLDER AliceVision
)

target_link_libraries(aliceVision_lInftyComputerVision
  PUBLIC aliceVision_linearProgramming
)

if(MOSEK_FOUND)  
    target_link_libraries(aliceVision_lInftyComputerVision PUBLIC ${MOSEK_LIB})
endif()

set_target_properties(aliceVision_lInftyComputerVision
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

install(TARGETS aliceVision_lInftyComputerVision
  DESTINATION lib
  EXPORT aliceVision-targets
)

set(LOCAL_LIBS
  aliceVision_multiview_test_data
  aliceVision_multiview
  aliceVision_lInftyComputerVision
)

# Linfinity Triangulation
UNIT_TEST(aliceVision lInftyCV_triangulation "${LOCAL_LIBS}")

# Linfinity Pose/Resection estimation
UNIT_TEST(aliceVision lInftyCV_resection        "${LOCAL_LIBS}")
UNIT_TEST(aliceVision lInftyCV_resection_robust "${LOCAL_LIBS}")

# Linfinity Global translations from triplets of heading directions
UNIT_TEST(aliceVision lInftyCV_global_translations_fromTriplets "${LOCAL_LIBS}")

# Linfinity Global translations from heading directions
UNIT_TEST(aliceVision lInftyCV_global_translations_fromTij "${LOCAL_LIBS}")

# Linfinity Translation and Structure 
UNIT_TEST(aliceVision lInftyCV_tijsAndXis "${LOCAL_LIBS}")

# Linfinity Translation and Structure with noise handling
UNIT_TEST(aliceVision lInftyCV_tijsAndXis_outlier "${LOCAL_LIBS}")

