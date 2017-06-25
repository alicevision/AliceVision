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

add_library(openMVG_lInftyComputerVision
  ${lInftycomputervision_headers}
  ${lInftycomputervision_sources}
)

target_link_libraries(openMVG_lInftyComputerVision
  openMVG_linearProgramming
  ${CLP_LIBRARIES}     # clp + solver wrapper
  ${COINUTILS_LIBRARY} # container tools
  ${OSI_LIBRARY}       # generic LP
)

if(MOSEK_FOUND)  
    target_link_libraries(openMVG_lInftyComputerVision ${MOSEK_LIB})
endif()

set_target_properties(openMVG_lInftyComputerVision
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

install(TARGETS openMVG_lInftyComputerVision
  DESTINATION lib
  EXPORT openMVG-targets
)

set(LOCAL_LIBS
  openMVG_multiview_test_data
  openMVG_multiview
  openMVG_lInftyComputerVision
)

# Linfinity Triangulation
UNIT_TEST(openMVG lInftyCV_triangulation "${LOCAL_LIBS}")

# Linfinity Pose/Resection estimation
UNIT_TEST(openMVG lInftyCV_resection        "${LOCAL_LIBS}")
UNIT_TEST(openMVG lInftyCV_resection_robust "${LOCAL_LIBS}")

# Linfinity Global translations from triplets of heading directions
UNIT_TEST(openMVG lInftyCV_global_translations_fromTriplets "${LOCAL_LIBS}")

# Linfinity Global translations from heading directions
UNIT_TEST(openMVG lInftyCV_global_translations_fromTij "${LOCAL_LIBS}")

# Linfinity Translation and Structure 
UNIT_TEST(openMVG lInftyCV_tijsAndXis "${LOCAL_LIBS}")

# Linfinity Translation and Structure with noise handling
UNIT_TEST(openMVG lInftyCV_tijsAndXis_outlier "${LOCAL_LIBS}")

