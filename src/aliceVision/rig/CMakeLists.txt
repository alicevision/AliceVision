# Headers
set(rig_files_headers
  Rig.hpp
  ResidualError.hpp
)

# Sources
set(rig_files_sources
  Rig.cpp
)

alicevision_add_library(aliceVision_rig
  SOURCES ${rig_files_headers} ${rig_files_sources}
  PUBLIC_LINKS
    aliceVision_camera
    aliceVision_geometry
    aliceVision_localization
    aliceVision_numeric
  PRIVATE_LINKS
    aliceVision_sfm
    aliceVision_system
    ${CERES_LIBRARIES}
)
