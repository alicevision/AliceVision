# Headers
set(rig_files_headers
  Rig.hpp
  ResidualError.hpp
)

# Sources
set(rig_files_sources
  Rig.cpp
)

add_library(aliceVision_rig
  ${rig_files_headers}
  ${rig_files_sources}
)

target_link_libraries(aliceVision_rig
PUBLIC
  aliceVision_camera
  aliceVision_geometry
  aliceVision_localization
  aliceVision_numeric
  ${CERES_LIBRARIES}
  Eigen3::Eigen
PRIVATE
  aliceVision_sfm
  aliceVision_system
)

set_target_properties(aliceVision_rig
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

set_property(TARGET aliceVision_rig
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_rig
  DESTINATION lib
  EXPORT aliceVision-targets
)
