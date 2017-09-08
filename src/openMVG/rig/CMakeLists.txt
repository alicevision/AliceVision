# Headers
set(rig_files_headers
  Rig.hpp
  rig_BA_ceres.hpp
)

# Sources
set(rig_files_sources
  Rig.cpp
)

add_library(aliceVision_rig
  ${rig_files_header}
  ${rig_files_sources}
)

target_link_libraries(aliceVision_rig
  aliceVision_localization
  ${LOG_LIB}
)

set_target_properties(aliceVision_rig
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

set_property(TARGET aliceVision_rig
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_rig
  DESTINATION lib
  EXPORT aliceVision-targets
)
