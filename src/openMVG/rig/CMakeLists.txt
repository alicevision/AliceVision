# Headers
set(rig_files_headers
  Rig.hpp
  rig_BA_ceres.hpp
)

# Sources
set(rig_files_sources
  Rig.cpp
)

add_library(openMVG_rig
  ${rig_files_header}
  ${rig_files_sources}
)

target_link_libraries(openMVG_rig
  openMVG_localization
  ${LOG_LIB}
)

set_target_properties(openMVG_rig
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

set_property(TARGET openMVG_rig
  PROPERTY FOLDER OpenMVG
)

install(TARGETS openMVG_rig
  DESTINATION lib
  EXPORT openMVG-targets
)
