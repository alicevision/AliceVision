# Headers
set(kvld_files_headers
  kvld.h
  kvld_draw.h
  algorithm.h
)

# Sources
set(kvld_files_sources
  kvld.cpp
  algorithm.cpp
)

add_library(openMVG_kvld
  ${kvld_files_headers}
  ${kvld_files_sources}
)

set_target_properties(openMVG_kvld
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

set_property(TARGET openMVG_kvld
  PROPERTY FOLDER OpenMVG/OpenMVG
)

install(TARGETS openMVG_kvld
  DESTINATION lib
  EXPORT openMVG-targets
)
