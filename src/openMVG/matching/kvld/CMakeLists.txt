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

add_library(aliceVision_kvld
  ${kvld_files_headers}
  ${kvld_files_sources}
)

set_target_properties(aliceVision_kvld
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

set_property(TARGET aliceVision_kvld
  PROPERTY FOLDER AliceVision/AliceVision
)

install(TARGETS aliceVision_kvld
  DESTINATION lib
  EXPORT aliceVision-targets
)
