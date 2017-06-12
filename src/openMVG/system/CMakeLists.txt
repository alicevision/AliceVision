# Headers
set(system_files_headers
  cpu.hpp
  memoryInfo.hpp
  system.hpp
  timer.hpp
)

# Sources
set(system_files_sources
  cpu.cpp
  memoryInfo.cpp
  timer.cpp
)

add_library(openMVG_system
  ${system_files_sources}
)

set_target_properties(openMVG_system
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

set_property(TARGET openMVG_system
  PROPERTY FOLDER OpenMVG/OpenMVG
)

install(TARGETS openMVG_system
  DESTINATION lib/
  EXPORT openMVG-targets
)

