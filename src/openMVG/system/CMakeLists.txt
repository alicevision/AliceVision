# Headers
set(system_files_headers
  cpu.hpp
  memoryInfo.hpp
  system.hpp
  timer.hpp
  Logger.hpp
)

# Sources
set(system_files_sources
  cpu.cpp
  memoryInfo.cpp
  timer.cpp
  Logger.cpp
)

add_library(openMVG_system
  ${system_files_headers}
  ${system_files_sources}
)

set_property(TARGET openMVG_system
  PROPERTY FOLDER OpenMVG/OpenMVG
)

if(OPENMVG_HAVE_BOOST)
  target_link_libraries(openMVG_system ${Boost_LIBRARIES})
endif()

set_target_properties(openMVG_system
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

install(TARGETS openMVG_system
  DESTINATION lib/
  EXPORT openMVG-targets
)

