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

add_library(aliceVision_system
  ${system_files_headers}
  ${system_files_sources}
)

set_property(TARGET aliceVision_system
  PROPERTY FOLDER AliceVision/AliceVision
)

if(OPENMVG_HAVE_BOOST)
  target_link_libraries(aliceVision_system ${Boost_LIBRARIES})
endif()

set_target_properties(aliceVision_system
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

install(TARGETS aliceVision_system
  DESTINATION lib/
  EXPORT aliceVision-targets
)

