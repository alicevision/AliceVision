
set(system_files_header timer.hpp)

set(system_files_cpp timer.cpp)

add_library(openMVG_system ${system_files_cpp})

set_target_properties(openMVG_system PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR} VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}")
set_propertY(TARGET openMVG_system PROPERTY FOLDER OpenMVG/OpenMVG)
install(TARGETS openMVG_system DESTINATION lib/ EXPORT openMVG-targets)

