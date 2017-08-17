# Headers
set(linearProgramming_headers
    bisectionLP.hpp
    linearProgramming.hpp
    linearProgrammingInterface.hpp
    linearProgrammingOSI_X.hpp
)

# Sources
set(linearProgramming_sources
  ""
)

# Dependencies
set(openMVG_linearProgramming_DEPS
  ${CLP_LIBRARIES}     # clp + solver wrapper
  ${COINUTILS_LIBRARY} # container tools
  ${OSI_LIBRARY}       # generic LP
)

set_source_files_properties(${linearProgramming_headers} PROPERTIES LANGUAGE CXX)

if (NOT MOSEK_FOUND)
  add_library(openMVG_linearProgramming INTERFACE)

  target_link_libraries(openMVG_linearProgramming
    INTERFACE ${openMVG_linearProgramming_DEPS}
  )

else ()

  # Add MOSEK related sources
  list(APPEND linearProgramming_cpp linearProgrammingMOSEK.cpp)
  list(APPEND linearProgramming_headers linearProgrammingMOSEK.hpp)

  # MOSEK_FOUND is true
  add_library(openMVG_linearProgramming
    ${linearProgramming_headers}
    ${linearProgramming_sources}
  )

  set_property(TARGET openMVG_linearProgramming
    PROPERTY FOLDER OpenMVG/OpenMVG
  )

  target_link_libraries(openMVG_linearProgramming
      ${openMVG_linearProgramming_DEPS}
      lib_Osi_Msk   # OSI solver wrapper for the Mosek backend
      ${MOSEK_LIB}  # Mosek lib
  )

endif()

install(TARGETS openMVG_linearProgramming
  DESTINATION lib
  EXPORT openMVG-targets
)

UNIT_TEST(openMVG linearProgramming "openMVG_linearProgramming")

add_subdirectory(lInfinityCV)
