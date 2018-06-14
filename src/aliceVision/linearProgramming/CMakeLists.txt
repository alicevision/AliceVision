# Headers
set(linearProgramming_headers
    bisectionLP.hpp
    linearProgramming.hpp
    ISolver.hpp
    OSIXSolver.hpp
)

# Sources
set(linearProgramming_sources
  ""
)

# Tests
set(linearProgramming_files_test
  linearProgramming_test.cpp
)

# Dependencies
set(aliceVision_linearProgramming_DEPS
  aliceVision_numeric
  aliceVision_system
  ${CLP_LIBRARIES}     # clp + solver wrapper
  ${COINUTILS_LIBRARY} # container tools
  ${OSI_LIBRARY}       # generic LP
)

set_source_files_properties(${linearProgramming_headers} PROPERTIES LANGUAGE CXX)

if (NOT MOSEK_FOUND)
  add_library(aliceVision_linearProgramming INTERFACE)

  target_link_libraries(aliceVision_linearProgramming
    INTERFACE ${aliceVision_linearProgramming_DEPS}
  )
  add_custom_target(aliceVision_linearProgramming_ide SOURCES ${linearProgramming_headers} ${linearProgramming_files_test})

  set_property(TARGET aliceVision_linearProgramming_ide
    PROPERTY FOLDER AliceVision
  )

else ()

  # Add MOSEK related sources
  list(APPEND linearProgramming_sources MOSEKSolver.cpp)
  list(APPEND linearProgramming_headers MOSEKSolver.hpp)

  # MOSEK_FOUND is true
  add_library(aliceVision_linearProgramming
    ${linearProgramming_headers}
    ${linearProgramming_sources}
  )

  set_property(TARGET aliceVision_linearProgramming
    PROPERTY FOLDER AliceVision/AliceVision
  )

  target_link_libraries(aliceVision_linearProgramming
      ${aliceVision_linearProgramming_DEPS}
      lib_Osi_Msk   # OSI solver wrapper for the Mosek backend
      ${MOSEK_LIB}  # Mosek lib
  )

endif()

install(TARGETS aliceVision_linearProgramming
  DESTINATION lib
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision linearProgramming "aliceVision_linearProgramming")

add_subdirectory(lInfinityCV)
