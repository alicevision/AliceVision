set(linearProgramming_headers
    bisectionLP.hpp
    linearProgramming.hpp
    linearProgrammingInterface.hpp
    linearProgrammingOSI_X.hpp)

set(linearProgramming_cpp "")


SET_SOURCE_FILES_PROPERTIES(${linearProgramming_headers} PROPERTIES LANGUAGE CXX)

SET(openMVG_linearProgramming_DEPS
  ${CLP_LIBRARIES}     # clp + solver wrapper
  ${COINUTILS_LIBRARY} # container tools
  ${OSI_LIBRARY}       # generic LP
)

IF (NOT MOSEK_FOUND)
  ADD_LIBRARY(openMVG_linearProgramming INTERFACE)

  TARGET_LINK_LIBRARIES(openMVG_linearProgramming INTERFACE ${openMVG_linearProgramming_DEPS})

ELSE (NOT MOSEK_FOUND)
  # Add MOSEK related sources
  list(APPEND linearProgramming_cpp linearProgrammingMOSEK.cpp)
  list(APPEND linearProgramming_headers linearProgrammingMOSEK.hpp)

  # MOSEK_FOUND is true
  ADD_LIBRARY(openMVG_linearProgramming ${linearProgramming_headers} ${linearProgramming_cpp})
  SET_PROPERTY(TARGET openMVG_linearProgramming PROPERTY FOLDER OpenMVG/OpenMVG)

  TARGET_LINK_LIBRARIES(openMVG_linearProgramming
    ${openMVG_linearProgramming_DEPS}
    lib_Osi_Msk          # OSI solver wrapper for the Mosek backend
    ${MOSEK_LIB}    # Mosek lib
    )
ENDIF()

INSTALL(TARGETS openMVG_linearProgramming DESTINATION lib EXPORT openMVG-targets)

UNIT_TEST(openMVG_linearProgramming linearProgramming "openMVG_linearProgramming")

ADD_SUBDIRECTORY(lInfinityCV)
