FILE(
	GLOB
	linearProgramming_headers
	*.hpp
)
FILE(
	GLOB
	linearProgramming_cpp
	*.cpp
)
FILE(GLOB_RECURSE REMOVEFILESUNITTEST *_test.cpp)

SET_SOURCE_FILES_PROPERTIES(${linearProgramming_headers} PROPERTIES LANGUAGE CXX)

#Remove the future main files
LIST(REMOVE_ITEM linearProgramming_cpp ${REMOVEFILESUNITTEST})

SET(openMVG_linearProgramming_DEPS
  ${CLP_LIBRARIES}     # clp + solver wrapper
  ${COINUTILS_LIBRARY} # container tools
  ${OSI_LIBRARY}       # generic LP
)

IF (NOT MOSEK_FOUND)
  ADD_LIBRARY(openMVG_linearProgramming INTERFACE)

  TARGET_LINK_LIBRARIES(openMVG_linearProgramming INTERFACE ${openMVG_linearProgramming_DEPS})

ELSE (NOT MOSEK_FOUND)
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
