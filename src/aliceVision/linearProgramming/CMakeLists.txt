# Headers
set(linearProgramming_files_headers
    bisectionLP.hpp
    linearProgramming.hpp
    ISolver.hpp
    OSIXSolver.hpp
)

# Dependencies
set(aliceVision_linearProgramming_DEPS
  aliceVision_numeric
  aliceVision_system
  ${CLP_LIBRARIES}     # clp + solver wrapper
  ${COINUTILS_LIBRARY} # container tools
  ${OSI_LIBRARY}       # generic LP
)

if (NOT MOSEK_FOUND)
alicevision_add_interface(aliceVision_linearProgramming
  SOURCES ${linearProgramming_files_headers} ${linearProgramming_files_sources}
  LINKS ${aliceVision_linearProgramming_DEPS}
)
else ()
  # Add MOSEK related sources
  list(APPEND linearProgramming_files_sources MOSEKSolver.cpp)
  list(APPEND linearProgramming_files_headers MOSEKSolver.hpp)
  alicevision_add_library(aliceVision_linearProgramming
    SOURCES ${linearProgramming_files_headers}
    PUBLIC_LINKS
      ${aliceVision_linearProgramming_DEPS}
      lib_Osi_Msk   # OSI solver wrapper for the Mosek backend
      ${MOSEK_LIB}  # Mosek lib
  )
endif()

# Unit tests
alicevision_add_test(linearProgramming_test.cpp NAME "linearProgramming" LINKS aliceVision_linearProgramming)

# lInfinityCV library
add_subdirectory(lInfinityCV)
