# Headers
set(system_files_headers
  cpu.hpp
  gpu.hpp
  MemoryInfo.hpp
  system.hpp
  Timer.hpp
  Logger.hpp
  nvtx.hpp
)

# Sources
set(system_files_sources
  cpu.cpp
  MemoryInfo.cpp
  Timer.cpp
  Logger.cpp
  nvtx.cpp
)

if(DONTUSETHIS_ALICEVISION_USE_NVTX_PROFILING)
    set(DONTUSETHIS_ALICEVISION_NVTX_LIBRARY ${CUDA_NVTX_LIBRARY})
else()
    set(DONTUSETHIS_ALICEVISION_NVTX_LIBRARY "")
endif()

alicevision_add_library(aliceVision_system
  SOURCES ${system_files_headers} ${system_files_sources}
  PUBLIC_LINKS
    ${Boost_LOG_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_LOG_SETUP_LIBRARY}
    ${Boost_DETAIL_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${ALICEVISION_NVTX_LIBRARY}
  PUBLIC_INCLUDE_DIRS
    ${Boost_INCLUDE_DIR}
)

if(ALICEVISION_USE_NVTX_PROFILING)
target_link_libraries(aliceVision_system ${CUDA_NVTX_LIBRARY})
endif()


set_target_properties(aliceVision_system
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

install(TARGETS aliceVision_system
  DESTINATION lib/
  EXPORT aliceVision-targets
)

