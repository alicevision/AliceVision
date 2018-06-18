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

add_library(aliceVision_system
  ${system_files_headers}
  ${system_files_sources}
)

set_property(TARGET aliceVision_system
  PROPERTY FOLDER AliceVision
)

if(ALICEVISION_USE_NVTX_PROFILING)
    set(ALICEVISION_NVTX_LIBRARY ${CUDA_NVTX_LIBRARY})
else()
    set(ALICEVISION_NVTX_LIBRARY "")
endif()

target_link_libraries(aliceVision_system
  PUBLIC
    ${Boost_LOG_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_LOG_SETUP_LIBRARY}
    ${Boost_DETAIL_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${ALICEVISION_NVTX_LIBRARY}
)

set_target_properties(aliceVision_system
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

install(TARGETS aliceVision_system
  DESTINATION lib/
  EXPORT aliceVision-targets
)

