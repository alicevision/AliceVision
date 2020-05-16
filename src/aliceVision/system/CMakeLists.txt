# Headers
set(system_files_headers
  cpu.hpp
  main.hpp
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

alicevision_add_library(aliceVision_system
  SOURCES ${system_files_headers} ${system_files_sources}
  PUBLIC_LINKS
    Boost::log
    Boost::log_setup
    Boost::thread
    Boost::system
    Boost::date_time
    Boost::program_options
    ${ALICEVISION_NVTX_LIBRARY}
  PRIVATE_LINKS
    Boost::boost
)

alicevision_add_test(Logger_test.cpp NAME "system_Logger" LINKS aliceVision_system)