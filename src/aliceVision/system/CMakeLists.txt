if(ALICEVISION_HAVE_TBB)
  set(system_files_headers_tbb ParallelismBackendTBB.hpp)
  set(system_files_sources_tbb ParallelismBackendTBB.cpp)
  set(system_tbb_library TBB::tbb)
  set(system_tbb_definitions "-DALICEVISION_HAVE_TBB=1")
endif()

# Headers
set(system_files_headers
  cpu.hpp
  main.hpp
  MemoryInfo.hpp
  system.hpp
  Timer.hpp
  Logger.hpp
  ParallelFor.hpp
  ParallelismBackend.hpp
  ParallelismBackendOpenMP.hpp
  ${system_files_headers_tbb}
  ParallelSettings.hpp
  ProgressDisplay.hpp
  nvtx.hpp
)

# Sources
set(system_files_sources
  cpu.cpp
  MemoryInfo.cpp
  Timer.cpp
  Logger.cpp
  ParallelismBackend.cpp
  ParallelismBackendOpenMP.cpp
  ${system_files_sources_tbb}
  ProgressDisplay.cpp
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
    ${system_tbb_library}
  PRIVATE_DEFINITIONS
    ${system_tbb_definitions}
)

alicevision_add_test(Logger_test.cpp NAME "system_Logger" LINKS aliceVision_system)
