# Sources
set(cmdline_files_sources
  cmdline.cpp
)

alicevision_add_library(aliceVision_cmdline
  SOURCES ${cmdline_files_sources}
  PUBLIC_LINKS
    Boost::program_options
  PRIVATE_LINKS
    aliceVision_system
)
