# Headers
set(kvld_files_headers
  kvld.h
  kvld_draw.h
  algorithm.h
)

# Sources
set(kvld_files_sources
  kvld.cpp
  algorithm.cpp
)

alicevision_add_library(aliceVision_kvld
  SOURCES ${kvld_files_headers} ${kvld_files_sources}
  PUBLIC_LINKS
    aliceVision_image
    aliceVision_feature
    aliceVision_numeric
    aliceVision_system
)
