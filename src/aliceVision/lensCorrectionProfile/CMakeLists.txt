# Headers
set(lensCorrectionProfile_files_headers
  lcp.hpp
)

# Sources
set(lensCorrectionProfile_files_sources
  lcp.cpp
)

alicevision_add_library(aliceVision_lensCorrectionProfile
  SOURCES ${lensCorrectionProfile_files_headers} ${lensCorrectionProfile_files_sources}
  PRIVATE_LINKS
    Boost::log
    expat::expat
)
