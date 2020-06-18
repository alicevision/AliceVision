# Headers
set(utils_files_headers
  Histogram.hpp
  regexFilter.hpp
)

alicevision_add_interface(aliceVision_utils
  SOURCES ${utils_files_headers}
)