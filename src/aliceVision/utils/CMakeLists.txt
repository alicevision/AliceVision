# Headers
set(utils_files_headers
  Histogram.hpp
  regexFilter.hpp
  filesIO.hpp
)

alicevision_add_interface(aliceVision_utils
  SOURCES ${utils_files_headers}
)

# Unit tests
alicevision_add_test(regexFilter_test.cpp NAME "utils_regexFilter" LINKS aliceVision_utils)