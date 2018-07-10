# Headers
set(numeric_files_headers
  numeric.hpp
)

# Sources
set(numeric_files_sources
  numeric.cpp
)

alicevision_add_library(aliceVision_numeric
  SOURCES ${numeric_files_headers} ${numeric_files_sources}
  PUBLIC_LINKS
    Eigen3::Eigen
)

# Unit tests
alicevision_add_test(numeric_test.cpp    NAME "numeric"            LINKS aliceVision_numeric)
alicevision_add_test(polynomial_test.cpp NAME "numeric_polynomial" LINKS aliceVision_numeric)
alicevision_add_test(lmFunctor_test.cpp  NAME "numeric_lmFunctor"  LINKS aliceVision_numeric)
