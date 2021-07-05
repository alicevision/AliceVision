# Headers
set(numeric_files_headers
  numeric.hpp
  projection.hpp
  gps.hpp
)

# Sources
set(numeric_files_sources
  numeric.cpp
  projection.cpp
  gps.cpp
)

alicevision_add_library(aliceVision_numeric
  SOURCES ${numeric_files_headers} ${numeric_files_sources}
  PUBLIC_LINKS
    Eigen3::Eigen
    Boost::boost
    Boost::log
    Boost::log_setup
  PUBLIC_DEFINITIONS
    ${AV_EIGEN_DEFINITIONS}
)

# Unit tests
alicevision_add_test(numeric_test.cpp    NAME "numeric"            LINKS aliceVision_numeric)
alicevision_add_test(polynomial_test.cpp NAME "numeric_polynomial" LINKS aliceVision_numeric)
alicevision_add_test(lmFunctor_test.cpp  NAME "numeric_lmFunctor"  LINKS aliceVision_numeric)
alicevision_add_test(projection_test.cpp NAME "numeric_projection" LINKS aliceVision_numeric)
alicevision_add_test(gps_test.cpp NAME "numeric_gps" LINKS aliceVision_numeric)
