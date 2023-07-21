# Headers
set(registration_files_headers
  nanoflannKDTreeEigenAdaptor.hpp
  rigidMotionEstimator.hpp
  directSimilarityEstimator.hpp
  ICP.hpp
  SICP.hpp
  PointcloudRegistration.hpp
)

# Sources
set(registration_files_sources
  PointcloudRegistration.cpp
)

alicevision_add_library(aliceVision_registration
        SOURCES ${registration_files_headers} ${registration_files_sources}
        PUBLIC_LINKS
        Boost::filesystem
        ${PCL_LIBRARIES}
        nanoflann
        )
