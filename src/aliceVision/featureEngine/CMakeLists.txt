# Headers
set(featureEngine_files_headers
  FeatureExtractor.hpp
)

# Sources
set(featureEngine_files_sources
  FeatureExtractor.cpp
)

# Library
alicevision_add_library(aliceVision_featureEngine
  SOURCES ${featureEngine_files_headers} ${featureEngine_files_sources}
  PUBLIC_LINKS
    aliceVision_feature
    aliceVision_image
    aliceVision_sfmData
    aliceVision_system
)
