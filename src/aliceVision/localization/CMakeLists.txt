# Headers
set(localization_files_headers
  LocalizationResult.hpp
  VoctreeLocalizer.hpp
  optimization.hpp
  reconstructed_regions.hpp
  ILocalizer.hpp
  rigResection.hpp
)

# Sources
set(localization_files_sources
  LocalizationResult.cpp
  VoctreeLocalizer.cpp
  optimization.cpp
  rigResection.cpp
)

if (ALICEVISION_HAVE_CCTAG)
  list(APPEND localization_files_headers CCTagLocalizer.hpp)
  list(APPEND localization_files_sources CCTagLocalizer.cpp)
endif()

alicevision_add_library(aliceVision_localization
  SOURCES ${localization_files_headers} ${localization_files_sources}
  PUBLIC_LINKS
    aliceVision_camera
    aliceVision_feature
    aliceVision_geometry
    aliceVision_image
    aliceVision_matching
    aliceVision_numeric
    aliceVision_robustEstimation
    aliceVision_sfm
    aliceVision_voctree
  PRIVATE_LINKS
    aliceVision_system
    aliceVision_matchingImageCollection
    Boost::filesystem
    Boost::boost
)

if(ALICEVISION_HAVE_CCTAG)
  target_link_libraries(aliceVision_localization PRIVATE CCTag::CCTag)
endif()

if(ALICEVISION_HAVE_OPENGV)
  target_link_libraries(aliceVision_localization PRIVATE opengv)
endif()

# Unit tests
alicevision_add_test(LocalizationResult_test.cpp NAME "localization_localizationResult" LINKS aliceVision_localization)

if(ALICEVISION_HAVE_OPENGV)
  alicevision_add_test(rigResection_test.cpp NAME "localization_rigResection" LINKS aliceVision_localization)
endif()
