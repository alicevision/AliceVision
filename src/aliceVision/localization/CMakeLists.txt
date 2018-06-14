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

add_library(aliceVision_localization
  ${localization_files_headers}
  ${localization_files_sources}
)

target_link_libraries(aliceVision_localization
  PUBLIC 
    aliceVision_camera
    aliceVision_feature
    aliceVision_geometry
    aliceVision_image
    aliceVision_matching
    aliceVision_numeric
    aliceVision_robustEstimation
    aliceVision_sfm
    aliceVision_voctree
  PRIVATE
    aliceVision_system
    aliceVision_matchingImageCollection
    ${Boost_FILESYSTEM_LIBRARY}
)

if(ALICEVISION_HAVE_CCTAG)
  target_link_libraries(aliceVision_localization PRIVATE CCTag::CCTag)
endif()

if(ALICEVISION_HAVE_OPENGV)
  target_link_libraries(aliceVision_localization PRIVATE ${OPENGV_LIBRARY})
endif()

set_target_properties(aliceVision_localization
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

set_property(TARGET aliceVision_localization
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_localization
  DESTINATION lib
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision LocalizationResult "aliceVision_localization")

if(ALICEVISION_HAVE_OPENGV)
  UNIT_TEST(aliceVision rigResection  "aliceVision_localization")
endif()
