include_directories (${PCL_INCLUDE_DIRS})
message(STATUS "PCL version: ${PCL_VERSION}")
link_directories (${PCL_LIBRARY_DIRS})
add_definitions (${PCL_DEFINITIONS})

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

add_library(aliceVision_registration
  ${registration_files_headers}
  ${registration_files_sources}
)

target_link_libraries(aliceVision_registration
  PUBLIC
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_LOG_LIBRARY}
    ${PCL_LIBRARIES}
    nanoflann
)

set_target_properties(aliceVision_registration
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

set_property(TARGET aliceVision_registration
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_registration
  DESTINATION lib
  EXPORT aliceVision-targets
)
