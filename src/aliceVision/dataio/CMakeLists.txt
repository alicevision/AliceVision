# Headers
set(dataio_headers
  FeedProvider.hpp
  IFeed.hpp
  ImageFeed.hpp
)

# Sources
set(dataio_sources
  FeedProvider.cpp
  IFeed.cpp
  ImageFeed.cpp
)

if(ALICEVISION_HAVE_OPENCV)
  list(APPEND dataio_headers VideoFeed.hpp)
  list(APPEND dataio_sources VideoFeed.cpp)
endif()

set_source_files_properties(${dataio_sources} PROPERTIES LANGUAGE CXX)

add_library(aliceVision_dataio
  ${dataio_headers}
  ${dataio_sources}
 )

set_property(TARGET aliceVision_dataio
  PROPERTY FOLDER AliceVision
)

target_link_libraries(aliceVision_dataio
  PUBLIC
    aliceVision_camera
    aliceVision_image
  PRIVATE
    aliceVision_sfm
    aliceVision_system
    ${Boost_FILESYSTEM_LIBRARY}
)

if(ALICEVISION_HAVE_OPENCV)
  target_link_libraries(aliceVision_dataio PRIVATE ${OpenCV_LIBS})
endif()

set_target_properties(aliceVision_dataio
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

install(TARGETS aliceVision_dataio
  DESTINATION lib/
  EXPORT aliceVision-targets
)
