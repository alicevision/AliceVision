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
  PROPERTY FOLDER AliceVision/AliceVision
)

target_link_libraries(aliceVision_dataio
  aliceVision_sfm
  ${Boost_LIBRARIES}
  ${LOG_LIB}
)

if(ALICEVISION_HAVE_OPENCV)
  target_link_libraries(aliceVision_dataio ${OpenCV_LIBS})
endif()

set_target_properties(aliceVision_dataio
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

install(TARGETS aliceVision_dataio
  DESTINATION lib/
  EXPORT aliceVision-targets
)
