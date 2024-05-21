# Headers
set(dataio_files_headers
  FeedProvider.hpp
  IFeed.hpp
  ImageFeed.hpp
  SfMDataFeed.hpp
  json.hpp
)

# Sources
set(dataio_files_sources
  FeedProvider.cpp
  IFeed.cpp
  ImageFeed.cpp
  SfMDataFeed.cpp
  json.cpp
)

if(ALICEVISION_HAVE_OPENCV)
  list(APPEND dataio_files_headers VideoFeed.hpp)
  list(APPEND dataio_files_sources VideoFeed.cpp)
endif()

if(NOT ALICEVISION_BUILD_LIDAR STREQUAL "OFF")
  list(APPEND dataio_files_headers E57Reader.hpp)
  list(APPEND dataio_files_sources E57Reader.cpp)
endif()

alicevision_add_library(aliceVision_dataio
  SOURCES ${dataio_files_headers} ${dataio_files_sources}
  PUBLIC_LINKS
    aliceVision_camera
    aliceVision_image
  PRIVATE_LINKS
    aliceVision_sfmData
    aliceVision_sfmDataIO
    aliceVision_system
    Boost::boost
    Boost::json
)

if(ALICEVISION_HAVE_OPENCV)
  target_link_libraries(aliceVision_dataio PRIVATE ${OpenCV_LIBS})
endif()

if(NOT ALICEVISION_BUILD_LIDAR STREQUAL "OFF")
  target_link_libraries(aliceVision_dataio PRIVATE E57Format)
endif()
