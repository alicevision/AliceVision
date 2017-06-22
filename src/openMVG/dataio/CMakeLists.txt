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

if(OPENMVG_HAVE_OPENCV)
    list(APPEND dataio_headers VideoFeed.hpp)
    list(APPEND dataio_sources VideoFeed.cpp)
endif()

set_source_files_properties(${dataio_sources} PROPERTIES LANGUAGE CXX)

add_library(openMVG_dataio
  ${dataio_headers}
  ${dataio_sources}
 )

set_property(TARGET openMVG_dataio
  PROPERTY FOLDER OpenMVG/OpenMVG
)

target_link_libraries(openMVG_dataio
  openMVG_sfm
  ${Boost_LIBRARIES}
  ${LOG_LIB}
)

if(OPENMVG_HAVE_OPENCV)
  target_link_libraries(openMVG_dataio ${OpenCV_LIBS})
endif()

set_target_properties(openMVG_dataio
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

install(TARGETS openMVG_dataio
  DESTINATION lib/
  EXPORT openMVG-targets
)
