set(dataio_header
    FeedProvider.hpp
    IFeed.hpp
    ImageFeed.hpp)

set(dataio_cpp
    FeedProvider.cpp
    IFeed.cpp
    ImageFeed.cpp)

if(OpenMVG_USE_OPENCV)
    list(APPEND dataio_header VideoFeed.hpp)
    list(APPEND dataio_cpp VideoFeed.cpp)
endif(OpenMVG_USE_OPENCV)

set_source_files_properties(${dataio_cpp} PROPERTIES LANGUAGE CXX)
add_library(openMVG_dataio
  ${dataio_header}
  ${dataio_cpp})
set_property(TARGET openMVG_dataio PROPERTY FOLDER OpenMVG/OpenMVG)
target_link_libraries( openMVG_dataio openMVG_sfm ${Boost_LIBRARIES} ${LOG_LIB})
if(OpenMVG_USE_OPENCV)
  target_link_libraries( openMVG_dataio ${OpenCV_LIBS})
endif(OpenMVG_USE_OPENCV)
set_target_properties(openMVG_dataio PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR} VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}")

install(TARGETS openMVG_dataio DESTINATION lib/ EXPORT openMVG-targets)
