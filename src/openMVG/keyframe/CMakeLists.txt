if(OpenMVG_USE_OIIO)

set(keyframe_files_header
    KeyframeSelector.hpp)
set(keyframe_files_cpp
    KeyframeSelector.cpp)

add_library(openMVG_keyframe ${keyframe_files_header} ${keyframe_files_cpp})
target_link_libraries(
  openMVG_keyframe
  openMVG_features
  openMVG_dataio
  openMVG_voctree
  ${OpenImageIO_LIBRARY}
)
set_target_properties(openMVG_keyframe PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR} VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}")
install(TARGETS openMVG_keyframe DESTINATION lib EXPORT openMVG-targets)

endif(OpenMVG_USE_OIIO)