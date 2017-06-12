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


if (OpenMVG_USE_CCTAG)
  list(APPEND localization_files_headers CCTagLocalizer.hpp)
  list(APPEND localization_files_sources CCTagLocalizer.cpp)
endif()

add_library(openMVG_localization
  ${localization_files_headers}
  ${localization_files_sources}
)

target_link_libraries(openMVG_localization
  PUBLIC openMVG_sfm
         openMVG_voctree
         openMVG_numeric
  PRIVATE openMVG_system
          stlplus
          ${Boost_LIBRARIES}
          ${LOG_LIB}
)

if(OpenMVG_USE_CCTAG)
  target_link_libraries(openMVG_localization PRIVATE CCTag::CCTag)
endif()

if(OpenMVG_USE_OPENGV)
  target_link_libraries(openMVG_localization PRIVATE ${OPENGV_LIBRARY})
endif()

set_target_properties(openMVG_localization
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

set_property(TARGET openMVG_localization
  PROPERTY FOLDER OpenMVG
)

install(TARGETS openMVG_localization
  DESTINATION lib
  EXPORT openMVG-targets
)

UNIT_TEST(openMVG LocalizationResult "openMVG_localization")

if(OpenMVG_USE_OPENGV)
  UNIT_TEST(openMVG rigResection  "openMVG_localization")
endif()
