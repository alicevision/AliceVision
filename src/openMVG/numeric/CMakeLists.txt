add_library(openMVG_numeric
  numeric.cpp
  numeric.h
)

target_link_libraries(openMVG_numeric ${LOG_LIB})

set_target_properties(openMVG_numeric
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

set_property(TARGET openMVG_numeric
  PROPERTY FOLDER AliceVision/AliceVision
)

install(TARGETS openMVG_numeric
  DESTINATION lib
  EXPORT openMVG-targets
)

UNIT_TEST(openMVG numeric openMVG_numeric)
UNIT_TEST(openMVG poly    openMVG_numeric)
UNIT_TEST(openMVG lm      openMVG_numeric)
