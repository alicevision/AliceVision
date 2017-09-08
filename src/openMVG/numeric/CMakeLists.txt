add_library(aliceVision_numeric
  numeric.cpp
  numeric.h
)

target_link_libraries(aliceVision_numeric ${LOG_LIB})

set_target_properties(aliceVision_numeric
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

set_property(TARGET aliceVision_numeric
  PROPERTY FOLDER AliceVision/AliceVision
)

install(TARGETS aliceVision_numeric
  DESTINATION lib
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision numeric aliceVision_numeric)
UNIT_TEST(aliceVision poly    aliceVision_numeric)
UNIT_TEST(aliceVision lm      aliceVision_numeric)
