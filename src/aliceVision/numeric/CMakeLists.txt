add_library(aliceVision_numeric
  numeric.cpp
  numeric.hpp
)

target_link_libraries(aliceVision_numeric
  PUBLIC
    ${LOG_LIB}
)

set_target_properties(aliceVision_numeric
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

set_property(TARGET aliceVision_numeric
  PROPERTY FOLDER AliceVision/AliceVision
)

install(TARGETS aliceVision_numeric
  DESTINATION lib
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision numeric       aliceVision_numeric)
UNIT_TEST(aliceVision polynomial    aliceVision_numeric)
UNIT_TEST(aliceVision lmFunctor     aliceVision_numeric)
