# libs should be static
set(BUILD_SHARED_LIBS OFF)

# use PIC code for link into shared lib
if(UNIX)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
endif()

include_directories(./vl)

set(FEATS
  vl/generic.c
  vl/imopv_sse2.c
  vl/sift.c
  vl/imopv.c
  vl/mathop_sse2.c
  vl/sift.c
  vl/host.c
  vl/mathop.c
  vl/random.c
)

set_source_files_properties(${FEATS} PROPERTIES LANGUAGE C)

add_library(vlsift ${FEATS})

install(TARGETS vlsift
  DESTINATION lib
  EXPORT aliceVision-targets
)

set_property(TARGET vlsift
  PROPERTY FOLDER nonFree
)

install(
  DIRECTORY .
  DESTINATION include/aliceVision_dependencies/nonFree/sift
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
