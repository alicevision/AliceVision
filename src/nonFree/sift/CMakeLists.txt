# libs should be static
set(BUILD_SHARED_LIBS ON)

# use PIC code for link into shared lib
if(UNIX)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
endif()

include_directories(./vl)

set(FEATS
  vl/covdet.c
  vl/generic.c
  vl/host.c
  vl/imopv.c
  vl/imopv_sse2.c
  vl/mathop.c
  vl/mathop_avx.c
  vl/mathop_sse2.c
  vl/mser.c
  vl/random.c
  vl/scalespace.c
  vl/sift.c
  vl/stringop.c
)

set(FEATS_H
  vl/covdet.h
  vl/float.th
  vl/generic.h
  vl/host.h
  vl/imopv.h
  vl/imopv_sse2.h
  vl/mathop.h
  vl/mathop_avx.h
  vl/mathop_sse2.h
  vl/mser.h
  vl/random.h
  vl/scalespace.h
  vl/sift.h
  vl/stringop.h
)

set_source_files_properties(${FEATS} ${FEATS_H} PROPERTIES LANGUAGE C)

add_definitions(-DVL_DISABLE_AVX -DVL_BUILD_DLL)

add_library(vlsift ${FEATS} ${FEATS_H})


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
