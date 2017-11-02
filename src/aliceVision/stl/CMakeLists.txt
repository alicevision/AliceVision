# Headers
set(stl_files_headers
  bitmask.hpp
  DynamicBitset.hpp
  FlatMap.hpp
  FlatSet.hpp
  hash.hpp
  indexedSort.hpp
  split.hpp
  stl.hpp
  mapUtils.hpp
)

add_library(aliceVision_stl INTERFACE)

# target_sources(aliceVision_stl INTERFACE ${stl_files_headers}) # TODO

target_link_libraries(aliceVision_stl
  INTERFACE ${LOG_LIB}
)

UNIT_TEST(aliceVision split         "aliceVision_stl")
UNIT_TEST(aliceVision dynamicBitset "aliceVision_stl")

add_custom_target(aliceVision_stl_ide SOURCES ${stl_files_headers})
