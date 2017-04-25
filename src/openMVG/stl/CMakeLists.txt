
set(stl_files_header
  bitmask.hpp
  dynamic_bitset.hpp
  flatMap.hpp
  flatSet.hpp
  hash.hpp
  indexed_sort.hpp
  split.hpp
  stl.hpp
  stlMap.hpp
)
add_library(openMVG_stl INTERFACE)
# target_sources(openMVG_stl INTERFACE ${stl_files_header}) # TODO
target_link_libraries(openMVG_stl INTERFACE ${LOG_LIB})

UNIT_TEST(openMVG split "openMVG_stl")
UNIT_TEST(openMVG dynamic_bitset "openMVG_stl")
