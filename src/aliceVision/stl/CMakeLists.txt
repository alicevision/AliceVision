# Headers
set(stl_files_headers
  bitmask.hpp
  DynamicBitset.hpp
  FlatMap.hpp
  FlatSet.hpp
  hash.hpp
  indexedSort.hpp
  stl.hpp
  mapUtils.hpp
)

alicevision_add_interface(aliceVision_stl
  SOURCES ${stl_files_headers}
  LINKS Boost::container
)

# Unit tests
alicevision_add_test(dynamicBitset_test.cpp NAME "stl_dynamicBitset" LINKS aliceVision_stl)
