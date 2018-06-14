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

add_library(aliceVision_stl INTERFACE)

# target_sources(aliceVision_stl INTERFACE ${stl_files_headers}) # TODO

target_link_libraries(aliceVision_stl
  INTERFACE
    ${Boost_CONTAINER_LIBRARIES}
)

install(TARGETS aliceVision_stl 
  EXPORT aliceVision-targets 
)

UNIT_TEST(aliceVision dynamicBitset "aliceVision_stl")

add_custom_target(aliceVision_stl_ide SOURCES ${stl_files_headers})

set_property(TARGET aliceVision_stl_ide
  PROPERTY FOLDER AliceVision
)