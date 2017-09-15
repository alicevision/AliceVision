# Headers
set(voctree_headers
  database.hpp
  databaseIO.hpp
  descriptor_loader.hpp
  descriptor_loader.tcc
  distance.hpp
  feature_allocator.hpp
  mutable_tree.hpp
  simple_kmeans.hpp
  tree_builder.hpp
  vocabulary_tree.hpp
)

# Sources
set(voctree_sources
  database.cpp
  descriptor_loader.cpp
  vocabulary_tree.cpp
)

add_library(aliceVision_voctree
  ${voctree_headers}
  ${voctree_sources}
)

set_property(TARGET aliceVision_voctree
  PROPERTY FOLDER AliceVision/AliceVision
)

target_link_libraries(aliceVision_voctree
  aliceVision_feature
  aliceVision_sfm
  ${Boost_LIBRARIES}
  ${LOG_LIB}
)

set_target_properties(aliceVision_voctree
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

install(TARGETS aliceVision_voctree
  DESTINATION lib/
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision voctree      "aliceVision_voctree")
UNIT_TEST(aliceVision kmeans       "aliceVision_voctree")
UNIT_TEST(aliceVision voctreebuild "aliceVision_voctree")
