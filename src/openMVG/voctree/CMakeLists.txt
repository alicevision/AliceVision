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
)

add_library(openMVG_voctree
  ${voctree_headers}
  ${voctree_sources}
)

set_property(TARGET openMVG_voctree
  PROPERTY FOLDER OpenMVG/OpenMVG
)

target_link_libraries(openMVG_voctree
  openMVG_features
  openMVG_sfm
  ${Boost_LIBRARIES}
  ${LOG_LIB}
)

set_target_properties(openMVG_voctree
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

install(TARGETS openMVG_voctree
  DESTINATION lib/
  EXPORT openMVG-targets
)

UNIT_TEST(openMVG voctree      "openMVG_voctree")
UNIT_TEST(openMVG kmeans       "openMVG_voctree")
UNIT_TEST(openMVG voctreebuild "openMVG_voctree")
