# Headers
set(voctree_headers
  Database.hpp
  databaseIO.hpp
  descriptorLoader.hpp
  descriptorLoader.tcc
  distance.hpp
  DefaultAllocator.hpp
  MutableVocabularyTree.hpp
  SimpleKmeans.hpp
  TreeBuilder.hpp
  VocabularyTree.hpp
)

# Sources
set(voctree_sources
  Database.cpp
  descriptorLoader.cpp
  VocabularyTree.cpp
)

add_library(aliceVision_voctree
  ${voctree_headers}
  ${voctree_sources}
)

set_property(TARGET aliceVision_voctree
  PROPERTY FOLDER AliceVision
)

target_link_libraries(aliceVision_voctree
  PUBLIC
    aliceVision_feature
    aliceVision_sfm
    aliceVision_system
    Eigen3::Eigen
  PRIVATE
    ${Boost_FILESYSTEM_LIBRARY}
)

set_target_properties(aliceVision_voctree
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

install(TARGETS aliceVision_voctree
  DESTINATION lib/
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision kmeans               "aliceVision_voctree")
UNIT_TEST(aliceVision vocabularyTree       "aliceVision_voctree")
UNIT_TEST(aliceVision vocabularyTreeBuild  "aliceVision_voctree")
