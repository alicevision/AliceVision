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

alicevision_add_library(aliceVision_voctree
  SOURCES ${voctree_headers} ${voctree_sources}
  PUBLIC_LINKS
    aliceVision_feature
    aliceVision_sfmData
    aliceVision_system
    Boost::boost
  PRIVATE_LINKS
    Boost::filesystem
)

# Unit tests
alicevision_add_test(kmeans_test.cpp              NAME "voctree_kmeans"              LINKS aliceVision_voctree)
alicevision_add_test(vocabularyTree_test.cpp      NAME "voctree_vocabularyTree"      LINKS aliceVision_voctree)
alicevision_add_test(vocabularyTreeBuild_test.cpp NAME "voctree_vocabularyTreeBuild" LINKS aliceVision_voctree)
