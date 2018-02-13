# Headers
set(mesh_files_headers
  Mesh.hpp
  MeshAnalyze.hpp
  MeshClean.hpp
  MeshEnergyOpt.hpp
  MeshEnergyOptPhotoMem.hpp
  meshVisibility.hpp
  Retexturer.hpp
  UVAtlas.hpp
  plyLoader.hpp
  plySaver.hpp
  rply.h
)

# Sources
set(mesh_files_sources
  Mesh.cpp
  MeshAnalyze.cpp
  MeshClean.cpp
  MeshEnergyOpt.cpp
  MeshEnergyOptPhotoMem.cpp
  meshVisibility.cpp
  Retexturer.cpp
  UVAtlas.cpp
  plyLoader.cpp
  plySaver.cpp
  rply.c
)

add_library(aliceVision_mesh
  ${mesh_files_headers}
  ${mesh_files_sources}
)

target_include_directories(aliceVision_mesh
  PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
)

target_link_libraries(aliceVision_mesh
  PUBLIC aliceVision_structures
         aliceVision_common
         aliceVision_imageIO
         aliceVision_depthMap
         Geogram::geogram
         ${Boost_FILESYSTEM_LIBRARIES}
)

set_property(TARGET aliceVision_mesh
  PROPERTY FOLDER AliceVision
)
