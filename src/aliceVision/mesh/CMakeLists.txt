# Headers
set(mesh_files_headers
  geoMesh.hpp
  Material.hpp
  Mesh.hpp
  MeshAnalyze.hpp
  MeshClean.hpp
  MeshEnergyOpt.hpp
  meshPostProcessing.hpp
  meshVisibility.hpp
  Texturing.hpp
  UVAtlas.hpp
)

# Sources
set(mesh_files_sources
  Material.cpp
  Mesh.cpp
  MeshAnalyze.cpp
  MeshClean.cpp
  MeshEnergyOpt.cpp
  meshPostProcessing.cpp
  meshVisibility.cpp
  Texturing.cpp
  UVAtlas.cpp
)

alicevision_add_library(aliceVision_mesh
  SOURCES ${mesh_files_headers} ${mesh_files_sources}
  PUBLIC_LINKS
    aliceVision_mvsData
    aliceVision_mvsUtils
    Geogram::geogram
    assimp::assimp
  PRIVATE_LINKS
    aliceVision_system
    Boost::boost
)

