# Headers
set(mesh_files_headers
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
  Mesh.cpp
  MeshAnalyze.cpp
  MeshClean.cpp
  MeshEnergyOpt.cpp
  meshPostProcessing.cpp
  meshVisibility.cpp
  Texturing.cpp
  UVAtlas.cpp
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
  PUBLIC 
    aliceVision_mvsData
    aliceVision_mvsUtils
    aliceVision_imageIO
    Geogram::geogram
    ${Boost_FILESYSTEM_LIBRARY}
  PRIVATE 
    aliceVision_system
)

set_property(TARGET aliceVision_mesh
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_mesh
  DESTINATION lib
  EXPORT aliceVision-targets
)
