## AliceVision
## Pipeline software

# CMPMVS all-in-one software

add_executable(aliceVision_CMPMVS CMPMVS.cpp)

target_link_libraries(aliceVision_CMPMVS
  PUBLIC aliceVision_structures
         aliceVision_common
         aliceVision_mesh
         aliceVision_largeScale
         aliceVision_delaunayCut
         ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_CMPMVS
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_CMPMVS
  DESTINATION bin/
)

# Compute Nearby Cameras

add_executable(aliceVision_cameraConnection main_cameraConnection.cpp)

target_link_libraries(aliceVision_cameraConnection
  PUBLIC aliceVision_structures
         aliceVision_common
         aliceVision_mesh
         aliceVision_largeScale
         aliceVision_delaunayCut
         ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_cameraConnection
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_cameraConnection
  DESTINATION bin/
)

# Depth Map Estimation

add_executable(aliceVision_depthMapEstimation main_depthMapEstimation.cpp)

target_link_libraries(aliceVision_depthMapEstimation
  PUBLIC aliceVision_structures
         aliceVision_common
         aliceVision_mesh
         aliceVision_largeScale
         aliceVision_delaunayCut
         ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_depthMapEstimation
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_depthMapEstimation
  DESTINATION bin/
)

# Depth Map Filtering

add_executable(aliceVision_depthMapFiltering main_depthMapFiltering.cpp)

target_link_libraries(aliceVision_depthMapFiltering
  PUBLIC aliceVision_structures
         aliceVision_common
         aliceVision_mesh
         aliceVision_largeScale
         aliceVision_delaunayCut
         ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_depthMapFiltering
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_depthMapFiltering
  DESTINATION bin/
)

# Meshing

add_executable(aliceVision_meshing main_meshing.cpp)

target_link_libraries(aliceVision_meshing
  PUBLIC aliceVision_structures
         aliceVision_common
         aliceVision_mesh
         aliceVision_largeScale
         aliceVision_delaunayCut
         ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_meshing
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_meshing
  DESTINATION bin/
)

# Mesh Resampling

add_executable(aliceVision_meshResampling main_meshResampling.cpp)

target_link_libraries(aliceVision_meshResampling
  PUBLIC aliceVision_structures
         aliceVision_common
         Geogram::geogram
         ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_meshResampling
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_meshResampling
  DESTINATION bin/
)

# Texturing

add_executable(aliceVision_texturing main_texturing.cpp)

target_link_libraries(aliceVision_texturing
  PUBLIC aliceVision_structures
         aliceVision_common
         aliceVision_mesh
         aliceVision_largeScale
         aliceVision_delaunayCut
         ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_texturing
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_texturing
  DESTINATION bin/
)
