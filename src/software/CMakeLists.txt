
# Runtime library paths
SET(LIB_RUNTIME_PATHS ${Boost_LIBRARY_DIRS})

# "Escape" the list of library paths using $<SEMICOLON> to use it as a custom command argument
string (REPLACE ";" "$<SEMICOLON>" LIB_RUNTIME_PATHS_ARG "${LIB_RUNTIME_PATHS}")



set(SOURCES_files_CMPMVS CMPMVS.cpp)
source_group("CMPMVS" FILES CMPMVS.cpp)

add_executable(CMPMVS ${SOURCES_files_CMPMVS})
target_link_libraries(CMPMVS PUBLIC mv_structures
                                    aliceVision_common
                                    mv_cudaInterfaces
                                    mv_mesh mv_largeScale
                                    mv_delaunaycut
                                    mv_cudaInterfaces mv_prematching
                                    eig33
                                    ${Boost_LIBRARIES} ${ZLIB_LIBRARIES}
                                    ${GMP_LIBRARIES} ${MPFR_LIBRARIES})

## Install rule
install(TARGETS CMPMVS DESTINATION bin/)

## Bundle rule
# Custom target that calls fixup_bundle to copy needed runtime libraries
# next to the target executable

# Full path to executable
set(CMPMVS_EXE ${CMAKE_INSTALL_PREFIX}/bin/CMPMVS${CMAKE_EXECUTABLE_SUFFIX})

# Bundle target (see MakeBundle.cmake)
# Note: require that the install rule has been executed
add_custom_target("bundle" 
                  ${CMAKE_COMMAND}
                  -DEXECUTABLE=${CMPMVS_EXE} -DLPATHS=${LIB_RUNTIME_PATHS_ARG} 
                  -P ${CMAKE_CURRENT_SOURCE_DIR}/MakeBundle.cmake)
set_target_properties("bundle" PROPERTIES FOLDER Packaging)




set(SOURCES_files_exportDepthMap main_exportDepthMap.cpp)
source_group("aliceVision_exportDepthMap" FILES main_exportDepthMap.cpp)

add_executable(aliceVision_exportDepthMap ${SOURCES_files_exportDepthMap})
target_link_libraries(aliceVision_exportDepthMap PUBLIC mv_structures
                                    aliceVision_common
                                    mv_cudaInterfaces
                                    mv_mesh mv_largeScale
                                    mv_delaunaycut
                                    mv_cudaInterfaces mv_prematching
                                    eig33
                                    ${Boost_LIBRARIES} ${ZLIB_LIBRARIES}
                                    ${GMP_LIBRARIES} ${MPFR_LIBRARIES})

## Install rule
install(TARGETS aliceVision_exportDepthMap DESTINATION bin/)


## Compute Nearby Cameras
set(SOURCES_files_cameraConnection main_cameraConnection.cpp)
source_group("aliceVision_cameraConnection" FILES main_cameraConnection.cpp)

add_executable(aliceVision_cameraConnection ${SOURCES_files_cameraConnection})
target_link_libraries(aliceVision_cameraConnection PUBLIC mv_structures
                                    aliceVision_common
                                    mv_cudaInterfaces
                                    mv_mesh mv_largeScale
                                    mv_delaunaycut
                                    mv_cudaInterfaces mv_prematching
                                    eig33
                                    ${Boost_LIBRARIES} ${ZLIB_LIBRARIES}
                                    ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
install(TARGETS aliceVision_cameraConnection DESTINATION bin/)


## Depth Map Estimation
set(SOURCES_files_depthMapEstimation main_depthMapEstimation.cpp)
source_group("aliceVision_depthMapEstimation" FILES main_depthMapEstimation.cpp)

add_executable(aliceVision_depthMapEstimation ${SOURCES_files_depthMapEstimation})
target_link_libraries(aliceVision_depthMapEstimation PUBLIC mv_structures
                                    aliceVision_common
                                    mv_cudaInterfaces
                                    mv_mesh mv_largeScale
                                    mv_delaunaycut
                                    mv_cudaInterfaces mv_prematching
                                    eig33
                                    ${Boost_LIBRARIES} ${ZLIB_LIBRARIES}
                                    ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
install(TARGETS aliceVision_depthMapEstimation DESTINATION bin/)

## Depth Map Filtering
set(SOURCES_files_depthMapFiltering main_depthMapFiltering.cpp)
source_group("aliceVision_depthMapFiltering" FILES main_depthMapFiltering.cpp)

add_executable(aliceVision_depthMapFiltering ${SOURCES_files_depthMapFiltering})
target_link_libraries(aliceVision_depthMapFiltering PUBLIC mv_structures
                                    aliceVision_common
                                    mv_cudaInterfaces
                                    mv_mesh mv_largeScale
                                    mv_delaunaycut
                                    mv_cudaInterfaces mv_prematching
                                    eig33
                                    ${Boost_LIBRARIES} ${ZLIB_LIBRARIES}
                                    ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
install(TARGETS aliceVision_depthMapFiltering DESTINATION bin/)


## Meshing
set(SOURCES_files_meshing main_meshing.cpp)
source_group("aliceVision_meshing" FILES main_meshing.cpp)

add_executable(aliceVision_meshing ${SOURCES_files_meshing})
target_link_libraries(aliceVision_meshing PUBLIC mv_structures
                                    aliceVision_common
                                    mv_cudaInterfaces
                                    mv_mesh mv_largeScale
                                    mv_delaunaycut
                                    mv_cudaInterfaces mv_prematching
                                    eig33
                                    ${Boost_LIBRARIES} ${ZLIB_LIBRARIES}
                                    ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
install(TARGETS aliceVision_meshing DESTINATION bin/)


## Mesh Resampling
set(SOURCES_files_meshResampling main_meshResampling.cpp)
source_group("aliceVision_meshResampling" FILES main_meshResampling.cpp)

add_executable(aliceVision_meshResampling ${SOURCES_files_meshResampling})
target_link_libraries(aliceVision_meshResampling PUBLIC mv_structures
                                    aliceVision_common
                                    ${Boost_LIBRARIES}
                                    Geogram::geogram)
install(TARGETS aliceVision_meshResampling DESTINATION bin/)


## Texturing
set(SOURCES_files_texturing main_texturing.cpp)
source_group("aliceVision_texturing" FILES main_texturing.cpp)

add_executable(aliceVision_texturing ${SOURCES_files_texturing})
target_link_libraries(aliceVision_texturing PUBLIC mv_structures
                                    aliceVision_common
                                    mv_cudaInterfaces
                                    mv_mesh mv_largeScale
                                    mv_delaunaycut
                                    mv_cudaInterfaces mv_prematching
                                    eig33
                                    ${Boost_LIBRARIES} ${ZLIB_LIBRARIES}
                                    ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
install(TARGETS aliceVision_texturing DESTINATION bin/)
