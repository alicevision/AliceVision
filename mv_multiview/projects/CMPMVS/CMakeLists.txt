
# Runtime library paths
SET(LIB_RUNTIME_PATHS ${Boost_LIBRARY_DIRS}
                      ${_OpenCV_LIB_PATH})
# "Escape" the list of library paths using $<SEMICOLON> to use it as a custom command argument
string (REPLACE ";" "$<SEMICOLON>" LIB_RUNTIME_PATHS_ARG "${LIB_RUNTIME_PATHS}")



set(SOURCES_files_CMPMVS CMPMVS.cpp)
source_group("CMPMVS" FILES CMPMVS.cpp)

add_executable(CMPMVS ${SOURCES_files_CMPMVS})
target_link_libraries(CMPMVS PUBLIC mv_structures mv_cudaInterfaces
                                    mv_mesh mv_largeScale
                                    mv_filter mv_delaunaycut
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
target_link_libraries(aliceVision_exportDepthMap PUBLIC mv_structures mv_cudaInterfaces
                                    mv_mesh mv_largeScale
                                    mv_filter mv_delaunaycut
                                    mv_cudaInterfaces mv_prematching
                                    eig33
                                    ${Boost_LIBRARIES} ${ZLIB_LIBRARIES}
                                    ${GMP_LIBRARIES} ${MPFR_LIBRARIES})

## Install rule
install(TARGETS aliceVision_exportDepthMap DESTINATION bin/)




set(SOURCES_files_texturing main_texturing.cpp)
source_group("aliceVision_texturing" FILES main_texturing.cpp)

add_executable(aliceVision_texturing ${SOURCES_files_texturing})
target_link_libraries(aliceVision_texturing PUBLIC mv_structures mv_cudaInterfaces
                                    mv_mesh mv_largeScale
                                    mv_filter mv_delaunaycut
                                    mv_cudaInterfaces mv_prematching
                                    eig33
                                    ${Boost_LIBRARIES} ${ZLIB_LIBRARIES}
                                    ${GMP_LIBRARIES} ${MPFR_LIBRARIES})

## Install rule
install(TARGETS aliceVision_texturing DESTINATION bin/)
