add_subdirectory(common)
add_subdirectory(imageIO)
add_subdirectory(mesh)
add_subdirectory(meshConstruction)
add_subdirectory(structures)

if(ALICEVISION_HAVE_CUDA)
  add_subdirectory(depthMap)
endif()
