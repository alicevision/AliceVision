## AliceVision dependencies

# libs should be static
set(BUILD_SHARED_LIBS OFF)

# use PIC code for link into shared lib
if(UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif(UNIX)

# Basic filesystem utils
set(STLPLUS_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/aliceVision/dependencies/stlplus3)
add_subdirectory(stlplus3)

# Add graph library
if(DEFINED ALICEVISION_USE_INTERNAL_LEMON)
  add_subdirectory(lemon)
  set_property(TARGET lemon
    PROPERTY FOLDER AliceVision/Dependencies/Lemon
  )
  set_property(TARGET check
    PROPERTY FOLDER AliceVision/Dependencies/Lemon
  )
endif()

# Add svg library
add_subdirectory(vectorGraphics)

# Add ceres-solver (A Nonlinear Least Squares Minimizer)
if(ALICEVISION_USE_INTERNAL_CERES)
  set(CXSPARSE_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/aliceVision/dependencies/cxsparse)
  add_subdirectory(cxsparse)
  set(CERES_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/aliceVision/dependencies/ceres)
  add_subdirectory(ceres-solver)
  set_property(TARGET cxsparse
    PROPERTY FOLDER AliceVision/Dependencies/Ceres
  )
  set_property(TARGET ceres
    PROPERTY FOLDER AliceVision/Dependencies/Ceres
  )
endif()

# Add an Approximate Nearest Neighbor library
if(DEFINED ALICEVISION_USE_INTERNAL_FLANN)
  set(FLANN_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/aliceVision/dependencies/flann/src/cpp)
  add_subdirectory(flann)
  set_property(TARGET flann_cpp_s
    PROPERTY FOLDER AliceVision/Dependencies/Flann
  )
endif()

##
# Install Header only libraries if necessary
## 
if(DEFINED ALICEVISION_USE_INTERNAL_EIGEN)
  # Configure Eigen install
  set(EIGEN_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/aliceVision/dependencies/eigen)
  add_subdirectory(eigen)
endif()

list(APPEND directories histogram htmlDoc vectorGraphics)
foreach(inDirectory ${directories})
  install(
    DIRECTORY ./${inDirectory}
    DESTINATION include/aliceVision/dependencies/
    COMPONENT headers
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
  )
endforeach()
