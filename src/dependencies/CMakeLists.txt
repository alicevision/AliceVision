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

##
# Image I/O
##
if(NOT JPEG_FOUND)
  set(ALICEVISION_USE_INTERNAL_JPEG ON PARENT_SCOPE)
  set(JPEG_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/aliceVision/dependencies/jpeg)
  add_subdirectory(jpeg)
  list(APPEND JPEG_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/jpeg ${CMAKE_CURRENT_BINARY_DIR}/jpeg/config)
  set(JPEG_INCLUDE_DIR ${JPEG_INCLUDE_DIRECTORIES})
  set(JPEG_LIBRARIES jpeg PARENT_SCOPE)
  set(JPEG_INCLUDE_DIR ${JPEG_INCLUDE_DIR} PARENT_SCOPE)
endif()

# TIFF and PNG depend on zlib, if one of them is not found add the internal zlib
if(NOT PNG_FOUND OR NOT TIFF_FOUND)
  add_subdirectory(zlib)
endif()

if(NOT PNG_FOUND)
  set(ALICEVISION_USE_INTERNAL_PNG ON PARENT_SCOPE)
  set(ZLIB_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/aliceVision/dependencies/zlib)
  set(PNG_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/aliceVision/dependencies/png)
  add_subdirectory(png)
  set(PNG_LIBRARIES png zlib)
  list(APPEND PNG_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/png ${CMAKE_CURRENT_SOURCE_DIR}/zlib ${CMAKE_CURRENT_BINARY_DIR}/png/config)
  set(PNG_INCLUDE_DIRS ${PNG_INCLUDE_DIRECTORIES})
  set(PNG_LIBRARIES ${PNG_LIBRARIES} PARENT_SCOPE)
  set(PNG_INCLUDE_DIRS ${PNG_INCLUDE_DIRS} PARENT_SCOPE)
endif()

if(NOT TIFF_FOUND)
  set(ALICEVISION_USE_INTERNAL_TIFF ON PARENT_SCOPE)
  set(TIFF_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/aliceVision/dependencies/tiff)
  add_subdirectory(tiff)
  list(APPEND TIFF_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/tiff ${CMAKE_CURRENT_BINARY_DIR}/tiff)
  set(TIFF_INCLUDE_DIR ${TIFF_INCLUDE_DIRECTORIES})
  set(TIFF_LIBRARIES tiff)
  set(TIFF_LIBRARIES ${TIFF_LIBRARIES} PARENT_SCOPE)
  set(TIFF_INCLUDE_DIR ${TIFF_INCLUDE_DIR} PARENT_SCOPE)
endif()
##
# End - Image I/O
##

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
