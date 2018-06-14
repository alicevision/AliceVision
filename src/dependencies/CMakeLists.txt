## AliceVision dependencies

# Add nanoflann (header-only)
add_library(nanoflann INTERFACE IMPORTED GLOBAL)
set(NANOFLANN_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/nanoflann/include")
set_target_properties(nanoflann PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${NANOFLANN_INCLUDE_DIR}")

# libs should be static
set(BUILD_SHARED_LIBS OFF)

# use PIC code for link into shared lib
if(UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif(UNIX)

# Add graph library
if(DEFINED ALICEVISION_USE_INTERNAL_LEMON)
  add_subdirectory(lemon)
  set_property(TARGET lemon
    PROPERTY FOLDER Dependencies/Lemon
  )
  set_property(TARGET check
    PROPERTY FOLDER Dependencies/Lemon
  )
endif()

# Add svg library
add_subdirectory(vectorGraphics)

# Add an Approximate Nearest Neighbor library
if(DEFINED ALICEVISION_USE_INTERNAL_FLANN)
  set(FLANN_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/aliceVision/dependencies/flann/src/cpp)
  add_subdirectory(flann)
  set_property(TARGET flann_cpp_s
    PROPERTY FOLDER Dependencies/Flann
  )
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
