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


# Add svg library
add_subdirectory(vectorGraphics)



list(APPEND directories histogram htmlDoc vectorGraphics)
foreach(inDirectory ${directories})
  install(
    DIRECTORY ./${inDirectory}
    DESTINATION include/aliceVision/dependencies/
    COMPONENT headers
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
  )
endforeach()
