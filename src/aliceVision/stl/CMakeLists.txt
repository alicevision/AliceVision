# Headers
set(stl_files_headers
    bitmask.hpp
    DynamicBitset.hpp
    FlatMap.hpp
    hash.hpp
    indexedSort.hpp
    stl.hpp
    mapUtils.hpp
)

alicevision_add_interface(aliceVision_stl
    SOURCES ${stl_files_headers}
    LINKS Boost::container
)

# Unit tests
alicevision_add_test(dynamicBitset_test.cpp NAME "stl_dynamicBitset" LINKS aliceVision_stl)

# SWIG Binding
if (ALICEVISION_BUILD_SWIG_BINDING)
    alicevision_swig_add_library(stl
        SOURCES Stl.i
        PUBLIC_LINKS
            aliceVision_stl
            aliceVision_system
            Boost::container
            ${Python3_LIBRARIES}
        PRIVATE_INCLUDE_DIRS
            ../include
            ${ALICEVISION_ROOT}/include
            ${Python3_INCLUDE_DIRS}
            ${Python3_NumPy_INCLUDE_DIRS}
    )
endif()
