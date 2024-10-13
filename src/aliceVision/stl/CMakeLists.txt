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
    set(UseSWIG_TARGET_NAME_PREFERENCE STANDARD)
    set_property(SOURCE Stl.i PROPERTY CPLUSPLUS ON)
    set_property(SOURCE Stl.i PROPERTY SWIG_MODULE_NAME stl)

    swig_add_library(stl
        TYPE MODULE
        LANGUAGE python
        SOURCES Stl.i
    )

    if (CMAKE_SYSTEM_NAME MATCHES "Linux")
        list(APPEND SWIG_EXTRA_COMPILE_OPTIONS "-DSWIGWORDSIZE64")
    endif()

    set_property(
        TARGET stl
        PROPERTY SWIG_COMPILE_OPTIONS -doxygen ${SWIG_EXTRA_COMPILE_OPTIONS}
    )

    target_include_directories(stl
    PRIVATE
        ../include
        ${ALICEVISION_ROOT}/include
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
    )
    set_property(
        TARGET stl
        PROPERTY SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON
    )
    set_property(
        TARGET stl
        PROPERTY COMPILE_OPTIONS -std=c++17
    )

    target_link_libraries(stl
    PUBLIC
        aliceVision_stl
        aliceVision_system
        Boost::container
        ${Python3_LIBRARIES}
    )

    install(
    TARGETS
        stl
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
    install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/stl.py
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
endif()