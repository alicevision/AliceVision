# Headers
set(numeric_files_headers
  numeric.hpp
  Container.hpp
  projection.hpp
  gps.hpp
  algebra.hpp
  BoxStats.hpp
)

# Sources
set(numeric_files_sources
  numeric.cpp
  Container.cpp
  projection.cpp
  gps.cpp
)

alicevision_add_library(aliceVision_numeric
  SOURCES ${numeric_files_headers} ${numeric_files_sources}
  PUBLIC_LINKS
    Eigen3::Eigen
    Boost::boost
    Boost::log
    Boost::log_setup
  PUBLIC_DEFINITIONS
    ${AV_EIGEN_DEFINITIONS}
)

# Unit tests
alicevision_add_test(numeric_test.cpp    NAME "numeric"            LINKS aliceVision_numeric)
alicevision_add_test(polynomial_test.cpp NAME "numeric_polynomial" LINKS aliceVision_numeric)
alicevision_add_test(lmFunctor_test.cpp  NAME "numeric_lmFunctor"  LINKS aliceVision_numeric)
alicevision_add_test(projection_test.cpp NAME "numeric_projection" LINKS aliceVision_numeric)
alicevision_add_test(gps_test.cpp NAME "numeric_gps" LINKS aliceVision_numeric)


# SWIG Binding
if (ALICEVISION_BUILD_SWIG_BINDING)
    set(UseSWIG_TARGET_NAME_PREFERENCE STANDARD)
    set_property(SOURCE numeric.i PROPERTY CPLUSPLUS ON)
    set_property(SOURCE numeric.i PROPERTY SWIG_MODULE_NAME numeric)

    swig_add_library(numeric
        TYPE MODULE
        LANGUAGE python
        SOURCES numeric.i
    )

    set_property(
        TARGET numeric
        PROPERTY SWIG_COMPILE_OPTIONS -doxygen
    )

    target_include_directories(numeric
    PRIVATE
        ../include
        ${ALICEVISION_ROOT}/include
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
    )
    set_property(
        TARGET numeric
        PROPERTY SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON
    )
    set_property(
        TARGET numeric
        PROPERTY COMPILE_OPTIONS -std=c++20
    )

    target_link_libraries(numeric
    PUBLIC
        aliceVision_numeric
    )

    install(
    TARGETS
    numeric
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
    install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/numeric.py
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
endif()