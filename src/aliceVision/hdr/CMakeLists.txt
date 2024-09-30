
# Headers
set(hdr_files_headers
 LaguerreBACalibration.hpp
 sampling.hpp
 rgbCurve.hpp
 hdrMerge.hpp
 DebevecCalibrate.hpp
 GrossbergCalibrate.hpp
 emorCurve.hpp
 QuadProg++.hpp
 brackets.hpp
)

# Sources
set(hdr_files_sources
 LaguerreBACalibration.cpp
 sampling.cpp
 rgbCurve.cpp
 hdrMerge.cpp
 DebevecCalibrate.cpp
 GrossbergCalibrate.cpp
 emorCurve.cpp
 emorCurveInv.cpp
 QuadProg++.cpp
 brackets.cpp
)

alicevision_add_library(aliceVision_hdr
  SOURCES ${hdr_files_headers} ${hdr_files_sources}
  PRIVATE_LINKS
    aliceVision_system
    aliceVision_image
    aliceVision_sfmData
    ${CERES_LIBRARIES}
)

# Unit tests
alicevision_add_test(hdrDebevec_test.cpp
    NAME "hdr_debevec"
    LINKS aliceVision_image aliceVision_hdr)

alicevision_add_test(hdrGrossberg_test.cpp
    NAME "hdr_grossberg"
    LINKS aliceVision_image aliceVision_hdr)

alicevision_add_test(hdrLaguerre_test.cpp
    NAME "hdr_laguerre"
    LINKS aliceVision_image aliceVision_hdr)


# SWIG Binding
if (ALICEVISION_BUILD_SWIG_BINDING)
    set(UseSWIG_TARGET_NAME_PREFERENCE STANDARD)
    set_property(SOURCE Hdr.i PROPERTY CPLUSPLUS ON)
    set_property(SOURCE Hdr.i PROPERTY SWIG_MODULE_NAME hdr)

    swig_add_library(hdr
        TYPE MODULE
        LANGUAGE python
        SOURCES Hdr.i
    )

    if (CMAKE_SYSTEM_NAME MATCHES "Linux")
        list(APPEND SWIG_EXTRA_COMPILE_OPTIONS "-DSWIGWORDSIZE64")
    endif()

    set_property(
        TARGET hdr
        PROPERTY SWIG_COMPILE_OPTIONS -doxygen ${SWIG_EXTRA_COMPILE_OPTIONS}
    )

    target_include_directories(hdr
    PRIVATE
        ../include
        ${ALICEVISION_ROOT}/include
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
    )
    set_property(
        TARGET hdr
        PROPERTY SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON
    )
    set_property(
        TARGET hdr
        PROPERTY COMPILE_OPTIONS -std=c++17
    )

    target_link_libraries(hdr
    PUBLIC
        aliceVision_hdr
        aliceVision_system
        aliceVision_numeric
        aliceVision_image
        aliceVision_sfmData
        ${Python3_LIBRARIES}
    )

    install(
    TARGETS
        hdr
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
    install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/hdr.py
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
endif()