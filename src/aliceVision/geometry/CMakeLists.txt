# Headers
set(geometry_files_headers
    Frustum.hpp
    HalfPlane.hpp
    lie.hpp
    Pose3.hpp
    rigidTransformation3D.hpp
    Similarity3.hpp
    Intersection.hpp
)

# Sources
set(geometry_files_sources
    rigidTransformation3D.cpp
    Intersection.cpp
)

alicevision_add_library(aliceVision_geometry
  SOURCES ${geometry_files_headers} ${geometry_files_sources}
  PUBLIC_LINKS
    aliceVision_numeric
    aliceVision_linearProgramming
  # PRIVATE_LINKS
  # TODO: fix cross-dependency
  # aliceVision_robustEstimation
)

# Unit tests
alicevision_add_test(rigidTransformation3D_test.cpp NAME "geometry_rigidTransformation3D" LINKS aliceVision_geometry)
alicevision_add_test(halfSpaceIntersection_test.cpp NAME "geometry_halfSpaceIntersection" LINKS aliceVision_geometry)
alicevision_add_test(Pose3d_test.cpp NAME "geometry_pose3D" LINKS aliceVision_geometry)
alicevision_add_test(frustumIntersection_test.cpp
  NAME "geometry_frustumIntersection"
  LINKS aliceVision_geometry
        aliceVision_multiview
        aliceVision_multiview_test_data
)

# SWIG Binding
if (ALICEVISION_BUILD_SWIG_BINDING)
    set(UseSWIG_TARGET_NAME_PREFERENCE STANDARD)
    set_property(SOURCE Geometry.i PROPERTY CPLUSPLUS ON)
    set_property(SOURCE Geometry.i PROPERTY SWIG_MODULE_NAME geometry)

    swig_add_library(geometry
        TYPE MODULE
        LANGUAGE python
        SOURCES Geometry.i
    )

    if (CMAKE_SYSTEM_NAME MATCHES "Linux")
        list(APPEND SWIG_EXTRA_COMPILE_OPTIONS "-DSWIGWORDSIZE64")
    endif()

    set_property(
        TARGET geometry
        PROPERTY SWIG_COMPILE_OPTIONS -doxygen ${SWIG_EXTRA_COMPILE_OPTIONS}
    )

    target_include_directories(geometry
    PRIVATE
        ../include
        ${ALICEVISION_ROOT}/include
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
    )
    set_property(
        TARGET geometry
        PROPERTY SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON
    )
    set_property(
        TARGET geometry
        PROPERTY COMPILE_OPTIONS -std=c++17
    )

    target_link_libraries(geometry
    PUBLIC
        aliceVision_geometry
        ${Python3_LIBRARIES}
    )

    install(
    TARGETS
        geometry
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
    install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/geometry.py
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
endif()