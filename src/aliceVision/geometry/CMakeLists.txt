# Headers
set(geometryData_files_headers
    Frustum.hpp
    Pose3.hpp
    rigidTransformation3D.hpp
    Similarity3.hpp
)

# Sources
set(geometryData_files_sources
    rigidTransformation3D.cpp
)

alicevision_add_library(aliceVision_geometryData
  SOURCES ${geometryData_files_headers} ${geometryData_files_sources}
  PUBLIC_LINKS
    aliceVision_numeric
    aliceVision_system
)

alicevision_add_test(rigidTransformation3D_test.cpp NAME "geometry_rigidTransformation3D" LINKS aliceVision_geometryData)

if(ALICEVISION_BUILD_SFM)

    # Headers
    set(geometry_files_headers
        HalfPlane.hpp
    )

    # Sources
    set(geometry_files_sources
      HalfPlane.cpp
    )

    alicevision_add_library(aliceVision_geometry
      SOURCES ${geometry_files_headers} ${geometry_files_sources}
      PUBLIC_LINKS
        aliceVision_geometryData
        aliceVision_linearProgramming
      # PRIVATE_LINKS
      # TODO: fix cross-dependency
      # aliceVision_robustEstimation
    )

    # Unit tests
    alicevision_add_test(halfSpaceIntersection_test.cpp NAME "geometry_halfSpaceIntersection" LINKS aliceVision_geometry)

    alicevision_add_test(frustumIntersection_test.cpp
        NAME "geometry_frustumIntersection"
        LINKS aliceVision_geometry
            aliceVision_multiview
            aliceVision_multiview_test_data
    )
    endif()
