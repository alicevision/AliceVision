# Headers
set(sfm_files_headers
  pipelines/global/GlobalSfM_rotation_averaging.hpp
  pipelines/global/GlobalSfM_translation_averaging.hpp
  pipelines/global/mutexSet.hpp
  pipelines/global/sfm_global_engine_relative_motions.hpp
  pipelines/global/sfm_global_reindex.hpp
  pipelines/global/triplet_t_ACRansac_kernelAdaptator.hpp
  pipelines/localization/SfM_Localizer.hpp
  pipelines/localization/SfM_Localizer_Single_3DTrackObservation_Database.hpp
  pipelines/sequential/sequential_SfM.hpp
  pipelines/sfm_engine.hpp
  pipelines/sfm_matches_provider.hpp
  pipelines/sfm_robust_model_estimation.hpp
  pipelines/structure_from_known_poses/structure_estimator.hpp
  pipelines/RegionsIO.hpp
  sfm.hpp
  sfm_data.hpp
  sfm_data_BA.hpp
  sfm_data_BA_ceres.hpp
  sfm_data_BA_ceres_camera_functor.hpp
  sfm_data_filters.hpp
  sfm_data_filters_frustum.hpp
  sfm_data_io.hpp
  sfm_data_io_baf.hpp
  sfm_data_io_cereal.hpp
  sfm_data_io_gt.hpp
  sfm_data_io_ply.hpp
  sfm_data_triangulation.hpp
  sfm_data_utils.hpp
  sfm_filters.hpp
  sfm_landmark.hpp
  sfm_report.hpp
  sfm_view.hpp
  Rig.hpp
  utils/alignment.hpp
  utils/sfm_data_UID_utils.hpp
)

# Sources
set(sfm_files_sources
  pipelines/global/GlobalSfM_rotation_averaging.cpp
  pipelines/global/GlobalSfM_translation_averaging.cpp
  pipelines/global/sfm_global_engine_relative_motions.cpp
  pipelines/localization/SfM_Localizer.cpp
  pipelines/localization/SfM_Localizer_Single_3DTrackObservation_Database.cpp
  pipelines/sequential/sequential_SfM.cpp
  pipelines/sfm_robust_model_estimation.cpp
  pipelines/structure_from_known_poses/structure_estimator.cpp
  pipelines/RegionsIO.cpp
  sfm_data.cpp
  sfm_data_BA_ceres.cpp
  sfm_data_filters_frustum.cpp
  sfm_data_io.cpp
  sfm_data_io_gt.cpp
  sfm_data_triangulation.cpp
  sfm_data_utils.cpp
  utils/alignment.cpp
  utils/sfm_data_UID_utils.cpp
)

if(ALICEVISION_HAVE_ALEMBIC)
  list(APPEND sfm_files_headers
    AlembicExporter.hpp
    AlembicImporter.hpp
  )
  list(APPEND sfm_files_sources
    AlembicExporter.cpp
    AlembicImporter.cpp
  )
endif()

add_library(aliceVision_sfm
  ${sfm_files_headers}
  ${sfm_files_sources}
)

target_link_libraries(aliceVision_sfm
  aliceVision_multiview
  stlplus
  ${CERES_LIBRARIES}
  aliceVision_lInftyComputerVision
  aliceVision_system
  aliceVision_matching
  aliceVision_feature
  aliceVision_exif
  ${LOG_LIB}
)

if(Boost_FOUND)
  target_link_libraries(aliceVision_sfm
    ${Boost_LIBRARIES}
  )
endif()

set_target_properties(aliceVision_sfm
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

if(ALICEVISION_HAVE_ALEMBIC)
  target_link_libraries(aliceVision_sfm
    ${ABC_LIBRARIES}
  )
endif()

install(TARGETS aliceVision_sfm
  DESTINATION lib
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision sfm_data_io    "aliceVision_feature;aliceVision_sfm;aliceVision_system;stlplus")
UNIT_TEST(aliceVision sfm_data_BA    "aliceVision_multiview_test_data;aliceVision_feature;aliceVision_multiview;aliceVision_sfm;aliceVision_system;stlplus")
UNIT_TEST(aliceVision sfm_data_utils "aliceVision_feature;aliceVision_multiview;aliceVision_system;aliceVision_sfm;stlplus")
UNIT_TEST(aliceVision Rig            "aliceVision_feature;aliceVision_sfm;aliceVision_system;stlplus")

if(ALICEVISION_HAVE_ALEMBIC)
  UNIT_TEST(aliceVision AlembicImporter "aliceVision_sfm;${ABC_LIBRARIES}")
endif()

add_subdirectory(pipelines)

