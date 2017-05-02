
set(sfm_files_header
  AlembicExporter.hpp
  AlembicImporter.hpp
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
  sfm_view_metadata.hpp
  utils/alignment.hpp
  utils/sfm_data_UID_utils.hpp
)
set(sfm_files_cpp
  AlembicExporter.cpp
  AlembicImporter.cpp
  pipelines/global/GlobalSfM_rotation_averaging.cpp
  pipelines/global/GlobalSfM_translation_averaging.cpp
  pipelines/global/sfm_global_engine_relative_motions.cpp
  pipelines/localization/SfM_Localizer.cpp
  pipelines/localization/SfM_Localizer_Single_3DTrackObservation_Database.cpp
  pipelines/sequential/sequential_SfM.cpp
  pipelines/RegionsIO.cpp
  pipelines/sfm_robust_model_estimation.cpp
  pipelines/structure_from_known_poses/structure_estimator.cpp
  sfm_data.cpp
  sfm_data_BA_ceres.cpp
  #sfm_data_BA_test.cpp
  sfm_data_filters_frustum.cpp
  sfm_data_io.cpp
  #sfm_data_io_test.cpp
  sfm_data_triangulation.cpp
  sfm_data_utils.cpp
  #sfm_data_utils_test.cpp
  utils/alignment.cpp
  utils/sfm_data_UID_utils.cpp
  sfm_data_io_gt.cpp
)


add_library(openMVG_sfm ${sfm_files_header} ${sfm_files_cpp})
target_link_libraries(
  openMVG_sfm
  openMVG_multiview
  stlplus
  ${CERES_LIBRARIES}
  openMVG_lInftyComputerVision
  openMVG_system
  openMVG_matching
  openMVG_features
  openMVG_exif
  ${LOG_LIB}
)
set_target_properties(openMVG_sfm PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR} VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}")

if(OpenMVG_USE_ALEMBIC)
  target_link_libraries( openMVG_sfm ${ABC_LIBRARIES})
endif(OpenMVG_USE_ALEMBIC)

install(TARGETS openMVG_sfm DESTINATION lib EXPORT openMVG-targets)

UNIT_TEST(openMVG sfm_data_io
  "openMVG_features;openMVG_sfm;openMVG_system;stlplus")
UNIT_TEST(openMVG sfm_data_BA
  "openMVG_multiview_test_data;openMVG_features;openMVG_multiview;openMVG_sfm;openMVG_system;stlplus")
UNIT_TEST(openMVG sfm_data_utils
  "openMVG_features;openMVG_multiview;openMVG_system;openMVG_sfm;stlplus")

if(OpenMVG_USE_ALEMBIC)
  UNIT_TEST(openMVG AlembicImporter "openMVG_sfm;${ABC_LIBRARIES}")
endif(OpenMVG_USE_ALEMBIC)

add_subdirectory(pipelines)

