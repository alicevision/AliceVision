# Headers
set(sfm_files_headers
  pipeline/global/GlobalSfMRotationAveragingSolver.hpp
  pipeline/global/GlobalSfMTranslationAveragingSolver.hpp
  pipeline/global/MutexSet.hpp
  pipeline/global/ReconstructionEngine_globalSfM.hpp
  pipeline/global/reindexGlobalSfM.hpp
  pipeline/global/TranslationTripletKernelACRansac.hpp
  pipeline/localization/SfMLocalizer.hpp
  pipeline/localization/SfMLocalizationSingle3DTrackObservationDatabase.hpp
  pipeline/sequential/ReconstructionEngine_sequentialSfM.hpp
  pipeline/ReconstructionEngine.hpp
  pipeline/pairwiseMatchesIO.hpp
  pipeline/RelativePoseInfo.hpp
  pipeline/structureFromKnownPoses/StructureEstimationFromKnownPoses.hpp
  pipeline/regionsIO.hpp
  sfm.hpp
  SfMData.hpp
  BundleAdjustment.hpp
  BundleAdjustmentCeres.hpp
  LocalBundleAdjustmentCeres.hpp
  LocalBundleAdjustmentData.hpp
  ResidualErrorFunctor.hpp
  sfmDataFilters.hpp
  FrustumFilter.hpp
  sfmDataIO.hpp
  sfmDataIO_baf.hpp
  sfmDataIO_gt.hpp
  sfmDataIO_json.hpp
  sfmDataIO_ply.hpp
  sfmDataTriangulation.hpp
  filters.hpp
  Landmark.hpp
  generateReport.hpp
  View.hpp
  viewIO.hpp
  Rig.hpp
  utils/alignment.hpp
  utils/uid.hpp
  utils/statistics.hpp
  utils/syntheticScene.hpp
)

# Sources
set(sfm_files_sources
  pipeline/global/GlobalSfMRotationAveragingSolver.cpp
  pipeline/global/GlobalSfMTranslationAveragingSolver.cpp
  pipeline/global/ReconstructionEngine_globalSfM.cpp
  pipeline/localization/SfMLocalizer.cpp
  pipeline/localization/SfMLocalizationSingle3DTrackObservationDatabase.cpp
  pipeline/sequential/ReconstructionEngine_sequentialSfM.cpp
  pipeline/RelativePoseInfo.cpp
  pipeline/structureFromKnownPoses/StructureEstimationFromKnownPoses.cpp
  pipeline/regionsIO.cpp
  SfMData.cpp
  BundleAdjustmentCeres.cpp
  LocalBundleAdjustmentCeres.cpp
  LocalBundleAdjustmentData.cpp
  sfmDataFilters.cpp
  FrustumFilter.cpp
  sfmDataIO.cpp
  sfmDataIO_baf.cpp
  sfmDataIO_gt.cpp
  sfmDataIO_json.cpp
  sfmDataIO_ply.cpp
  sfmDataTriangulation.cpp
  generateReport.cpp
  viewIO.cpp
  utils/alignment.cpp
  utils/uid.cpp
  utils/statistics.cpp
  utils/syntheticScene.cpp
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
  PUBLIC
  aliceVision_numeric
  aliceVision_track
  aliceVision_multiview
  stlplus
  ${CERES_LIBRARIES}
  aliceVision_lInftyComputerVision
  aliceVision_system
  aliceVision_matching
  aliceVision_feature
  aliceVision_geometry
  aliceVision_track
  ${Boost_LIBRARIES}
  ${LOG_LIB}
)

set_target_properties(aliceVision_sfm
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

if(ALICEVISION_HAVE_ALEMBIC)
  target_link_libraries(aliceVision_sfm
    PRIVATE Alembic::Alembic
  )
endif()

install(TARGETS aliceVision_sfm
  DESTINATION lib
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision sfmDataIO          "aliceVision_feature;aliceVision_sfm;aliceVision_system;stlplus")
UNIT_TEST(aliceVision bundleAdjustment   "aliceVision_multiview_test_data;aliceVision_feature;aliceVision_multiview;aliceVision_sfm;aliceVision_system;stlplus")
UNIT_TEST(aliceVision rig                "aliceVision_feature;aliceVision_sfm;aliceVision_system")

if(ALICEVISION_HAVE_ALEMBIC)
  UNIT_TEST(aliceVision alembicIO "aliceVision_sfm;Alembic::Alembic")
endif()

add_subdirectory(pipeline)

