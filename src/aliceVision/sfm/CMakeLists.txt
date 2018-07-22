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
  CameraPose.hpp
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

alicevision_add_library(aliceVision_sfm
  SOURCES ${sfm_files_headers} ${sfm_files_sources}
  PUBLIC_LINKS
    aliceVision_numeric
    aliceVision_track
    aliceVision_multiview
    aliceVision_lInftyComputerVision
    aliceVision_system
    aliceVision_matching
    aliceVision_feature
    aliceVision_geometry
    aliceVision_track
    aliceVision_camera
    aliceVision_graph
    aliceVision_linearProgramming
    ${Boost_FILESYSTEM_LIBRARY}
    ${CERES_LIBRARIES}
  PRIVATE_LINKS
    ${Boost_REGEX_LIBRARY}
)


if(ALICEVISION_HAVE_ALEMBIC)
  target_link_libraries(aliceVision_sfm
    PRIVATE Alembic::Alembic
  )
  # There is a missing include dependency in Alembic cmake export.
  target_include_directories(aliceVision_sfm
    PRIVATE "${ILMBASE_INCLUDE_DIR}"
  )
endif()

# Unit tests
if(ALICEVISION_HAVE_ALEMBIC)
  alicevision_add_test(alembicIO_test.cpp NAME "sfm_alembicIO" LINKS aliceVision_sfm Alembic::Alembic)
endif()

alicevision_add_test(sfmDataIO_test.cpp
  NAME "sfm_sfmDataIO"
  LINKS aliceVision_sfm
        aliceVision_feature
        aliceVision_system
)

alicevision_add_test(bundleAdjustment_test.cpp
  NAME "sfm_bundleAdjustment"
  LINKS aliceVision_sfm
        aliceVision_multiview
        aliceVision_multiview_test_data
        aliceVision_feature
        aliceVision_system
)

alicevision_add_test(rig_test.cpp
  NAME "sfm_rig"
  LINKS aliceVision_sfm
        aliceVision_feature
        aliceVision_system
)

add_subdirectory(pipeline)

