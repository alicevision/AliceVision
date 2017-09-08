// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_SFM_HPP
#define OPENMVG_SFM_HPP

#include "aliceVision/types.hpp"
#include "aliceVision/numeric/numeric.h"

//-----------------
// SfM data
//-----------------
#include "aliceVision/sfm/sfm_data.hpp"
#include "aliceVision/sfm/sfm_data_utils.hpp"
#include "aliceVision/sfm/sfm_data_io.hpp"
#include "aliceVision/sfm/sfm_data_filters.hpp"
#include "aliceVision/sfm/sfm_data_filters_frustum.hpp"
#include "aliceVision/sfm/sfm_data_BA.hpp"
#include "aliceVision/sfm/sfm_data_BA_ceres.hpp"

#include "aliceVision/sfm/sfm_filters.hpp"
#include "aliceVision/sfm/sfm_data_triangulation.hpp"

//-----------------
// SfM pipelines
//-----------------
#include "aliceVision/sfm/sfm_report.hpp"
#include "aliceVision/sfm/pipelines/sfm_engine.hpp"
#include "aliceVision/features/FeaturesPerView.hpp"
#include "aliceVision/features/RegionsPerView.hpp"
#include "aliceVision/sfm/pipelines/sfm_matches_provider.hpp"

#include "aliceVision/sfm/pipelines/sfm_robust_model_estimation.hpp"

#include "aliceVision/sfm/pipelines/sequential/sequential_SfM.hpp"

#include "aliceVision/sfm/pipelines/global/sfm_global_reindex.hpp"
#include "aliceVision/sfm/pipelines/global/sfm_global_engine_relative_motions.hpp"

#include "aliceVision/sfm/pipelines/structure_from_known_poses/structure_estimator.hpp"

#include "aliceVision/sfm/pipelines/localization/SfM_Localizer.hpp"
#include "aliceVision/sfm/pipelines/localization/SfM_Localizer_Single_3DTrackObservation_Database.hpp"

#endif // OPENMVG_SFM_HPP
