// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_SFM_HPP
#define OPENMVG_SFM_HPP

#include "openMVG/types.hpp"
#include "openMVG/numeric/numeric.h"

//-----------------
// SfM data
//-----------------
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_utils.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data_filters.hpp"
#include "openMVG/sfm/sfm_data_filters_frustum.hpp"
#include "openMVG/sfm/sfm_data_BA.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres.hpp"

#include "openMVG/sfm/sfm_filters.hpp"
#include "openMVG/sfm/sfm_data_triangulation.hpp"

//-----------------
// SfM pipelines
//-----------------
#include "openMVG/sfm/sfm_report.hpp"
#include "openMVG/sfm/pipelines/sfm_engine.hpp"
#include "openMVG/features/FeaturesPerView.hpp"
#include "openMVG/features/RegionsPerView.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"

#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"

#include "openMVG/sfm/pipelines/sequential/sequential_SfM.hpp"

#include "openMVG/sfm/pipelines/global/sfm_global_reindex.hpp"
#include "openMVG/sfm/pipelines/global/sfm_global_engine_relative_motions.hpp"

#include "openMVG/sfm/pipelines/structure_from_known_poses/structure_estimator.hpp"

#include "openMVG/sfm/pipelines/localization/SfM_Localizer.hpp"
#include "openMVG/sfm/pipelines/localization/SfM_Localizer_Single_3DTrackObservation_Database.hpp"

#endif // OPENMVG_SFM_HPP
