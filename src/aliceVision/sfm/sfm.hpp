// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// SfM

#include <aliceVision/sfm/filters.hpp>
#include <aliceVision/sfm/FrustumFilter.hpp>
#include <aliceVision/sfm/BundleAdjustment.hpp>
#include <aliceVision/sfm/BundleAdjustmentCeres.hpp>
#include <aliceVision/sfm/LocalBundleAdjustmentGraph.hpp>
#include <aliceVision/sfm/generateReport.hpp>
#include <aliceVision/sfm/sfmFilters.hpp>
#include <aliceVision/sfm/sfmTriangulation.hpp>

// SfM pipeline

#include <aliceVision/sfm/pipeline/ReconstructionEngine.hpp>
#include <aliceVision/sfm/pipeline/pairwiseMatchesIO.hpp>
#include <aliceVision/sfm/pipeline/RelativePoseInfo.hpp>
#include <aliceVision/sfm/pipeline/global/reindexGlobalSfM.hpp>
#include <aliceVision/sfm/pipeline/global/ReconstructionEngine_globalSfM.hpp>
#include <aliceVision/sfm/pipeline/panorama/ReconstructionEngine_panorama.hpp>
#include <aliceVision/sfm/pipeline/sequential/ReconstructionEngine_sequentialSfM.hpp>
#include <aliceVision/sfm/pipeline/structureFromKnownPoses/StructureEstimationFromKnownPoses.hpp>
#include <aliceVision/sfm/pipeline/localization/SfMLocalizer.hpp>
#include <aliceVision/sfm/pipeline/localization/SfMLocalizationSingle3DTrackObservationDatabase.hpp>

