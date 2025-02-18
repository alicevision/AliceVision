// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/pipeline/bootstrapping/PairsScoring.hpp>
#include <aliceVision/sfm/pipeline/bootstrapping/EstimateAngle.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionPolicyLegacy.hpp>
#include <aliceVision/multiview/triangulation/triangulationDLT.hpp>

namespace aliceVision {
namespace sfm {

IndexT findBestPair(const sfmData::SfMData & sfmData, 
                const std::vector<sfm::ReconstructedPair> & pairs,
                const track::TracksMap& tracksMap, 
                const track::TracksPerView & tracksPerView, 
                double minAngle,
                double maxAngle)
{
    IndexT bestPair = UndefinedIndexT;
    double maxScore = std::numeric_limits<double>::lowest();
    
    for (IndexT pairId = 0; pairId < pairs.size(); pairId++)
    {
        const sfm::ReconstructedPair & pair = pairs[pairId];
        
        double angle = 0.0;
        std::vector<size_t> usedTracks;
        if (!sfm::estimatePairAngle(sfmData, pair.reference, pair.next, pair.pose, tracksMap, tracksPerView, angle, usedTracks))
        {
            continue;
        }

        if (radianToDegree(angle) > maxAngle)
        {
            continue;
        }

       

        const sfmData::View & vref = sfmData.getView(pair.reference);
        const sfmData::View & vnext = sfmData.getView(pair.next);

        int maxref = std::max(vref.getImage().getWidth(), vref.getImage().getHeight());
        int maxnext = std::max(vnext.getImage().getWidth(), vnext.getImage().getHeight());
        

        double refScore = sfm::ExpansionPolicyLegacy::computeScore(tracksMap, usedTracks, pair.reference, maxref, 5);
        double nextScore = sfm::ExpansionPolicyLegacy::computeScore(tracksMap, usedTracks, pair.next, maxnext, 5);

        double score = std::min(refScore, nextScore) * std::max(1e-12, radianToDegree(angle));
         //If the angle is too small, then dramatically reduce its chances
        if (radianToDegree(angle) < minAngle)
        {
            score = -1.0 / score;
        }

        if (score > maxScore)
        {
            maxScore = score;
            bestPair = pairId;
        }
    }
    
    return bestPair;
}

}
}