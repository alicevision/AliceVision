// This file is part of the AliceVision project.
// Copyright (c) 2027 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/utils/gauge.hpp>

namespace aliceVision
{
namespace sfm
{

bool selectTripletForGaugeRemoval(const sfmData::SfMData& sfmData, std::array<IndexT, 3>& selectedSet, int nbSamples)
{
    // Check landmarks
    const auto & landmarks = sfmData.getLandmarks();
    if (landmarks.size() < 3)
    {
        return false;
    }

    // Get a consecutive vector of landmarks indices
    std::vector<IndexT> landmarksIndices;
    landmarksIndices.reserve(landmarks.size());
    for (const auto & [key, _] : landmarks)
    {
        landmarksIndices.push_back(key);
    }

    std::mt19937 gen;
    std::uniform_int_distribution<size_t> randIdx(0, landmarks.size() - 1);

    double maxScore = -1.0;
    for (int i = 0; i < nbSamples; i++)
    {   
        // Select 3 unique indices
        size_t i0 = randIdx(gen);
        size_t i1, i2;
        do { i1 = randIdx(gen); } while (i1 == i0);
        do { i2 = randIdx(gen); } while (i2 == i0 || i2 == i1);

        IndexT idx1 = landmarksIndices[i0];
        IndexT idx2 = landmarksIndices[i1];
        IndexT idx3 = landmarksIndices[i2];

        const Vec3 & p1 = landmarks.at(idx1).getX();
        const Vec3 & p2 = landmarks.at(idx2).getX();
        const Vec3 & p3 = landmarks.at(idx3).getX();

        const Vec3 d1 = p2 - p1;
        const Vec3 d2 = p3 - p1;
        const Vec3 d3 = p3 - p2;
        
        // Compute area of the triangle
        const Vec3 crossProduct = d1.cross(d2);
        const double areaDouble = crossProduct.norm();

        // Privilege large area while penalizing elongated triangles
        // Which may lead to colinearity
        const double longestSize = std::max({d1.norm(), d2.norm(), d3.norm()});
        const double score = areaDouble / longestSize;

        if (score > maxScore)
        {
            maxScore = score;
            selectedSet[0] = idx1;
            selectedSet[1] = idx2;
            selectedSet[2] = idx3;
        }
    }

    return true;
}

}
}
