// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Universe.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>

namespace aliceVision {
namespace depthMap {

class DepthSim
{
public:
    union {
        struct
        {
            float depth, sim;
        };
        float m[2];
    };

    inline DepthSim()
    {
        depth = 0.0;
        sim = 0.0;
    }

    inline DepthSim(float _depth, float _sim)
    {
        depth = _depth;
        sim = _sim;
    }

    inline DepthSim& operator=(const DepthSim& param)
    {
        depth = param.depth;
        sim = param.sim;
        return *this;
    }
};


class DepthSimMap
{
public:
    mvsUtils::MultiViewParams& _mp;
    const int scale;
    const int step;
    int rc, w, h;
    StaticVector<DepthSim> dsm; //< depth similarity map

    DepthSimMap(int rc, mvsUtils::MultiViewParams& mp, int _scale, int _step);
    ~DepthSimMap(void);

    void initJustFromDepthMap(const StaticVector<float>& depthMap, float defaultSim);
    void initFromDepthMapAndSimMap(StaticVector<float>* depthMapT, StaticVector<float>* simMapT,
                                     int depthSimMapsScale);

    void add11(const DepthSimMap& depthSimMap);
    void add(const DepthSimMap& depthSimMap);

    Point2d getMaxMinDepth() const;
    Point2d getMaxMinSim() const;

    float getPercentileDepth(float perc) const;
    void getDepthMapStep1(StaticVector<float>& out_depthMap) const;
    void getSimMapStep1(StaticVector<float>& out_simMap) const;
    void getDepthMap(StaticVector<float>& out_depthMap) const;

    void getDepthMapStep1XPart(StaticVector<float>& out_depthMap, int xFrom, int partW);
    void getSimMapStep1XPart(StaticVector<float>& out_depthMap, int xFrom, int partW);

    void saveToImage(const std::string& filename, float simThr) const;
    void save(int rc, const StaticVector<int>& tcams) const;
    void load(int rc, int fromScale);
    void saveRefine(int rc, const std::string& depthMapFileName, const std::string& simMapFileName) const;

    float getCellSmoothStep(int rc, const int cellId);
    float getCellSmoothStep(int rc, const Pixel& cell);
};

} // namespace depthMap
} // namespace aliceVision
