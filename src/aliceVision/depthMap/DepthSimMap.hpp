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
    mvsUtils::MultiViewParams* mp;
    const int scale;
    const int step;
    int rc, w, h;
    StaticVector<DepthSim>* dsm; //< depth similarity map

    DepthSimMap(int rc, mvsUtils::MultiViewParams* _mp, int _scale, int _step);
    ~DepthSimMap(void);

    void initJustFromDepthMapT(StaticVector<float>* depthMapT, float defaultSim);
    void initJustFromDepthMap(StaticVector<float>* depthMap, float defaultSim);
    void initFromDepthMapTAndSimMapT(StaticVector<float>* depthMapT, StaticVector<float>* simMapT,
                                     int depthSimMapsScale);

    void add11(DepthSimMap* depthSimMap);
    void add(DepthSimMap* depthSimMap);

    Point2d getMaxMinDepth() const;
    Point2d getMaxMinSim() const;

    float getPercentileDepth(float perc);
    StaticVector<float>* getDepthMapStep1();
    StaticVector<float>* getDepthMapTStep1();
    StaticVector<float>* getSimMapStep1();
    StaticVector<float>* getDepthMap();

    StaticVector<float>* getDepthMapStep1XPart(int xFrom, int partW);
    StaticVector<float>* getSimMapStep1XPart(int xFrom, int partW);

    void saveToImage(std::string pngFileName, float simThr);
    void save(int rc, const StaticVector<int>& tcams);
    void load(int rc, int fromScale);
    void saveRefine(int rc, std::string depthMapFileName, std::string simMapFileName);

    float getCellSmoothStep(int rc, const int cellId);
    float getCellSmoothStep(int rc, const Pixel& cell);
};

} // namespace depthMap
} // namespace aliceVision
