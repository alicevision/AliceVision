// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/Universe.hpp>
#include <aliceVision/structures/Pixel.hpp>
#include <aliceVision/structures/Point2d.hpp>
#include <aliceVision/structures/Point3d.hpp>
#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/common/MultiViewParams.hpp>

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
    MultiViewParams* mp;
    int rc, w, h, scale, step;
    StaticVector<DepthSim>* dsm; //< depth similarity map

    DepthSimMap(int rc, MultiViewParams* _mp, int _scale, int _step);
    ~DepthSimMap(void);

    Point3d get3DPtOfPixel(const Pixel& pix, int pixScale, int rc);
    float getFPDepthOfPixel(const Pixel& pix, int pixScale, int rc);

    void initJustFromDepthMapT(StaticVector<float>* depthMapT, float defaultSim);
    void initJustFromDepthMap(StaticVector<float>* depthMap, float defaultSim);
    void initFromDepthMapTAndSimMapT(StaticVector<float>* depthMapT, StaticVector<float>* simMapT,
                                     int depthSimMapsScale);
    void initFromDepthMapAndSimMap(StaticVector<float>* depthMap, StaticVector<float>* simMap, int depthSimMapsScale);
    void setUsedCellsSimTo(float defaultSim);

    void add11(DepthSimMap* depthSimMap);
    void add(DepthSimMap* depthSimMap);

    void getReconstructedPixelsDepthsSims(StaticVector<Pixel>* pixels, StaticVector<float>* depths,
                                          StaticVector<float>* sims);

    Point2d getMaxMinDepth();
    Point2d getMaxMinSim();
    float getPercentileDepth(float perc);
    StaticVector<float>* getDepthMapStep1();
    StaticVector<float>* getDepthMapTStep1();
    StaticVector<float>* getSimMapStep1();
    StaticVector<float>* getSimMapTStep1();
    StaticVector<float>* getDepthMap();

    StaticVector<float>* getDepthMapStep1XPart(int xFrom, int partW);
    StaticVector<float>* getSimMapStep1XPart(int xFrom, int partW);

    void saveToImage(std::string pngFileName, float simThr);
    void save(int rc, StaticVector<int>* tcams);
    void load(int rc, int fromScale);

    void saveRefine(int rc, std::string depthMapFileName, std::string simMapFileName);
    bool loadRefine(std::string depthMapFileName, std::string simMapFileName);
    bool loadRefine(std::string depthMapFileName, float defaultSim);

    Universe* segment(float alpha, int rc);
    void removeSmallSegments(int minSegSize, float alpha, int rc);
    void cutout(const Pixel& LU, const Pixel& RD);

    float getAngleBetwABandACdepth(int rc, const Pixel& cellA, float dA, const Pixel& cellB, float dB,
                                   const Pixel& cellC, float dC);
    float getCellSmoothEnergy(int rc, const int cellId, float defaultE);
    float getCellSmoothEnergy(int rc, const Pixel& cell, float defaultE);

    float getASmoothStepBetwABandACdepth(int rc, const Pixel& cellA, float dA, const Pixel& cellB, float dB,
                                         const Pixel& cellC, float dC);
    float getCellSmoothStep(int rc, const int cellId);
    float getCellSmoothStep(int rc, const Pixel& cell);
};
