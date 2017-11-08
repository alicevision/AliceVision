// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/output3D/mv_output3D.hpp>
#include <aliceVision/structures/mv_universe.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

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


class ps_depthSimMap
{
public:
    multiviewParams* mp;
    mv_output3D* o3d;
    int rc, w, h, scale, step;
    staticVector<DepthSim>* dsm; //< depth similarity map

    ps_depthSimMap(int rc, multiviewParams* _mp, int _scale, int _step);
    ~ps_depthSimMap(void);

    point3d get3DPtOfPixel(const pixel& pix, int pixScale, int rc);
    float getFPDepthOfPixel(const pixel& pix, int pixScale, int rc);

    void initJustFromDepthMapT(staticVector<float>* depthMapT, float defaultSim);
    void initJustFromDepthMap(staticVector<float>* depthMap, float defaultSim);
    void initFromDepthMapTAndSimMapT(staticVector<float>* depthMapT, staticVector<float>* simMapT,
                                     int depthSimMapsScale);
    void initFromDepthMapAndSimMap(staticVector<float>* depthMap, staticVector<float>* simMap, int depthSimMapsScale);
    void setUsedCellsSimTo(float defaultSim);

    void add11(ps_depthSimMap* depthSimMap);
    void add(ps_depthSimMap* depthSimMap);

    void getReconstructedPixelsDepthsSims(staticVector<pixel>* pixels, staticVector<float>* depths,
                                          staticVector<float>* sims);

    point2d getMaxMinDepth();
    point2d getMaxMinSim();
    float getPercentileDepth(float perc);
    staticVector<float>* getDepthMapStep1();
    staticVector<float>* getDepthMapTStep1();
    staticVector<float>* getSimMapStep1();
    staticVector<float>* getSimMapTStep1();
    staticVector<float>* getDepthMap();

    staticVector<float>* getDepthMapStep1XPart(int xFrom, int partW);
    staticVector<float>* getSimMapStep1XPart(int xFrom, int partW);

    IplImage* convertToImage(float simThr);
    void show(float simThr);
    void saveToPng(std::string pngFileName, float simThr);
    void saveToWrl(std::string wrlFileName, int rc);
    void saveToWrlPng(std::string wrlFileName, int rc, float simThr);
    void save(int rc, staticVector<int>* tcams);
    void load(int rc, int fromScale);

    void saveToBin(std::string depthMapFileName, std::string simMapFileName);
    bool loadFromBin(std::string depthMapFileName, std::string simMapFileName);
    bool loadFromBin(std::string depthMapFileName, float defaultSim);

    mv_universe* segment(float alpha, int rc);
    void removeSmallSegments(int minSegSize, float alpha, int rc);
    void cutout(const pixel& LU, const pixel& RD);

    float getAngleBetwABandACdepth(int rc, const pixel& cellA, float dA, const pixel& cellB, float dB,
                                   const pixel& cellC, float dC);
    float getCellSmoothEnergy(int rc, const int cellId, float defaultE);
    float getCellSmoothEnergy(int rc, const pixel& cell, float defaultE);

    float getASmoothStepBetwABandACdepth(int rc, const pixel& cellA, float dA, const pixel& cellB, float dB,
                                         const pixel& cellC, float dC);
    float getCellSmoothStep(int rc, const int cellId);
    float getCellSmoothStep(int rc, const pixel& cell);
};
