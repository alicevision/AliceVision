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

    inline DepthSim& operator=(const DepthSim& v)
    {
        depth = v.depth;
        sim = v.sim;
        return *this;
    }

    inline DepthSim operator+(const DepthSim& v) const
    {
        DepthSim out;
        out.depth = depth + v.depth;
        out.sim = sim + v.sim;
        return out;
    }
    inline DepthSim operator-(const DepthSim& v) const
    {
        DepthSim out;
        out.depth = depth - v.depth;
        out.sim = sim - v.sim;
        return out;
    }
    inline DepthSim operator*(float v) const
    {
        DepthSim out;
        out.depth = depth * v;
        out.sim = sim * v;
        return out;
    }
    inline DepthSim operator/(float v) const
    {
        DepthSim out;
        out.depth = depth / v;
        out.sim = sim / v;
        return out;
    }
    inline bool operator<(const DepthSim& other) const
    {
        if(depth == other.depth)
            return sim < other.sim;
        return (depth < other.depth);
    }
};


class DepthSimMap
{
public:
    const mvsUtils::MultiViewParams& _mp;
    const int _scale;
    const int _step;
    int _rc, _w, _h;
    StaticVector<DepthSim> _dsm; //< depth similarity map

    DepthSimMap(int rc, const mvsUtils::MultiViewParams& mp, int scale, int step);
    ~DepthSimMap();

    inline int getScale() const { return _scale; }
    inline int getStep() const { return _step; }
    inline int getRc() const { return _rc; }
    inline int getWidth() const { return _w; }
    inline int getHeight() const { return _h; }
    inline const DepthSim& getDepthSim(int x, int y) const { return _dsm[y * _w + x]; }
    inline const StaticVector<DepthSim>& getData() const { return _dsm; }

    inline DepthSim& getDepthSim(int x, int y) { return _dsm[y * _w + x]; }
    inline StaticVector<DepthSim>& getData() { return _dsm; }

    void initJustFromDepthMap(const StaticVector<float>& depthMap, float defaultSim);
    void initJustFromDepthMap(const DepthSimMap& depthSimMap, float defaultSim);
    void initFromDepthMapAndSimMap(StaticVector<float>* depthMapT, StaticVector<float>* simMapT,
                                     int depthSimMapsScale);

    void initFromSmaller(const DepthSimMap& depthSimMap);
    void init(const DepthSimMap& depthSimMap);

    Point2d getMaxMinDepth() const;
    Point2d getMaxMinSim() const;

    float getPercentileDepth(float perc) const;
    void getDepthMapStep1(StaticVector<float>& out_depthMap) const;
    void getSimMapStep1(StaticVector<float>& out_simMap) const;
    void getDepthMap(StaticVector<float>& out_depthMap) const;
    void getSimMap(StaticVector<float>& out_simMap) const;

    void getDepthMapStep1XPart(StaticVector<float>& out_depthMap, int xFrom, int partW);
    void getSimMapStep1XPart(StaticVector<float>& out_depthMap, int xFrom, int partW);

    void saveToImage(const std::string& filename, float simThr) const;
    void save(const std::string& customSuffix = "", bool useStep1 = false) const;
    void load(int fromScale);
};

} // namespace depthMap
} // namespace aliceVision
