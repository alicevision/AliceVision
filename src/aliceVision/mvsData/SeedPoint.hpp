// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/OrientedPoint.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/SeedPointCams.hpp>

namespace aliceVision {

class SeedPoint
{
public:
    OrientedPoint op;    // 18 Bytes
    Point3d xax;
    Point3d yax;
    float pixSize;       // 4 bytes
    unsigned long area;  // 4 bytes
    unsigned long segId; // 4-Bytes : 0 to 4,294,967,295
    SeedPointCams cams;  // n * 2
    // std::vector<unsigned short> cams;
    // TOTAL: floating

    SeedPoint()
    {
        op = OrientedPoint();
        pixSize = 0.0;
        area = 0;
    }

    ~SeedPoint()
    {
        cams.resize(0);
    }

    SeedPoint& operator=(const SeedPoint& param)
    {
        op = param.op;
        xax = param.xax;
        yax = param.yax;

        cams.resize(0);
        for(int i = 0; i < param.cams.size(); i++)
        {
            cams.push_back(param.cams[i]);
        };
        for(int i = 0; i < param.cams.size() + 1; i++)
        {
            cams.shifts[i] = param.cams.shifts[i];
        };

        area = param.area;
        pixSize = param.pixSize;
        segId = param.segId;
        return *this;
    }

    bool operator>(const SeedPoint& param) const
    {
        return (op.sim > param.op.sim);
    }
};

} // namespace aliceVision
