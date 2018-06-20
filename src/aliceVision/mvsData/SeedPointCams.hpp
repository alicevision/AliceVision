// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/Point2d.hpp>

namespace aliceVision {

class SeedPointCams
{
public:
    static const int SP_NCAMS = 20;

    unsigned short cams[SP_NCAMS];
    Point2d shifts[SP_NCAMS];
    int n = 0;

    SeedPointCams() {}
    ~SeedPointCams() {}

    inline const unsigned short& operator[](int index) const
    {
        return cams[index];
    }

    inline unsigned short& operator[](int index)
    {
        return cams[index];
    }

    inline void push_back(unsigned short cam)
    {
        if(n == SP_NCAMS - 1)
        {
            // printf("push_back too many cams\n");
            // exit(1);
        }
        else
        {
            cams[n] = cam;
            shifts[n] = Point2d(0.0f, 0.0f);
            n++;
        }
    }

    inline int size() const
    {
        return n;
    }

    inline void resize(int newSize)
    {
        n = newSize;
    }

    inline void reserve(int newSize)
    {
        if(SP_NCAMS < newSize)
        {
            ALICEVISION_LOG_ERROR("reserve too many cams\n");
            // exit(1);
        }
    }

    inline int indexOf(int what)
    {
        int isthereindex = -1;
        int i = 0;
        while((i < n) && (isthereindex == -1))
        {
            if(cams[i] == what)
            {
                isthereindex = i;
            };
            i++;
        };
        return isthereindex;
    }
};

} // namespace aliceVision
