// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <queue>
#include <vector>

class TopN
{
public:
    /**
    @brief Constructur
    @param cacheSize the maximal number of element kept
    */
    TopN(size_t cacheSize)
    : _cacheSize(cacheSize)
    {

    }

    void add(const double & value)
    {
        if (_minHeap.size() < _cacheSize)
        {
            _minHeap.push(value);
            return;
        }

        /*Only add if value is greater than minimal value*/
        if (value > _minHeap.top())
        {
            _minHeap.pop();
            _minHeap.push(value);
        }
    }

    std::vector<double> getAndReset()
    {
        std::vector<double> result;

        //Result will be in ascending order
        while (!_minHeap.empty())
        {
            result.push_back(_minHeap.top());
            _minHeap.pop();
        }

        return result;
    }

private:
    /**
    Top value will be the smallest value
    */
    std::priority_queue<double, std::vector<double>, std::greater<double>> _minHeap;

    size_t _cacheSize;
};