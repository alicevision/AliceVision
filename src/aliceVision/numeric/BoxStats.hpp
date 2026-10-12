// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once


#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace aliceVision {


/** Get back the min, mean, median and the max
 *  values of an iterable sequence.
 */
template<typename Type>
struct BoxStats
{
    Type min{}, max{}, mean{}, median{}, firstQuartile{}, thirdQuartile{};

    BoxStats() = default;

    template<typename DataInputIterator>
    BoxStats(DataInputIterator begin, DataInputIterator end)
    {
        compute(begin, end);
    }

    template<typename DataInputIterator>
    void compute(DataInputIterator begin, DataInputIterator end)
    {
        if (std::distance(begin, end) < 1)
        {
            min = 0;
            max = 0;
            mean = 0;
            median = 0;
            firstQuartile = 0;
            thirdQuartile = 0;
            return;
        }

        std::vector<Type> vec_val(begin, end);
        std::sort(vec_val.begin(), vec_val.end());
        min = vec_val[0];
        max = vec_val[vec_val.size() - 1];
        mean = accumulate(vec_val.begin(), vec_val.end(), Type(0)) / static_cast<Type>(vec_val.size());
        median = vec_val[vec_val.size() / 2];
        firstQuartile = vec_val[vec_val.size() / 4];
        thirdQuartile = vec_val[(vec_val.size() * 3) / 4];
    }
};

template<typename Type>
inline std::ostream& operator<<(std::ostream& os, const BoxStats<Type> obj)
{
    os << "\t min: " << obj.min
       << "\n"
          "\t mean: "
       << obj.mean
       << "\n"
          "\t median: "
       << obj.median
       << "\n"
          "\t max: "
       << obj.max
       << "\n"
          "\t first quartile: "
       << obj.firstQuartile
       << "\n"
          "\t third quartile: "
       << obj.thirdQuartile;

    return os;
}


}  // namespace aliceVision
