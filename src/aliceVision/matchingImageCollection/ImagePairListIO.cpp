// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImagePairListIO.hpp"
#include <aliceVision/system/Logger.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <fstream>

namespace aliceVision {
namespace matchingImageCollection {


bool loadPairsFromFile(const std::string& sFileName,  // filename of the list file,
                       PairSet& pairs,
                       bool useSymmetry)
{
    std::ifstream in(sFileName);
    if (!in.is_open())
    {
        ALICEVISION_LOG_WARNING("loadPairsFromFile: Impossible to read the specified file: \"" << sFileName << "\".");
        return false;
    }

    std::size_t nbLine = 0;
    std::string sValue;

    for (; std::getline(in, sValue); ++nbLine)
    {
        std::vector<std::string> vec_str;
        boost::trim(sValue);
        boost::split(vec_str, sValue, boost::is_any_of("\t "), boost::token_compress_on);

        const size_t str_size = vec_str.size();
        if (str_size < 2)
        {
            ALICEVISION_LOG_WARNING("loadPairsFromFile: Invalid input file.");
            return false;
        }

        std::stringstream oss;
        oss.clear();
        oss.str(vec_str[0]);
        size_t I, J;
        oss >> I;

        for (size_t i = 1; i < str_size; ++i)
        {
            oss.clear();
            oss.str(vec_str[i]);
            oss >> J;

            Pair pairToInsert;

            if (useSymmetry)
            {
                pairToInsert = (I < J) ? std::make_pair(I, J) : std::make_pair(J, I);
            }
            else
            {
                pairToInsert = std::make_pair(I, J);
            }

            ALICEVISION_LOG_TRACE("loadPairsFromFile: image pair (" << I << ", " << J << ") added.");
            pairs.insert(pairToInsert);
        }
    }

    return true;
}

bool savePairsToFile(const std::string& sFileName, const PairSet& pairs)
{
    std::ofstream outStream(sFileName);
    if (!outStream.is_open())
    {
        ALICEVISION_LOG_WARNING("savePairsToFile: Impossible to open the output specified file: \"" << sFileName << "\".");
        return false;
    }

    if (pairs.empty())
    {
        return true;
    }
    
    outStream << pairs.begin()->first << " " << pairs.begin()->second;
    IndexT previousIndex = pairs.begin()->first;

    // Pairs is sorted so we will always receive elements with the same first pair ID in
    // continuous blocks.
    for (auto it = std::next(pairs.begin()); it != pairs.end(); ++it)
    {
        if (it->first == previousIndex)
        {
            outStream << " " << it->second;
        }
        else
        {
            outStream << "\n" << it->first << " " << it->second;
            previousIndex = it->first;
        }
    }
    outStream << "\n";

    return !outStream.bad();
}

}  // namespace matchingImageCollection
}  // namespace aliceVision
