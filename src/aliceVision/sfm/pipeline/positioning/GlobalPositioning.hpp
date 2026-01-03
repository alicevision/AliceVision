// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <random>
#include <map>
#include <ceres/ceres.h>

namespace aliceVision 
{
namespace sfm
{

class GlobalPositioning
{
public:
    /**
     * @brief compute an approximation of the landmarks position
     * and an approximation of the structure assuming correct rotation
     * and correct intrinsics
     * 
     * @param sfmData the input and output sfmData. It will remain unchanged on error
     * @param generator manage the random number generator (for reproductibility).
    */
    bool process(sfmData::SfMData & sfmData, std::mt19937 & generator);

private:

    /**
     * @brief create ceres problem using sfmData measures
     * @param sfmData the input sfmData containing all the current state
     * @param generator the random generator for reproductibility
    */
    bool createStructure(const sfmData::SfMData & sfmData, std::mt19937 & generator);

    /**
     * @brief create ceres problem using sfmData measures
     * @param problem output variable containing the problem to solve
     * @param ordering the output requested ordering of variables
     * @param sfmData the input sfmData containing all the current state
    */
    bool createProblem(ceres::Problem & problem, ceres::ParameterBlockOrdering & ordering, const sfmData::SfMData & sfmData);
    
    /**
     * @brief update sfmData with values in the class properties
     * @param sfmData the sfmData to update
     */
    void updateSfmData(sfmData::SfMData & sfmData);

    /**
     * @brief erase incorrect observations from sfmData
     * @param sfmData the in/out sfmData to process
     * @return true if something was erased
    */
    bool filter(sfmData::SfMData & sfmData);

    /**
     * @brief erase observations from sfmData
     * @param sfmData the in/out sfmData to process
     * @return count erased
    */
    size_t eraseObservationsWithAngularError(sfmData::SfMData & sfmData);

private:
    std::map<IndexT, Vec3> _landmarks;
    std::map<IndexT, Vec3> _centers;
    std::map<Pair, double> _scales;
    std::map<Pair, Vec3> _observations;
   
private:
    double _huberScale = 0.1;
    double _maxAngle = 2.0;
};

}
}