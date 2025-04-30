// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <array>
#include <ceres/ceres.h>

struct ForceDepthErrorFunctor
{
    explicit ForceDepthErrorFunctor(double depth, double weight)
    : pmean(mean)
    , pweight(weight)
    {        
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residuals) const
    {       
        const T* parameter_point = parameters[0];

        residuals[0] = pweight * (parameter_point[2] - T(pmean));

        return true;
    }

    inline static ceres::CostFunction* createCostFunction(double depth, double weight)
    {
        auto costFunction = new ceres::DynamicAutoDiffCostFunction<ForceDepthErrorFunctor>(
            new ForceDepthErrorFunctor(depth, weight));

        costFunction->AddParameterBlock(3);
        costFunction->SetNumResiduals(1);

        return costFunction;
    }

    double pmean;
    double pweight;
};