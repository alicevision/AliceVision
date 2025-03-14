// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>
#include "aliceVision/linearProgramming/ISolver.hpp"
#include "aliceVision/linearProgramming/OSIXSolver.hpp"
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
    #include "aliceVision/linearProgramming/MOSEKSolver.hpp"
#endif
#include "aliceVision/linearProgramming/bisectionLP.hpp"

namespace aliceVision {
namespace lInfinity {

struct relativeColorHistogramEdge
{
    std::size_t I, J;
    std::vector<size_t> histoI, histoJ;

    relativeColorHistogramEdge() {}

    relativeColorHistogramEdge(size_t i, size_t j, const std::vector<std::size_t>& histogramI, const std::vector<std::size_t>& histogramJ)
      : I(i),
        J(j),
        histoI(histogramI),
        histoJ(histogramJ)
    {}
};

namespace histogram {

// Normalize a distribution function
template<typename T>
inline void normalizeHisto(const std::vector<T>& vecDf, std::vector<double>& vecNormalizedDf)
{
    double totalCount = static_cast<double>(std::accumulate(vecDf.begin(), vecDf.end(), 0));
    vecNormalizedDf.resize(vecDf.size(), 0.0);
    for (std::size_t i = 0; i < vecDf.size(); ++i)
        vecNormalizedDf[i] = vecDf[i] / totalCount;
}

// Compute cumulative distribution functions (cdf)
template<typename T>
inline void cdf(const std::vector<T>& vecDf, std::vector<T>& vecCdf)
{
    vecCdf = vecDf;
    for (size_t i = 1; i < vecCdf.size(); i++)
        vecCdf[i] = vecCdf[i] + vecCdf[i - 1];
}

};  // namespace histogram

// Implementation of the formula (1) of [1] with 10 quantiles.
//-- L_infinity alignment of pair of histograms over a graph thanks to a linear program.
void Encode_histo_relation(const std::size_t nImage,
                           const std::vector<relativeColorHistogramEdge>& vec_relativeHistograms,
                           const std::vector<std::size_t>& vec_indexToFix,
                           sRMat& A,
                           Vec& C,
                           std::vector<linearProgramming::LPConstraints::eLP_SIGN>& vec_sign,
                           std::vector<double>& vec_costs,
                           std::vector<std::pair<double, double>>& vec_bounds);

struct GainOffsetConstraintBuilder
{
    GainOffsetConstraintBuilder(const std::vector<relativeColorHistogramEdge>& vecRelativeHistograms, const std::vector<std::size_t>& vecIndexToFix)
      : _vecRelative(vecRelativeHistograms),
        _vecIndexToFix(vecIndexToFix)
    {
        // Count the number of images
        std::set<std::size_t> countSet;
        for (int i = 0; i < static_cast<int>(_vecRelative.size()); ++i)
        {
            countSet.insert(_vecRelative[i].I);
            countSet.insert(_vecRelative[i].J);
        }
        _Nima = countSet.size();
    }

    /// Setup constraints for the translation and structure problem,
    ///  in the LPConstraints object.
    bool Build(linearProgramming::LPConstraintsSparse& constraint)
    {
        Encode_histo_relation(_Nima,
                              _vecRelative,
                              _vecIndexToFix,
                              constraint._constraintMat,
                              constraint._Cst_objective,
                              constraint._vec_sign,
                              constraint._vec_cost,
                              constraint._vec_bounds);

        // it's a minimization problem over the gamma variable
        constraint._bminimize = true;

        //-- Setup additional information about the Linear Program constraint
        constraint._nbParams = _Nima * 2 + 1;
        return true;
    }
    // Internal data
    size_t _Nima;
    const std::vector<relativeColorHistogramEdge>& _vecRelative;
    const std::vector<std::size_t>& _vecIndexToFix;
};

};  // namespace lInfinity
};  // namespace aliceVision
