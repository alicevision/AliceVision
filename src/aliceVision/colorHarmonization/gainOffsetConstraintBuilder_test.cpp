// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <cstdio>
#include <iostream>
#include <string>
#include <cstdlib>
#include <cmath>

// ColorHarmonization solver
#include <aliceVision/colorHarmonization/GainOffsetConstraintBuilder.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/config.hpp>

#include <aliceVision/utils/Histogram.hpp>
#include "dependencies/htmlDoc/htmlDoc.hpp"

#define BOOST_TEST_MODULE GainOffsetConstraintBuilder

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::linearProgramming;
using namespace aliceVision::lInfinity;

double const pi = 4.0 * std::atan(1.0);

// simple functor for normal distribution
class NormalDistribution
{
  public:
    NormalDistribution(double m, double s)
      : mu(m),
        sigma(s)
    {}

    double operator()() const  // returns a single normally distributed number
    {
        double r1 = (std::rand() + 1.0) / (RAND_MAX + 1.0);  // gives equal distribution in (0, 1]
        double r2 = (std::rand() + 1.0) / (RAND_MAX + 1.0);
        return mu + sigma * std::sqrt(-2 * std::log(r1)) * std::cos(2 * pi * r2);
    }

  private:
    const double mu, sigma;
};

BOOST_AUTO_TEST_CASE(ColorHarmonisation_Simple_offset)
{
    utils::Histogram<double> histo(0, 256, 255);
    for (std::size_t i = 0; i < 6000; i++)
    {
        histo.Add(NormalDistribution(127, 10)());
    }

    const size_t OFFET_VALUE = 20;
    std::vector<std::size_t> reference = histo.GetHist();
    std::vector<std::size_t> shifted = reference;
    rotate(shifted.begin(), shifted.begin() + OFFET_VALUE, shifted.end());

    //-- Try to solve the color consistency between the two histograms
    //-- We are looking for gain and offset parameter for each image {g;o}
    //--  and the upper bound precision found by Linfinity minimization.
    std::vector<double> solution(2 * 2 + 1);

    //-- Setup the problem data in the container
    std::vector<relativeColorHistogramEdge> relativeHistograms;
    relativeHistograms.push_back(relativeColorHistogramEdge(0, 1, reference, shifted));
    //-- First image will be considered as reference and don't move
    std::vector<std::size_t> indexToFix(1, 0);

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
    typedef MOSEKSolver SOLVER_LP_T;
#else
    typedef OSI_CISolverWrapper SOLVER_LP_T;
#endif
    // Red channel
    {
        SOLVER_LP_T lpSolver(solution.size());

        GainOffsetConstraintBuilder cstBuilder(relativeHistograms, indexToFix);
        LPConstraintsSparse constraint;
        cstBuilder.Build(constraint);
        lpSolver.setup(constraint);
        lpSolver.solve();
        lpSolver.getSolution(solution);
    }

    ALICEVISION_LOG_DEBUG("Found solution:");
    std::copy(solution.begin(), solution.end(), std::ostream_iterator<double>(std::cout, " "));

    double g0 = solution[0];
    double o0 = solution[1];
    double g1 = solution[2];
    double o1 = solution[3];
    double gamma = solution[4];

    BOOST_CHECK_SMALL(1. - g0, 1e-2);
    BOOST_CHECK_SMALL(0. - o0, 1e-2);
    BOOST_CHECK_SMALL(1. - g1, 1e-2);
    BOOST_CHECK_SMALL(OFFET_VALUE - o1, 1e-2);
    BOOST_CHECK_SMALL(0. - gamma, 1e-2);  // Alignment must be perfect
}

BOOST_AUTO_TEST_CASE(ColorHarmonisation_Offset_gain)
{
    utils::Histogram<double> histoRef(0, 256, 255);
    utils::Histogram<double> histoOffsetGain(0, 256, 255);
    const double GAIN = 3.0;
    const double OFFSET = 160;
    // const double GAIN = 2.0;
    // const double OFFSET = 50;
    for (std::size_t i = 0; i < 10000; i++)
    {
        double val = NormalDistribution(127, 10)();
        histoRef.Add(val);
        histoOffsetGain.Add((val - 127) * GAIN + OFFSET);
    }
    std::vector<std::size_t> reference = histoRef.GetHist();
    std::vector<std::size_t> shifted = histoOffsetGain.GetHist();

    //-- Try to solve the color consistency between the two histograms
    //-- We are looking for gain and offset parameter for each image {g;o}
    //--  and the upper bound precision found by Linfinity minimization.
    std::vector<double> solution(3 * 2 + 1);

    //-- Setup the problem data in the container
    std::vector<relativeColorHistogramEdge> relativeHistograms;
    relativeHistograms.push_back(relativeColorHistogramEdge(0, 1, reference, shifted));
    relativeHistograms.push_back(relativeColorHistogramEdge(1, 2, shifted, reference));
    relativeHistograms.push_back(relativeColorHistogramEdge(0, 2, reference, reference));
    //-- First image will be considered as reference and don't move
    std::vector<size_t> indexToFix(1, 0);

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
    typedef MOSEKSolver SOLVER_LP_T;
#else
    typedef OSI_CISolverWrapper SOLVER_LP_T;
#endif
    // Red channel
    {
        SOLVER_LP_T lpSolver(solution.size());

        GainOffsetConstraintBuilder cstBuilder(relativeHistograms, indexToFix);
        LPConstraintsSparse constraint;
        cstBuilder.Build(constraint);
        lpSolver.setup(constraint);
        lpSolver.solve();
        lpSolver.getSolution(solution);
    }

    ALICEVISION_LOG_DEBUG("Found solution:");
    std::copy(solution.begin(), solution.end(), std::ostream_iterator<double>(std::cout, " "));

    double g0 = solution[0];
    double o0 = solution[1];
    double g1 = solution[2];
    double o1 = solution[3];
    double g2 = solution[4];
    double o2 = solution[5];
    double gamma = solution[6];

    // The minimal solution must be {0,1,1/gain, 127-offset/gain,1,0}
    // gain and offset 2 must not move since it is equal to reference and link to the reference.

    BOOST_CHECK_SMALL(1. - g0, 1e-2);
    BOOST_CHECK_SMALL(0. - o0, 1e-2);
    BOOST_CHECK_SMALL((1. / GAIN) - g1, 1e-1);
    BOOST_CHECK_SMALL((127 - OFFSET / GAIN) - o1, 2.);  // +/- quantization error (2 gray levels)
    BOOST_CHECK_SMALL(1. - g2, 1e-2);
    BOOST_CHECK_SMALL(0. - o2, 1e-2);
    BOOST_CHECK(gamma < 1.0);  // Alignment must be below one gray level

    //-- Visual HTML export
    using namespace htmlDocument;
    htmlDocument::htmlDocumentStream _htmlDocStream("Global Multiple-View Color Consistency.");
    // Reference histogram
    {
        htmlDocument::JSXGraphWrapper jsxGraph;
        jsxGraph.init("test0", 600, 300);
        jsxGraph.addYChart(histoRef.GetHist(), "point");
        jsxGraph.UnsuspendUpdate();
        std::vector<double> xBin = histoRef.GetXbinsValue();
        std::pair<std::pair<double, double>, std::pair<double, double>> range = autoJSXGraphViewport<double>(xBin, histoRef.GetHist());
        jsxGraph.setViewport(range);
        jsxGraph.close();
        _htmlDocStream.pushInfo(jsxGraph.toStr());
    }
    // Histogram with gain and offset change
    {
        htmlDocument::JSXGraphWrapper jsxGraph;
        jsxGraph.init("test1", 600, 300);
        jsxGraph.addYChart(histoOffsetGain.GetHist(), "point");
        jsxGraph.UnsuspendUpdate();
        std::vector<double> xBin = histoOffsetGain.GetXbinsValue();
        std::pair<std::pair<double, double>, std::pair<double, double>> range = autoJSXGraphViewport<double>(xBin, histoOffsetGain.GetHist());
        jsxGraph.setViewport(range);
        jsxGraph.close();
        _htmlDocStream.pushInfo(jsxGraph.toStr());
    }

    std::ofstream htmlFileStream("test.html");
    htmlFileStream << _htmlDocStream.getDoc();
}
