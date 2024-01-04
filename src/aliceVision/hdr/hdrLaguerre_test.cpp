// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE hdr_debevec

#include "hdrTestCommon.hpp"
#include "LaguerreBACalibration.hpp"

#include <boost/test/unit_test.hpp>

using namespace aliceVision;

BOOST_AUTO_TEST_CASE(hdr_laguerre)
{
    std::vector<std::string> paths;
    std::vector<double> times;

    const size_t quantization = pow(2, 10);
    hdr::rgbCurve gt_curve(quantization);

    std::array<float, 3> laguerreParams = {-0.2, 0.4, -0.3};
    for (int i = 0; i < quantization; i++)
    {
        float x = float(i) / float(quantization - 1);
        gt_curve.getCurve(0)[i] = hdr::laguerreFunction(laguerreParams[0], x);
        gt_curve.getCurve(1)[i] = hdr::laguerreFunction(laguerreParams[1], x);
        gt_curve.getCurve(2)[i] = hdr::laguerreFunction(laguerreParams[2], x);
    }

    hdr::test::buildBrackets(paths, times, gt_curve);

    std::vector<std::vector<std::string>> all_paths;
    all_paths.push_back(paths);
    std::vector<std::vector<double>> exposures;
    exposures.push_back(times);
    hdr::LaguerreBACalibration calib;
    hdr::rgbCurve response(quantization);

    std::vector<std::vector<hdr::ImageSample>> samples;
    hdr::test::extractSamplesGroups(samples, all_paths, exposures, quantization);
    calib.process(samples, exposures, quantization, false, response);

    for (int imageId = 0; imageId < paths.size() - 1; imageId++)
    {
        image::Image<image::RGBfColor> imgA, imgB;
        image::readImage(paths[imageId], imgA, image::EImageColorSpace::LINEAR);
        image::readImage(paths[imageId + 1], imgB, image::EImageColorSpace::LINEAR);

        BOOST_CHECK(imgA.size() == imgB.size());
        double ratioExposures = times[imageId] / times[imageId + 1];

        double max_diff = 0.0;
        for (int i = 0; i < imgA.height(); i++)
        {
            for (int j = 0; j < imgA.width(); j++)
            {
                image::RGBfColor Ba = imgA(i, j);
                image::RGBfColor Bb = imgB(i, j);
                for (int k = 0; k < 3; k++)
                {
                    double responseA = response(Ba(k), k);
                    double responseB = response(Bb(k), k);
                    double hdrA = responseA / times[imageId];
                    double hdrB = responseB / times[imageId + 1];
                    double diff = std::abs(responseA - ratioExposures * responseB);

                    if (Bb(k) > 0.99)
                        diff = 0.0;
                    if (hdrA > 1.0)
                        diff = 0.0;
                    if (hdrB > 1.0)
                        diff = 0.0;

                    max_diff = std::max(diff, max_diff);
                }
            }
        }

        BOOST_CHECK(std::isfinite(max_diff));
        BOOST_CHECK_SMALL(max_diff, 1e-3);
    }
}
