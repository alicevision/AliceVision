// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE hdr_debevec

#include "hdrTestCommon.hpp"
#include "GrossbergCalibrate.hpp"

#include <boost/test/unit_test.hpp>

using namespace aliceVision;

BOOST_AUTO_TEST_CASE(hdr_grossberg)
{
    std::vector<std::string> paths;
    std::vector<double> times;

    const size_t quantization = pow(2, 10);
    hdr::rgbCurve gt_curve(quantization);

    gt_curve.setEmor(0);
    std::array<double, 3> grossberg_params[3] = {{0.8, 0.3, 0.2}, {-0.1, -0.3, 0.2}, {2.1, 0.05, 0.4}};

    for (int dim = 0; dim < 3; dim++)
    {
        hdr::rgbCurve dim_curve(quantization);
        dim_curve.setEmor(dim + 1);

        for (int k = 0; k < quantization; k++)
        {
            gt_curve.getCurve(0)[k] += grossberg_params[0][dim] * dim_curve.getCurve(0)[k];
            gt_curve.getCurve(1)[k] += grossberg_params[1][dim] * dim_curve.getCurve(0)[k];
            gt_curve.getCurve(2)[k] += grossberg_params[2][dim] * dim_curve.getCurve(0)[k];
        }
    }

    hdr::test::buildBrackets(paths, times, gt_curve);

    std::vector<std::vector<std::string>> all_paths;
    std::vector<std::vector<double>> exposures;

    all_paths.push_back(paths);
    exposures.push_back(times);

    hdr::GrossbergCalibrate calib(9);
    hdr::rgbCurve response(quantization);

    std::vector<std::vector<hdr::ImageSample>> samples;
    hdr::test::extractSamplesGroups(samples, all_paths, exposures, quantization);
    calib.process(samples, exposures, quantization, response);

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

                    if (Bb(k) > 0.93)
                        diff = 0.0;
                    if (hdrA > 0.99)
                        diff = 0.0;
                    if (hdrB > 0.99)
                        diff = 0.0;

                    max_diff = std::max(diff, max_diff);
                }
            }
        }

        BOOST_CHECK(std::isfinite(max_diff));
        BOOST_CHECK_SMALL(max_diff, 2.0 * 1e-3);
    }
}
