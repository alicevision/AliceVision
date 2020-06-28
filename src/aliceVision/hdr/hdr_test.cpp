#include <boost/filesystem.hpp>

#define BOOST_TEST_MODULE hdr
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>

#include <aliceVision/image/all.hpp>
#include <aliceVision/system/Logger.hpp>

#include "DebevecCalibrate.hpp"
#include "LaguerreBACalibration.hpp"
#include "GrossbergCalibrate.hpp"
#include "sampling.hpp"

#include <random>
#include <array>
#include <boost/filesystem.hpp>

using namespace aliceVision;
namespace fs = boost::filesystem;

bool buildBrackets(std::vector<std::string>& paths, std::vector<float>& times, const hdr::rgbCurve& gt_response)
{
    /* Exposure time for each bracket */
    times = {0.05f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f, 1.1f};

    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.0f, 1.0f);

    const size_t size_region = 11;
    const size_t regions_count_per_row = 64;
    const size_t regions_count_per_col = 64;

    /* Generate a random image made of flat blocks (to pass variancetest )*/
    image::Image<image::RGBfColor> img(512, 512, true, image::RGBfColor(0.0f));
    int val = 0;
    for(int i = 0; i < img.Height(); i++)
    {
        int y = i / size_region;

        for(int j = 0; j < img.Width(); j++)
        {
            int x = j / size_region;
            int val = y * regions_count_per_col + x;
            val = val % 4096;

            float r = float(val) / 1023.0;
            float g = float(val) / 1023.0;
            float b = float(val) / 1023.0;

            img(i, j) = image::RGBfColor(r, g, b);
        }
    }

    for(double time : times)
    {
        image::Image<image::RGBfColor> img_bracket(img.Width(), img.Height());
        for(int i = 0; i < img.Height(); i++)
        {
            for(int j = 0; j < img.Width(); j++)
            {
                image::RGBfColor color = img(i, j);

                for(int k = 0; k < 3; k++)
                {
                    float radiance = color[k];
                    float radiance_dt = radiance * time;
                    float val = gt_response(radiance_dt, k);
                    val = std::min(1.0f, val);
                    img_bracket(i, j)[k] = val;
                }
            }
        }

        boost::filesystem::path temp = boost::filesystem::temp_directory_path();
        temp /= boost::filesystem::unique_path();
        temp += ".exr";

        ALICEVISION_LOG_INFO("writing to " << temp.string());

        image::writeImage(temp.string(), img_bracket, image::EImageColorSpace::LINEAR);
        paths.push_back(temp.string());
    }

    return true;
}

BOOST_AUTO_TEST_CASE(hdr_laguerre)
{
    std::vector<std::string> paths;
    std::vector<float> times;

    const size_t quantization = pow(2, 10);
    hdr::rgbCurve gt_curve(quantization);

    std::array<float, 3> laguerreParams = {-0.2, 0.4, -0.3};
    for(int i = 0; i < quantization; i++)
    {
        float x = float(i) / float(quantization - 1);
        gt_curve.getCurve(0)[i] = hdr::laguerreFunction(laguerreParams[0], x);
        gt_curve.getCurve(1)[i] = hdr::laguerreFunction(laguerreParams[1], x);
        gt_curve.getCurve(2)[i] = hdr::laguerreFunction(laguerreParams[2], x);
    }

    buildBrackets(paths, times, gt_curve);

    std::vector<std::vector<std::string>> all_paths;
    all_paths.push_back(paths);
    std::vector<std::vector<float>> exposures;
    exposures.push_back(times);
    hdr::LaguerreBACalibration calib;
    hdr::rgbCurve response(quantization);
    calib.process(all_paths, quantization, exposures, false, response);

    for(int imageId = 0; imageId < paths.size() - 1; imageId++)
    {
        image::Image<image::RGBfColor> imgA, imgB;
        image::readImage(paths[imageId], imgA, image::EImageColorSpace::LINEAR);
        image::readImage(paths[imageId + 1], imgB, image::EImageColorSpace::LINEAR);

        BOOST_CHECK(imgA.size() == imgB.size());
        double ratioExposures = times[imageId] / times[imageId + 1];

        double max_diff = 0.0;
        for(int i = 0; i < imgA.Height(); i++)
        {
            for(int j = 0; j < imgA.Width(); j++)
            {
                image::RGBfColor Ba = imgA(i, j);
                image::RGBfColor Bb = imgB(i, j);
                for(int k = 0; k < 3; k++)
                {
                    double responseA = response(Ba(k), k);
                    double responseB = response(Bb(k), k);
                    double hdrA = responseA / times[imageId];
                    double hdrB = responseB / times[imageId + 1];
                    double diff = std::abs(responseA - ratioExposures * responseB);

                    if (Bb(k) > 0.99) diff = 0.0;
                    if (hdrA > 1.0) diff = 0.0;
                    if (hdrB > 1.0) diff = 0.0;
                    
                    max_diff = std::max(diff, max_diff);
                    
                }
            }
        }

        
        BOOST_CHECK(std::isfinite(max_diff));
        BOOST_CHECK_SMALL(max_diff, 1e-3);
    }
}


BOOST_AUTO_TEST_CASE(hdr_debevec)
{
    std::vector<std::string> paths;
    std::vector<float> times;

    const size_t quantization = pow(2, 10);
    hdr::rgbCurve gt_curve(quantization);

    std::array<float, 3> laguerreParams = {-0.8, 0.8, -0.3};
    for(int i = 0; i < quantization; i++)
    {
        float x = float(i) / float(quantization - 1);
        gt_curve.getCurve(0)[i] = hdr::laguerreFunction(laguerreParams[0], x);
        gt_curve.getCurve(1)[i] = hdr::laguerreFunction(laguerreParams[1], x);
        gt_curve.getCurve(2)[i] = hdr::laguerreFunction(laguerreParams[2], x);
    }

    buildBrackets(paths, times, gt_curve);

    std::vector<std::vector<std::string>> all_paths;
    all_paths.push_back(paths);

    std::vector<std::vector<float>> exposures;
    exposures.push_back(times);

    hdr::DebevecCalibrate calib;
    hdr::rgbCurve response(quantization);
    hdr::rgbCurve calibrationWeight(quantization);

    calibrationWeight.setTriangular();
    calib.process(all_paths, quantization, exposures, calibrationWeight, 0.001, response);
    response.exponential();

    for(int imageId = 0; imageId < paths.size() - 1; imageId++)
    {
        image::Image<image::RGBfColor> imgA, imgB;
        image::readImage(paths[imageId], imgA, image::EImageColorSpace::LINEAR);
        image::readImage(paths[imageId + 1], imgB, image::EImageColorSpace::LINEAR);

        BOOST_CHECK(imgA.size() == imgB.size());
        double ratioExposures = times[imageId] / times[imageId + 1];

        double max_diff = 0.0;
        for(int i = 0; i < imgA.Height(); i++)
        {
            for(int j = 0; j < imgA.Width(); j++)
            {
                image::RGBfColor Ba = imgA(i, j);
                image::RGBfColor Bb = imgB(i, j);
                for(int k = 0; k < 3; k++)
                {
                    double responseA = response(Ba(k), k);
                    double responseB = response(Bb(k), k);
                    double hdrA = responseA / times[imageId];
                    double hdrB = responseB / times[imageId + 1];
                    double diff = std::abs(responseA - ratioExposures * responseB);

                    if (Ba(k) < 0.01) diff = 0.0;
                    if (Bb(k) > 0.96) diff = 0.0;
                    if (hdrA > 0.99) diff = 0.0;
                    if (hdrB > 0.99) diff = 0.0;
                    
                    max_diff = std::max(diff, max_diff);
                    
                }
            }
        }

        
        BOOST_CHECK(std::isfinite(max_diff));
        BOOST_CHECK_SMALL(max_diff, 0.01);
    }
}


BOOST_AUTO_TEST_CASE(hdr_grossberg)
{
    std::vector<std::string> paths;
    std::vector<float> times;

    const size_t quantization = pow(2, 10);
    hdr::rgbCurve gt_curve(quantization);

    gt_curve.setEmor(0);
    std::array<double, 3> grossberg_params[3] = {
                {0.8, 0.3, 0.2}, 
                {-0.1, -0.3, 0.2}, 
                {2.1, 0.05, 0.4}
            };

    for(int dim = 0; dim < 3; dim++)
    {
        hdr::rgbCurve dim_curve(quantization);
        dim_curve.setEmor(dim + 1);

        for(int k = 0; k < quantization; k++)
        {
            gt_curve.getCurve(0)[k] += grossberg_params[0][dim] * dim_curve.getCurve(0)[k];
            gt_curve.getCurve(1)[k] += grossberg_params[1][dim] * dim_curve.getCurve(0)[k];
            gt_curve.getCurve(2)[k] += grossberg_params[2][dim] * dim_curve.getCurve(0)[k];
        }
    }

    buildBrackets(paths, times, gt_curve);

    std::vector<std::vector<std::string>> all_paths;
    std::vector<std::vector<float>> exposures;

    all_paths.push_back(paths);
    exposures.push_back(times);

    hdr::GrossbergCalibrate calib(9);
    hdr::rgbCurve response(quantization);

    calib.process(all_paths, quantization, exposures, response);

    for(int imageId = 0; imageId < paths.size() - 1; imageId++)
    {
        image::Image<image::RGBfColor> imgA, imgB;
        image::readImage(paths[imageId], imgA, image::EImageColorSpace::LINEAR);
        image::readImage(paths[imageId + 1], imgB, image::EImageColorSpace::LINEAR);

        BOOST_CHECK(imgA.size() == imgB.size());
        double ratioExposures = times[imageId] / times[imageId + 1];

        double max_diff = 0.0;
        for(int i = 0; i < imgA.Height(); i++)
        {
            for(int j = 0; j < imgA.Width(); j++)
            {
                image::RGBfColor Ba = imgA(i, j);
                image::RGBfColor Bb = imgB(i, j);
                for(int k = 0; k < 3; k++)
                {
                    double responseA = response(Ba(k), k);
                    double responseB = response(Bb(k), k);
                    double hdrA = responseA / times[imageId];
                    double hdrB = responseB / times[imageId + 1];
                    double diff = std::abs(responseA - ratioExposures * responseB);

                    if (Bb(k) > 0.93) diff = 0.0;
                    if (hdrA > 0.99) diff = 0.0;
                    if (hdrB > 0.99) diff = 0.0;
                    
                    max_diff = std::max(diff, max_diff);
                    
                }
            }
        }

        
        BOOST_CHECK(std::isfinite(max_diff));
        BOOST_CHECK_SMALL(max_diff, 2.0 * 1e-3);
    }
}

