// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>
#include <aliceVision/sensorDB/parseDatabase.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/utils/regexFilter.hpp>
#include <aliceVision/utils/filesIO.hpp>
#include <aliceVision/stl/mapUtils.hpp>
#include <aliceVision/lensCorrectionProfile/lcp.hpp>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
    #include <opencv2/imgproc.hpp>
    #include <opencv2/photo.hpp>
#endif

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>
#include <OpenImageIO/color.h>

#include <filesystem>
#include <string>
#include <cmath>
#include <vector>
#include <map>
#include <iostream>
#include <algorithm>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 3
#define ALICEVISION_SOFTWARE_VERSION_MINOR 3

using namespace aliceVision;
namespace po = boost::program_options;
namespace fs = std::filesystem;

struct LensCorrectionParams
{
    bool enabled = false;
    bool geometry = false;
    bool vignetting = false;
    bool chromaticAberration = false;

    std::vector<float> gParams;
    std::vector<float> vParams;
    std::vector<float> caGParams;
    std::vector<float> caRGParams;
    std::vector<float> caBGParams;

    RectilinearModel geometryModel = RectilinearModel();

    RectilinearModel caGModel = RectilinearModel();
    RectilinearModel caBGModel = RectilinearModel();
    RectilinearModel caRGModel = RectilinearModel();
};

std::istream& operator>>(std::istream& in, LensCorrectionParams& lcParams)
{
    std::string token;
    in >> token;
    std::vector<std::string> splitParams;
    boost::split(splitParams, token, boost::algorithm::is_any_of(":"));
    if (splitParams.size() != 4)
        throw std::invalid_argument("Failed to parse LensCorrectionParams from: " + token);
    lcParams.enabled = boost::to_lower_copy(splitParams[0]) == "true";
    lcParams.geometry = boost::to_lower_copy(splitParams[1]) == "true";
    lcParams.vignetting = boost::to_lower_copy(splitParams[2]) == "true";
    lcParams.chromaticAberration = boost::to_lower_copy(splitParams[3]) == "true";

    return in;
}

inline std::ostream& operator<<(std::ostream& os, const LensCorrectionParams& lcParams)
{
    os << lcParams.enabled << ":" << lcParams.geometry << ":" << lcParams.vignetting << ":" << lcParams.chromaticAberration;
    return os;
}

struct SharpenParams
{
    bool enabled;
    int width;
    float contrast;
    float threshold;
};

std::istream& operator>>(std::istream& in, SharpenParams& sParams)
{
    std::string token;
    in >> token;
    std::vector<std::string> splitParams;
    boost::split(splitParams, token, boost::algorithm::is_any_of(":"));
    if (splitParams.size() != 4)
        throw std::invalid_argument("Failed to parse SharpenParams from: " + token);
    sParams.enabled = boost::to_lower_copy(splitParams[0]) == "true";
    sParams.width = boost::lexical_cast<int>(splitParams[1]);
    sParams.contrast = boost::lexical_cast<float>(splitParams[2]);
    sParams.threshold = boost::lexical_cast<float>(splitParams[3]);

    return in;
}

inline std::ostream& operator<<(std::ostream& os, const SharpenParams& sParams)
{
    os << sParams.enabled << ":" << sParams.width << ":" << sParams.contrast << ":" << sParams.threshold;
    return os;
}

struct BilateralFilterParams
{
    bool enabled;
    int distance;
    float sigmaColor;
    float sigmaSpace;
};

std::istream& operator>>(std::istream& in, BilateralFilterParams& bfParams)
{
    std::string token;
    in >> token;
    std::vector<std::string> splitParams;
    boost::split(splitParams, token, boost::algorithm::is_any_of(":"));
    if (splitParams.size() != 4)
        throw std::invalid_argument("Failed to parse BilateralFilterParams from: " + token);
    bfParams.enabled = boost::to_lower_copy(splitParams[0]) == "true";
    bfParams.distance = boost::lexical_cast<int>(splitParams[1]);
    bfParams.sigmaColor = boost::lexical_cast<float>(splitParams[2]);
    bfParams.sigmaSpace = boost::lexical_cast<float>(splitParams[3]);

    return in;
}

inline std::ostream& operator<<(std::ostream& os, const BilateralFilterParams& bfParams)
{
    os << bfParams.enabled << ":" << bfParams.distance << ":" << bfParams.sigmaColor << ":" << bfParams.sigmaSpace;
    return os;
}

struct ClaheFilterParams
{
    bool enabled;
    float clipLimit;
    int tileGridSize;
};

std::istream& operator>>(std::istream& in, ClaheFilterParams& cfParams)
{
    std::string token;
    in >> token;
    std::vector<std::string> splitParams;
    boost::split(splitParams, token, boost::algorithm::is_any_of(":"));
    if (splitParams.size() != 3)
        throw std::invalid_argument("Failed to parse ClaheFilterParams from: " + token);
    cfParams.enabled = boost::to_lower_copy(splitParams[0]) == "true";
    cfParams.clipLimit = boost::lexical_cast<float>(splitParams[1]);
    cfParams.tileGridSize = boost::lexical_cast<int>(splitParams[2]);

    return in;
}

inline std::ostream& operator<<(std::ostream& os, const ClaheFilterParams& cfParams)
{
    os << cfParams.enabled << ":" << cfParams.clipLimit << ":" << cfParams.tileGridSize;
    return os;
}

enum class ENoiseMethod
{
    uniform,
    gaussian,
    salt
};

inline std::string ENoiseMethod_enumToString(ENoiseMethod noiseMethod)
{
    switch (noiseMethod)
    {
        case ENoiseMethod::uniform:
            return "uniform";
        case ENoiseMethod::gaussian:
            return "gaussian";
        case ENoiseMethod::salt:
            return "salt";
    }
    throw std::invalid_argument("Invalid ENoiseMethod Enum");
}

inline ENoiseMethod ENoiseMethod_stringToEnum(std::string noiseMethod)
{
    boost::to_lower(noiseMethod);
    if (noiseMethod == "uniform")
        return ENoiseMethod::uniform;
    if (noiseMethod == "gaussian")
        return ENoiseMethod::gaussian;
    if (noiseMethod == "salt")
        return ENoiseMethod::salt;

    throw std::invalid_argument("Unrecognized noise method '" + noiseMethod + "'");
}

struct NoiseFilterParams
{
    bool enabled;
    ENoiseMethod method;
    float A;
    float B;
    bool mono;
};

std::istream& operator>>(std::istream& in, NoiseFilterParams& nfParams)
{
    std::string token;
    in >> token;
    std::vector<std::string> splitParams;
    boost::split(splitParams, token, boost::algorithm::is_any_of(":"));
    if (splitParams.size() != 5)
        throw std::invalid_argument("Failed to parse NoiseFilterParams from: " + token);
    nfParams.enabled = boost::to_lower_copy(splitParams[0]) == "true";
    nfParams.method = ENoiseMethod_stringToEnum(splitParams[1]);
    nfParams.A = boost::lexical_cast<float>(splitParams[2]);
    nfParams.B = boost::lexical_cast<float>(splitParams[3]);
    nfParams.mono = boost::to_lower_copy(splitParams[4]) == "true";
    return in;
}

inline std::ostream& operator<<(std::ostream& os, const NoiseFilterParams& nfParams)
{
    os << nfParams.enabled << ":" << ENoiseMethod_enumToString(nfParams.method) << ":" << nfParams.A << ":" << nfParams.B << ":" << nfParams.mono;
    return os;
}

enum class EImageFormat
{
    RGBA,
    RGB,
    Grayscale
};

inline std::string EImageFormat_enumToString(EImageFormat imageFormat)
{
    switch (imageFormat)
    {
        case EImageFormat::RGBA:
            return "rgba";
        case EImageFormat::RGB:
            return "rgb";
        case EImageFormat::Grayscale:
            return "grayscale";
    }
    throw std::invalid_argument("Invalid EImageFormat Enum");
}

inline EImageFormat EImageFormat_stringToEnum(std::string imageFormat)
{
    boost::to_lower(imageFormat);
    if (imageFormat == "rgba")
        return EImageFormat::RGBA;
    if (imageFormat == "rgb")
        return EImageFormat::RGB;
    if (imageFormat == "grayscale")
        return EImageFormat::Grayscale;

    throw std::invalid_argument("Unrecognized image format '" + imageFormat + "'");
}

inline std::ostream& operator<<(std::ostream& os, EImageFormat e) { return os << EImageFormat_enumToString(e); }

inline std::istream& operator>>(std::istream& in, EImageFormat& e)
{
    std::string token;
    in >> token;
    e = EImageFormat_stringToEnum(token);
    return in;
}

struct NLMeansFilterParams
{
    bool enabled;
    float filterStrength;
    float filterStrengthColor;
    int templateWindowSize;
    int searchWindowSize;
};

std::istream& operator>>(std::istream& in, NLMeansFilterParams& nlmParams)
{
    std::string token;
    in >> token;
    std::vector<std::string> splitParams;
    boost::split(splitParams, token, boost::algorithm::is_any_of(":"));
    if (splitParams.size() != 5)
        throw std::invalid_argument("Failed to parse NLMeansFilterParams from: " + token);
    nlmParams.enabled = boost::to_lower_copy(splitParams[0]) == "true";
    nlmParams.filterStrength = boost::lexical_cast<float>(splitParams[1]);
    nlmParams.filterStrengthColor = boost::lexical_cast<float>(splitParams[2]);
    nlmParams.templateWindowSize = boost::lexical_cast<int>(splitParams[3]);
    nlmParams.searchWindowSize = boost::lexical_cast<int>(splitParams[4]);

    return in;
}

inline std::ostream& operator<<(std::ostream& os, const NLMeansFilterParams& nlmParams)
{
    os << nlmParams.enabled << ":" << nlmParams.filterStrength << ":" << nlmParams.filterStrengthColor << ":" << nlmParams.templateWindowSize << ":"
       << nlmParams.searchWindowSize;
    return os;
}

struct pixelAspectRatioParams
{
    bool enabled;
    bool rowDecimation;
    float value;
};

std::istream& operator>>(std::istream& in, pixelAspectRatioParams& parParams)
{
    std::string token;
    in >> token;
    std::vector<std::string> splitParams;
    boost::split(splitParams, token, boost::algorithm::is_any_of(":"));
    if (splitParams.size() != 2)
        throw std::invalid_argument("Failed to parse pixelAspectRatioParams from: " + token);
    parParams.enabled = boost::to_lower_copy(splitParams[0]) == "true";
    parParams.rowDecimation = boost::to_lower_copy(splitParams[1]) == "true";
    return in;
}

inline std::ostream& operator<<(std::ostream& os, const pixelAspectRatioParams& parParams)
{
    os << parParams.enabled << ":" << parParams.rowDecimation;
    return os;
}

std::string getColorProfileDatabaseFolder()
{
    const char* value = std::getenv("ALICEVISION_COLOR_PROFILE_DB");
    return value ? value : "";
}

struct ProcessingParams
{
    bool reconstructedViewsOnly = false;
    bool keepImageFilename = false;
    bool exposureCompensation = false;
    bool rawAutoBright = false;
    float rawExposureAdjust = 0.0;
    EImageFormat outputFormat = EImageFormat::RGBA;
    float scaleFactor = 1.0f;
    unsigned int maxWidth = 0;
    unsigned int maxHeight = 0;
    float contrast = 1.0f;
    int medianFilter = 0;
    bool fillHoles = false;
    bool fixNonFinite = false;
    bool applyDcpMetadata = false;
    bool useDCPColorMatrixOnly = false;
    bool enableColorTempProcessing = false;
    double correlatedColorTemperature = -1.0;
    bool reorient = false;

    LensCorrectionParams lensCorrection = {
      false,  // enable
      false,  // geometry
      false,  // vignetting
      false   // chromatic aberration
    };

    SharpenParams sharpen = {
      false,  // enable
      3,      // width
      1.0f,   // contrast
      0.0f    // threshold
    };

    BilateralFilterParams bilateralFilter = {
      false,  // enable
      0,      // distance
      0.0f,   // sigmaColor
      0.0f    // sigmaSpace
    };

    ClaheFilterParams claheFilter = {
      false,  // enable
      4.0f,   // clipLimit
      8       // tileGridSize
    };

    NoiseFilterParams noise = {
      false,                  // enable
      ENoiseMethod::uniform,  // method
      0.0f,                   // A
      1.0f,                   // B
      true                    // mono
    };

    NLMeansFilterParams nlmFilter = {
      false,  // enable
      5.0f,   // filterStrength
      10.0f,  // filterStrengthColor
      7,      // templateWindowSize
      21      // searchWindowSize
    };

    pixelAspectRatioParams par = {
      false,  // enable
      false,  // rowDecimation
      1.0f    // value
    };
};

void undistortVignetting(aliceVision::image::Image<aliceVision::image::RGBAfColor>& img, const std::vector<float>& vparam)
{
    if (vparam.size() >= 7)
    {
        const float focX = vparam[0];
        const float focY = vparam[1];
        const float imageXCenter = vparam[2];
        const float imageYCenter = vparam[3];

        const float p1 = -vparam[4];
        const float p2 = vparam[4] * vparam[4] - vparam[5];
        const float p3 = -(vparam[4] * vparam[4] * vparam[4] - 2 * vparam[4] * vparam[5] + vparam[6]);
        const float p4 =
          vparam[4] * vparam[4] * vparam[4] * vparam[4] + vparam[5] * vparam[5] + 2 * vparam[4] * vparam[6] - 3 * vparam[4] * vparam[4] * vparam[5];

#pragma omp parallel for
        for (int j = 0; j < img.height(); ++j)
            for (int i = 0; i < img.width(); ++i)
            {
                const aliceVision::Vec2 p(i, j);

                aliceVision::Vec2 np;
                np(0) = ((p(0) / img.width()) - imageXCenter) / focX;
                np(1) = ((p(1) / img.height()) - imageYCenter) / focY;

                const float rsqr = np(0) * np(0) + np(1) * np(1);
                const float gain = 1.f + p1 * rsqr + p2 * rsqr * rsqr + p3 * rsqr * rsqr * rsqr + p4 * rsqr * rsqr * rsqr * rsqr;

                img(j, i) *= gain;
            }
    }
}

void undistortRectilinearGeometryLCP(const aliceVision::image::Image<aliceVision::image::RGBAfColor>& img,
                                     RectilinearModel& model,
                                     aliceVision::image::Image<aliceVision::image::RGBAfColor>& img_ud,
                                     const image::RGBAfColor fillcolor)
{
    if (!model.isEmpty && model.FocalLengthX != 0.0 && model.FocalLengthY != 0.0)
    {
        img_ud.resize(img.width(), img.height(), true, fillcolor);
        const image::Sampler2d<image::SamplerLinear> sampler;

        const float maxWH = std::max(img.width(), img.height());
        const float ppX = model.ImageXCenter * img.width();
        const float ppY = model.ImageYCenter * img.height();
        const float scaleX = model.FocalLengthX * maxWH;
        const float scaleY = model.FocalLengthY * maxWH;

#pragma omp parallel for
        for (int v = 0; v < img.height(); ++v)
            for (int u = 0; u < img.width(); ++u)
            {
                // image to camera
                const float x = (u - ppX) / scaleX;
                const float y = (v - ppY) / scaleY;

                // disto
                float xd, yd;
                model.distort(x, y, xd, yd);

                // camera to image
                const Vec2 distoPix(xd * scaleX + ppX, yd * scaleY + ppY);

                // pick pixel if it is in the image domain
                if (img.contains(distoPix(1), distoPix(0)))
                {
                    img_ud(v, u) = sampler(img, distoPix(1), distoPix(0));
                }
            }
    }
}

void undistortChromaticAberrations(const aliceVision::image::Image<aliceVision::image::RGBAfColor>& img,
                                   RectilinearModel& greenModel,
                                   RectilinearModel& blueGreenModel,
                                   RectilinearModel& redGreenModel,
                                   aliceVision::image::Image<aliceVision::image::RGBAfColor>& img_ud,
                                   const image::RGBAfColor fillcolor,
                                   bool undistortGeometry = false)
{
    if (!greenModel.isEmpty && greenModel.FocalLengthX != 0.0 && greenModel.FocalLengthY != 0.0)
    {
        img_ud.resize(img.width(), img.height(), true, fillcolor);
        const image::Sampler2d<image::SamplerLinear> sampler;

        const float maxWH = std::max(img.width(), img.height());
        const float ppX = greenModel.ImageXCenter * img.width();
        const float ppY = greenModel.ImageYCenter * img.height();
        const float scaleX = greenModel.FocalLengthX * maxWH;
        const float scaleY = greenModel.FocalLengthY * maxWH;

#pragma omp parallel for
        for (int v = 0; v < img.height(); ++v)
            for (int u = 0; u < img.width(); ++u)
            {
                // image to camera
                const float x = (u - ppX) / scaleX;
                const float y = (v - ppY) / scaleY;

                // disto
                float xdRed, ydRed, xdGreen, ydGreen, xdBlue, ydBlue;
                if (undistortGeometry)
                {
                    greenModel.distort(x, y, xdGreen, ydGreen);
                }
                else
                {
                    xdGreen = x;
                    ydGreen = y;
                }
                redGreenModel.distort(xdGreen, ydGreen, xdRed, ydRed);
                blueGreenModel.distort(xdGreen, ydGreen, xdBlue, ydBlue);

                // camera to image
                const Vec2 distoPixRed(xdRed * scaleX + ppX, ydRed * scaleY + ppY);
                const Vec2 distoPixGreen(xdGreen * scaleX + ppX, ydGreen * scaleY + ppY);
                const Vec2 distoPixBlue(xdBlue * scaleX + ppX, ydBlue * scaleY + ppY);

                // pick pixel if it is in the image domain
                if (img.contains(distoPixRed(1), distoPixRed(0)))
                {
                    img_ud(v, u)[0] = sampler(img, distoPixRed(1), distoPixRed(0))[0];
                }
                if (img.contains(distoPixGreen(1), distoPixGreen(0)))
                {
                    img_ud(v, u)[1] = sampler(img, distoPixGreen(1), distoPixGreen(0))[1];
                }
                if (img.contains(distoPixBlue(1), distoPixBlue(0)))
                {
                    img_ud(v, u)[2] = sampler(img, distoPixBlue(1), distoPixBlue(0))[2];
                }
            }
    }
}

void processImage(image::Image<image::RGBAfColor>& image,
                  ProcessingParams& pParams,
                  std::map<std::string, std::string>& imageMetadata,
                  std::shared_ptr<camera::IntrinsicBase> cam)
{
    const unsigned int nchannels = 4;

    // Fix non-finite pixels
    // Note: fill holes needs to fix non-finite values first
    if (pParams.fixNonFinite || pParams.fillHoles)
    {
        oiio::ImageBuf inBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        int pixelsFixed = 0;
        // Works inplace
        oiio::ImageBufAlgo::fixNonFinite(inBuf, inBuf, oiio::ImageBufAlgo::NonFiniteFixMode::NONFINITE_BOX3, &pixelsFixed);
        ALICEVISION_LOG_INFO("Fixed " << pixelsFixed << " non-finite pixels.");
    }

    if (pParams.lensCorrection.enabled)
    {
        if (pParams.lensCorrection.vignetting && !pParams.lensCorrection.vParams.empty())
        {
            undistortVignetting(image, pParams.lensCorrection.vParams);
        }
        else if (pParams.lensCorrection.vignetting && pParams.lensCorrection.vParams.empty())
        {
            ALICEVISION_LOG_WARNING("No distortion model available for lens vignetting correction.");
        }

        if (pParams.lensCorrection.chromaticAberration && !pParams.lensCorrection.caGModel.isEmpty)
        {
            const image::RGBAfColor FBLACK_A(.0f, .0f, .0f, 1.0f);
            image::Image<image::RGBAfColor> image_ud;
            undistortChromaticAberrations(
              image, pParams.lensCorrection.caGModel, pParams.lensCorrection.caBGModel, pParams.lensCorrection.caRGModel, image_ud, FBLACK_A, false);
            image = image_ud;
        }
        else if (pParams.lensCorrection.chromaticAberration && pParams.lensCorrection.caGModel.isEmpty)
        {
            ALICEVISION_LOG_WARNING("No distortion model available for lens chromatic aberration correction.");
        }

        if (pParams.lensCorrection.geometry && cam != NULL && cam->hasDistortion())
        {
            const image::RGBAfColor FBLACK_A(.0f, .0f, .0f, 1.0f);
            image::Image<image::RGBAfColor> image_ud;
            const image::Sampler2d<image::SamplerLinear> sampler;

            image_ud.resize(image.width(), image.height(), true, FBLACK_A);

            camera::UndistortImage(image, cam.get(), image_ud, FBLACK_A);

            image = image_ud;
        }
        else if (pParams.lensCorrection.geometry && cam != NULL && !cam->hasDistortion())
        {
            ALICEVISION_LOG_WARNING("No distortion model available for lens geometry distortion correction.");
        }
        else if (pParams.lensCorrection.geometry && cam == NULL)
        {
            ALICEVISION_LOG_WARNING("No intrinsics data available for lens geometry distortion correction.");
        }
    }

    const float sfw =
      (pParams.maxWidth != 0 && pParams.maxWidth < image.width()) ? static_cast<float>(pParams.maxWidth) / static_cast<float>(image.width()) : 1.0;
    const float sfh = (pParams.maxHeight != 0 && pParams.maxHeight < image.height())
                        ? static_cast<float>(pParams.maxHeight) / static_cast<float>(image.height())
                        : 1.0;
    const float scaleFactor = std::min(pParams.scaleFactor, std::min(sfw, sfh));

    if (scaleFactor != 1.0f || pParams.par.enabled)
    {
        const bool parRowDecimation = pParams.par.enabled && pParams.par.rowDecimation;
        const float widthRatio = scaleFactor * ((parRowDecimation || !pParams.par.enabled) ? 1.0 : pParams.par.value);
        const float heightRatio = scaleFactor * (parRowDecimation ? (1.0 / pParams.par.value) : 1.0);

        ALICEVISION_LOG_TRACE("widthRatio " << widthRatio);
        ALICEVISION_LOG_TRACE("heightRatio " << heightRatio);

        const unsigned int w = image.width();
        const unsigned int h = image.height();
        const unsigned int nw = static_cast<unsigned int>(floor(static_cast<float>(image.width()) * widthRatio));
        const unsigned int nh = static_cast<unsigned int>(floor(static_cast<float>(image.height()) * heightRatio));

        image::Image<image::RGBAfColor> rescaled(nw, nh);

        const oiio::ImageSpec imageSpecResized(nw, nh, nchannels, oiio::TypeDesc::FLOAT);
        const oiio::ImageSpec imageSpecOrigin(w, h, nchannels, oiio::TypeDesc::FLOAT);

        const oiio::ImageBuf inBuf(imageSpecOrigin, image.data());
        oiio::ImageBuf outBuf(imageSpecResized, rescaled.data());

        oiio::ImageBufAlgo::resize(outBuf, inBuf);

        image.swap(rescaled);
    }

    if ((pParams.reorient) && (imageMetadata.find("Orientation") != imageMetadata.end()))
    {
        oiio::ImageBuf inBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        inBuf.set_orientation(std::stoi(imageMetadata["Orientation"]));
        oiio::ImageBuf outBuf = oiio::ImageBufAlgo::reorient(inBuf);

        if (outBuf.spec().get_int_attribute("orientation") != inBuf.spec().get_int_attribute("orientation"))
        {
            imageMetadata.at("Orientation") = std::to_string(outBuf.spec().get_int_attribute("orientation"));

            image::Image<image::RGBAfColor> reoriented(outBuf.spec().width, outBuf.spec().height);

            oiio::ROI exportROI = outBuf.roi();
            exportROI.chbegin = 0;
            exportROI.chend = nchannels;
            outBuf.get_pixels(exportROI, oiio::TypeDesc::FLOAT, reoriented.data());

            image.swap(reoriented);
        }
    }

    if (pParams.contrast != 1.0f)
    {
        image::Image<image::RGBAfColor> filtered(image.width(), image.height());
        const oiio::ImageBuf inBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        oiio::ImageBuf outBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), filtered.data());
        oiio::ImageBufAlgo::contrast_remap(outBuf, inBuf, 0.0f, 1.0f, 0.0f, 1.0f, pParams.contrast);

        image.swap(filtered);
    }
    if (pParams.medianFilter >= 3)
    {
        image::Image<image::RGBAfColor> filtered(image.width(), image.height());
        const oiio::ImageBuf inBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        oiio::ImageBuf outBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), filtered.data());
        oiio::ImageBufAlgo::median_filter(outBuf, inBuf, pParams.medianFilter);

        image.swap(filtered);
    }
    if (pParams.sharpen.enabled)
    {
        image::Image<image::RGBAfColor> filtered(image.width(), image.height());
        const oiio::ImageBuf inBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        oiio::ImageBuf outBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), filtered.data());
        oiio::ImageBufAlgo::unsharp_mask(outBuf, inBuf, "gaussian", pParams.sharpen.width, pParams.sharpen.contrast, pParams.sharpen.threshold);

        image.swap(filtered);
    }

    if (pParams.bilateralFilter.enabled)
    {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
        // Create temporary OpenCV Mat (keep only 3 Channels) to handled Eigen data of our image
        cv::Mat openCVMatIn = image::imageRGBAToCvMatBGR(image, CV_32FC3);
        cv::Mat openCVMatOut(image.width(), image.height(), CV_32FC3);

        cv::bilateralFilter(
          openCVMatIn, openCVMatOut, pParams.bilateralFilter.distance, pParams.bilateralFilter.sigmaColor, pParams.bilateralFilter.sigmaSpace);

        // Copy filtered data from openCV Mat(3 channels) to our image(keep the alpha channel unfiltered)
        image::cvMatBGRToImageRGBA(openCVMatOut, image);

#else
        throw std::invalid_argument("Unsupported mode! If you intended to use a bilateral filter, please add OpenCV support.");
#endif
    }

    // Contrast Limited Adaptive Histogram Equalization
    if (pParams.claheFilter.enabled)
    {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
        // Convert alicevision::image to BGR openCV Mat
        cv::Mat BGRMat = image::imageRGBAToCvMatBGR(image);

        // Convert BGR format to Lab format
        cv::Mat labImg;
        cv::cvtColor(BGRMat, labImg, cv::COLOR_LBGR2Lab);

        // Extract the L channel
        cv::Mat L;
        cv::extractChannel(labImg, L, 0);

        // normalise L channel from [0, 100] to [0, 1]
        std::for_each(L.begin<float>(), L.end<float>(), [](float& pixel) { pixel /= 100.0; });

        // Convert float image to 16bit
        L.convertTo(L, CV_16U, 65535.0);

        // apply Clahe algorithm to the L channel
        {
            const cv::Ptr<cv::CLAHE> clahe =
              cv::createCLAHE(pParams.claheFilter.clipLimit, cv::Size(pParams.claheFilter.tileGridSize, pParams.claheFilter.tileGridSize));
            clahe->apply(L, L);
        }

        // Convert 16bit image to float
        L.convertTo(L, CV_32F, 1.0 / 65535.0);

        // normalise back L channel from [0, 1] to [0, 100]
        std::for_each(L.begin<float>(), L.end<float>(), [](float& pixel) { pixel *= 100.0; });

        // Merge back Lab colors channels
        cv::insertChannel(L, labImg, 0);

        // Convert Lab format to BGR format
        cv::cvtColor(labImg, BGRMat, cv::COLOR_Lab2LBGR);

        // Copy filtered data from openCV Mat to our alicevision image(keep the alpha channel unfiltered)
        image::cvMatBGRToImageRGBA(BGRMat, image);
#else
        throw std::invalid_argument("Unsupported mode! If you intended to use a Clahe filter, please add OpenCV support.");
#endif
    }
    if (pParams.fillHoles)
    {
        image::Image<image::RGBAfColor> filtered(image.width(), image.height());
        oiio::ImageBuf inBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        oiio::ImageBuf outBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), filtered.data());

        // Premult necessary to ensure that the fill holes works as expected
        oiio::ImageBufAlgo::premult(inBuf, inBuf);
        oiio::ImageBufAlgo::fillholes_pushpull(outBuf, inBuf);

        image.swap(filtered);
    }

    if (pParams.noise.enabled)
    {
        oiio::ImageBuf inBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
        oiio::ImageBufAlgo::noise(inBuf, ENoiseMethod_enumToString(pParams.noise.method), pParams.noise.A, pParams.noise.B, pParams.noise.mono);
    }

    if (pParams.nlmFilter.enabled)
    {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
        // Create temporary OpenCV Mat (keep only 3 channels) to handle Eigen data of our image
        cv::Mat openCVMatIn = image::imageRGBAToCvMatBGR(image, CV_8UC3);
        cv::Mat openCVMatOut(image.width(), image.height(), CV_8UC3);

        cv::fastNlMeansDenoisingColored(openCVMatIn,
                                        openCVMatOut,
                                        pParams.nlmFilter.filterStrength,
                                        pParams.nlmFilter.filterStrengthColor,
                                        pParams.nlmFilter.templateWindowSize,
                                        pParams.nlmFilter.searchWindowSize);

        // Copy filtered data from OpenCV Mat(3 channels) to our image (keep the alpha channel unfiltered)
        image::cvMatBGRToImageRGBA(openCVMatOut, image);

#else
        throw std::invalid_argument("Unsupported mode! If you intended to use a non-local means filter, please add OpenCV support.");
#endif
    }

    if (pParams.applyDcpMetadata || (pParams.enableColorTempProcessing && pParams.correlatedColorTemperature <= 0.0))
    {
        bool dcpMetadataOK = map_has_non_empty_value(imageMetadata, "AliceVision:DCP:Temp1") &&
                             map_has_non_empty_value(imageMetadata, "AliceVision:DCP:Temp2") &&
                             map_has_non_empty_value(imageMetadata, "AliceVision:DCP:ForwardMatrixNumber") &&
                             map_has_non_empty_value(imageMetadata, "AliceVision:DCP:ColorMatrixNumber");

        int colorMatrixNb;
        int fwdMatrixNb;

        if (dcpMetadataOK)
        {
            colorMatrixNb = std::stoi(imageMetadata.at("AliceVision:DCP:ColorMatrixNumber"));
            fwdMatrixNb = std::stoi(imageMetadata.at("AliceVision:DCP:ForwardMatrixNumber"));

            ALICEVISION_LOG_INFO("Matrix Number : " << colorMatrixNb << " ; " << fwdMatrixNb);

            dcpMetadataOK = !((colorMatrixNb == 0) || ((colorMatrixNb > 0) && !map_has_non_empty_value(imageMetadata, "AliceVision:DCP:ColorMat1")) ||
                              ((colorMatrixNb > 1) && !map_has_non_empty_value(imageMetadata, "AliceVision:DCP:ColorMat2")) ||
                              ((fwdMatrixNb > 0) && !map_has_non_empty_value(imageMetadata, "AliceVision:DCP:ForwardMat1")) ||
                              ((fwdMatrixNb > 1) && !map_has_non_empty_value(imageMetadata, "AliceVision:DCP:ForwardMat2")));
        }

        if (!dcpMetadataOK)
        {
            ALICEVISION_THROW_ERROR("Image Processing: All required DCP metadata cannot be found.\n" << imageMetadata);
        }

        image::DCPProfile dcpProf;

        dcpProf.info.temperature_1 = std::stof(imageMetadata.at("AliceVision:DCP:Temp1"));
        dcpProf.info.temperature_2 = std::stof(imageMetadata.at("AliceVision:DCP:Temp2"));
        dcpProf.info.has_color_matrix_1 = colorMatrixNb > 0;
        dcpProf.info.has_color_matrix_2 = colorMatrixNb > 1;
        dcpProf.info.has_forward_matrix_1 = fwdMatrixNb > 0;
        dcpProf.info.has_forward_matrix_2 = fwdMatrixNb > 1;

        std::vector<std::string> v_str;

        v_str.push_back(imageMetadata.at("AliceVision:DCP:ColorMat1"));
        if (colorMatrixNb > 1)
        {
            v_str.push_back(imageMetadata.at("AliceVision:DCP:ColorMat2"));
        }
        dcpProf.setMatricesFromStrings("color", v_str);

        v_str.clear();
        if (fwdMatrixNb > 0)
        {
            v_str.push_back(imageMetadata.at("AliceVision:DCP:ForwardMat1"));
            if (fwdMatrixNb > 1)
            {
                v_str.push_back(imageMetadata.at("AliceVision:DCP:ForwardMat2"));
            }
            dcpProf.setMatricesFromStrings("forward", v_str);
        }

        std::string cam_mul =
          map_has_non_empty_value(imageMetadata, "raw:cam_mul") ? imageMetadata.at("raw:cam_mul") : imageMetadata.at("AliceVision:raw:cam_mul");
        std::vector<float> v_mult;
        size_t last = 0;
        size_t next = 1;
        while ((next = cam_mul.find(",", last)) != std::string::npos)
        {
            v_mult.push_back(std::stof(cam_mul.substr(last, next - last)));
            last = next + 1;
        }
        v_mult.push_back(std::stof(cam_mul.substr(last, cam_mul.find("}", last) - last)));

        image::DCPProfile::Triple neutral;
        for (int i = 0; i < 3; i++)
        {
            neutral[i] = v_mult[i] / v_mult[1];
        }

        double cct = pParams.correlatedColorTemperature;
        double tint;

        if (pParams.enableColorTempProcessing)
        {
            dcpProf.getColorTemperatureAndTintFromNeutral(neutral, cct, tint);
        }

        if (pParams.applyDcpMetadata)
        {
            dcpProf.applyLinear(image, neutral, cct, true, pParams.useDCPColorMatrixOnly);
        }

        imageMetadata["AliceVision:ColorTemperature"] = std::to_string(cct);
    }
    else if (pParams.enableColorTempProcessing && pParams.correlatedColorTemperature > 0.0)
    {
        imageMetadata["AliceVision:ColorTemperature"] = std::to_string(pParams.correlatedColorTemperature);
    }
}

void saveImage(image::Image<image::RGBAfColor>& image,
               const std::string& inputPath,
               const std::string& outputPath,
               std::map<std::string, std::string> inputMetadata,
               const std::vector<std::string>& metadataFolders,
               const EImageFormat outputFormat,
               const image::ImageWriteOptions options)
{
    // Read metadata path
    std::string metadataFilePath;

    const std::string filename = fs::path(inputPath).filename().string();
    const std::string outExtension = boost::to_lower_copy(fs::path(outputPath).extension().string());
    const bool isEXR = (outExtension == ".exr");
    oiio::ParamValueList metadata;

    if (!inputMetadata.empty())  // If metadata are provided as input
    {
        // metadata name in "raw" domain must be updated otherwise values are discarded by oiio when writing exr format
        // we want to propagate them so we replace the domain "raw" with "AliceVision:raw"
        for (const auto& meta : inputMetadata)
        {
            if (meta.first.compare(0, 3, "raw") == 0)
            {
                metadata.add_or_replace(oiio::ParamValue("AliceVision:" + meta.first, meta.second));
            }
            else
            {
                metadata.add_or_replace(oiio::ParamValue(meta.first, meta.second));
            }
        }
    }
    else if (!metadataFolders.empty())  // If metadataFolders is specified
    {
        // The file must match the file name and extension to be used as a metadata replacement.
        const std::vector<std::string> metadataFilePaths =
          utils::getFilesPathsFromFolders(metadataFolders, [&filename](const fs::path& path) { return path.filename().string() == filename; });

        if (metadataFilePaths.size() > 1)
        {
            ALICEVISION_LOG_ERROR("Ambiguous case: Multiple path corresponding to this file was found for metadata replacement.");
            throw std::invalid_argument("Ambiguous case: Multiple path corresponding to this file was found for metadata replacement");
        }

        if (metadataFilePaths.empty())
        {
            ALICEVISION_LOG_WARNING("Metadata folders was specified but there is no matching for this image: "
                                    << filename << ". The default metadata will be used instead for this image.");
            metadataFilePath = inputPath;
        }
        else
        {
            ALICEVISION_LOG_TRACE("Metadata path found for the current image: " << filename);
            metadataFilePath = metadataFilePaths[0];
        }
        metadata = image::readImageMetadata(metadataFilePath);
    }
    else
    {
        // Metadata are extracted from the original images
        metadata = image::readImageMetadata(inputPath);
    }

    // Save image
    ALICEVISION_LOG_TRACE("Export image: '" << outputPath << "'.");

    if (outputFormat == EImageFormat::Grayscale)
    {
        image::Image<float> outputImage;
        image::ConvertPixelType(image, &outputImage);
        image::writeImage(outputPath, outputImage, options, metadata);
    }
    else if (outputFormat == EImageFormat::RGB)
    {
        image::Image<image::RGBfColor> outputImage;
        image::ConvertPixelType(image, &outputImage);
        image::writeImage(outputPath, outputImage, options, metadata);
    }
    else
    {
        // Already in RGBAf
        image::writeImage(outputPath, image, options, metadata);
    }
}

int aliceVision_main(int argc, char* argv[])
{
    std::string inputExpression;
    std::vector<std::string> inputFolders;
    std::vector<std::string> metadataFolders;
    std::string outputPath;
    EImageFormat outputFormat = EImageFormat::RGBA;
    image::EImageColorSpace inputColorSpace = image::EImageColorSpace::AUTO;
    image::EImageColorSpace workingColorSpace = image::EImageColorSpace::LINEAR;
    image::EImageColorSpace outputColorSpace = image::EImageColorSpace::LINEAR;
    image::EStorageDataType storageDataType = image::EStorageDataType::Float;
    image::EImageExrCompression exrCompressionMethod = image::EImageExrCompression::Auto;
    int exrCompressionLevel = 0;
    bool jpegCompress = true;
    int jpegQuality = 90;
    std::string extension;
    image::ERawColorInterpretation rawColorInterpretation = image::ERawColorInterpretation::DcpLinearProcessing;
    std::string colorProfileDatabaseDirPath = "";
    bool errorOnMissingColorProfile = true;
    bool useDCPColorMatrixOnly = true;
    bool doWBAfterDemosaicing = false;
    std::string demosaicingAlgo = "AHD";
    int highlightMode = 0;
    double correlatedColorTemperature = -1;
    std::string lensCorrectionProfileInfo;
    bool lensCorrectionProfileSearchIgnoreCameraModel = true;
    std::string sensorDatabasePath;

    ProcessingParams pParams;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputExpression)->default_value(inputExpression),
         "SfMData file input, image filenames or regex(es) on the image file path (supported regex: '#' matches a "
         "single digit, '@' one or more digits, '?' one character and '*' zero or more).")
        ("inputFolders", po::value<std::vector<std::string>>(&inputFolders)->multitoken(),
         "Use images from specific folder(s) instead of those specify in the SfMData file.")
        ("output,o", po::value<std::string>(&outputPath)->required(),
         "Output folder or output image if a single image is given as input.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("metadataFolders", po::value<std::vector<std::string>>(&metadataFolders)->multitoken(),
         "Use images metadata from specific folder(s) instead of those specified in the input images.")

        ("keepImageFilename", po::value<bool>(&pParams.keepImageFilename)->default_value(pParams.keepImageFilename),
         "Use original image names instead of view names when saving.")

        ("reconstructedViewsOnly", po::value<bool>(&pParams.reconstructedViewsOnly)->default_value(pParams.reconstructedViewsOnly),
         "Process only recontructed views or all views.")

        ("fixNonFinite", po::value<bool>(&pParams.fixNonFinite)->default_value(pParams.fixNonFinite),
         "Fill non-finite pixels.")

        ("scaleFactor", po::value<float>(&pParams.scaleFactor)->default_value(pParams.scaleFactor),
         "Scale Factor (1.0: no change).")

        ("maxWidth", po::value<unsigned int>(&pParams.maxWidth)->default_value(pParams.maxWidth),
         "Max width (0: no change).")

        ("maxHeight", po::value<unsigned int>(&pParams.maxHeight)->default_value(pParams.maxHeight),
         "Max height (0: no change).")

        ("exposureCompensation", po::value<bool>(&pParams.exposureCompensation)->default_value(pParams.exposureCompensation),
         "Exposure Compensation. Valid only if a sfmdata is set as input.")

        ("rawExposureAdjust", po::value<float>(&pParams.rawExposureAdjust)->default_value(pParams.rawExposureAdjust),
         "Exposure Adjustment in fstops limited to the range from -2 to +3 fstops.")

        ("rawAutoBright", po::value<bool>(&pParams.rawAutoBright)->default_value(pParams.rawAutoBright),
         "Enable automatic exposure adjustment for RAW images.")

        ("lensCorrection", po::value<LensCorrectionParams>(&pParams.lensCorrection)->default_value(pParams.lensCorrection),
         "Lens Correction parameters:\n"
         " * Enabled: Use automatic lens correction.\n"
         " * Geometry: For geometry if a model is available in SfM data.\n"
         " * Vignetting: For vignetting if model parameters is available in metadata.\n "
         " * Chromatic Aberration: For chromatic aberration (fringing) if model parameters is available in metadata.")

        ("contrast", po::value<float>(&pParams.contrast)->default_value(pParams.contrast),
         "Contrast Factor (1.0: no change).")

        ("medianFilter", po::value<int>(&pParams.medianFilter)->default_value(pParams.medianFilter),
         "Median Filter (0: no filter).")

        ("sharpenFilter", po::value<SharpenParams>(&pParams.sharpen)->default_value(pParams.sharpen),
         "Sharpen Filter parameters:\n"
         " * Enabled: Use Sharpen.\n"
         " * Width: Sharpen kernel width.\n"
         " * Contrast: Sharpen contrast value.\n "
         " * Threshold: Threshold for minimal variation for contrast to avoid sharpening of small noise (0.0: no noise threshold).")

        ("fillHoles", po::value<bool>(&pParams.fillHoles)->default_value(pParams.fillHoles),
         "Fill Holes.")

        ("bilateralFilter", po::value<BilateralFilterParams>(&pParams.bilateralFilter)->default_value(pParams.bilateralFilter),
         "Bilateral Filter parameters:\n"
         " * Enabled: Use bilateral filter.\n"
         " * Distance: Diameter of each pixel neighborhood that is used during filtering (if <= 0, it is computed "
         "proportionally from sigmaSpace).\n"
         " * SigmaSpace: Filter sigma in the coordinate space.\n "
         " * SigmaColor: Filter sigma in the color space.")

        ("claheFilter", po::value<ClaheFilterParams>(&pParams.claheFilter)->default_value(pParams.claheFilter),
         "Sharpen Filter parameters:\n"
         " * Enabled: Use Contrast Limited Adaptive Histogram Equalization (CLAHE).\n"
         " * ClipLimit: Sets threshold for contrast limiting.\n"
         " * TileGridSize: Sets size of grid for histogram equalization. Input image will be divided into equally "
         "sized rectangular tiles.")

        ("noiseFilter", po::value<NoiseFilterParams>(&pParams.noise)->default_value(pParams.noise),
         "Noise Filter parameters:\n"
         " * Enabled: Add noise.\n"
         " * method: There are several noise types to choose from:\n"
         "    - uniform: adds noise values uninformly distributed on range [A,B).\n"
         "    - gaussian: adds Gaussian (normal distribution) noise values with mean value A and standard deviation B.\n"
         "    - salt: changes to value A a portion of pixels given by B.\n"
         " * A, B: parameters that have a different interpretation depending on the method chosen.\n"
         " * mono: If is true, a single noise value will be applied to all channels otherwise a separate noise value "
         "will be computed for each channel.")

        ("nlmFilter", po::value<NLMeansFilterParams>(&pParams.nlmFilter)->default_value(pParams.nlmFilter),
         "Non-local means filter parameters:\n"
         " * Enabled: Use non-local means filter.\n"
         " * H: Parameter regulating filter strength. Bigger H value perfectly removes noise but also removes image "
         "details, smaller H value preserves details but also preserves some noise.\n"
         " * HColor: Parameter regulating filter strength for color images only. Normally same as Filtering "
         "Parameter H. Not necessary for grayscale images.\n "
         " * templateWindowSize: Size in pixels of the template patch that is used to compute weights. Should be odd.\n"
         " * searchWindowSize: Size in pixels of the window that is used to compute weighted average for a given pixel. "
         "Should be odd. Affects performance linearly: greater searchWindowsSize - greater denoising time.")

        ("parFilter", po::value<pixelAspectRatioParams>(&pParams.par)->default_value(pParams.par),
         "Pixel Aspect Ratio parameters:\n"
         " * Enabled: Apply pixel aspect ratio.\n"
         " * RowDecimation: Decimate rows (reduce image height) instead of upsampling columns (increase image width).")

        ("inputColorSpace", po::value<image::EImageColorSpace>(&inputColorSpace)->default_value(inputColorSpace),
         ("Input image color space: " + image::EImageColorSpace_informations()).c_str())

        ("workingColorSpace", po::value<image::EImageColorSpace>(&workingColorSpace)->default_value(workingColorSpace),
         ("Working color space: " + image::EImageColorSpace_informations()).c_str())

        ("outputFormat", po::value<EImageFormat>(&outputFormat)->default_value(outputFormat),
         "Output image format (rgba, rgb, grayscale).")

        ("outputColorSpace", po::value<image::EImageColorSpace>(&outputColorSpace)->default_value(outputColorSpace),
         ("Output color space: " + image::EImageColorSpace_informations()).c_str())

        ("rawColorInterpretation", po::value<image::ERawColorInterpretation>(&rawColorInterpretation)->default_value(rawColorInterpretation),
         ("RAW color interpretation: " + image::ERawColorInterpretation_informations() + "\n"
         "Default: DcpLinearProcessing").c_str())

        ("applyDcpMetadata", po::value<bool>(&pParams.applyDcpMetadata)->default_value(pParams.applyDcpMetadata),
         "Apply after all processings a linear DCP profile generated from the image DCP metadata if any.")

        ("colorProfileDatabase,c", po::value<std::string>(&colorProfileDatabaseDirPath)->default_value(""),
         "DNG Color Profiles (DCP) database path.")

        ("errorOnMissingColorProfile", po::value<bool>(&errorOnMissingColorProfile)->default_value(errorOnMissingColorProfile),
         "Rise an error if a DCP color profiles database is specified but no DCP file matches with the camera model "
         "(maker + name) extracted from metadata (only for RAW images).")

        ("useDCPColorMatrixOnly", po::value<bool>(&useDCPColorMatrixOnly)->default_value(useDCPColorMatrixOnly),
         "Use only color matrices of DCP profile, ignoring forward matrices if any. Default: False.\n"
         "In case white balancing has been done before demosaicing, the reverse operation is done before applying "
         "the color matrix.")

        ("doWBAfterDemosaicing", po::value<bool>(&doWBAfterDemosaicing)->default_value(doWBAfterDemosaicing),
         "Do not use libRaw white balancing. White balancing is applied just before DCP profile if "
         "useDCPColorMatrixOnly is set to False. Default: False.")

        ("demosaicingAlgo", po::value<std::string>(&demosaicingAlgo)->default_value(demosaicingAlgo),
         "Demosaicing algorithm (see libRaw documentation).\n"
         "Possible algos are: linear, VNG, PPG, AHD (default), DCB, AHD-Mod, AFD, VCD, Mixed, LMMSE, AMaZE, DHT, AAHD, none.")

        ("highlightMode", po::value<int>(&highlightMode)->default_value(highlightMode),
         "Highlight management (see libRaw documentation).\n"
         "0 = clip (default), 1 = unclip, 2 = blend, 3+ = rebuild.")
            
        ("lensCorrectionProfileInfo", po::value<std::string>(&lensCorrectionProfileInfo)->default_value(""),
         "Lens Correction Profile filepath or database directory path.")
            
        ("lensCorrectionProfileSearchIgnoreCameraModel", po::value<bool>(&lensCorrectionProfileSearchIgnoreCameraModel)->default_value(lensCorrectionProfileSearchIgnoreCameraModel),
         "Automatic LCP Search considers only the camera maker and the lens name.")

        ("correlatedColorTemperature", po::value<double>(&correlatedColorTemperature)->default_value(correlatedColorTemperature),
         "Correlated Color Temperature in Kelvin of scene illuminant.\n"
         "If less than or equal to 0.0, the value extracted from the metadata will be used.")

        ("sensorDatabase,s", po::value<std::string>(&sensorDatabasePath)->default_value(""),
         "Camera sensor width database path.")

        ("reorient", po::value<bool>(&pParams.reorient)->default_value(pParams.reorient),
         "Enable automatic reorientation of images.")

        ("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType),
         ("Storage data type: " + image::EStorageDataType_informations()).c_str())

        ("exrCompressionMethod", po::value<image::EImageExrCompression>(&exrCompressionMethod)->default_value(exrCompressionMethod),
         ("Compression method for EXR images: " + image::EImageExrCompression_informations()).c_str())

        ("exrCompressionLevel", po::value<int>(&exrCompressionLevel)->default_value(exrCompressionLevel),
         "Compression Level for EXR images.\n"
         "Only dwaa, dwab, zip and zips compression methods are concerned.\n"
         "dwaa/dwab: value must be strictly positive.\n"
         "zip/zips: value must be between 1 and 9.")
        
        ("jpegCompress", po::value<bool>(&jpegCompress)->default_value(jpegCompress),
         "Compress JPEG images.")
        
        ("jpegQuality", po::value<int>(&jpegQuality)->default_value(jpegQuality),
         "JPEG quality after compression (between 0 and 100).")

        ("extension", po::value<std::string>(&extension)->default_value(extension),
         "Output image extension (like exr, or empty to keep the source file format.");
    // clang-format on

    CmdLine cmdline("AliceVision imageProcessing");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // check user choose at least one input option
    if (inputExpression.empty() && inputFolders.empty())
    {
        ALICEVISION_LOG_ERROR("Program need at least --input or --inputFolders option." << std::endl << "No input images here.");
        return EXIT_FAILURE;
    }

#if !ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
    if (pParams.bilateralFilter.enabled || pParams.claheFilter.enabled || pParams.nlmFilter.enabled)
    {
        ALICEVISION_LOG_ERROR("Invalid option: BilateralFilter, claheFilter and nlmFilter can't be used without openCV !");
        return EXIT_FAILURE;
    }
#endif

    if (pParams.scaleFactor < 0.0001f || pParams.scaleFactor > 1.0f)
    {
        ALICEVISION_LOG_ERROR("Invalid scale factor, it should be in range [0.0001, 1].");
        return EXIT_FAILURE;
    }

    // Check if sfmInputDataFilename exist and is recognized as sfm data file
    const std::string inputExt = boost::to_lower_copy(fs::path(inputExpression).extension().string());
    static const std::array<std::string, 2> sfmSupportedExtensions = {".sfm", ".abc"};
    if (!inputExpression.empty() && std::find(sfmSupportedExtensions.begin(), sfmSupportedExtensions.end(), inputExt) != sfmSupportedExtensions.end())
    {
        sfmData::SfMData sfmData;
        if (!sfmDataIO::load(sfmData, inputExpression, sfmDataIO::ALL))
        {
            ALICEVISION_LOG_ERROR("The input SfMData file '" << inputExpression << "' cannot be read.");
            return EXIT_FAILURE;
        }

        // Map used to store paths of the views that need to be processed
        std::unordered_map<IndexT, std::string> ViewPaths;

        const bool checkInputFolders = !inputFolders.empty();
        // Iterate over all views
        for (const auto& viewIt : sfmData.getViews())
        {
            const sfmData::View& view = *(viewIt.second);
            // Only valid views if needed
            if (pParams.reconstructedViewsOnly && !sfmData.isPoseAndIntrinsicDefined(&view))
            {
                continue;
            }
            // if inputFolders are used
            if (checkInputFolders)
            {
                const std::vector<std::string> foundViewPaths = sfmDataIO::viewPathsFromFolders(view, inputFolders);

                // Checks if a file associated with a given view is found in the inputfolders
                if (foundViewPaths.empty())
                {
                    ALICEVISION_LOG_ERROR("Some views from SfmData cannot be found in folders passed in the parameters.\n"
                                          << "Use only SfmData input, use reconstructedViewsOnly or specify the correct inputFolders.");
                    return EXIT_FAILURE;
                }
                else if (foundViewPaths.size() > 1)
                {
                    ALICEVISION_LOG_ERROR("Ambiguous case: Multiple paths found in input folders for the corresponding view '"
                                          << view.getViewId() << "'.\n"
                                          << "Make sure that only one match can be found in input folders.");
                    return EXIT_FAILURE;
                }

                // Add to ViewPaths the new associated path
                ALICEVISION_LOG_TRACE("New path found for the view " << view.getViewId() << " '" << foundViewPaths[0] << "'");
                ViewPaths.insert({view.getViewId(), foundViewPaths[0]});
            }
            else
            {
                // Otherwise use the existing path
                ViewPaths.insert({view.getViewId(), view.getImage().getImagePath()});
            }
        }

        const int size = ViewPaths.size();
        int i = 0;

        for (auto& viewIt : ViewPaths)
        {
            const IndexT viewId = viewIt.first;
            const std::string viewPath = viewIt.second;
            sfmData::View& view = sfmData.getView(viewId);

            const bool isRAW = image::isRawFormat(viewPath);

            const fs::path fsPath = viewPath;
            const std::string fileName = fsPath.stem().string();
            const std::string fileExt = fsPath.extension().string();
            const std::string outputExt = extension.empty() ? (isRAW ? ".exr" : fileExt) : (std::string(".") + extension);
            const std::string outputfilePath =
              (fs::path(outputPath) / ((pParams.keepImageFilename ? fileName : std::to_string(viewId)) + outputExt)).generic_string();

            ALICEVISION_LOG_INFO(++i << "/" << size << " - Process view '" << viewId << "'.");

            auto metadata = view.getImage().getMetadata();

            if (pParams.applyDcpMetadata && metadata["AliceVision:ColorSpace"] != "no_conversion")
            {
                ALICEVISION_LOG_WARNING("A dcp profile will be applied on an image containing non raw data!");
            }

            image::ImageReadOptions options;
            options.workingColorSpace = pParams.applyDcpMetadata ? image::EImageColorSpace::NO_CONVERSION : workingColorSpace;

            if (isRAW)
            {
                if (rawColorInterpretation == image::ERawColorInterpretation::Auto)
                {
                    options.rawColorInterpretation = image::ERawColorInterpretation_stringToEnum(view.getImage().getRawColorInterpretation());
                    if (options.rawColorInterpretation == image::ERawColorInterpretation::DcpMetadata)
                    {
                        options.useDCPColorMatrixOnly = false;
                        options.doWBAfterDemosaicing = true;
                    }
                    else
                    {
                        options.useDCPColorMatrixOnly = useDCPColorMatrixOnly;
                        options.doWBAfterDemosaicing = doWBAfterDemosaicing;
                    }
                }
                else
                {
                    options.rawColorInterpretation = rawColorInterpretation;
                    options.useDCPColorMatrixOnly = useDCPColorMatrixOnly;
                    options.doWBAfterDemosaicing = doWBAfterDemosaicing;
                }
                options.colorProfileFileName = view.getImage().getColorProfileFileName();
                options.demosaicingAlgo = demosaicingAlgo;
                options.highlightMode = highlightMode;
                options.rawExposureAdjustment = std::pow(2.f, pParams.rawExposureAdjust);
                options.rawAutoBright = pParams.rawAutoBright;
                options.correlatedColorTemperature = correlatedColorTemperature;
                pParams.correlatedColorTemperature = correlatedColorTemperature;
                pParams.enableColorTempProcessing = options.rawColorInterpretation == image::ERawColorInterpretation::DcpLinearProcessing;
            }
            else
            {
                options.inputColorSpace = inputColorSpace;
            }

            if (pParams.lensCorrection.enabled && pParams.lensCorrection.vignetting)
            {
                if (!view.getImage().getVignettingParams(pParams.lensCorrection.vParams))
                {
                    pParams.lensCorrection.vParams.clear();
                }
            }

            if (pParams.lensCorrection.enabled && pParams.lensCorrection.chromaticAberration)
            {
                std::vector<float> caGParams, caBGParams, caRGParams;
                view.getImage().getChromaticAberrationParams(caGParams, caBGParams, caRGParams);

                pParams.lensCorrection.caGModel.init3(caGParams);
                pParams.lensCorrection.caBGModel.init3(caBGParams);
                pParams.lensCorrection.caRGModel.init3(caRGParams);

                if (pParams.lensCorrection.caGModel.FocalLengthX == 0.0)
                {
                    float sensorWidth = view.getImage().getSensorWidth();
                    pParams.lensCorrection.caGModel.FocalLengthX = view.getImage().getWidth() * view.getImage().getMetadataFocalLength() /
                                                                   sensorWidth / std::max(view.getImage().getWidth(), view.getImage().getHeight());
                }
                if (pParams.lensCorrection.caGModel.FocalLengthY == 0.0)
                {
                    float sensorHeight = view.getImage().getSensorHeight();
                    pParams.lensCorrection.caGModel.FocalLengthY = view.getImage().getHeight() * view.getImage().getMetadataFocalLength() /
                                                                   sensorHeight / std::max(view.getImage().getWidth(), view.getImage().getHeight());
                }

                if ((pParams.lensCorrection.caGModel.FocalLengthX <= 0.0) || (pParams.lensCorrection.caGModel.FocalLengthY <= 0.0))
                {
                    pParams.lensCorrection.caGModel.reset();
                    pParams.lensCorrection.caBGModel.reset();
                    pParams.lensCorrection.caRGModel.reset();
                }
            }

            // Read original image
            image::Image<image::RGBAfColor> image;
            image::readImage(viewPath, image, options);

            // If exposureCompensation is needed for sfmData files
            if (pParams.exposureCompensation)
            {
                const double medianCameraExposure = sfmData.getMedianCameraExposureSetting().getExposure();
                const double cameraExposure = view.getImage().getCameraExposureSetting().getExposure();
                const double ev = std::log2(1.0 / cameraExposure);
                const float compensationFactor = static_cast<float>(medianCameraExposure / cameraExposure);

                ALICEVISION_LOG_INFO("View: " << viewId << ", Ev: " << ev << ", Ev compensation: " << compensationFactor);

                for (int i = 0; i < image.width() * image.height(); ++i)
                {
                    image(i)[0] *= compensationFactor;
                    image(i)[1] *= compensationFactor;
                    image(i)[2] *= compensationFactor;
                }
            }

            sfmData::Intrinsics::const_iterator iterIntrinsic = sfmData.getIntrinsics().find(view.getIntrinsicId());
            std::shared_ptr<camera::IntrinsicBase> cam = iterIntrinsic->second;

            std::map<std::string, std::string> viewMetadata = view.getImage().getMetadata();

            if (pParams.par.enabled)
            {
                pParams.par.value = cam->getParams()[1] / cam->getParams()[0];
            }

            // Image processing
            processImage(image, pParams, viewMetadata, cam);

            if (pParams.applyDcpMetadata)
            {
                workingColorSpace = image::EImageColorSpace::ACES2065_1;
            }

            image::ImageWriteOptions writeOptions;

            writeOptions.fromColorSpace(workingColorSpace);
            writeOptions.toColorSpace(outputColorSpace);
            writeOptions.exrCompressionMethod(exrCompressionMethod);
            writeOptions.exrCompressionLevel(exrCompressionLevel);
            writeOptions.jpegCompress(jpegCompress);
            writeOptions.jpegQuality(jpegQuality);

            if (boost::to_lower_copy(fs::path(outputPath).extension().string()) == ".exr")
            {
                // Select storage data type
                writeOptions.storageDataType(storageDataType);
            }

            // Save the image
            saveImage(image, viewPath, outputfilePath, viewMetadata, metadataFolders, outputFormat, writeOptions);

            // Update view for this modification
            view.getImage().setImagePath(outputfilePath);
            view.getImage().setWidth(image.width());
            view.getImage().setHeight(image.height());
            view.getImage().addMetadata("AliceVision:ColorSpace", image::EImageColorSpace_enumToString(outputColorSpace));
            if (viewMetadata.find("Orientation") != viewMetadata.end())
                view.getImage().addMetadata("Orientation", viewMetadata.at("Orientation"));

            if (pParams.reorient && image.width() != cam->w() && image.width() == cam->h())  // The image has been rotated by automatic reorientation
            {
                camera::IntrinsicBase* cam2 = cam->clone();

                cam2->setWidth(image.width());
                cam2->setHeight(image.height());
                double sensorWidth = cam->sensorWidth();
                cam2->setSensorWidth(cam->sensorHeight());
                cam2->setSensorHeight(sensorWidth);

                IndexT intrinsicId = cam2->hashValue();
                view.setIntrinsicId(intrinsicId);
                sfmData.getIntrinsics().emplace(intrinsicId, cam2);
            }
        }

        if ((pParams.scaleFactor != 1.0f) || (pParams.par.enabled && pParams.par.value != 1.0))
        {
            const bool parRowDecimation = pParams.par.enabled && pParams.par.rowDecimation;

            const float scaleFactorW = pParams.scaleFactor * ((!pParams.par.enabled || parRowDecimation) ? 1.0 : pParams.par.value);
            const float scaleFactorH = pParams.scaleFactor * (parRowDecimation ? (1.0 / pParams.par.value) : 1.0);
            for (auto& i : sfmData.getIntrinsics())
            {
                i.second->rescale(scaleFactorW, scaleFactorH);
            }
        }

        // Save sfmData with modified path to images
        const std::string sfmfilePath = (fs::path(outputPath) / fs::path(inputExpression).filename()).generic_string();
        if (!sfmDataIO::save(sfmData, sfmfilePath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
        {
            ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmfilePath << "' cannot be written.");
            return EXIT_FAILURE;
        }
    }
    else
    {
        const fs::path inputPath(inputExpression);
        std::vector<std::string> filesStrPaths;

        // If sfmInputDataFilename is empty use imageFolders instead
        if (inputExpression.empty())
        {
            // Get supported files
            filesStrPaths =
              utils::getFilesPathsFromFolders(inputFolders, [](const fs::path& path) { return image::isSupported(path.extension().string()); });
        }
        else
        {
            // If you try to use both a regex-like filter expression and folders as input
            if (!inputFolders.empty())
            {
                ALICEVISION_LOG_WARNING("InputFolders and filter expression cannot be used at the same time, InputFolders are ignored here.");
            }

            if (fs::is_regular_file(inputPath))
            {
                filesStrPaths.push_back(inputPath.string());
            }
            else
            {
                ALICEVISION_LOG_INFO("Working directory Path '" + inputPath.parent_path().generic_string() + "'.");

                const std::regex regex = utils::filterToRegex(inputExpression);
                // Get supported files in inputPath directory which matches our regex filter
                filesStrPaths = utils::getFilesPathsFromFolder(inputPath.parent_path().generic_string(), [&regex](const fs::path& path) {
                    return image::isSupported(path.extension().string()) && std::regex_match(path.generic_string(), regex);
                });
            }
        }

        const int size = filesStrPaths.size();

        if (!size)
        {
            ALICEVISION_LOG_ERROR("Any images was found.");
            ALICEVISION_LOG_ERROR("Input folders or input expression '" << inputExpression << "' may be incorrect ?");
            return EXIT_FAILURE;
        }
        else
        {
            ALICEVISION_LOG_INFO(size << " images found.");
        }

        image::DCPDatabase dcpDatabase;
        LCPdatabase lcpStore(lensCorrectionProfileInfo, lensCorrectionProfileSearchIgnoreCameraModel);

        // check sensor database
        std::vector<sensorDB::Datasheet> sensorDatabase;
        if (pParams.lensCorrection.enabled && (pParams.lensCorrection.geometry || pParams.lensCorrection.chromaticAberration))
        {
            if (sensorDatabasePath.empty())
            {
                const auto root = image::getAliceVisionRoot();
                if (root.empty())
                {
                    ALICEVISION_LOG_WARNING("ALICEVISION_ROOT is not defined, default sensor database cannot be accessed.");
                }
                else
                {
                    sensorDatabasePath = root + "/share/aliceVision/cameraSensors.db";
                }
            }
            if (!sensorDatabasePath.empty() && !sensorDB::parseDatabase(sensorDatabasePath, sensorDatabase))
            {
                ALICEVISION_LOG_ERROR("Invalid input sensor database '" << sensorDatabasePath << "', please specify a valid file.");
                return EXIT_FAILURE;
            }
        }

        int i = 0;
        for (const std::string& inputFilePath : filesStrPaths)
        {
            const bool isRAW = image::isRawFormat(inputFilePath);

            const fs::path path = fs::path(inputFilePath);
            const std::string filename = path.stem().string();
            const std::string fileExt = path.extension().string();
            const std::string outputExt = extension.empty() ? (isRAW ? ".exr" : fileExt) : (std::string(".") + extension);

            ALICEVISION_LOG_INFO(++i << "/" << size << " - Process image '" << filename << fileExt << "'.");

            const std::string userExt = fs::path(outputPath).extension().string();
            std::string outputFilePath;

            if ((size == 1) && !userExt.empty())
            {
                if (image::isSupported(userExt))
                {
                    outputFilePath = fs::path(outputPath).generic_string();
                }
                else
                {
                    outputFilePath = (fs::path(outputPath).parent_path() / (filename + outputExt)).generic_string();
                    ALICEVISION_LOG_WARNING("Extension " << userExt << " is not supported! Output image saved in " << outputFilePath);
                }
            }
            else
            {
                outputFilePath = (fs::path(outputPath) / (filename + outputExt)).generic_string();
            }

            image::DCPProfile dcpProf;
            sfmData::View view;  // used to extract and complete metadata
            view.getImage().setImagePath(inputFilePath);
            int width, height;
            const auto metadata = image::readImageMetadata(inputFilePath, width, height);
            view.getImage().setMetadata(image::getMapFromMetadata(metadata));
            view.getImage().setWidth(width);
            view.getImage().setHeight(height);
            std::shared_ptr<camera::IntrinsicBase> intrinsicBase;
            // Get DSLR maker and model in view metadata.
            const std::string& make = view.getImage().getMetadataMake();
            const std::string& model = view.getImage().getMetadataModel();

            if (isRAW && (rawColorInterpretation == image::ERawColorInterpretation::DcpLinearProcessing ||
                          rawColorInterpretation == image::ERawColorInterpretation::DcpMetadata))
            {
                // Load DCP color profiles database if not already loaded
                dcpDatabase.load(colorProfileDatabaseDirPath.empty() ? getColorProfileDatabaseFolder() : colorProfileDatabaseDirPath, false);

                // Get DCP profile
                if (!dcpDatabase.retrieveDcpForCamera(make, model, dcpProf))
                {
                    if (errorOnMissingColorProfile)
                    {
                        ALICEVISION_LOG_ERROR("The specified DCP database does not contain an appropriate profil for DSLR " << make << " " << model);
                        return EXIT_FAILURE;
                    }
                    else
                    {
                        ALICEVISION_LOG_WARNING("Can't find color profile for input image " << inputFilePath);
                    }
                }

                // Add color profile info in metadata
                view.getImage().addDCPMetadata(dcpProf);
            }

            if (isRAW && pParams.lensCorrection.enabled &&
                (pParams.lensCorrection.geometry || pParams.lensCorrection.vignetting || pParams.lensCorrection.chromaticAberration))
            {
                // try to find an appropriate Lens Correction Profile
                LCPinfo* lcpData = nullptr;
                if (lcpStore.size() == 1)
                {
                    lcpData = lcpStore.retrieveLCP();
                }
                else if (!lcpStore.empty())
                {
                    // Find an LCP file that matches the camera model and the lens model.
                    const std::string& lensModel = view.getImage().getMetadataLensModel();
                    const int lensID = view.getImage().getMetadataLensID();

                    if (!make.empty() && !lensModel.empty())
                    {
#pragma omp critical(lcp)
                        lcpData = lcpStore.findLCP(make, model, lensModel, lensID, 1);
                    }
                }

                if ((lcpData != nullptr) && !(lcpData->isEmpty()))
                {
                    double focalLengthmm = view.getImage().getMetadataFocalLength();
                    const float apertureValue = 2.f * std::log(view.getImage().getMetadataFNumber()) / std::log(2.0);
                    const float focusDistance = 0.f;

                    LensParam lensParam;
                    lcpData->getDistortionParams(focalLengthmm, focusDistance, lensParam);
                    lcpData->getVignettingParams(focalLengthmm, apertureValue, lensParam);
                    lcpData->getChromaticParams(focalLengthmm, focusDistance, lensParam);

                    // Get sensor size by combining information from sensor database and view's metadata
                    double sensorWidth = -1.0;
                    double sensorHeight = -1.0;
                    camera::EInitMode intrinsicInitMode = camera::EInitMode::UNKNOWN;
                    view.getImage().getSensorSize(sensorDatabase, sensorWidth, sensorHeight, focalLengthmm, intrinsicInitMode, true);

                    if (lensParam.hasVignetteParams() && !lensParam.vignParams.isEmpty && pParams.lensCorrection.vignetting)
                    {
                        float FocX = lensParam.vignParams.FocalLengthX != 0.0 ? lensParam.vignParams.FocalLengthX
                                                                              : width * focalLengthmm / sensorWidth / std::max(width, height);
                        float FocY = lensParam.vignParams.FocalLengthY != 0.0 ? lensParam.vignParams.FocalLengthY
                                                                              : height * focalLengthmm / sensorHeight / std::max(width, height);

                        pParams.lensCorrection.vParams.clear();

                        if (FocX == 0.0 || FocY == 0.0)
                        {
                            ALICEVISION_LOG_WARNING("Vignetting correction is requested but cannot be applied due to missing info.");
                        }
                        else
                        {
                            pParams.lensCorrection.vParams.push_back(FocX);
                            pParams.lensCorrection.vParams.push_back(FocY);
                            pParams.lensCorrection.vParams.push_back(lensParam.vignParams.ImageXCenter);
                            pParams.lensCorrection.vParams.push_back(lensParam.vignParams.ImageYCenter);
                            pParams.lensCorrection.vParams.push_back(lensParam.vignParams.VignetteModelParam1);
                            pParams.lensCorrection.vParams.push_back(lensParam.vignParams.VignetteModelParam2);
                            pParams.lensCorrection.vParams.push_back(lensParam.vignParams.VignetteModelParam3);
                        }
                    }

                    if (pParams.lensCorrection.chromaticAberration && lensParam.hasChromaticParams() && !lensParam.ChromaticGreenParams.isEmpty)
                    {
                        if (lensParam.ChromaticGreenParams.FocalLengthX == 0.0)
                        {
                            lensParam.ChromaticGreenParams.FocalLengthX = width * focalLengthmm / sensorWidth / std::max(width, height);
                        }
                        if (lensParam.ChromaticGreenParams.FocalLengthY == 0.0)
                        {
                            lensParam.ChromaticGreenParams.FocalLengthY = height * focalLengthmm / sensorHeight / std::max(width, height);
                        }

                        if (lensParam.ChromaticGreenParams.FocalLengthX == 0.0 || lensParam.ChromaticGreenParams.FocalLengthY == 0.0)
                        {
                            pParams.lensCorrection.caGModel.reset();
                            pParams.lensCorrection.caBGModel.reset();
                            pParams.lensCorrection.caRGModel.reset();

                            ALICEVISION_LOG_WARNING("Chromatic Aberration correction is requested but cannot be applied due to missing info.");
                        }
                        else
                        {
                            pParams.lensCorrection.caGModel = lensParam.ChromaticGreenParams;
                            pParams.lensCorrection.caBGModel = lensParam.ChromaticBlueGreenParams;
                            pParams.lensCorrection.caRGModel = lensParam.ChromaticRedGreenParams;
                        }
                    }

                    if (pParams.lensCorrection.geometry)
                    {
                        // build intrinsic
                        const camera::EINTRINSIC defaultCameraModel = camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL3;
                        const camera::EINTRINSIC allowedCameraModels = camera::EINTRINSIC_parseStringToBitmask("radial3,fisheye4");
                        const double defaultFocalLength = -1.0;
                        const double defaultFieldOfView = -1.0;
                        const double defaultFocalRatio = 1.0;
                        const double defaultOffsetX = 0.0;
                        const double defaultOffsetY = 0.0;
                        intrinsicBase = sfmDataIO::getViewIntrinsic(view,
                                                                    focalLengthmm,
                                                                    sensorWidth,
                                                                    defaultFocalLength,
                                                                    defaultFieldOfView,
                                                                    defaultFocalRatio,
                                                                    defaultOffsetX,
                                                                    defaultOffsetY,
                                                                    &lensParam,
                                                                    defaultCameraModel,
                                                                    allowedCameraModels);

                        pParams.lensCorrection.geometryModel = lensParam.perspParams;
                    }
                }
                else
                {
                    ALICEVISION_LOG_WARNING("No LCP file found for image " << inputFilePath);
                    ALICEVISION_LOG_WARNING("Requested lens correction(s) won't be applied");
                }
            }

            std::map<std::string, std::string> md = view.getImage().getMetadata();

            pParams.par.value = 1.0;
            if (pParams.par.enabled)
            {
                double pixelAspectRatio = 1.0;
                view.getImage().getDoubleMetadata({"PixelAspectRatio"}, pixelAspectRatio);
                pParams.par.value = pixelAspectRatio;
                md["PixelAspectRatio"] = "1.0";
            }

            // set readOptions
            image::ImageReadOptions readOptions;

            if (isRAW)
            {
                readOptions.colorProfileFileName = dcpProf.info.filename;
                if (dcpProf.info.filename.empty() && ((rawColorInterpretation == image::ERawColorInterpretation::DcpLinearProcessing) ||
                                                      (rawColorInterpretation == image::ERawColorInterpretation::DcpMetadata)))
                {
                    // Fallback case of missing profile but no error requested
                    readOptions.rawColorInterpretation = image::ERawColorInterpretation::LibRawWhiteBalancing;
                }
                else
                {
                    readOptions.rawColorInterpretation = rawColorInterpretation;
                }

                if (pParams.applyDcpMetadata && md["AliceVision::ColorSpace"] != "no_conversion")
                {
                    ALICEVISION_LOG_WARNING("A dcp profile will be applied on an image containing non raw data!");
                }

                readOptions.useDCPColorMatrixOnly = useDCPColorMatrixOnly;
                readOptions.doWBAfterDemosaicing = doWBAfterDemosaicing;
                readOptions.demosaicingAlgo = demosaicingAlgo;
                readOptions.highlightMode = highlightMode;
                readOptions.rawExposureAdjustment = std::pow(2.f, pParams.rawExposureAdjust);
                readOptions.rawAutoBright = pParams.rawAutoBright;
                readOptions.correlatedColorTemperature = correlatedColorTemperature;
                pParams.correlatedColorTemperature = correlatedColorTemperature;
                pParams.enableColorTempProcessing = readOptions.rawColorInterpretation == image::ERawColorInterpretation::DcpLinearProcessing;

                pParams.useDCPColorMatrixOnly = useDCPColorMatrixOnly;
                if (pParams.applyDcpMetadata)
                {
                    workingColorSpace = image::EImageColorSpace::ACES2065_1;
                }
            }
            else
            {
                readOptions.inputColorSpace = inputColorSpace;
            }

            readOptions.workingColorSpace = pParams.applyDcpMetadata ? image::EImageColorSpace::NO_CONVERSION : workingColorSpace;

            // Read original image
            image::Image<image::RGBAfColor> image;
            image::readImage(inputFilePath, image, readOptions);

            // Image processing
            processImage(image, pParams, md, intrinsicBase);

            image::ImageWriteOptions writeOptions;

            writeOptions.fromColorSpace(workingColorSpace);
            writeOptions.toColorSpace(outputColorSpace);
            writeOptions.exrCompressionMethod(exrCompressionMethod);
            writeOptions.exrCompressionLevel(exrCompressionLevel);

            if (boost::to_lower_copy(fs::path(outputPath).extension().string()) == ".exr")
            {
                // Select storage data type
                writeOptions.storageDataType(storageDataType);
            }

            // Save the image
            saveImage(image, inputFilePath, outputFilePath, md, metadataFolders, outputFormat, writeOptions);
        }
    }

    return 0;
}
