// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "KeyframeSelector.hpp"
#include <aliceVision/image/all.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/dataio/FeedProvider.hpp>
#include <boost/filesystem.hpp>

#include <random>
#include <tuple>
#include <cassert>
#include <cstdlib>
#include <iomanip>

#include <opencv2/optflow.hpp>

namespace fs = boost::filesystem;

namespace aliceVision
{
namespace keyframe
{

/**
 * @brief Get a random int in order to generate uid.
 * @warning The random don't use a repeatable seed to avoid conflicts between different launches on different data sets.
 * @return int between 0 and std::numeric_limits<int>::max()
 */
int getRandomInt()
{
    std::random_device rd;             // will be used to obtain a seed for the random number engine
    std::mt19937 randomTwEngine(rd()); // standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> randomDist(0, std::numeric_limits<int>::max());
    return randomDist(randomTwEngine);
}

double computeSharpness(const cv::Mat& grayscaleImage, const int window_size)
{
    cv::Mat sum, sumsq, laplacian;
    cv::Laplacian(grayscaleImage, laplacian, CV_64F);
    cv::integral(laplacian, sum, sumsq);

    double n = window_size * window_size;
    double maxstd = 0.0;
    for (int i = 0; i < sum.rows - window_size; i++)
    {
        for (int j = 0; j < sum.cols - window_size; j++)
        {
            double tl = sum.at<double>(i, j);
            double tr = sum.at<double>(i, j + window_size);
            double bl = sum.at<double>(i + window_size, j);
            double br = sum.at<double>(i + window_size, j + window_size);
            double s1 = br + tl - tr - bl;

            tl = sumsq.at<double>(i, j);
            tr = sumsq.at<double>(i, j + window_size);
            bl = sumsq.at<double>(i + window_size, j);
            br = sumsq.at<double>(i + window_size, j + window_size);
            double s2 = br + tl - tr - bl;

            double std_2 = sqrt((s2 - (s1 * s1) / n) / n);

            maxstd = std::max(maxstd, std_2);
        }
    }

    return maxstd;
}

cv::Mat readImage(dataio::FeedProvider & feed, size_t max_width)
{
    image::Image<image::RGBColor> image;
    camera::PinholeRadialK3 queryIntrinsics;
    bool hasIntrinsics = false;
    std::string currentImgName;

    if (!feed.readImage(image, queryIntrinsics, currentImgName, hasIntrinsics))
    {
        ALICEVISION_LOG_ERROR("Cannot read frame '" << currentImgName << "' !");
        throw std::invalid_argument("Cannot read frame '" + currentImgName + "' !");
    }

    //Convert content to opencv
    cv::Mat cvFrame(cv::Size(image.cols(), image.rows()), CV_8UC3, image.data(), image.cols() * 3);

    cv::Mat cvGrayscale;
    //Convert to grayscale
    cv::cvtColor(cvFrame, cvGrayscale, cv::COLOR_BGR2GRAY);

    //Resize to smaller size
    cv::Mat cvRescaled;
    if (cvGrayscale.cols > max_width)
    {
        cv::resize(cvGrayscale, cvRescaled, cv::Size(max_width, double(cvGrayscale.rows) * double(max_width) / double(cvGrayscale.cols)));
    }
    else
    {
        cvRescaled = cvGrayscale;
    }

    return cvRescaled;
}

double estimateFlow(const std::vector<std::unique_ptr<dataio::FeedProvider>> & feeds, size_t max_width, int previous, int current)
{
    auto ptrFlow = cv::optflow::createOptFlow_DeepFlow();

    double minmaxflow = std::numeric_limits<double>::max();

    for (auto& feed : feeds)
    {
        feed->goToFrame(previous);
        cv::Mat first = readImage(*feed, max_width);

        feed->goToFrame(current);
        cv::Mat second = readImage(*feed, max_width);

        cv::Mat flow;
        ptrFlow->calc(first, second, flow);

        cv::Mat sumflow;
        cv::integral(flow, sumflow, CV_64F);

        int ws = 20;
        double n = ws * ws;
        double maxflow = 0.0;
        size_t count = 0;

        cv::Mat matnorm(flow.size(), CV_32FC1);
        for (int i = 10; i < flow.rows - ws - 10; i++)
        {
            for (int j = 10; j < flow.cols - ws - 10; j++)
            {
                cv::Point2d tl = sumflow.at<cv::Point2d>(i, j);
                cv::Point2d tr = sumflow.at<cv::Point2d>(i, j + ws);
                cv::Point2d bl = sumflow.at<cv::Point2d>(i + ws, j);
                cv::Point2d br = sumflow.at<cv::Point2d>(i + ws, j + ws);
                cv::Point2d s1 = br + tl - tr - bl;

                cv::Point fl = flow.at<cv::Point2f>(i, j);

                double norm = std::hypot(s1.x, s1.y) / n;
                maxflow = std::max(maxflow, norm);
            }
        }

        minmaxflow = std::min(minmaxflow, maxflow);
    }

    return minmaxflow;
}

void KeyframeSelector::processSimple(const std::vector<std::string>& mediaPaths)
{
    std::size_t nbFrames = std::numeric_limits<std::size_t>::max();
    std::vector<std::unique_ptr<dataio::FeedProvider>> feeds;

    _selected.clear();

    for (std::size_t mediaIndex = 0; mediaIndex < mediaPaths.size(); ++mediaIndex)
    {
        const auto& path = mediaPaths.at(mediaIndex);

        // create a feed provider per mediaPaths
        feeds.emplace_back(new dataio::FeedProvider(path));

        const auto& feed = *feeds.back();

        // check if feed is initialized
        if (!feed.isInit())
        {
            ALICEVISION_LOG_ERROR("Cannot initialize the FeedProvider with " << path);
            throw std::invalid_argument("Cannot while initialize the FeedProvider with " + path);
        }

        // update minimum number of frames
        nbFrames = std::min(nbFrames, (size_t)feed.nbFrames());
    }

    // check if minimum number of frame is zero
    if (nbFrames == 0)
    {
        ALICEVISION_LOG_ERROR("One or multiple medias can't be found or empty !");
        throw std::invalid_argument("One or multiple medias can't be found or empty !");
    }

    int step = _minFrameStep;
    if (_maxOutFrame > 0)
    {
        step = int(std::floor(double(nbFrames) / double(_maxOutFrame)));
    }

    for (int id = 0; id < nbFrames; id += step)
    {
        _selected.push_back(id);
    }
}

void KeyframeSelector::processSmart(const std::vector<std::string> & mediaPaths)
{
    // create feeds and count minimum number of frames
    std::size_t nbFrames = std::numeric_limits<std::size_t>::max();
    std::vector<std::unique_ptr<dataio::FeedProvider>> feeds;
    const size_t processWidth = 720;
    const double thresholdSharpness = 10.0;
    const double thresholdFlow = 10.0;

    for(std::size_t mediaIndex = 0; mediaIndex < mediaPaths.size(); ++mediaIndex)
    {
        const auto& path = mediaPaths.at(mediaIndex);

        // create a feed provider per mediaPaths
        feeds.emplace_back(new dataio::FeedProvider(path));

        const auto& feed = *feeds.back();

        // check if feed is initialized
        if(!feed.isInit())
        {
            ALICEVISION_LOG_ERROR("Cannot initialize the FeedProvider with " << path);
            throw std::invalid_argument("Cannot while initialize the FeedProvider with " + path);
        }

        // update minimum number of frames
        nbFrames = std::min(nbFrames, (size_t)feed.nbFrames());
    }

    // check if minimum number of frame is zero
    if(nbFrames == 0)
    {
        ALICEVISION_LOG_ERROR("One or multiple medias can't be found or empty !");
        throw std::invalid_argument("One or multiple medias can't be found or empty !");
    }

    const int searchWindowSize = (_maxOutFrame <= 0) ? 1 : std::floor(double(nbFrames) / double(_maxOutFrame));

    // feed provider variables
    image::Image<image::RGBColor> image;     // original image
    camera::PinholeRadialK3 queryIntrinsics; // image associated camera intrinsics
    bool hasIntrinsics = false;              // true if queryIntrinsics is valid
    std::string currentImgName;              // current image name


    // feed and metadata initialization
    for (std::size_t mediaIndex = 0; mediaIndex < feeds.size(); ++mediaIndex)
    {
        // first frame with offset
        feeds.at(mediaIndex)->goToFrame(0);

        if (!feeds.at(mediaIndex)->readImage(image, queryIntrinsics, currentImgName, hasIntrinsics))
        {
            ALICEVISION_LOG_ERROR("Cannot read media first frame " << mediaPaths[mediaIndex]);
            throw std::invalid_argument("Cannot read media first frame " + mediaPaths[mediaIndex]);
        }
    }

    size_t currentFrame = 0;

    std::vector<double> sharpnessScores;

    while (currentFrame < nbFrames)
    {
        double minimalSharpness = std::numeric_limits<double>::max();

        for(std::size_t mediaIndex = 0; mediaIndex < feeds.size(); ++mediaIndex)
        {
            ALICEVISION_LOG_DEBUG("media : " << mediaPaths.at(mediaIndex));
            auto& feed = *feeds.at(mediaIndex);

            //Resize to smaller size
            cv::Mat cvRescaled = readImage(feed, processWidth);

            double score = computeSharpness(cvRescaled, 200);
            minimalSharpness = std::min(minimalSharpness, score);

            feed.goToNextFrame();
        }

        sharpnessScores.push_back(minimalSharpness);
        currentFrame++;
    }


    std::vector<unsigned int> indices;

    int startPosition = 0;

    while (startPosition < sharpnessScores.size())
    {
        int endPosition = std::min(int(sharpnessScores.size()), startPosition + searchWindowSize);
        auto maxIter = std::max_element(sharpnessScores.begin() + startPosition, sharpnessScores.begin() + endPosition);
        size_t index = maxIter - sharpnessScores.begin();
        double maxval = *maxIter;
        if (maxval < thresholdSharpness)
        {
            //This value means that the image is completely blurry.
            //We consider we should not select a value here
            startPosition += searchWindowSize;
            continue;
        }

        if (indices.size() == 0)
        {
            //No previous, so no flow check
            startPosition = index + std::max(int(_minFrameStep), searchWindowSize);
            indices.push_back(index);
            continue;
        }

        int previous = indices.back();

        double flow = estimateFlow(feeds, processWidth, previous, index);
        if (flow < thresholdFlow)
        {
            //Continue with next frame
            startPosition = index + 1;
            continue;
        }

        indices.push_back(index);
        startPosition = index + std::max(int(_minFrameStep), searchWindowSize);
    }

    _selected = indices;
}

bool KeyframeSelector::writeSelection(const std::string & outputFolder, const std::vector<std::string>& mediaPaths, const std::vector<std::string>& brands, const std::vector<std::string>& models, const std::vector<float>& mmFocals)
{
    image::Image< image::RGBColor> image;
    camera::PinholeRadialK3 queryIntrinsics;
    bool hasIntrinsics = false;
    std::string currentImgName;

    for (int id = 0; id < mediaPaths.size(); id++)
    {
        const auto& path = mediaPaths.at(id);

        // create a feed provider per mediaPaths
        dataio::FeedProvider feed(path);

        // check if feed is initialized
        if (!feed.isInit())
        {
            ALICEVISION_LOG_ERROR("Cannot initialize the FeedProvider with " << path);
            return false;
        }

        std::string processedOutputFolder = outputFolder;
        if (mediaPaths.size() > 1)
        {
            const std::string rigFolder = outputFolder + "/rig/";
            if (!fs::exists(rigFolder))
            {
                fs::create_directory(rigFolder);
            }

            processedOutputFolder = rigFolder + std::to_string(id);
            if (!fs::exists(processedOutputFolder))
            {
                fs::create_directory(processedOutputFolder);
            }
        }

        for (auto pos : _selected)
        {
            feed.goToFrame(pos);

            if (!feed.readImage(image, queryIntrinsics, currentImgName, hasIntrinsics))
            {
                ALICEVISION_LOG_ERROR("Error reading image");
                return false;
            }

            oiio::ImageSpec spec(image.Width(), image.Height(), 3, oiio::TypeDesc::UINT8);

            spec.attribute("Make", brands[id]);
            spec.attribute("Model", models[id]);
            spec.attribute("Exif:BodySerialNumber", std::to_string(getRandomInt())); // TODO: use Exif:OriginalRawFileName instead
            spec.attribute("Exif:FocalLength", mmFocals[id]);
            spec.attribute("Exif:ImageUniqueID", std::to_string(getRandomInt()));

            fs::path folder = outputFolder;
            std::ostringstream filenameSS;
            filenameSS << std::setw(5) << std::setfill('0') << pos << ".exr";
            const auto filepath = (processedOutputFolder / fs::path(filenameSS.str())).string();


            std::unique_ptr<oiio::ImageOutput> out(oiio::ImageOutput::create(filepath));

            if (out.get() == nullptr)
            {
                throw std::invalid_argument("Cannot create image file : " + filepath);
            }

            if (!out->open(filepath, spec))
            {
                throw std::invalid_argument("Cannot open image file : " + filepath);
            }

            out->write_image(oiio::TypeDesc::UINT8, image.data());
            out->close();
        }

    }

    return true;
}

} // namespace keyframe
} // namespace aliceVision
