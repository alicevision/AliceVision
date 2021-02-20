// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)

#include "aliceVision/image/Image.hpp"
#include <aliceVision/numeric/numeric.hpp>

#include <opencv2/core.hpp>

namespace aliceVision{
namespace image
{


/**
 * @brief Sets the value at the specified pixel of a OpenCV BGR image matrix
 * Ignores the alpha channel of the source image
 * @tparam VecType - OpenCV vector type to interpret the mat values with
 * @tparam ValueType - numeric type to cast the color values
 * @param[inout] mat - OpenCV mat to set
 * @param[in] i - row index to set
 * @param[in] j - column index to set
 * @param[in] color - value to set
 * @param[in] factor - optional scale factor
 * @param[in] delta - optional delta added to the scaled values
 * @return the resulting openCV image
 */
template <typename VecType, typename ValueType>
inline void setValueCvMatBGR(cv::Mat& mat, int i, int j, const image::RGBAfColor& color, float factor = 1.f,
                             float delta = 0.f)
{
    mat.at<VecType>(i, j)[0] = (ValueType) clamp(color.b() * factor + delta, 0.f, 255.f);
    mat.at<VecType>(i, j)[1] = (ValueType) clamp(color.g() * factor + delta, 0.f, 255.f);
    mat.at<VecType>(i, j)[2] = (ValueType) clamp(color.r() * factor + delta, 0.f, 255.f);
}


/**
 * @brief Converts an aliceVision image to an openCv image (cv::Mat) in BGR
 * Ignores the alpha channel of the source image
 * @param[in] img - Input RGBA aliceVision image
 * @param[in] cvtype - OpenCV mat type (supported values: CV_32FC3, CV_8UC3)
 * @return the resulting openCV image
 */
inline cv::Mat imageRGBAToCvMatBGR(const image::Image<image::RGBAfColor>& img, int cvtype = CV_32FC3)
{
    cv::Mat mat(img.Height(), img.Width(), cvtype);
    for(int i = 0; i < img.Height(); i++)
    {
        for(int j = 0; j < img.Width(); j++)
        {
            switch(cvtype)
            {
                case CV_32FC3:
                    setValueCvMatBGR<cv::Vec3f, float>(mat, i, j, img(i, j));
                    break;
                case CV_8UC3:
                    setValueCvMatBGR<cv::Vec3b, uint8_t>(mat, i, j, img(i, j), 255.f);
                    break;
                default:
                    std::runtime_error("Cannot handle OpenCV matrix type '" + std::to_string(cvtype) + "'.");
            }
        }
    }
    return mat;
}


/**
 * @brief Converts an openCv image (cv::Mat) in BGR to an aliceVision image
 * Keeps the alpha channel of the output image unchanged
 * @param[in] img - Input openCV image (cv::Mat)
 * @param[out] img - output RGBA aliceVision image
 * @return the resulting regex
 */
inline void cvMatBGRToImageRGBA(const cv::Mat& img, image::Image<image::RGBAfColor>& imageOut)
{
    for(int row = 0; row < imageOut.Height(); row++)
    {
        const cv::Vec3f* rowPtr = img.ptr<cv::Vec3f>(row);
        for(int col = 0; col < imageOut.Width(); col++)
        {
            const cv::Vec3f& matPixel = rowPtr[col];
            imageOut(row, col) = image::RGBAfColor(matPixel[2], matPixel[1], matPixel[0], imageOut(row, col).a());
        }
    }
}

} // namespace image
} // namespace aliceVision

#endif // ALICEVISION_HAVE_OPENCV
