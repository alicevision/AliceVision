// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/image/imageAlgo.hpp>
#include <aliceVision/image/drawing.hpp>

#include <random>
#include <algorithm>
#include <filesystem>

#include <boost/program_options.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace std {
std::ostream& operator<<(std::ostream& os, const std::pair<double, double>& v)
{
    os << v.first << " " << v.second;
    return os;
}

std::istream& operator>>(std::istream& in, std::pair<double, double>& v)
{
    std::string token;
    in >> token;
    v.first = boost::lexical_cast<double>(token);
    in >> token;
    v.second = boost::lexical_cast<double>(token);
    return in;
}
}  // namespace std

namespace po = boost::program_options;
namespace fs = std::filesystem;
namespace pt = boost::property_tree;

/**
 * A simple class for gaussian pyramid
 */
class PyramidFloat
{
  public:
    PyramidFloat(size_t width, size_t height, size_t minimal_size)
    {
        // minimal_size * 2^n = minside
        size_t minside = std::min(width, height);
        double scale = std::log2(double(minside) / double(minimal_size));
        int levels = std::floor(scale);

        size_t cwidth = width;
        size_t cheight = height;
        for (int i = 0; i <= levels; i++)
        {
            _levels.push_back(image::Image<float>(cwidth, cheight));

            cheight /= 2;
            cwidth /= 2;
        }
    }

    bool apply(const image::Image<float>& grayscale_input)
    {
        // First of all, build pyramid for filtering high frequencies
        _levels[0] = grayscale_input;
        for (int level = 1; level < _levels.size(); level++)
        {
            size_t sw = _levels[level - 1].width();
            size_t sh = _levels[level - 1].height();
            size_t dw = _levels[level].width();
            size_t dh = _levels[level].height();

            oiio::ImageSpec spec_src(sw, sh, 1, oiio::TypeDesc::FLOAT);
            oiio::ImageSpec spec_dst(dw, dh, 1, oiio::TypeDesc::FLOAT);

            oiio::ImageBuf buf_src(spec_src, const_cast<float*>(_levels[level - 1].data()));
            oiio::ImageBuf buf_dst(spec_dst, const_cast<float*>(_levels[level].data()));

            oiio::ImageBufAlgo::resize(buf_dst, buf_src, "gaussian");
        }

        return true;
    }

    size_t countLevels() const { return _levels.size(); }

    const image::Image<float>& getLevel(size_t level) const { return _levels[level]; }

  private:
    std::vector<image::Image<float>> _levels;
};

class CircleDetector
{
  public:
    CircleDetector() = delete;

    CircleDetector(size_t width, size_t height, size_t minimal_size)
      : _source_width(width),
        _source_height(height),
        _minimal_size(minimal_size),
        _radius(0)
    {}

    void setDebugDirectory(const std::string& dir) { _debugDirectory = dir; }

    bool appendImage(const image::Image<float>& grayscale_input)
    {
        if (grayscale_input.width() != _source_width)
        {
            return false;
        }

        if (grayscale_input.height() != _source_height)
        {
            return false;
        }

        // Store pyramid for this image, will be processed later
        // This way we ensure we do not loose intermediate information
        PyramidFloat pyramid(_source_width, _source_height, _minimal_size);
        if (!pyramid.apply(grayscale_input))
        {
            return false;
        }
        _pyramids.push_back(pyramid);

        return true;
    }

    bool preprocessLevel(const PyramidFloat& pyramid, size_t pyramid_id, size_t level)
    {
        if (level >= pyramid.countLevels())
        {
            return false;
        }

        const image::Image<float>& source = pyramid.getLevel(level);

        // Adapt current center to level
        double level_centerx = _center_x / pow(2.0, level);
        double level_centery = _center_y / pow(2.0, level);

        // Compute the maximal distance between the borders and the circle center
        double max_rx = std::max(double(source.width()) - level_centerx, level_centerx);
        double max_ry = std::max(double(source.height()) - level_centery, level_centery);

        // Just in case the smallest side is cropped inside the circle.
        double max_radius = std::min(max_rx, max_ry);
        max_radius *= 1.5;

        size_t max_radius_i = size_t(std::ceil(max_radius));
        double angles_bins = size_t(std::ceil(max_radius * M_PI));

        // Build a polar image using the estimated circle center
        image::Image<float> polarImage(max_radius_i, angles_bins, true, 0.0f);
        image::Image<unsigned char> polarImageMask(max_radius_i, angles_bins, true, 0);
        if (!buildPolarImage(polarImage, polarImageMask, source, level_centerx, level_centery))
        {
            return false;
        }

        debugImage(polarImage, "polarImage", pyramid_id, level);

        // Use a custom edge detector
        // int max = pyramid.countLevels() - 1;
        // int diff = max - level;
        int min_radius = 8;
        int radius = min_radius;  // * pow(2, diff);
        image::Image<float> gradientImage(max_radius_i, angles_bins);
        if (!buildGradientImage(gradientImage, polarImage, polarImageMask, radius))
        {
            return false;
        }

        debugImage(gradientImage, "gradientImage", pyramid_id, level);

        if (_gradientImage.width() != gradientImage.width() || _gradientImage.height() != gradientImage.height())
        {
            _gradientImage = gradientImage;
        }
        else
        {
            _gradientImage += gradientImage;
        }

        return true;
    }

    bool process()
    {
        if (_pyramids.empty())
        {
            return false;
        }

        // Initialize parameters with most common case
        _center_x = _source_width / 2;
        _center_y = _source_height / 2;
        _radius = std::min(_source_width / 4, _source_height / 4);
        size_t last_level_inliers = 0;
        int last_valid_level = -1;

        for (int current_level = _pyramids[0].countLevels() - 1; current_level > 1; current_level--)
        {
            // Compute gradients
            size_t current_pyramid_id = 0;
            for (PyramidFloat& pyr : _pyramids)
            {
                if (!preprocessLevel(pyr, current_pyramid_id, current_level))
                {
                    return false;
                }
                current_pyramid_id++;
            }

            // Estimate the search area
            int uncertainty = 50;
            if (current_level == _pyramids[0].countLevels() - 1)
            {
                uncertainty = std::max(_source_width, _source_height);
            }

            debugImage(_gradientImage, "globalGradientImage", 0, current_level);

            // Perform estimation
            if (!processLevel(current_level, uncertainty, last_level_inliers))
            {
                break;
            }

            last_valid_level = current_level;
        }

        // Check that the circle was detected at some level
        if (last_valid_level < 0)
        {
            return false;
        }

        return true;
    }

    bool processLevel(size_t level, int uncertainty, size_t& last_level_inliers)
    {
        image::Image<float> gradients = _gradientImage;

        image::Image<unsigned char> selection(gradients.width(), gradients.height(), true, 0);

        // Adapt current center to level
        const double level_centerx = _center_x / pow(2.0, level);
        const double level_centery = _center_y / pow(2.0, level);
        const double level_radius = _radius / pow(2.0, level);
        const int min_radius = gradients.width() / 2;

        // Extract maximas of response
        std::vector<Eigen::Vector2d> selected_points;
        for (int y = 0; y < gradients.height(); y++)
        {
            const double rangle = double(y) * (2.0 * M_PI / double(gradients.height()));
            const double cangle = cos(rangle);
            const double sangle = sin(rangle);

            // Lookup possible radius
            const int start = std::max(min_radius, int(level_radius) - uncertainty);
            const int end = std::min(gradients.width() - 1, int(level_radius) + uncertainty);

            // Remove non maximas
            for (int x = start; x < end; x++)
            {
                if (gradients(y, x) < gradients(y, x + 1))
                {
                    gradients(y, x) = 0.0f;
                }
            }

            // Remove non maximas
            for (int x = end; x > start; x--)
            {
                if (gradients(y, x) < gradients(y, x - 1))
                {
                    gradients(y, x) = 0.0f;
                }
            }

            // Store maximas
            for (int x = start; x <= end; x++)
            {
                if (gradients(y, x) > 0.0f)
                {
                    const double nx = level_centerx + cangle * double(x);
                    const double ny = level_centery + sangle * double(x);
                    selected_points.push_back({nx, ny});
                    selection(y, x) = 255;
                }
            }
        }

        if (selected_points.size() < 3)
        {
            return false;
        }

        debugImage(selection, "selected", 0, level);

        //
        // RANSAC
        //
        std::default_random_engine generator;
        std::uniform_int_distribution<int> distribution(0, selected_points.size() - 1);

        size_t maxcount = 0;
        Eigen::Vector3d best_params;
        for (int i = 0; i < 10000; i++)
        {
            const int id1 = distribution(generator);
            const int id2 = distribution(generator);
            const int id3 = distribution(generator);

            if (id1 == id2 || id1 == id3 || id2 == id3)
                continue;

            const Eigen::Vector2d p1 = selected_points[id1];
            const Eigen::Vector2d p2 = selected_points[id2];
            const Eigen::Vector2d p3 = selected_points[id3];

            Eigen::Vector3d res;
            if (!fitCircle(res, p1, p2, p3))
            {
                continue;
            }

            size_t count = 0;
            for (const auto& point : selected_points)
            {
                const double cx = point(0) - res(0);
                const double cy = point(1) - res(1);
                const double r = res(2);

                const double dist = std::abs(sqrt(cx * cx + cy * cy) - r);
                if (dist < 3)
                {
                    count++;
                }
            }

            if (count > maxcount)
            {
                maxcount = count;
                best_params = res;
            }
        }

        if (maxcount < last_level_inliers)
        {
            return false;
        }
        last_level_inliers = maxcount;

        //
        // Minimize
        //
        double sigma = 2.0;
        double c = sigma * 4.6851;
        Eigen::Vector3d previous_good = best_params;
        double last_good_error = std::numeric_limits<double>::max();

        for (int iter = 0; iter < 1000; iter++)
        {
            double sum_error = 0.0;
            for (int i = 0; i < selected_points.size(); i++)
            {
                const double cx = selected_points[i](0) - best_params(0);
                const double cy = selected_points[i](1) - best_params(1);
                const double r = best_params(2);
                const double dist = pow(sqrt(cx * cx + cy * cy) - r, 2.0);

                double w = 0.0;
                if (dist < c)
                {
                    const double xoc = dist / c;
                    const double hw = 1.0 - xoc * xoc;
                    w = hw * hw;
                }

                sum_error += w * dist;
            }

            if (sum_error > last_good_error)
            {
                best_params = previous_good;
                break;
            }

            last_good_error = sum_error;

            Eigen::Matrix3d JtJ = Eigen::Matrix3d::Zero();
            Eigen::Vector3d Jte = Eigen::Vector3d::Zero();
            for (auto& pt : selected_points)
            {
                const double cx = pt(0) - best_params(0);
                const double cy = pt(1) - best_params(1);
                const double r = best_params(2);
                const double normsq = cx * cx + cy * cy;
                const double norm = sqrt(normsq);
                const double dist = norm - r;

                double w = 0.0;
                if (dist < c)
                {
                    double xoc = dist / c;
                    double hw = 1.0 - xoc * xoc;
                    w = hw * hw;
                }

                Eigen::Vector3d J;
                if (std::abs(normsq) < 1e-12)
                {
                    J.fill(0);
                    J(2) = -w;
                }
                else
                {
                    J(0) = -w * cx / norm;
                    J(1) = -w * cy / norm;
                    J(2) = -w;
                }

                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        JtJ(i, j) += J(i) * J(j);
                    }

                    Jte(i) += J(i) * dist;
                }
            }

            previous_good = best_params;
            best_params -= JtJ.inverse() * Jte;
        }

        _center_x = best_params(0) * pow(2.0, level);
        _center_y = best_params(1) * pow(2.0, level);
        _radius = best_params(2) * pow(2.0, level);

        return true;
    }

    bool fitCircle(Eigen::Vector3d& output, const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2, const Eigen::Vector2d& pt3)
    {
        /*
        Solve :
        (pt1x - centerx)^2 + (pt1y - centery)^2 - r^2 = 0
        (pt2x - centerx)^2 + (pt2y - centery)^2 - r^2 = 0
        (pt3x - centerx)^2 + (pt3y - centery)^2 - r^2 = 0

        -----
        l1 : pt1x^2 + centerx^2 - 2*pt1x*centerx + pt1y^2 + centery^2 - 2*pt1y*centery - r^2 = 0
        l2 : pt2x^2 + centerx^2 - 2*pt2x*centerx + pt2y^2 + centery^2 - 2*pt2y*centery - r^2 = 0
        l3 : pt3x^2 + centerx^2 - 2*pt3x*centerx + pt3y^2 + centery^2 - 2*pt3y*centery - r^2 = 0
        -----
        l2 - l1 : (pt2x^2 + centerx^2 - 2*pt2x*centerx + pt2y^2 + centery^2 - 2*pt2y*centery - r^2) - (pt1x^2 +
        centerx^2 - 2*pt1x*centerx + pt1y^2 + centery^2 - 2*pt1y*centery - r^2) = 0 l3 - l1 : (pt3x^2 + centerx^2 -
        2*pt3x*centerx + pt3y^2 + centery^2 - 2*pt3y*centery - r^2) - (pt1x^2 + centerx^2 - 2*pt1x*centerx + pt1y^2 +
        centery^2 - 2*pt1y*centery - r^2) = 0
        -----
        l2 - l1 : pt2x^2 + centerx^2 - 2*pt2x*centerx + pt2y^2 + centery^2 - 2*pt2y*centery - r^2 - pt1x^2 - centerx^2 +
        2*pt1x*centerx - pt1y^2 - centery^2 + 2*pt1y*centery + r^2) = 0 l3 - l1 : pt3x^2 + centerx^2 - 2*pt3x*centerx +
        pt3y^2 + centery^2 - 2*pt3y*centery - r^2 - pt1x^2 - centerx^2 + 2*pt1x*centerx - pt1y^2 - centery^2 +
        2*pt1y*centery + r^2) = 0
        -----
        l2 - l1 : pt2x^2 - 2*pt2x*centerx + pt2y^2 - 2*pt2y*centery - pt1x^2 + 2*pt1x*centerx - pt1y^2 + 2*pt1y*centery
        = 0 l3 - l1 : pt3x^2 - 2*pt3x*centerx + pt3y^2 - 2*pt3y*centery - pt1x^2 + 2*pt1x*centerx - pt1y^2 +
        2*pt1y*centery = 0
        -----
        l2 - l1 : pt2x^2 + pt2y^2 - pt1x^2 - pt1y^2 + 2*pt1x*centerx - 2*pt2x*centerx + 2*pt1y*centery - 2*pt2y*centery
        = 0 l3 - l1 : pt3x^2 + pt3y^2 - pt1x^2 - pt1y^2 - 2*pt3x*centerx - 2*pt3y*centery + 2*pt1x*centerx +
        2*pt1y*centery = 0
        -----
        l2 - l1 : pt2x^2 + pt2y^2 - pt1x^2 - pt1y^2 + 2*(pt1x - pt2x)*centerx + 2*(pt1y-pt2y)*centery = 0
        l3 - l1 : pt3x^2 + pt3y^2 - pt1x^2 - pt1y^2 + 2*(pt1x - pt3x)*centerx + 2*(pt1y-pt3y)*centery = 0
        -----
        l2 - l1 : A + C*centerx + E*centery = 0
        l3 - l1 : B + D*centerx + F*centery = 0
        -----
        l2 - l1 : centerx = - (A + E * centery) / C
        l3 - l1 : B + D * centerx + F * centery = 0
        -----
        l2 - l1 : centerx = G + H * centery;
        l3 - l1 : B + D * (G + H * centery) + F * centery = 0
        -----
        l2 - l1 : centerx = G + H * centery;
        l3 - l1 : B + D * G + D * H * centery + F * centery = 0
        -----
        l2 - l1 : centerx = G + H * centery;
        l3 - l1 : B + D * G + (D * H + F) * centery = 0
        -----
        l2 - l1 : centerx = G + H * centery;
        l3 - l1 : centery = - (B + D * G) / (D * H + F);

        ----
        l1 : (pt1x - centerx)^2 + (pt1y - centery)^2 = r^2
        */

        const double A = pt2(0) * pt2(0) + pt2(1) * pt2(1) - pt1(0) * pt1(0) - pt1(1) * pt1(1);
        const double B = pt3(0) * pt3(0) + pt3(1) * pt3(1) - pt1(0) * pt1(0) - pt1(1) * pt1(1);
        const double C = 2.0 * (pt1(0) - pt2(0));
        const double D = 2.0 * (pt1(0) - pt3(0));
        const double E = 2.0 * (pt1(1) - pt2(1));
        const double F = 2.0 * (pt1(1) - pt3(1));
        if (std::abs(C) < 1e-12)
            return false;

        const double G = -A / C;
        const double H = -E / C;

        if (std::abs(D * H + F) < 1e-12)
            return false;

        const double centery = -(B + D * G) / (D * H + F);
        const double centerx = G + H * centery;

        output(0) = centerx;
        output(1) = centery;
        output(2) = sqrt((pt1(0) - centerx) * (pt1(0) - centerx) + (pt1(1) - centery) * (pt1(1) - centery));

        return true;
    }

    bool buildPolarImage(image::Image<float>& dst,
                         image::Image<unsigned char>& dstmask,
                         const image::Image<float>& src,
                         float center_x,
                         float center_y)
    {
        image::Sampler2d<image::SamplerLinear> sampler;
        const size_t count_angles = dst.height();

        for (int angle = 0; angle < count_angles; angle++)
        {
            const double rangle = angle * (2.0 * M_PI / double(count_angles));
            const double cangle = cos(rangle);
            const double sangle = sin(rangle);

            for (int amplitude = 0; amplitude < dst.width(); amplitude++)
            {
                const double x = center_x + cangle * double(amplitude);
                const double y = center_y + sangle * double(amplitude);

                dst(angle, amplitude) = 0;
                dstmask(angle, amplitude) = 0;

                if (x < 0 || y < 0)
                    continue;
                if (x >= src.width() || y >= src.height())
                    continue;
                dst(angle, amplitude) = sampler(src, y, x);
                dstmask(angle, amplitude) = 255;
            }
        }

        return true;
    }

    bool buildGradientImage(image::Image<float>& dst, const image::Image<float>& src, const image::Image<unsigned char>& srcMask, size_t radius_size)
    {
        // Build gradient for x coordinates image
        dst.fill(0);

        int kernel_radius = radius_size;
        for (int angle = 0; angle < src.height(); angle++)
        {
            int start = radius_size;
            int end = src.width() - kernel_radius * 2;

            for (int amplitude = start; amplitude < end; amplitude++)
            {
                dst(angle, amplitude) = 0.0;

                float sum_inside = 0.0;
                float sum_outside = 0.0;

                unsigned char valid = 255;

                for (int dx = -kernel_radius; dx < 0; dx++)
                {
                    sum_inside += src(angle, amplitude + dx);
                    valid &= srcMask(angle, amplitude + dx);
                }
                for (int dx = 1; dx <= kernel_radius * 2; dx++)
                {
                    sum_outside += src(angle, amplitude + dx);
                    valid &= srcMask(angle, amplitude + dx);
                }

                if (valid)
                {
                    dst(angle, amplitude) = std::max(0.0f, (sum_inside - sum_outside));
                }
                else
                {
                    dst(angle, amplitude) = 0.0f;
                }
            }
        }

        return true;
    }

    double getCircleCenterX() const { return _center_x; }

    double getCircleCenterY() const { return _center_y; }

    double getCircleRadius() const { return _radius; }

    template<class T>
    void debugImage(const image::Image<T>& toSave, const std::string& name, int pyramid_id, int level)
    {
        // Only export debug image if there is a debug output folder defined.
        if (_debugDirectory.empty())
            return;

        fs::path filepath = fs::path(_debugDirectory) / (name + "_" + std::to_string(pyramid_id) + "_" + std::to_string(level) + ".exr");
        image::writeImage(filepath.string(), toSave, image::ImageWriteOptions());
    }

  private:
    std::vector<PyramidFloat> _pyramids;
    image::Image<float> _gradientImage;
    std::string _debugDirectory;

    double _center_x;
    double _center_y;
    double _radius;

    size_t _source_width;
    size_t _source_height;
    size_t _minimal_size;
};

/**
 * @brief Utility function for resizing an image.
 */
void resample(image::Image<image::RGBfColor>& output, const image::Image<image::RGBfColor>& input)
{
    const oiio::ImageBuf inBuf(oiio::ImageSpec(input.width(), input.height(), 3, oiio::TypeDesc::FLOAT), const_cast<image::RGBfColor*>(input.data()));

    oiio::ImageBuf outBuf(oiio::ImageSpec(output.width(), output.height(), 3, oiio::TypeDesc::FLOAT), (image::RGBfColor*)output.data());

    oiio::ImageBufAlgo::resample(outBuf, inBuf, false);
}

/**
 * @brief Utility function for rotating an image given its orientation metadata.
 */
void applyOrientation(image::Image<image::RGBfColor>& output, const image::Image<image::RGBfColor>& input, sfmData::EEXIFOrientation orientation)
{
    const oiio::ImageBuf inBuf(oiio::ImageSpec(input.width(), input.height(), 3, oiio::TypeDesc::FLOAT), const_cast<image::RGBfColor*>(input.data()));

    oiio::ImageBuf outBuf(oiio::ImageSpec(output.width(), output.height(), 3, oiio::TypeDesc::FLOAT), (image::RGBfColor*)output.data());

    switch (orientation)
    {
        case sfmData::EEXIFOrientation::UPSIDEDOWN:
            oiio::ImageBufAlgo::rotate180(outBuf, inBuf);
            break;
        case sfmData::EEXIFOrientation::LEFT:
            oiio::ImageBufAlgo::rotate90(outBuf, inBuf);
            break;
        case sfmData::EEXIFOrientation::RIGHT:
            oiio::ImageBufAlgo::rotate270(outBuf, inBuf);
            break;
        default:
            outBuf.copy(inBuf);
            break;
    }
}

/**
 * @brief Utility struct for contact sheet elements.
 */
struct Contact
{
    int rank;
    std::string path;
    int width;
    int height;
    sfmData::EEXIFOrientation orientation;
};

/**
 * @brief Width of contact sheet element, taking into account orientation metadata.
 */
int orientedWidth(const Contact& contact)
{
    switch (contact.orientation)
    {
        case sfmData::EEXIFOrientation::LEFT:
        case sfmData::EEXIFOrientation::RIGHT:
            return contact.height;
        default:
            return contact.width;
    }
}

/**
 * @brief Height of contact sheet element, taking into account orientation metadata.
 */
int orientedHeight(const Contact& contact)
{
    switch (contact.orientation)
    {
        case sfmData::EEXIFOrientation::LEFT:
        case sfmData::EEXIFOrientation::RIGHT:
            return contact.width;
        default:
            return contact.height;
    }
}

bool buildContactSheetImage(image::Image<image::RGBfColor>& output,
                            const std::map<int, std::map<int, Contact>>& contactSheetInfo,
                            int contactSheetItemMaxSize)
{
    const int space = 10;

    // Compute ratio for resizing inputs
    int maxdim = 0;
    for (const auto& rowpair : contactSheetInfo)
    {
        for (const auto& item : rowpair.second)
        {
            maxdim = std::max(maxdim, orientedWidth(item.second));
            maxdim = std::max(maxdim, orientedHeight(item.second));
        }
    }
    double ratioResize = double(contactSheetItemMaxSize) / double(maxdim);

    // Compute output size
    int totalHeight = space;
    int maxWidth = 0;
    for (const auto& rowpair : contactSheetInfo)
    {
        int rowHeight = 0;
        int rowWidth = space;

        for (const auto& item : rowpair.second)
        {
            int resizedHeight = int(ratioResize * double(orientedHeight(item.second)));
            int resizedWidth = int(ratioResize * double(orientedWidth(item.second)));

            rowHeight = std::max(rowHeight, resizedHeight);
            rowWidth += resizedWidth + space;
        }

        totalHeight += rowHeight + space;
        maxWidth = std::max(maxWidth, rowWidth);
    }

    if (totalHeight == 0 || maxWidth == 0)
    {
        return false;
    }

    int rowCount = 0;
    int posY = space;
    output = image::Image<image::RGBfColor>(maxWidth, totalHeight, true);
    for (const auto& rowpair : contactSheetInfo)
    {
        ALICEVISION_LOG_INFO("Build contact sheet row " << rowCount + 1 << "/" << contactSheetInfo.size());
        int rowHeight = 0;
        int rowWidth = space;

        for (const auto& item : rowpair.second)
        {
            int resizedHeight = int(ratioResize * double(orientedHeight(item.second)));
            int resizedWidth = int(ratioResize * double(orientedWidth(item.second)));

            rowHeight = std::max(rowHeight, resizedHeight);
            rowWidth += resizedWidth + space;
        }

        // Create row thumbnails
        image::Image<image::RGBfColor> rowOutput(rowWidth, rowHeight, true);

        int posX = space;
        for (const auto& item : rowpair.second)
        {
            int rawResizedHeight = int(ratioResize * double(item.second.height));
            int rawResizedWidth = int(ratioResize * double(item.second.width));

            int resizedHeight = int(ratioResize * double(orientedHeight(item.second)));
            int resizedWidth = int(ratioResize * double(orientedWidth(item.second)));

            image::Image<image::RGBfColor> input;
            image::Image<image::RGBfColor> rawThumbnail(rawResizedWidth, rawResizedHeight);
            image::Image<image::RGBfColor> thumbnail(resizedWidth, resizedHeight);

            image::readImage(item.second.path, input, image::EImageColorSpace::SRGB);

            resample(rawThumbnail, input);
            applyOrientation(thumbnail, rawThumbnail, item.second.orientation);

            rowOutput.block(0, posX, resizedHeight, resizedWidth) = thumbnail;
            posX += resizedWidth + space;
        }

        int centeredX = (maxWidth - rowWidth) / 2;

        // Concatenate
        output.block(posY, centeredX, rowOutput.height(), rowOutput.width()) = rowOutput;

        posY += rowHeight + space;
        rowCount++;
    }

    return true;
}

int main(int argc, char* argv[])
{
    using namespace aliceVision;

    std::string externalInfoFilepath;
    std::string sfmInputDataFilepath;
    std::string sfmOutputDataFilepath;
    std::string inputAngleString;
    std::string initializeCameras;
    std::string nbViewsPerLineString;

    bool yawCW = true;
    bool useFisheye = false;
    bool estimateFisheyeCircle = true;
    Vec2 fisheyeCenterOffset(0, 0);
    double fisheyeRadius = 96.0;
    float additionalAngle = 0.0f;
    bool debugFisheyeCircleEstimation = false;
    bool buildContactSheet = false;
    int contactSheetItemMaxSize = 256;

    // Command line parameters
    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilepath)->required(),
         "SfMData file input.")
        ("outSfMData,o", po::value<std::string>(&sfmOutputDataFilepath)->required(),
         "SfMData file output.");

    po::options_description motorizedHeadParams("Motorized Head parameters");
    motorizedHeadParams.add_options()
        ("config,c", po::value<std::string>(&externalInfoFilepath),
         "External info xml file from a motorized head system.")
        ("inputAngle,a", po::value<std::string>(&inputAngleString),
         "External info xml additional angle.")
        ("yawCW", po::value<bool>(&yawCW),
         "Yaw rotation is ClockWise or ConterClockWise.")
        ("initializeCameras", po::value<std::string>(&initializeCameras),
         "Initialization type for the cameras poses.")
        ("nbViewsPerLine", po::value<std::string>(&nbViewsPerLineString),
         "Number of views per line splitted by comma. For instance, \"2,4,*,4,2\".")
        ("buildContactSheet", po::value<bool>(&buildContactSheet)->default_value(buildContactSheet),
         "Build a contact sheet");

    po::options_description fisheyeParams("Fisheye parameters");
    fisheyeParams.add_options()
        ("useFisheye", po::value<bool>(&useFisheye),
         "Declare all input images as fisheye with 'equidistant' model.")
        ("estimateFisheyeCircle", po::value<bool>(&estimateFisheyeCircle),
         "Automatically estimate the Fisheye Circle center and radius instead of using user values.")
        ("fisheyeCenterOffset_x", po::value<double>(&fisheyeCenterOffset(0)),
         "Fisheye circle's center offset X (pixels).")
        ("fisheyeCenterOffset_y", po::value<double>(&fisheyeCenterOffset(1)),
         "Fisheye circle's center offset Y (pixels).")
        ("fisheyeRadius,r", po::value<double>(&fisheyeRadius),
         "Fisheye circle's radius (% of image shortest side).")
        ("debugFisheyeCircleEstimation", po::value<bool>(&debugFisheyeCircleEstimation),
         "Debug fisheye circle detection.");
    // clang-format on

    CmdLine cmdline("Initialize information on the panorama pipeline's input images, specifically from a file "
                    "generated by a motorized head system.\n"
                    "AliceVision panoramaInit");
    cmdline.add(requiredParams);
    cmdline.add(motorizedHeadParams);
    cmdline.add(fisheyeParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    if (inputAngleString == "rotate90")
    {
        additionalAngle = -M_PI_2;
    }
    else if (inputAngleString == "rotate180")
    {
        additionalAngle = -M_PI;
    }
    else if (inputAngleString == "rotate270")
    {
        additionalAngle = M_PI_2;
    }

    std::map<int, std::map<int, Contact>> contactSheetInfo;

    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmInputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    {
        // Setup known poses from XML file or user expression
        std::map<int, Eigen::Matrix3d> rotations;

        boost::to_lower(initializeCameras);
        if (initializeCameras == "no") {}
        else if (initializeCameras == "file" || (initializeCameras.empty() && !externalInfoFilepath.empty()))
        {
            if (externalInfoFilepath.empty())
            {
                ALICEVISION_LOG_ERROR("Init cameras from file, but path is not set.");
                return EXIT_FAILURE;
            }

            pt::ptree tree;

            try
            {
                pt::read_xml(externalInfoFilepath, tree);
            }
            catch (...)
            {
                ALICEVISION_CERR("Error parsing input file");
                return EXIT_FAILURE;
            }

            pt::ptree shoot = tree.get_child("papywizard.shoot");

            // Get a set of unique ids
            std::set<int> uniqueIds;
            for (const auto it : shoot)
            {
                int id = it.second.get<double>("<xmlattr>.id");
                if (uniqueIds.find(id) != uniqueIds.end())
                {
                    ALICEVISION_CERR("Multiple xml attributes with a same id: " << id);
                    return EXIT_FAILURE;
                }

                uniqueIds.insert(id);
            }

            // Make sure a map of id is available to get rank (position in ascending order)
            // note that set is ordered automatically.
            int pos = 0;
            std::map<int, int> idToRank;
            for (const auto id : uniqueIds)
            {
                idToRank[id] = pos;
                pos++;
            }

            // Group shoots by "rows" (common pitch) assuming they are acquired row by row with a common pitch
            if (buildContactSheet)
            {
                for (const auto it : shoot)
                {
                    int id = it.second.get<double>("<xmlattr>.id");
                    int bracket = it.second.get<double>("<xmlattr>.bracket");
                    int rank = idToRank[id];

                    const double yaw_degree = it.second.get<double>("position.<xmlattr>.yaw");
                    const double pitch_degree = it.second.get<double>("position.<xmlattr>.pitch");

                    int ipitch_degree = -int(pitch_degree);  // minus to be sure rows are going top to bottom
                    int iyaw_degree = int(yaw_degree);

                    // Store also the yaw to be able to sort left to right
                    contactSheetInfo[ipitch_degree][iyaw_degree].rank = rank;
                }
            }

            for (const auto it : shoot)
            {
                int id = it.second.get<double>("<xmlattr>.id");
                int bracket = it.second.get<double>("<xmlattr>.bracket");
                int rank = idToRank[id];

                const double yaw_degree = it.second.get<double>("position.<xmlattr>.yaw");
                const double pitch_degree = it.second.get<double>("position.<xmlattr>.pitch");
                const double roll_degree = it.second.get<double>("position.<xmlattr>.roll");

                const double yaw = degreeToRadian(yaw_degree);
                const double pitch = degreeToRadian(pitch_degree);
                const double roll = degreeToRadian(roll_degree);

                const Eigen::AngleAxis<double> Myaw(yaw, Eigen::Vector3d::UnitY());
                const Eigen::AngleAxis<double> Mpitch(pitch, Eigen::Vector3d::UnitX());
                const Eigen::AngleAxis<double> Mroll(roll, Eigen::Vector3d::UnitZ());
                const Eigen::AngleAxis<double> Mimage(additionalAngle - M_PI_2, Eigen::Vector3d::UnitZ());

                const Eigen::Matrix3d oRc =
                  Myaw.toRotationMatrix() * Mpitch.toRotationMatrix() * Mroll.toRotationMatrix() * Mimage.toRotationMatrix();

                rotations[rank] = oRc.transpose();
            }

            if (sfmData.getViews().size() != rotations.size())
            {
                ALICEVISION_LOG_ERROR("The input SfMData has not the same number of views than the config file (sfmData views:"
                                      << sfmData.getViews().size() << ", config file rotations: " << rotations.size() << ").");
                return EXIT_FAILURE;
            }
        }
        else if (boost::algorithm::contains(initializeCameras, "horizontal"))
        {
            constexpr double zenithPitch = 0.5 * boost::math::constants::pi<double>();
            const Eigen::AngleAxis<double> zenithMpitch(zenithPitch, Eigen::Vector3d::UnitX());
            const Eigen::AngleAxis<double> zenithMroll(additionalAngle, Eigen::Vector3d::UnitZ());
            const Eigen::Matrix3d oRzenith = zenithMpitch.toRotationMatrix() * zenithMroll.toRotationMatrix();

            const bool withZenith = boost::algorithm::contains(initializeCameras, "zenith");
            if (initializeCameras == "zenith+horizontal")
            {
                ALICEVISION_LOG_TRACE("Add zenith first");
                rotations[rotations.size()] = oRzenith.transpose();
            }

            const std::size_t nbHorizontalViews = sfmData.getViews().size() - int(withZenith);
            for (int x = 0; x < nbHorizontalViews; ++x)
            {
                double yaw = 0;
                if (nbHorizontalViews > 1)
                {
                    // Vary horizontally between -180 and +180 deg
                    yaw = (yawCW ? 1.0 : -1.0) * x * 2.0 * boost::math::constants::pi<double>() / double(nbHorizontalViews);
                }

                const Eigen::AngleAxis<double> Myaw(yaw, Eigen::Vector3d::UnitY());
                const Eigen::AngleAxis<double> Mroll(additionalAngle, Eigen::Vector3d::UnitZ());

                const Eigen::Matrix3d oRc = Myaw.toRotationMatrix() * Mroll.toRotationMatrix();

                ALICEVISION_LOG_TRACE("Add rotation: yaw=" << yaw);
                rotations[rotations.size()] = oRc.transpose();
            }
            if (initializeCameras == "horizontal+zenith")
            {
                ALICEVISION_LOG_TRACE("Add zenith");
                rotations[rotations.size()] = oRzenith.transpose();
            }
        }
        else if (initializeCameras == "spherical" || (initializeCameras.empty() && !nbViewsPerLineString.empty()))
        {
            if (nbViewsPerLineString.empty())
            {
                ALICEVISION_LOG_ERROR("Init cameras from Sperical, but 'nbViewsPerLine' is not set.");
                return EXIT_FAILURE;
            }

            std::vector<std::string> nbViewsStrPerLine;
            boost::split(nbViewsStrPerLine, nbViewsPerLineString, boost::is_any_of(", "));
            const int totalNbViews = sfmData.getViews().size();
            std::vector<int> nbViewsPerLine;
            int nbAutoSize = 0;
            int sum = 0;
            for (const std::string& nbViewsStr : nbViewsStrPerLine)
            {
                if (nbViewsStr == "*")
                {
                    nbViewsPerLine.push_back(-1);
                    ++nbAutoSize;
                }
                else
                {
                    int v = boost::lexical_cast<int>(nbViewsStr);
                    nbViewsPerLine.push_back(v);
                    if (v == -1)
                    {
                        ++nbAutoSize;
                    }
                    else
                    {
                        sum += v;
                    }
                }
            }
            if (sum > totalNbViews)
            {
                ALICEVISION_LOG_ERROR("The input SfMData has less cameras declared than the number of cameras declared "
                                      "in the expression (sfmData views:"
                                      << sfmData.getViews().size() << ", expression sum: " << sum << ").");
                return EXIT_FAILURE;
            }
            if (nbAutoSize > 0)
            {
                std::replace(nbViewsPerLine.begin(), nbViewsPerLine.end(), -1, (totalNbViews - sum) / nbAutoSize);
            }
            if (nbViewsPerLine.empty())
            {
                // If no expression assume that it is a pure rotation around one axis
                nbViewsPerLine.push_back(totalNbViews);
            }
            const std::size_t newSum = std::accumulate(nbViewsPerLine.begin(), nbViewsPerLine.end(), 0);

            if (newSum != totalNbViews)
            {
                ALICEVISION_LOG_ERROR("The number of cameras in the input SfMData does not match with the number of cameras declared "
                                      "in the expression (sfmData views:"
                                      << sfmData.getViews().size() << ", expression sum: " << newSum << ").");
                return EXIT_FAILURE;
            }

            int i = 0;
            for (int y = 0; y < nbViewsPerLine.size(); ++y)
            {
                double pitch = 0;
                if (nbViewsPerLine.size() > 1)
                {
                    // Vary vertically between -90 and +90 deg
                    pitch = (-0.5 * boost::math::constants::pi<double>()) + y * boost::math::constants::pi<double>() / double(nbViewsPerLine.size());
                }

                const int nbViews = nbViewsPerLine[y];
                for (int x = 0; x < nbViews; ++x)
                {
                    double yaw = 0;
                    if (nbViews > 1)
                    {
                        // Vary horizontally between -180 and +180 deg
                        yaw = (yawCW ? 1.0 : -1.0) * x * 2.0 * boost::math::constants::pi<double>() / double(nbViews);
                    }
                    const double roll = 0;

                    const Eigen::AngleAxis<double> Myaw(yaw, Eigen::Vector3d::UnitY());
                    const Eigen::AngleAxis<double> Mpitch(pitch, Eigen::Vector3d::UnitX());
                    const Eigen::AngleAxis<double> Mroll(roll + additionalAngle, Eigen::Vector3d::UnitZ());

                    const Eigen::Matrix3d oRc = Myaw.toRotationMatrix() * Mpitch.toRotationMatrix() * Mroll.toRotationMatrix();

                    ALICEVISION_LOG_TRACE("Add rotation: yaw=" << yaw << ", pitch=" << pitch << ", roll=" << roll << ".");
                    rotations[i++] = oRc.transpose();
                }
            }
        }

        std::map<int, Contact> contacts;

        if (!rotations.empty())
        {
            ALICEVISION_LOG_TRACE("Apply rotations from nbViewsPerLine expressions: " << nbViewsPerLineString << ".");

            if (rotations.size() != sfmData.getViews().size())
            {
                ALICEVISION_LOG_ERROR("The number of cameras in the input SfMData does not match with the number of "
                                      "rotations to apply (sfmData nb views:"
                                      << sfmData.getViews().size() << ", nb rotations: " << rotations.size() << ").");
                return EXIT_FAILURE;
            }

            // HEURISTIC:
            // The xml file describe rotations for views which are not correlated with AliceVision views.
            // We assume that the order of the xml view ids correspond to the lexicographic order of the image names.
            std::vector<std::pair<std::string, int>> namesWithRank;
            for (const auto& v : sfmData.getViews())
            {
                fs::path path_image(v.second->getImage().getImagePath());
                namesWithRank.push_back(std::make_pair(path_image.stem().string(), v.first));
            }
            std::sort(namesWithRank.begin(), namesWithRank.end());

            // If we are trying to build a contact sheet
            if (contactSheetInfo.size() > 0)
            {
                // Fill information in contact sheet
                for (auto& rowpair : contactSheetInfo)
                {
                    for (auto& item : rowpair.second)
                    {
                        int rank = item.second.rank;
                        IndexT viewId = namesWithRank[rank].second;

                        const sfmData::View& v = sfmData.getView(viewId);

                        item.second.path = v.getImage().getImagePath();
                        item.second.width = v.getImage().getWidth();
                        item.second.height = v.getImage().getHeight();
                        item.second.orientation = v.getImage().getMetadataOrientation();
                    }
                }

                image::Image<image::RGBfColor> contactSheetImage;
                if (buildContactSheetImage(contactSheetImage, contactSheetInfo, contactSheetItemMaxSize))
                {
                    image::writeImage((fs::path(sfmOutputDataFilepath).parent_path() / "contactSheetImage.jpg").string(),
                                      contactSheetImage,
                                      image::ImageWriteOptions());
                }
            }

            size_t index = 0;
            for (const auto& item_rotation : rotations)
            {
                IndexT viewIdx = namesWithRank[index].second;
                const sfmData::View& v = sfmData.getView(viewIdx);

                sfmData::EEXIFOrientation orientation = v.getImage().getMetadataOrientation();
                double orientationAngle = 0.;
                switch (orientation)
                {
                    case sfmData::EEXIFOrientation::UPSIDEDOWN:
                        orientationAngle = boost::math::constants::pi<double>();
                        break;
                    case sfmData::EEXIFOrientation::LEFT:
                        orientationAngle = boost::math::constants::pi<double>() * .5;
                        break;
                    case sfmData::EEXIFOrientation::RIGHT:
                        orientationAngle = boost::math::constants::pi<double>() * -.5;
                        break;
                    default:
                        break;
                }

                const Eigen::AngleAxis<double> Morientation(orientationAngle, Eigen::Vector3d::UnitZ());

                const Eigen::Matrix3d viewRotation = Morientation.toRotationMatrix().transpose() * item_rotation.second;

                if (viewRotation.trace() != 0)
                {
                    sfmData::CameraPose pose(geometry::Pose3(viewRotation, Eigen::Vector3d::Zero()));
                    sfmData.setAbsolutePose(viewIdx, pose);
                }
                ++index;
            }
        }
    }

    if (useFisheye)
    {
        sfmData::Intrinsics& intrinsics = sfmData.getIntrinsics();
        for (auto& intrinsic_pair : intrinsics)
        {
            std::shared_ptr<camera::IntrinsicBase>& intrinsic = intrinsic_pair.second;
            std::shared_ptr<camera::IntrinsicScaleOffset> intrinsicSO = std::dynamic_pointer_cast<camera::IntrinsicScaleOffset>(intrinsic);
            std::shared_ptr<camera::Equidistant> equidistant = std::dynamic_pointer_cast<camera::Equidistant>(intrinsic);

            if (intrinsicSO != nullptr && equidistant == nullptr)
            {
                ALICEVISION_LOG_INFO("Replace intrinsic " << intrinsic_pair.first << " of type " << intrinsic->getTypeStr()
                                                          << " to an Equidistant camera model.");
                // convert non-Equidistant intrinsics to Equidistant
                std::shared_ptr<camera::Equidistant> newEquidistant =
                  std::dynamic_pointer_cast<camera::Equidistant>(camera::createIntrinsic(camera::EINTRINSIC::EQUIDISTANT_CAMERA_RADIAL3));

                newEquidistant->copyFrom(*intrinsicSO);
                // "radius" and "center" will be set later from the input parameters in another loop

                // replace the intrinsic
                intrinsic = newEquidistant;
            }
        }
    }

    {
        int equidistantCount = 0;

        if (useFisheye && estimateFisheyeCircle)
        {
            if (sfmData.getIntrinsics().size() != 1)
            {
                ALICEVISION_LOG_ERROR("Only one intrinsic allowed (" << sfmData.getIntrinsics().size() << " found)");
                return EXIT_FAILURE;
            }

            std::shared_ptr<camera::IntrinsicBase> intrinsic = sfmData.getIntrinsics().begin()->second;
            if (!intrinsic)
            {
                ALICEVISION_LOG_ERROR("No valid intrinsic");
                return EXIT_FAILURE;
            }

            if (camera::isEquidistant(intrinsic->getType()))
            {
                CircleDetector detector(intrinsic->w(), intrinsic->h(), 256);
                if (debugFisheyeCircleEstimation)
                {
                    fs::path path(sfmOutputDataFilepath);
                    detector.setDebugDirectory(path.parent_path().string());
                }
                for (const auto& v : sfmData.getViews())
                {
                    // Read original image
                    image::Image<float> grayscale;
                    image::readImage(v.second->getImage().getImagePath(), grayscale, image::EImageColorSpace::SRGB);

                    const bool res = detector.appendImage(grayscale);
                    if (!res)
                    {
                        ALICEVISION_LOG_ERROR("Image is incompatible with fisheye detection");
                        return EXIT_FAILURE;
                    }
                }

                if (!detector.process())
                {
                    ALICEVISION_LOG_ERROR("Failed to find circle");
                    return EXIT_FAILURE;
                }

                const double cx = detector.getCircleCenterX();
                const double cy = detector.getCircleCenterY();
                const double r = detector.getCircleRadius();

                // Update parameters with estimated values
                fisheyeCenterOffset(0) = cx - 0.5 * double(intrinsic->w());
                fisheyeCenterOffset(1) = cy - 0.5 * double(intrinsic->h());
                fisheyeRadius = 98.0 * r / (0.5 * std::min(double(intrinsic->w()), double(intrinsic->h())));

                ALICEVISION_LOG_INFO("Computing automatic fisheye circle");
                ALICEVISION_LOG_INFO(" * Center Offset: " << fisheyeCenterOffset);
                ALICEVISION_LOG_INFO(" * Radius: " << fisheyeRadius);
            }
        }

        sfmData::Intrinsics& intrinsics = sfmData.getIntrinsics();
        for (const auto& intrinsic_pair : intrinsics)
        {
            std::shared_ptr<camera::IntrinsicBase> intrinsic = intrinsic_pair.second;
            std::shared_ptr<camera::Equidistant> equidistant = std::dynamic_pointer_cast<camera::Equidistant>(intrinsic);
            if (!equidistant)
            {
                // skip non equidistant cameras
                continue;
            }
            ALICEVISION_LOG_INFO("Update Equidistant camera intrinsic " << intrinsic_pair.first << " with center and offset.");

            equidistant->setCircleCenterX(double(equidistant->w()) / 2.0 + fisheyeCenterOffset(0));
            equidistant->setCircleCenterY(double(equidistant->h()) / 2.0 + fisheyeCenterOffset(1));

            equidistant->setCircleRadius(fisheyeRadius / 100.0 * 0.5 * std::min(double(equidistant->w()), double(equidistant->h())));
            ++equidistantCount;
        }

        ALICEVISION_LOG_INFO(equidistantCount << " equidistant camera intrinsics have been updated");
    }

    ALICEVISION_LOG_INFO("Export SfM: " << sfmOutputDataFilepath);
    if (!sfmDataIO::save(sfmData, sfmOutputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmOutputDataFilepath << "' cannot be write.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
