// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "checkerDetector.hpp"

#include <aliceVision/system/Logger.hpp>

#include <OpenImageIO/imagebufalgo.h>

#include <boost/math/constants/constants.hpp>

#include <limits>
#include <algorithm>

// TODO: to remove when moving to eigen 3.4
namespace Eigen {
template<typename Type, int Size>
using Vector = Matrix<Type, Size, 1>;
template<typename Type, int Size>
using RowVector = Matrix<Type, 1, Size>;
}  // namespace Eigen

namespace aliceVision {
namespace calibration {

bool CheckerDetector::process(const image::Image<image::RGBColor>& source, bool useNestedGrids, bool debug)
{
    image::Image<float> grayscale;
    image::ConvertPixelType(source, &grayscale);

    const Vec2 center(grayscale.width() / 2, grayscale.height() / 2);

    const double scales[] = {1.0, 0.75, 0.5, 0.25};

    std::vector<IntermediateCorner> allCorners;
    for (double scale : scales)
    {
        ALICEVISION_LOG_INFO("[CheckerDetector] extracting corners at scale " << scale);
        std::vector<Vec2> corners;
        if (!processLevel(corners, grayscale, scale))
        {
            ALICEVISION_LOG_DEBUG("[CheckerDetector] detection failed");
            return false;
        }

        ALICEVISION_LOG_DEBUG("[CheckerDetector] detected " << corners.size() << " corners");

        // Merge with previous level corners
        const double distMerge = 5.0;
        for (Vec2 c : corners)
        {
            bool keep = true;

            for (const IntermediateCorner& oc : allCorners)
            {
                const double dist = (oc.center - c).norm();

                if (dist < distMerge)
                {
                    keep = false;
                    break;
                }
            }

            if (!keep)
                continue;

            allCorners.emplace_back(c, scale);
        }
    }

    ALICEVISION_LOG_DEBUG("[CheckerDetector] kept " << allCorners.size() << " corners positions after merge between levels");

    // Normalize image between 0 and 1
    image::Image<float> normalized;
    normalizeImage(normalized, grayscale);

    std::vector<CheckerBoardCorner> fittedCorners;
    fitCorners(fittedCorners, allCorners, normalized);

    // Remove multiple points at the same position
    const double distSamePosition = 2.0;
    for (std::size_t i = 0; i < fittedCorners.size(); ++i)
    {
        const CheckerBoardCorner& ci = fittedCorners[i];

        bool found = false;
        for (std::size_t j = i + 1; j < fittedCorners.size(); ++j)
        {
            const CheckerBoardCorner& cj = fittedCorners[j];

            if ((ci.center - cj.center).norm() < distSamePosition)
            {
                found = true;
            }
        }

        if (!found)
        {
            _corners.push_back(ci);
        }
    }

    ALICEVISION_LOG_INFO("[CheckerDetector] extracting checkerboards with " << _corners.size() << " corners");

    buildCheckerboards(_boards, _corners, normalized);

    ALICEVISION_LOG_DEBUG("[CheckerDetector] built " << _boards.size() << " checkerboards");

    // Try to merge together checkerboards if connected
    if (!mergeCheckerboards())
    {
        ALICEVISION_LOG_DEBUG("[CheckerDetector] failed to merge boards");
        return false;
    }

    ALICEVISION_LOG_DEBUG("[CheckerDetector] " << _boards.size() << " checkerboards after merge");

    if (!removeInvalidCheckerboards())
    {
        ALICEVISION_LOG_DEBUG("[CheckerDetector] failed to remove invalid boards");
        return false;
    }

    ALICEVISION_LOG_DEBUG("[CheckerDetector] " << _boards.size() << " checkerboards after filtering");

    if (useNestedGrids)
    {
        ALICEVISION_LOG_INFO("[CheckerDetector] computing nested checkerboard sequence");

        // Sort checkerboards by distance to center
        sortCheckerBoardsByDistanceToCenter(center);
        filterNestedCheckerBoards(source.rows(), source.cols());
        buildNestedConnectors();
        groupNestedCheckerboards();
    }

    if (debug)
    {
        _debugImage = source;
        drawCheckerBoard(_debugImage, useNestedGrids);
    }

    return true;
}

bool CheckerDetector::processLevel(std::vector<Vec2>& corners, const image::Image<float>& input, double scale) const
{
    // Get resized size
    const unsigned int w = input.width();
    const unsigned int h = input.height();
    const unsigned int nw = static_cast<unsigned int>(floor(static_cast<float>(w) * scale));
    const unsigned int nh = static_cast<unsigned int>(floor(static_cast<float>(h) * scale));

    // Resize image
    image::Image<float> toUpdate = input;
    image::Image<float> rescaled(nw, nh);
    const oiio::ImageSpec imageSpecResized(nw, nh, 1, oiio::TypeDesc::FLOAT);
    const oiio::ImageSpec imageSpecOrigin(w, h, 1, oiio::TypeDesc::FLOAT);
    const oiio::ImageBuf inBuf(imageSpecOrigin, toUpdate.data());
    oiio::ImageBuf outBuf(imageSpecResized, rescaled.data());
    oiio::ImageBufAlgo::resize(outBuf, inBuf);

    // Normalize image between 0 and 1
    image::Image<float> normalized;
    normalizeImage(normalized, rescaled);

    image::Image<float> hessian;
    computeHessianResponse(hessian, normalized);

    std::vector<Vec2> rawCorners;
    extractCorners(rawCorners, hessian);

    std::vector<Vec2> refinedCorners;
    refineCorners(refinedCorners, rawCorners, normalized);

    pruneCorners(corners, refinedCorners, normalized);

    for (Vec2& v : corners)
    {
        v.x() /= scale;
        v.y() /= scale;
    }

    return true;
}

void CheckerDetector::pruneCorners(std::vector<Vec2>& prunedCorners, const std::vector<Vec2>& rawCorners, const image::Image<float>& input) const
{
    const image::Sampler2d<image::SamplerLinear> sampler;
    const int radius = 5;
    const int samples = 50;

    float vector[samples];

    for (const Vec2& corner : rawCorners)
    {
        float min = std::numeric_limits<float>::max();
        float max = std::numeric_limits<float>::min();

        // Sample grayscale values on a circle around corner position
        for (int sample = 0; sample < samples; ++sample)
        {
            const double angle = 2.0 * boost::math::constants::pi<double>() * static_cast<double>(sample) / static_cast<double>(samples);

            const double x = corner(0) + cos(angle) * static_cast<double>(radius);
            const double y = corner(1) + sin(angle) * static_cast<double>(radius);

            vector[sample] = sampler(input, y, x);

            min = std::min(min, vector[sample]);
            max = std::max(max, vector[sample]);
        }

        // Normalize these values
        for (int sample = 0; sample < samples; ++sample)
        {
            vector[sample] = 2.0 * ((vector[sample] - min) / (max - min)) - 1.0;
        }

        // Count the number of sign changes between consecutive samples
        int count = 0;
        for (int sample = 0; sample < samples; ++sample)
        {
            int next = sample + 1;
            if (next == samples)
                next = 0;

            float test = vector[sample] * vector[next];
            if (test < 0.0)
            {
                count++;
            }
        }

        // Count should be 4 to ensure the corner corresponds to an intersection of black and white tiles
        if (count != 4)
            continue;

        prunedCorners.push_back(corner);
    }
}

void CheckerDetector::normalizeImage(image::Image<float>& output, const image::Image<float>& input) const
{
    float min = 0.0f, max = 0.0f;
    getMinMax(min, max, input);

    output.resize(input.width(), input.height());
    for (int y = 0; y < output.height(); ++y)
    {
        for (int x = 0; x < output.width(); ++x)
        {
            output(y, x) = (input(y, x) - min) / (max - min);
        }
    }
}

void CheckerDetector::computeHessianResponse(image::Image<float>& output, const image::Image<float>& input) const
{
    image::Image<float> smoothed;
    image::imageGaussianFilter(input, 1.5, smoothed, 2);

    // First order derivatives
    image::Image<float> gx, gy;
    imageXDerivative(smoothed, gx, true);
    imageYDerivative(smoothed, gy, true);

    // Second order derivatives
    image::Image<float> gxx, gxy, gyy;
    imageXDerivative(gx, gxx, true);
    imageXDerivative(gy, gxy, true);
    imageYDerivative(gy, gyy, true);

    output.resize(input.width(), input.height());
    for (int y = 0; y < input.height(); ++y)
    {
        for (int x = 0; x < input.width(); ++x)
        {
            output(y, x) = std::abs(gxx(y, x) * gyy(y, x) - 2.0 * gxy(y, x));
        }
    }
}

void CheckerDetector::extractCorners(std::vector<Vec2>& rawCorners, const image::Image<float>& hessianResponse) const
{
    float min = 0.0f, max = 0.0f;
    getMinMax(min, max, hessianResponse);

    const float threshold = max * 0.1f;
    const int radius = 7;

    // Find peaks (local maxima) of the Hessian response
    image::Image<float> output(hessianResponse.width(), hessianResponse.height(), true, 0.0f);
    for (int i = radius; i < hessianResponse.height() - radius; ++i)
    {
        for (int j = radius; j < hessianResponse.width() - radius; ++j)
        {
            bool isMaximal = true;
            const float val = hessianResponse(i, j);

            // Compare value to neighborhood
            for (int k = -radius; k <= radius; ++k)
            {
                for (int l = -radius; l <= radius; ++l)
                {
                    if (hessianResponse(i + k, j + l) > val)
                    {
                        isMaximal = false;
                    }
                }
            }

            if (!isMaximal)
                continue;

            // Peak must be higher than a global threshold
            if (val > threshold)
            {
                rawCorners.emplace_back(j, i);
            }
        }
    }
}

void CheckerDetector::getMinMax(float& min, float& max, const image::Image<float>& input) const
{
    min = std::numeric_limits<float>::max();
    max = std::numeric_limits<float>::min();
    for (int y = 0; y < input.height(); ++y)
    {
        for (int x = 0; x < input.width(); ++x)
        {
            min = std::min(min, input(y, x));
            max = std::max(max, input(y, x));
        }
    }
}

void CheckerDetector::refineCorners(std::vector<Vec2>& refinedCorners, const std::vector<Vec2>& rawCorners, const image::Image<float>& input) const
{
    image::Image<float> gx, gy;
    imageXDerivative(input, gx, true);
    imageYDerivative(input, gy, true);

    const int radius = 5;

    for (const Vec2& pt : rawCorners)
    {
        if (pt.x() < radius)
            continue;
        if (pt.y() < radius)
            continue;
        if (pt.x() >= gx.width() - radius)
            continue;
        if (pt.y() >= gx.height() - radius)
            continue;

        Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
        Eigen::Vector2d b = Eigen::Vector2d::Zero();
        for (int k = -radius; k <= radius; ++k)
        {
            const int i = pt.y() + k;

            for (int l = -radius; l <= radius; ++l)
            {
                if (l == 0 && k == 0)
                    continue;

                const int j = pt.x() + l;

                float du = gx(i, j);
                float dv = gy(i, j);
                const float norm = std::sqrt(du * du + dv * dv);
                if (norm < 0.01)
                    continue;

                du /= norm;
                dv /= norm;

                A(0, 0) += du * du;
                A(0, 1) += du * dv;
                A(1, 0) += du * dv;
                A(1, 1) += dv * dv;

                b(0) += du * du * static_cast<double>(j) + du * dv * static_cast<double>(i);
                b(1) += dv * du * static_cast<double>(j) + dv * dv * static_cast<double>(i);
            }
        }

        Vec2 update = A.inverse() * b;

        // Make sure the update is coherent
        const double dist = (update - pt).norm();
        if (dist > radius)
            continue;

        refinedCorners.push_back(update);
    }
}

void CheckerDetector::fitCorners(std::vector<CheckerBoardCorner>& refinedCorners,
                                 const std::vector<IntermediateCorner>& rawCorners,
                                 const image::Image<float>& input) const
{
    // Build kernel
    const int radius = 4;
    const int diameter = 2 * radius + 1;
    Eigen::MatrixXd kernel(diameter, diameter);

    double norm = 0.0;
    for (int i = -radius; i <= radius; ++i)
    {
        const int di = i + radius;
        for (int j = -radius; j <= radius; ++j)
        {
            const int dj = j + radius;

            const double val = std::max(0.0, radius + 1 - std::sqrt(i * i + j * j));
            norm += val;

            kernel(di, dj) = val;
        }
    }

    kernel /= norm;

    image::Image<float> filtered;
    image::imageConvolution(input, kernel, filtered);

    Eigen::MatrixXd AtA(6, 6);
    Eigen::Vector<double, 6> Atb;

    const image::Sampler2d<image::SamplerLinear> sampler;

    for (const IntermediateCorner& sc : rawCorners)
    {
        Vec2 corner = sc.center;
        bool isValid = true;

        const double cx = corner(0);
        const double cy = corner(1);

        for (int iter = 0; iter < 20; ++iter)
        {
            AtA.fill(0);
            Atb.fill(0);

            for (int i = -radius; i <= radius; ++i)
            {
                const int di = corner(1) + i;
                for (int j = -radius; j <= radius; ++j)
                {
                    const int dj = corner(0) + j;

                    Eigen::Vector<double, 6> rowA;
                    rowA(0) = j * j;
                    rowA(1) = i * j;
                    rowA(2) = i * i;
                    rowA(3) = j;
                    rowA(4) = i;
                    rowA(5) = 1.0;

                    AtA += rowA * rowA.transpose();
                    Atb += rowA * sampler(filtered, di, dj);
                }
            }

            Eigen::Vector<double, 6> x = AtA.inverse() * Atb;

            // f(x,y) = a1x**2 + a2xy + a3y**2 + a4x + a5y + a6
            // df(x)/dx = 2a1x + a2y + a4
            // df(x)/dy = 2a3y + a2x + a5
            // H = |2a1 a2 |
            //     |a2  2a3|
            // det(H) = 2*2*a1*a3 - 2*a2
            // inv(H) = |2a3 -a2|/det(H)
            //          |-a2 2a1|
            const double a1 = x(0);
            const double a2 = x(1);
            const double a3 = x(2);
            const double a4 = x(3);
            const double a5 = x(4);
            const double a6 = x(5);

            const double determinantH = 4.0 * a1 * a3 - a2 * a2;
            if (std::abs(determinantH) < 1e-6)
            {
                isValid = false;
                break;
            }

            Eigen::Matrix2d H;
            H(0, 0) = 2.0 * a1;
            H(0, 1) = a2;
            H(1, 0) = a2;
            H(1, 1) = 2.0 * a3;

            Eigen::Vector2d vecB;
            vecB(0) = -a4;
            vecB(1) = -a5;

            Vec2 update = H.inverse() * vecB;
            corner += update;

            Eigen::EigenSolver<Eigen::Matrix2d> solver(H, true);
            const double l1 = solver.eigenvalues()(0).real();
            const double l2 = solver.eigenvalues()(1).real();
            if (l1 * l2 > -1e-12)
            {
                isValid = false;
                break;
            }

            if (corner(0) < radius || corner(0) >= input.width() - radius || corner(1) < radius || corner(1) >= input.height() - radius)
            {
                isValid = false;
                break;
            }

            if (update.norm() < 1e-5)
                break;
        }

        if (isValid)
        {
            refinedCorners.emplace_back(corner, sc.scale);
        }
    }

    for (CheckerBoardCorner& corner : refinedCorners)
    {
        const double cx = corner.center(0);
        const double cy = corner.center(1);

        AtA.fill(0);
        Atb.fill(0);

        for (int i = -radius; i <= radius; ++i)
        {
            const int di = corner.center(1) + i;
            for (int j = -radius; j <= radius; ++j)
            {
                if (i == j)
                    continue;

                const int dj = corner.center(0) + j;

                Eigen::Vector<double, 6> rowA;
                rowA(0) = 1.0;
                rowA(1) = j;
                rowA(2) = i;
                rowA(3) = 2.0 * j * i;
                rowA(4) = j * j - i * i;
                rowA(5) = j * j + i * i;

                AtA += rowA * rowA.transpose();
                Atb += rowA * (2.0 * sampler(filtered, di, dj) - 1.0);
            }
        }

        Eigen::Vector<double, 6> x = AtA.inverse() * Atb;
        const double c1 = x(0);
        const double c2 = x(1);
        const double c3 = x(2);
        const double c4 = x(3);
        const double c5 = x(4);
        const double c6 = x(5);

        const double K = std::sqrt(c5 * c5 + c4 * c4);

        const double cos2phi = (-c6 / K);
        const double cos2theta = (c5 / K);
        const double sin2theta = (c4 / K);

        const double phi = acos(cos2phi) * 0.5;
        const double theta = atan2(sin2theta, cos2theta) * 0.5;

        if (theta != theta || phi != phi)
            continue;

        double angle = theta - phi;
        corner.dir1(0) = cos(angle);
        corner.dir1(1) = sin(angle);

        angle = theta + phi;
        corner.dir2(0) = cos(angle);
        corner.dir2(1) = sin(angle);
    }
}

IndexT CheckerDetector::findClosestCorner(const Vec2& center, const Vec2& dir, const std::vector<CheckerBoardCorner>& refinedCorners) const
{
    IndexT ret = UndefinedIndexT;
    double min = std::numeric_limits<double>::max();
    double angle = 0.0;

    for (IndexT cid = 0; cid < refinedCorners.size(); ++cid)
    {
        const CheckerBoardCorner& c = refinedCorners[cid];

        Vec2 diff = c.center - center;
        const double dist = diff.norm();
        if (dist < 5.0)
            continue;

        // Check that corner belongs to cone starting at center, directed at dir, with angle PI/8
        if (std::abs(std::atan2(diff.y(), diff.x()) - std::atan2(dir.y(), dir.x())) > boost::math::constants::pi<double>() * 0.125)
        {
            continue;
        }

        if (dist < min)
        {
            min = dist;
            ret = cid;
            angle = std::abs(std::atan2(diff.y(), diff.x()) - std::atan2(dir.y(), dir.x()));
        }
    }

    return ret;
}

bool CheckerDetector::getSeedCheckerboard(CheckerBoard& board, IndexT seed, const std::vector<CheckerBoardCorner>& refinedCorners) const
{
    IndexT right, bottom, bottom_right, check;
    const CheckerBoardCorner& referenceCorner = refinedCorners[seed];

    right = findClosestCorner(referenceCorner.center, referenceCorner.dir1, refinedCorners);
    if (right == UndefinedIndexT)
    {
        return false;
    }

    bottom = findClosestCorner(referenceCorner.center, referenceCorner.dir2, refinedCorners);
    if (bottom == UndefinedIndexT)
    {
        return false;
    }

    const CheckerBoardCorner& bottomCorner = refinedCorners[bottom];
    const CheckerBoardCorner& rightCorner = refinedCorners[right];

    bottom_right = findClosestCorner(bottomCorner.center, bottomCorner.dir2, refinedCorners);
    if (bottom_right == UndefinedIndexT)
    {
        return false;
    }

    check = findClosestCorner(rightCorner.center, -rightCorner.dir1, refinedCorners);
    if (check == UndefinedIndexT)
    {
        return false;
    }

    if (check != bottom_right)
    {
        return false;
    }

    board.resize(2, 2);
    board(0, 0) = seed;
    board(0, 1) = right;
    board(1, 0) = bottom;
    board(1, 1) = bottom_right;

    return true;
}

bool CheckerDetector::getCandidates(std::vector<NewPoint>& candidates, const CheckerBoard& board, bool inside) const
{
    if (board.rows() < 2)
    {
        return false;
    }

    if (!inside)
    {
        for (int col = 0; col < board.cols(); ++col)
        {
            int srow = -1;
            NewPoint p;
            p.col = col;
            p.score = std::numeric_limits<double>::max();

            for (int row = 0; row < board.rows() - 2; ++row)
            {
                if (board(row, col) != UndefinedIndexT)
                    break;

                srow = row;
            }

            if (srow < 0)
                continue;
            if (board(srow + 1, col) == UndefinedIndexT)
                continue;
            if (board(srow + 2, col) == UndefinedIndexT)
                continue;

            p.row = srow;
            candidates.push_back(p);
        }
    }
    else
    {
        for (int col = 0; col < board.cols(); ++col)
        {
            NewPoint p;
            p.col = col;
            p.score = std::numeric_limits<double>::max();

            for (int row = board.rows() - 3; row >= 0; --row)
            {
                if (board(row, col) != UndefinedIndexT)
                    continue;

                if (board(row + 1, col) == UndefinedIndexT)
                    continue;

                if (board(row + 2, col) == UndefinedIndexT)
                    continue;

                p.row = row;
                candidates.push_back(p);
            }
        }
    }

    return true;
}

bool CheckerDetector::growIterationUp(CheckerBoard& board, const std::vector<CheckerBoardCorner>& refinedCorners, bool nested) const
{
    const double referenceEnergy = computeEnergy(board, refinedCorners);

    // Enlarge board and fill with empty indices
    CheckerBoard nboard(board.rows() + 1, board.cols());
    nboard.fill(UndefinedIndexT);
    nboard.block(1, 0, board.rows(), board.cols()) = board;
    board = nboard;

    std::unordered_set<IndexT> used;
    for (int i = 0; i < board.rows(); ++i)
    {
        for (int j = 0; j < board.cols(); ++j)
        {
            used.insert(board(i, j));
        }
    }

    std::vector<NewPoint> candidates;
    if (!getCandidates(candidates, board, nested))
    {
        return false;
    }

    // Search for new corners
    for (auto& candidate : candidates)
    {
        const IndexT rci = board(candidate.row + 2, candidate.col);
        const IndexT rcj = board(candidate.row + 1, candidate.col);

        const CheckerBoardCorner& ci = refinedCorners[rci];
        const CheckerBoardCorner& cj = refinedCorners[rcj];

        IndexT minIndex = UndefinedIndexT;
        double minE = std::numeric_limits<double>::max();

        for (std::size_t id = 0; id < refinedCorners.size(); ++id)
        {
            if (used.find(id) != used.end())
                continue;

            const CheckerBoardCorner& ck = refinedCorners[id];

            const double E = ((ci.center - cj.center) + (ck.center - cj.center)).norm() / (ck.center - ci.center).norm();
            if (E < minE)
            {
                minE = E;
                minIndex = id;
            }
        }

        // Check if this neighbor is aimed by another candidate (keep the best in this case)
        bool alreadyTook = false;
        for (auto& candidateCheck : candidates)
        {
            const IndexT idx = board(candidateCheck.row, candidateCheck.col);
            if (idx != minIndex)
                continue;

            if (candidateCheck.score < minE)
            {
                alreadyTook = true;
                continue;
            }

            board(candidateCheck.row, candidateCheck.col) = UndefinedIndexT;
            candidate.score = std::numeric_limits<double>::max();
        }

        if (alreadyTook)
            continue;

        board(candidate.row, candidate.col) = minIndex;
        candidate.score = minE;
    }

    if (computeEnergy(board, refinedCorners) < referenceEnergy)
    {
        return true;
    }

    std::sort(candidates.begin(), candidates.end(), [](const NewPoint& p1, const NewPoint p2) { return p1.score < p2.score; });
    while (!candidates.empty())
    {
        const NewPoint c = candidates.back();
        candidates.pop_back();

        // Get the worst candidate and remove it
        const IndexT j = c.col;
        const IndexT i = c.row;
        board(i, j) = UndefinedIndexT;

        // Check if the next candidate's score is below the threshold, otherwise continue
        if (c.score > 1e6)
            continue;

        // After removal, some corners may be isolated, they should be removed
        bool changed = false;

        if (!nested)
        {
            for (std::size_t id = 0; id < candidates.size(); ++id)
            {
                const int ni = candidates[id].row;
                const int nj = candidates[id].col;

                int countH = 0;
                if (nj - 1 >= 0)
                {
                    if (board(ni, nj - 1) != UndefinedIndexT)
                    {
                        countH++;
                    }
                }

                if (nj + 1 < board.cols())
                {
                    if (board(ni, nj + 1) != UndefinedIndexT)
                    {
                        countH++;
                    }
                }

                if (countH == 0)
                {
                    candidates[id].score = std::numeric_limits<double>::max();
                    changed = true;
                }
            }
        }

        if (changed)
        {
            std::sort(candidates.begin(), candidates.end(), [](const NewPoint& p1, const NewPoint p2) { return p1.score < p2.score; });
            continue;
        }

        // Check if energy is lower than before
        const double E = computeEnergy(board, refinedCorners);
        if (E < referenceEnergy)
        {
            return true;
        }
    }

    return false;
}

double CheckerDetector::computeEnergy(const CheckerBoard& board, const std::vector<CheckerBoardCorner>& refinedCorners) const
{
    // Count valid corners in board
    size_t countValid = 0;
    for (int i = 0; i < board.rows(); ++i)
    {
        for (int j = 0; j < board.cols(); ++j)
        {
            const IndexT id = board(i, j);
            if (id != UndefinedIndexT)
            {
                countValid++;
            }
        }
    }

    // Compute maximum local distortion along rows and columns
    double maxE = 0;

    if (board.cols() > 2)
    {
        for (int i = 0; i < board.rows(); ++i)
        {
            for (int j = 0; j < board.cols() - 2; ++j)
            {
                const IndexT id1 = board(i, j);
                const IndexT id2 = board(i, j + 1);
                const IndexT id3 = board(i, j + 2);

                if (id1 == UndefinedIndexT || id2 == UndefinedIndexT || id3 == UndefinedIndexT)
                {
                    continue;
                }

                const CheckerBoardCorner& ci = refinedCorners[id1];
                const CheckerBoardCorner& cj = refinedCorners[id2];
                const CheckerBoardCorner& ck = refinedCorners[id3];

                const double E = ((ci.center - cj.center) + (ck.center - cj.center)).norm() / (ck.center - ci.center).norm();
                if (E > maxE)
                {
                    maxE = E;
                }
            }
        }
    }

    if (board.rows() > 2)
    {
        for (int i = 0; i < board.rows() - 2; ++i)
        {
            for (int j = 0; j < board.cols(); ++j)
            {
                const IndexT id1 = board(i, j);
                const IndexT id2 = board(i + 1, j);
                const IndexT id3 = board(i + 2, j);

                if (id1 == UndefinedIndexT || id2 == UndefinedIndexT || id3 == UndefinedIndexT)
                {
                    continue;
                }

                const CheckerBoardCorner& ci = refinedCorners[id1];
                const CheckerBoardCorner& cj = refinedCorners[id2];
                const CheckerBoardCorner& ck = refinedCorners[id3];

                const double E = ((ci.center - cj.center) + (ck.center - cj.center)).norm() / (ck.center - ci.center).norm();
                if (E > maxE)
                {
                    maxE = E;
                }
            }
        }
    }

    // Energy formula
    return static_cast<double>(countValid) * (maxE - 1);
}

bool CheckerDetector::growIteration(CheckerBoard& board, const std::vector<CheckerBoardCorner>& refinedCorners) const
{
    if (board.rows() < 2)
        return false;
    if (board.cols() < 2)
        return false;

    const double originalE = computeEnergy(board, refinedCorners);
    double minE = std::numeric_limits<double>::max();

    CheckerBoard board_original = board;

    CheckerBoard board_up = board_original;
    if (growIterationUp(board_up, refinedCorners, false))
    {
        const double E = computeEnergy(board_up, refinedCorners);
        if (E < minE)
        {
            board = board_up;
            minE = E;
        }
    }

    CheckerBoard board_down = board_original.colwise().reverse();
    if (growIterationUp(board_down, refinedCorners, false))
    {
        const double E = computeEnergy(board_down, refinedCorners);
        if (E < minE)
        {
            board = board_down.colwise().reverse();
            minE = E;
        }
    }

    CheckerBoard board_right = board_original.transpose().colwise().reverse();
    if (growIterationUp(board_right, refinedCorners, false))
    {
        const double E = computeEnergy(board_right, refinedCorners);
        if (E < minE)
        {
            board = board_right.colwise().reverse().transpose();
            minE = E;
        }
    }

    CheckerBoard board_left = board_original.transpose();
    if (growIterationUp(board_left, refinedCorners, false))
    {
        const double E = computeEnergy(board_left, refinedCorners);
        if (E < minE)
        {
            board = board_left.transpose();
            minE = E;
        }
    }

    if (minE < originalE)
    {
        return true;
    }

    board = board_original;

    return false;
}

void CheckerDetector::buildCheckerboards(std::vector<CheckerBoard>& boards,
                                         const std::vector<CheckerBoardCorner>& refinedCorners,
                                         const image::Image<float>& input) const
{
    double minE = std::numeric_limits<double>::max();

    std::vector<bool> used(refinedCorners.size(), false);
    for (IndexT cid = 0; cid < refinedCorners.size(); ++cid)
    {
        const CheckerBoardCorner& seed = refinedCorners[cid];
        if (used[cid])
            continue;

        // Check if corner can be used as seed
        CheckerBoard board;
        if (!getSeedCheckerboard(board, cid, refinedCorners))
        {
            continue;
        }

        // Check if board contains already used corners
        bool valid = true;
        for (int i = 0; i < board.rows(); ++i)
        {
            for (int j = 0; j < board.cols(); ++j)
            {
                if (used[board(i, j)])
                {
                    valid = false;
                }
            }
        }

        if (!valid)
            continue;

        // Extend board as much as possible
        while (growIteration(board, refinedCorners)) {}

        // Check that board has more than 10 corners
        int count = 0;
        for (int i = 0; i < board.rows(); ++i)
        {
            for (int j = 0; j < board.cols(); ++j)
            {
                if (board(i, j) == UndefinedIndexT)
                    continue;
                count++;
            }
        }

        if (count < 10)
            continue;

        // Update used corners
        for (int i = 0; i < board.rows(); ++i)
        {
            for (int j = 0; j < board.cols(); ++j)
            {
                if (board(i, j) == UndefinedIndexT)
                    continue;

                const IndexT id = board(i, j);
                used[id] = true;
            }
        }

        // Threshold on energy value
        if (computeEnergy(board, refinedCorners) / static_cast<double>(count) > -0.8)
        {
            continue;
        }

        boards.push_back(board);
    }
}

void CheckerDetector::drawCheckerBoard(image::Image<image::RGBColor>& img, bool nestedCheckers) const
{
    for (auto c : _corners)
    {
        image::drawLine(c.center.x() + 2.0, c.center.y(), c.center.x() - 2.0, c.center.y(), image::RGBColor(255, 255, 0), &img);
        image::drawLine(c.center.x(), c.center.y() + 2.0, c.center.x(), c.center.y() - 2.0, image::RGBColor(255, 255, 0), &img);
    }

    std::vector<image::RGBColor> colors;
    colors.emplace_back(255, 0, 0);
    colors.emplace_back(255, 255, 0);
    colors.emplace_back(255, 255, 255);
    colors.emplace_back(0, 255, 0);
    colors.emplace_back(0, 255, 255);
    colors.emplace_back(0, 0, 255);
    colors.emplace_back(255, 0, 255);

    // Utility lambda to draw the horizontal or vertical edges of a checkerboard on the input image
    auto drawEdges = [&](const CheckerBoard& board, const image::RGBColor& color, bool horizontal) -> void {
        const Vec2i delta = horizontal ? Vec2i{1, 0} : Vec2i{0, 1};

        for (int i = 0; i < board.rows() - delta.y(); ++i)
        {
            for (int j = 0; j < board.cols() - delta.x(); ++j)
            {
                const IndexT p1 = board(i, j);
                if (p1 == UndefinedIndexT)
                    continue;

                IndexT p2 = board(i + delta.y(), j + delta.x());

                if (nestedCheckers && p2 == UndefinedIndexT && i + 2 * delta.y() < board.rows() && j + 2 * delta.x() < board.cols())
                {
                    p2 = board(i + 2 * delta.y(), j + 2 * delta.x());

                    if (p2 == UndefinedIndexT && i + 4 * delta.y() < board.rows() && j + 4 * delta.x() < board.cols())
                    {
                        p2 = board(i + 4 * delta.y(), j + 4 * delta.x());
                    }
                }

                if (p2 == UndefinedIndexT)
                    continue;

                const CheckerBoardCorner& c1 = _corners[p1];
                const CheckerBoardCorner& c2 = _corners[p2];

                image::drawLineThickness(c1.center.x(), c1.center.y(), c2.center.x(), c2.center.y(), color, 5, &img);
            }
        }
    };

    std::size_t idx = 0;
    for (const auto& board : _boards)
    {
        drawEdges(board, colors[idx], true);
        drawEdges(board, colors[idx], false);

        idx++;
        if (idx >= _boards.size())
            idx = 0;
    }
}

CheckerDetector::CheckerBoard CheckerDetector::trimEmptyBorders(const CheckerBoard& board) const
{
    int miny = board.rows() - 1;
    int maxy = 0;
    int minx = board.cols() - 1;
    int maxx = 0;

    for (int i = 0; i < board.rows(); ++i)
    {
        for (int j = 0; j < board.cols(); ++j)
        {
            if (board(i, j) != UndefinedIndexT)
            {
                miny = std::min(miny, i);
                maxy = std::max(maxy, i);
                minx = std::min(minx, j);
                maxx = std::max(maxx, j);
            }
        }
    }

    const int width = maxx - minx + 1;
    const int height = maxy - miny + 1;

    return board.block(miny, minx, height, width);
}

bool CheckerDetector::findCommonCorner(const CheckerBoard& board,
                                       const std::unordered_map<IndexT, Vec2i>& cornerLookup,
                                       Vec2i& coordsRef,
                                       Vec2i& coordsBoard) const
{
    for (int i = 0; i < board.rows(); ++i)
    {
        for (int j = 0; j < board.cols(); ++j)
        {
            if (board(i, j) == UndefinedIndexT)
            {
                continue;
            }

            auto lookup = cornerLookup.find(board(i, j));
            if (lookup == cornerLookup.end())
            {
                continue;
            }

            coordsRef = lookup->second;
            coordsBoard = Vec2i(j, i);
            return true;
        }
    }

    return false;
}

bool CheckerDetector::mergeCheckerboardsPair(const CheckerBoard& boardRef,
                                             const CheckerBoard& board,
                                             const Vec2i& coordsRef,
                                             const Vec2i& coordsBoard,
                                             CheckerBoard& boardMerge) const
{
    const Vec2i offset = coordsRef - coordsBoard;

    const int right = std::max(boardRef.cols() - 1, board.cols() + offset.x() - 1);
    const int bottom = std::max(boardRef.rows() - 1, board.rows() + offset.y() - 1);
    const int left = std::min(0, offset.x());
    const int top = std::min(0, offset.y());
    const int nwidth = right - left + 1;
    const int nheight = bottom - top + 1;

    const int shiftRefX = std::max(0, -offset.x());
    const int shiftRefY = std::max(0, -offset.y());

    const int shiftCurX = std::max(0, offset.x());
    const int shiftCurY = std::max(0, offset.y());

    boardMerge.resize(nheight, nwidth);
    boardMerge.fill(UndefinedIndexT);
    boardMerge.block(shiftRefY, shiftRefX, boardRef.rows(), boardRef.cols()) = boardRef;

    for (int i = 0; i < board.rows(); ++i)
    {
        for (int j = 0; j < board.cols(); ++j)
        {
            const IndexT curval = board(i, j);
            if (curval == UndefinedIndexT)
                continue;

            const int rx = j + shiftCurX;
            const int ry = i + shiftCurY;

            const IndexT compare = boardMerge(ry, rx);
            if (compare != UndefinedIndexT && compare != curval)
            {
                return false;
            }

            boardMerge(ry, rx) = curval;
        }
    }

    return true;
}

void CheckerDetector::filterOverlapping(const std::vector<std::pair<CheckerBoard, double>>& checkersWithScore, std::vector<std::size_t>& toKeep) const
{
    for (std::size_t idRef = 0; idRef < checkersWithScore.size(); ++idRef)
    {
        const CheckerBoard baseBoard = checkersWithScore[idRef].first;

        std::set<IndexT> usedCorners;
        for (int i = 0; i < baseBoard.rows(); ++i)
        {
            for (int j = 0; j < baseBoard.cols(); ++j)
            {
                const IndexT curval = baseBoard(i, j);
                if (curval == UndefinedIndexT)
                    continue;

                usedCorners.insert(curval);
            }
        }

        bool keep = true;
        for (std::size_t idCur = 0; idCur < checkersWithScore.size(); ++idCur)
        {
            if (idCur == idRef)
                continue;

            const CheckerBoard& currentBoard = checkersWithScore[idCur].first;

            size_t count_similar = 0;
            for (int i = 0; i < currentBoard.rows(); ++i)
            {
                for (int j = 0; j < currentBoard.cols(); ++j)
                {
                    const IndexT curval = currentBoard(i, j);
                    if (curval == UndefinedIndexT)
                        continue;

                    if (usedCorners.find(curval) != usedCorners.end())
                    {
                        count_similar++;
                    }
                }
            }

            // If the other checker contains almost all the corners of this checkerboard and has a better score
            const double ratio = static_cast<double>(count_similar) / static_cast<double>(usedCorners.size());
            if (ratio > 0.8)
            {
                if (checkersWithScore[idRef].second > checkersWithScore[idCur].second)
                {
                    keep = false;
                    break;
                }
            }
        }

        if (keep)
        {
            toKeep.push_back(idRef);
        }
    }
}

bool CheckerDetector::mergeCheckerboards()
{
    using CheckerBoardWithScore = std::pair<CheckerBoard, double>;
    std::vector<CheckerBoardWithScore> checkers;

    // Remove empty borders
    for (std::size_t id = 0; id < _boards.size(); ++id)
    {
        auto b = _boards[id];
        _boards[id] = trimEmptyBorders(b);
    }

    if (_boards.size() <= 1)
    {
        return true;
    }

    for (auto b : _boards)
    {
        CheckerBoardWithScore cbws;
        cbws.first = b;
        cbws.second = computeEnergy(b, _corners);
        checkers.push_back(cbws);
    }

    bool hadMerged;
    do
    {
        hadMerged = false;
        std::sort(checkers.begin(), checkers.end(), [](const CheckerBoardWithScore& cb1, const CheckerBoardWithScore& cb2) {
            return cb1.second < cb2.second;
        });

        for (std::size_t idRef = 0; idRef < checkers.size(); ++idRef)
        {
            const CheckerBoard baseBoard = checkers[idRef].first;

            // Build dictionnary of corners for faster lookup
            std::unordered_map<IndexT, Vec2i> baseCorners;
            for (int i = 0; i < baseBoard.rows(); ++i)
            {
                for (int j = 0; j < baseBoard.cols(); ++j)
                {
                    if (baseBoard(i, j) != UndefinedIndexT)
                    {
                        baseCorners[baseBoard(i, j)] = Vec2i(j, i);
                    }
                }
            }

            for (std::size_t idCur = idRef + 1; idCur < checkers.size(); ++idCur)
            {
                const CheckerBoard& currentBoard = checkers[idCur].first;

                // Find a common corner
                Vec2i coordsRef;
                Vec2i coordsCur;
                bool foundCommon = findCommonCorner(currentBoard, baseCorners, coordsRef, coordsCur);
                if (!foundCommon)
                    continue;

                // Merge reference board with current one
                CheckerBoard newBoard;
                bool mergeSuccess = mergeCheckerboardsPair(baseBoard, currentBoard, coordsRef, coordsCur, newBoard);
                if (!mergeSuccess)
                    continue;

                const double newEnergy = computeEnergy(newBoard, _corners);

                // Add a small constant to avoid equality problem (if equality, merge)
                if (newEnergy < (checkers[idRef].second + 1e-6))
                {
                    hadMerged = true;

                    checkers[idRef].first = newBoard;
                    checkers[idRef].second = newEnergy;
                    checkers.erase(checkers.begin() + idCur);

                    break;
                }
            }

            if (hadMerged)
                break;
        }
    } while (hadMerged);

    // Remove overlapping
    std::vector<std::size_t> toKeep;
    filterOverlapping(checkers, toKeep);

    // Copy result
    _boards.clear();
    for (auto id : toKeep)
    {
        _boards.push_back(checkers[id].first);
    }

    return true;
}

bool CheckerDetector::removeInvalidCheckerboards()
{
    std::vector<CheckerBoard> filtered_boards;

    // Utility lambda to check invalidity criteria of a board along horizontal or vertical axis
    auto checkInvalid = [this](const CheckerBoard& board, bool horizontal) -> bool {
        bool isInvalid = false;

        const Vec2i delta = horizontal ? Vec2i{1, 0} : Vec2i{0, 1};

        for (int i = 0; i < board.rows() - delta.y(); ++i)
        {
            Vec2 sum{0, 0};
            int count = 0;

            std::vector<Vec2> dirs;

            // Store directions between consecutive corners in row
            for (int j = 0; j < board.cols() - delta.x(); ++j)
            {
                const IndexT c1 = board(i, j);
                const IndexT c2 = board(i + delta.y(), j + delta.x());

                if (c1 == UndefinedIndexT || c2 == UndefinedIndexT)
                    continue;

                const Vec2 dir = (_corners[c2].center - _corners[c1].center).normalized();
                dirs.push_back(dir);
            }

            // Check that angle between two directions is less than PI/4
            for (const Vec2& ref : dirs)
            {
                for (const Vec2& cur : dirs)
                {
                    const double angle = acos(ref.dot(cur));
                    if (std::abs(angle) > boost::math::constants::pi<double>() * 0.25)
                    {
                        isInvalid = true;
                        break;
                    }
                }

                if (isInvalid)
                    break;
            }
        }

        return isInvalid;
    };

    for (auto b : _boards)
    {
        if (checkInvalid(b, true) || checkInvalid(b, false))
        {
            continue;
        }

        filtered_boards.push_back(b);
    }

    _boards = filtered_boards;

    return true;
}

double CheckerDetector::minDistanceToCenter(const CheckerBoard& board, const Vec2& center) const
{
    double mindist = std::numeric_limits<double>::max();
    for (int i = 0; i < board.rows(); ++i)
    {
        for (int j = 0; j < board.cols(); ++j)
        {
            const IndexT cid = board(i, j);
            if (cid == UndefinedIndexT)
                continue;

            const CheckerBoardCorner& c = _corners[cid];
            double dist = (c.center - center).norm();
            mindist = std::min(dist, mindist);
        }
    }
    return mindist;
}

void CheckerDetector::sortCheckerBoardsByDistanceToCenter(const Vec2& center)
{
    // Sort boards by their minimal distance to image center
    std::sort(_boards.begin(), _boards.end(), [this, &center](const CheckerBoard& first, const CheckerBoard& second) {
        return minDistanceToCenter(first, center) < minDistanceToCenter(second, center);
    });
}

void CheckerDetector::filterNestedCheckerBoards(const size_t& height, const size_t& width)
{
    std::vector<CheckerBoard> updated_boards;

    double previous_area = 0.0;

    // Assume all checkerboards have been sorted wrt their distance to center
    for (std::size_t idx_board = 0; idx_board < _boards.size(); ++idx_board)
    {
        const CheckerBoard& board = _boards[idx_board];

        // Count the number of valid corners
        size_t count_points = 0;
        Vec2 mean = {0, 0};
        for (int i = 0; i < board.rows(); ++i)
        {
            for (int j = 0; j < board.cols(); ++j)
            {
                const IndexT cid = board(i, j);
                if (board(i, j) == UndefinedIndexT)
                    continue;

                mean += _corners[cid].center;

                count_points++;
            }
        }

        mean /= static_cast<double>(count_points);

        // Filter out small checkerboards
        if (count_points < 30)
            continue;

        // The first checkerboard must be in the center of the image
        // Other checkerboards more distant may have been separated because of point of view
        if (updated_boards.size() == 0)
        {
            const Vec2 size = {width, height};
            const Vec2 dist = (mean - (size / 2));

            // Make it relative to the image size
            if (std::abs(dist.x() / size.x()) > 0.1)
            {
                _boards.clear();
                return;
            }
        }

        // Compute the mean area of the squares
        double mean_squares = 0.0;
        int count_squares = 0;
        for (int i = 0; i < board.rows() - 1; ++i)
        {
            for (int j = 0; j < board.cols() - 1; ++j)
            {
                if (board(i, j) == UndefinedIndexT)
                    continue;

                if (board(i, j + 1) == UndefinedIndexT)
                    continue;

                if (board(i + 1, j) == UndefinedIndexT)
                    continue;

                if (board(i + 1, j + 1) == UndefinedIndexT)
                    continue;

                const Vec2 pt11 = _corners[board(i, j)].center;
                const Vec2 pt12 = _corners[board(i, j + 1)].center;
                const Vec2 pt21 = _corners[board(i + 1, j)].center;
                const Vec2 pt22 = _corners[board(i + 1, j + 1)].center;

                const double area = 0.5 * std::abs(((pt11.x() * pt12.y() + pt12.x() * pt21.y() + pt21.x() * pt22.y() + pt22.x() * pt11.y()) -
                                                    (pt12.x() * pt11.y() + pt21.x() * pt12.y() + pt22.x() * pt21.y() + pt11.x() * pt22.y())));
                mean_squares += area;
                count_squares++;
            }
        }

        double mean_area = mean_squares / count_squares;

        // Check that square size function is increasing
        if (mean_area < previous_area * 0.8)
            continue;

        previous_area = mean_area;

        updated_boards.push_back(board);
    }

    _boards = updated_boards;
}

void CheckerDetector::buildNestedConnectors()
{
    if (_boards.size() == 0)
    {
        return;
    }

    // Try to extend boards with checkerboards links
    for (std::size_t idx_board = 0; idx_board < _boards.size(); ++idx_board)
    {
        auto& board_original = _boards[idx_board];
        double minE = computeEnergy(board_original, _corners);

        CheckerBoard board_up = board_original;
        if (growIterationUp(board_up, _corners, true))
        {
            const double E = computeEnergy(board_up, _corners);
            if (E < minE)
            {
                board_original = board_up;
                minE = E;
            }
        }

        CheckerBoard board_down = board_original.colwise().reverse();
        if (growIterationUp(board_down, _corners, true))
        {
            const double E = computeEnergy(board_down, _corners);
            if (E < minE)
            {
                board_original = board_down.colwise().reverse();
                minE = E;
            }
        }

        CheckerBoard board_right = board_original.transpose().colwise().reverse();
        if (growIterationUp(board_right, _corners, true))
        {
            const double E = computeEnergy(board_right, _corners);
            if (E < minE)
            {
                board_original = board_right.colwise().reverse().transpose();
                ;
                minE = E;
            }
        }

        CheckerBoard board_left = board_original.transpose();
        if (growIterationUp(board_left, _corners, true))
        {
            const double E = computeEnergy(board_left, _corners);
            if (E < minE)
            {
                board_original = board_left.transpose();
                minE = E;
            }
        }
    }
}

bool CheckerDetector::groupNestedCheckerboardsPair(Vec2i& ref_center, const IndexT& other, int scale)
{
    const CheckerBoard ref_board = _boards[0];
    CheckerBoard cur_board = _boards[other];

    // Find corners present in both checkerboards
    std::vector<std::pair<Vec2, Vec2>> list_same_corners;
    for (int ref_i = 0; ref_i < ref_board.rows(); ++ref_i)
    {
        for (int ref_j = 0; ref_j < ref_board.cols(); ++ref_j)
        {
            bool found = false;
            const IndexT ref_val = ref_board(ref_i, ref_j);
            if (ref_val == UndefinedIndexT)
                continue;

            for (int cur_i = 0; cur_i < cur_board.rows(); ++cur_i)
            {
                for (int cur_j = 0; cur_j < cur_board.cols(); ++cur_j)
                {
                    const IndexT cur_val = cur_board(cur_i, cur_j);
                    if (cur_val == ref_val)
                    {
                        list_same_corners.push_back(std::make_pair(Vec2(ref_j, ref_i), Vec2(cur_j, cur_i)));
                        found = true;
                        break;
                    }
                }

                if (found)
                    break;
            }
        }
    }

    // Vote for the displacement between frame of current to frame of reference
    std::map<std::pair<int, int>, int> votes;
    for (auto& p : list_same_corners)
    {
        std::pair<int, int> offset{p.first.x() - (p.second.x() * scale), p.first.y() - (p.second.y() * scale)};

        if (votes.find(offset) == votes.end())
        {
            votes[offset] = 0;
        }
        else
        {
            votes[offset]++;
        }
    }

    // Find the best vote
    int max_count = 0;
    Vec2 max_offset;
    for (auto& v : votes)
    {
        if (v.second > max_count)
        {
            max_count = v.second;
            max_offset.x() = v.first.first;
            max_offset.y() = v.first.second;
        }
    }

    const int minVotes = 5;
    if (max_count < minVotes)
    {
        return false;
    }

    // If needed, double the current checkerboard size to give the same discretization of space
    if (scale > 1)
    {
        CheckerBoard double_current_board(cur_board.rows() * scale, cur_board.cols() * scale);

        double_current_board.fill(UndefinedIndexT);

        for (int i = 0; i < cur_board.rows(); ++i)
        {
            for (int j = 0; j < cur_board.cols(); ++j)
            {
                double_current_board(i * scale, j * scale) = cur_board(i, j);
            }
        }

        cur_board = double_current_board;
    }

    int cur_board_left = max_offset.x();
    int cur_board_top = max_offset.y();
    int ref_board_left = 0;
    int ref_board_top = 0;

    if (cur_board_left < 0)
    {
        cur_board_left = 0;
        ref_board_left = -max_offset.x();
    }

    if (cur_board_top < 0)
    {
        cur_board_top = 0;
        ref_board_top = -max_offset.y();
    }

    const int width_output = std::max(cur_board_left + cur_board.cols(), ref_board_left + ref_board.cols());
    const int height_output = std::max(cur_board_top + cur_board.rows(), ref_board_top + ref_board.rows());

    CheckerBoard output(height_output, width_output);
    output.fill(UndefinedIndexT);

    ref_center.x() += ref_board_left;
    ref_center.y() += ref_board_top;

    for (int i = 0; i < ref_board.rows(); ++i)
    {
        for (int j = 0; j < ref_board.cols(); ++j)
        {
            output(ref_board_top + i, ref_board_left + j) = ref_board(i, j);
        }
    }

    for (int i = 0; i < cur_board.rows(); ++i)
    {
        for (int j = 0; j < cur_board.cols(); ++j)
        {
            const IndexT id = cur_board(i, j);
            if (id != UndefinedIndexT)
            {
                output(cur_board_top + i, cur_board_left + j) = cur_board(i, j);
            }
        }
    }

    std::vector<CheckerBoard> output_boards;
    output_boards.push_back(output);

    for (std::size_t idx = 1; idx < _boards.size(); ++idx)
    {
        if (idx != other)
        {
            output_boards.push_back(_boards[idx]);
        }
    }

    _boards = output_boards;

    return true;
}

void CheckerDetector::groupNestedCheckerboards()
{
    int current_scale = 1;
    bool change = true;

    if (_boards.size() == 0)
    {
        return;
    }

    Vec2i center = {_boards[0].cols() / 2, _boards[0].rows() / 2};

    while (change)
    {
        change = false;

        // Try to find another checkerboard with the same level
        for (std::size_t idx_compare = 1; idx_compare < _boards.size(); ++idx_compare)
        {
            if (groupNestedCheckerboardsPair(center, idx_compare, current_scale))
            {
                change = true;
                break;
            }
        }

        if (change)
            continue;

        current_scale *= 2;

        // Try to find another checkerboard with the next level
        for (std::size_t idx_compare = 1; idx_compare < _boards.size(); ++idx_compare)
        {
            if (groupNestedCheckerboardsPair(center, idx_compare, current_scale))
            {
                change = true;
                break;
            }
        }
    }

    const int max_x = std::max(center.x(), static_cast<int>(_boards[0].cols() - 1 - center.x()));
    const int max_y = std::max(center.y(), static_cast<int>(_boards[0].rows() - 1 - center.y()));

    const Vec2i newcenter = {max_x, max_y};
    const Vec2i update = newcenter - center;

    CheckerBoard newboard(max_y * 2 + 1, max_x * 2 + 1);
    newboard.fill(UndefinedIndexT);
    newboard.block(update.y(), update.x(), _boards[0].rows(), _boards[0].cols()) = _boards[0];
    _boards[0] = newboard;
}

}  // namespace calibration
}  // namespace aliceVision
