// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "poseFilter.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/geometry/lie.hpp>
#include <aliceVision/geometry/Pose3.hpp>
#include <aliceVision/sfm/sfmFilters.hpp>

#include <ceres/rotation.h>

namespace aliceVision {
namespace sfm {

bool poseFilter::process(sfmData::SfMData& sfmData, const bool filterPosition, const bool filterRotation, const int scaleFactor, const int iterationCount)
{
    using namespace Eigen;

    ALICEVISION_LOG_INFO("poseFilter::process start");

    const int viewCount = sfmData.getViews().size();

    std::vector<IndexT> viewIdsVec(viewCount);

    // Get the temporally ordered view IDs in viewIdsVec
    if (viewCount == 0 || !getOrderedViewIds(sfmData, viewIdsVec))
    {
        return false;
    }

    tempFilter tFilter;

    tFilter.init();

    MatrixXd viewRotations(4, viewCount);
    MatrixXd viewCenters(3, viewCount);

    // Get the temporally ordered view positions and orientations
    for (int frameIdx = 0; frameIdx < viewCount; frameIdx++)
    {
        const sfmData::View& frameView = sfmData.getView(viewIdsVec[frameIdx]);
        const sfmData::CameraPose framePose = sfmData.getPose(frameView);
        viewCenters.col(frameIdx) = framePose.getTransform().center();
        AngleAxisd aa(framePose.getTransform().rotation());
        viewRotations.col(frameIdx) << aa.angle(), aa.axis();
    }

    // Apply a temporal filter to view positions
    if (filterPosition)
    {
        viewCenters = tFilter.applyMultiscale(viewCenters, scaleFactor, iterationCount, false);
    }

    // Apply a temporal filter to view orientations
    if (filterRotation)
    {
        viewRotations = tFilter.applyMultiscale(viewRotations, scaleFactor, iterationCount, true);
    }

    // Save the temporally filtered poses
    for (int frameIdx = 0; frameIdx < viewCount; frameIdx++)
    {
        AngleAxisd aa(viewRotations(0, frameIdx), viewRotations(seqN(1,3), frameIdx));
        geometry::Pose3 newPose(aa.toRotationMatrix(), viewCenters.col(frameIdx));
        const sfmData::View& frameView = sfmData.getView(viewIdsVec[frameIdx]);
        sfmData.setPose(frameView, sfmData::CameraPose(newPose));
   }

    ALICEVISION_LOG_INFO("poseFilter::process end");
    return true;
}


bool poseFilter::interpolateMissingPoses(sfmData::SfMData& sfmData, const bool ignoreFirstAndLast)
{
    using namespace Eigen;

    ALICEVISION_LOG_INFO("poseFilter::interpolateMissingPoses start");

    const int viewCount = sfmData.getViews().size();

    std::vector<IndexT> viewIdsVec(viewCount);

    IndexT firstViewFrameId = sfmData.getViews().begin()->second->getFrameId();
    IndexT minFrameId = firstViewFrameId;  // Arbitrary frameId init
    IndexT maxFrameId = firstViewFrameId;  // Arbitrary frameId init
    IndexT minFrameIdWithPose;
    IndexT maxFrameIdWithPose;

    bool existingPoseFound = false;

    // Get the frameIDs range and the frameID of the first and last views with an existing pose
    for (const auto& [viewID, viewPtr] : sfmData.getViews())
    {
        const IndexT frameId = viewPtr->getFrameId();
        if (frameId < minFrameId)
        {
            minFrameId = frameId;
        }
        if (frameId > maxFrameId)
        {
            maxFrameId = frameId;
        }
        if ( sfmData.existsPose(*viewPtr) )
        {
            if (!existingPoseFound || frameId < minFrameIdWithPose)
                minFrameIdWithPose = frameId;

            if (!existingPoseFound || frameId > maxFrameIdWithPose)
                maxFrameIdWithPose = frameId;

            existingPoseFound = true;
        }
    }

    const int frameIdRange = maxFrameId - minFrameId + 1;

    if ( !existingPoseFound || (frameIdRange != viewCount) )
    {
        return false;
    }

    // Store the temporally ordered view IDs
    for (const auto& [viewID, viewPtr] : sfmData.getViews())
    {
        const IndexT frameId = viewPtr->getFrameId();
        viewIdsVec[frameId-minFrameId] = viewID;

        if (!sfmData.existsPose(*viewPtr))
            ALICEVISION_LOG_INFO(" frameId without pose : " << frameId);
    }

    const sfmData::View& lastValidView = sfmData.getView(viewIdsVec[minFrameIdWithPose-minFrameId]);
    sfmData::CameraPose lastValidPose = sfmData.getPose(lastValidView);

    VectorX<bool> missingPosesMask(viewCount);
    missingPosesMask.setZero();

    // Initialize the pose of the views without pose
    for (int frameIdx = 0; frameIdx < viewCount; frameIdx++)
    {
        IndexT frameViewID = viewIdsVec[frameIdx];
        const sfmData::View& currentView = sfmData.getView(frameViewID);

        if (!sfmData.existsPose(currentView))
        {
            // Create a binary mask to be able to restore the original values
            missingPosesMask(frameIdx) = true;

            if (!ignoreFirstAndLast || (minFrameIdWithPose < (frameIdx + minFrameId) && (frameIdx + minFrameId) < maxFrameIdWithPose) )
                sfmData.setPose(currentView, lastValidPose);
        }
        else
            lastValidPose = sfmData.getPose(currentView);
    }

    tempFilter tFilter;

    tFilter.init();

    MatrixXd viewRotations(4, viewCount);
    MatrixXd viewCenters(3, viewCount);

    // Get the temporally ordered view positions and orientations
    for (int frameIdx = 0; frameIdx < viewCount; frameIdx++)
    {
        const sfmData::View& frameView = sfmData.getView(viewIdsVec[frameIdx]);
        const sfmData::CameraPose framePose = sfmData.getPose(frameView);
        viewCenters.col(frameIdx) = framePose.getTransform().center();
        AngleAxisd aa(framePose.getTransform().rotation());
        viewRotations.col(frameIdx) << aa.angle(), aa.axis();
    }

    int scaleFactor = 8;
    int iterationCount = 1000;

    // Apply a temporal filter to view positions
    viewCenters = tFilter.applyMultiscale(viewCenters, scaleFactor, iterationCount, false, missingPosesMask);

    // Apply a temporal filter to view orientations
    viewRotations = tFilter.applyMultiscale(viewRotations, scaleFactor, iterationCount, true, missingPosesMask);

    // Save the temporally filtered poses
    for (int frameIdx = 0; frameIdx < viewCount; frameIdx++)
    {
        if (!missingPosesMask(frameIdx))
            continue;

        IndexT frameViewID = viewIdsVec[frameIdx];
        AngleAxisd aa(viewRotations(0, frameIdx), viewRotations(seqN(1,3), frameIdx));
        geometry::Pose3 newPose(aa.toRotationMatrix(), viewCenters.col(frameIdx));
        const sfmData::View& frameView = sfmData.getView(frameViewID);
        sfmData.setPose(frameView, sfmData::CameraPose(newPose));
   }

    ALICEVISION_LOG_INFO("poseFilter::interpolateMissingPoses end");
    return true;
}


bool poseFilter::getOrderedViewIds(sfmData::SfMData& sfmData, std::vector<IndexT>& viewIdsVec)
{
    const int viewCount = sfmData.getViews().size();

    IndexT firstViewFrameId = sfmData.getViews().begin()->second->getFrameId();
    IndexT minFrameId = firstViewFrameId;  // Arbitrary frameId init
    IndexT maxFrameId = firstViewFrameId;  // Arbitrary frameId init
    IndexT minFrameIdWithPose;

    bool existingPoseFound = false;

    // Get the frameIDs range and the frameID of the first view with an existing pose
    for (const auto& [viewID, viewPtr] : sfmData.getViews())
    {
        const IndexT frameId = viewPtr->getFrameId();
        if (frameId < minFrameId)
        {
            minFrameId = frameId;
        }
        if ( (!existingPoseFound || frameId < minFrameIdWithPose) && sfmData.existsPose(*viewPtr) )
        {
            existingPoseFound = true;
            minFrameIdWithPose = frameId;
        }
        if (frameId > maxFrameId)
            maxFrameId = frameId;
    }

    const int frameIdRange = maxFrameId - minFrameId + 1;

    ALICEVISION_LOG_DEBUG(" minFrameId : " << minFrameId);

    ALICEVISION_LOG_DEBUG(" maxFrameId : " << maxFrameId);

    if ( !existingPoseFound || (frameIdRange != viewCount) )
    {
        return false;
    }

    // Store the temporally ordered view IDs
    for (const auto& [viewID, viewPtr] : sfmData.getViews())
    {
        const IndexT frameId = viewPtr->getFrameId();
        viewIdsVec[frameId-minFrameId] = viewID;

        if (!sfmData.existsPose(*viewPtr))
            ALICEVISION_LOG_INFO(" frameId without pose : " << frameId);
    }

    ALICEVISION_LOG_DEBUG(" minFrameIdWithPose : " << minFrameIdWithPose);

    const sfmData::View& lastValidView = sfmData.getView(viewIdsVec[minFrameIdWithPose-minFrameId]);
    sfmData::CameraPose lastValidPose = sfmData.getPose(lastValidView);

    // Fill in the blanks within the camera poses list (so that every view gets a pose)
    // using the first view with an existing pose for the first views without existing pose
    // and using the last known view with an existing pose for any other view without existing pose

    for (IndexT frameViewID : viewIdsVec)
    {
        const sfmData::View& currentView = sfmData.getView(frameViewID);

        if (!sfmData.existsPose(currentView))
            sfmData.setPose(currentView, lastValidPose);
        else
            lastValidPose = sfmData.getPose(currentView);
    }

    return true;
}


bool getOrderedPoseIds(const sfmData::SfMData& sfmData, std::vector<IndexT>& poseIdsVec, IndexT& firstViewWithPose, IndexT& lastViewWithPose)
{
    std::map<std::string, std::vector<IndexT>> candidateViewIdLists;
    std::map<std::string, std::vector<IndexT>> candidateFrameIdLists;

    // Group views by common filename pattern
    for (const auto& [viewID, viewPtr] : sfmData.getViews())
    {
        const std::filesystem::path imagePath = std::filesystem::path(viewPtr->getImage().getImagePath());
        std::string parentPath = imagePath.parent_path().string();
        // Remove every digit in the filename
        std::string filePattern = std::regex_replace(imagePath.filename().string(), std::regex("\\d"), "");
        candidateViewIdLists[parentPath+filePattern].push_back(viewID);
        candidateFrameIdLists[parentPath+filePattern].push_back(viewPtr->getFrameId());
    }

    IndexT minFrameId;

    int longestValidRange = 1;
    std::string longestValidList;

    // Select the longest list of views having no hole in the frameIds range
    for (const auto& [filePattern, frameIDs] : candidateFrameIdLists)
    {
        auto [listMin, listMax] = std::minmax_element(frameIDs.begin(), frameIDs.end());
        auto listRange = *listMax + 1 - *listMin;
        bool noHole = frameIDs.size() == listRange;
        if (noHole && listRange > longestValidRange)
        {
            longestValidRange = listRange;
            longestValidList = filePattern;
            minFrameId = *listMin;
        }
    }

    if (longestValidRange == 1)
        return false;

    poseIdsVec.resize(longestValidRange);

    bool existingPoseFound = false;
    IndexT minFrameIdWithPose;
    IndexT maxFrameIdWithPose;

    // Get the frameID of the first and last views with an existing pose
    // And store the temporally ordered poseIDs
    for (const auto& viewID : candidateViewIdLists[longestValidList])
    {
        const sfmData::View& view = sfmData.getView(viewID);

        if ( sfmData.existsPose(view) )
        {
            const IndexT frameId = view.getFrameId();

            if (!existingPoseFound || frameId < minFrameIdWithPose)
                minFrameIdWithPose = frameId;

            if (!existingPoseFound || frameId > maxFrameIdWithPose)
                maxFrameIdWithPose = frameId;

            existingPoseFound = true;

            poseIdsVec[frameId-minFrameId] = view.getPoseId();
        }
    }

    if (!existingPoseFound)
        return false;

    firstViewWithPose = minFrameIdWithPose - minFrameId;
    lastViewWithPose = maxFrameIdWithPose - minFrameId;

    return true;
}

} // namespace sfm
} // namespace aliceVision


bool tempFilter::init()
{
    using namespace Eigen;
    using namespace Eigen::indexing;

    // Savitzky-Golay smoothing filter
    // Reference: https://en.wikipedia.org/wiki/Savitzky-Golay_filter

    filterCoeff.resize(kernelSize);
    // Savitzky-Golay smoothing filter coefficients (window size 9, polynomial order 2)
    filterCoeff << -21., 14., 39., 54., 59., 54., 39., 14., -21.;
    filterCoeff = filterCoeff / 231.;

    VectorXd filterCoeff_b(kernelSize);
    // Savitzky-Golay linear term coefficients
    filterCoeff_b << -4., -3., -2., -1., 0., 1., 2., 3., 4.;
    filterCoeff_b = filterCoeff_b / 60.;

    VectorXd filterCoeff_c(kernelSize);
    // Savitzky-Golay quadratic term coefficients
    filterCoeff_c << 28., 7., -8., -17., -20., -17., -8., 7., 28.;
    filterCoeff_c = filterCoeff_c / 924.;

    MatrixXd filterCoeff_x = MatrixXd(kernelSize, kernelSize);

    // The above filter coefficients are defined for the center position of the filter window
    // Below, the filter coefficients for any position in the filter window are computed
    for (int coeffIndex = 0; coeffIndex < kernelSize; coeffIndex++)
    {
        double x = coeffIndex - kernelSize / 2;
        for (int filterIndex = 0; filterIndex < kernelSize; filterIndex++)
        {
            filterCoeff_x(coeffIndex, filterIndex) = filterCoeff(filterIndex) + filterCoeff_b(filterIndex) * x + filterCoeff_c(filterIndex) * (x * x);
        }
    }

    // We extract the filter coefficients for the window tail and the window head (respectively the first and last positions)
    // These filters are respectively used for the first frames and the last frames
    tailFilter = filterCoeff_x(all, seq(0, last/2-1));
    headFilter = filterCoeff_x(all, seq(last/2+1, last));

    diffFilterCoeff.resize(kernelSize-1);
    // These filter coefficients (dfc) applied to the temporal delta signal
    // are equivalent to filterCoeff (fc) applied to the same signal (s)
    // i.e. sum( dfc(i) * (s(i+1)-s(i)) ) = sum( fc(i) * s(i) )
    diffFilterCoeff << 21., 7., -32., -86., 86., 32., -7., -21.;
    diffFilterCoeff = diffFilterCoeff / 231.;

    MatrixXd diffFilterCoeff_x = MatrixXd(kernelSize-1, kernelSize);

    // These filter coefficients (diffFilterCoeff_x) are equivalent to filterCoeff_x for the temporal delta signal
    for (int filterIndex = 0; filterIndex < kernelSize; filterIndex++)
    {
        if (filterIndex > 0)
            diffFilterCoeff_x(0, filterIndex) = -filterCoeff_x(0, filterIndex);

        for (int coeffIndex = 1; coeffIndex < filterIndex; coeffIndex++)
            diffFilterCoeff_x(coeffIndex, filterIndex) = diffFilterCoeff_x(coeffIndex-1, filterIndex) - filterCoeff_x(coeffIndex, filterIndex);

        if (filterIndex < kernelSize-1)
            diffFilterCoeff_x(kernelSize-2, filterIndex) = filterCoeff_x(kernelSize-1, filterIndex);

        for (int coeffIndex = kernelSize-3; coeffIndex >= filterIndex; coeffIndex--)
            diffFilterCoeff_x(coeffIndex, filterIndex) = diffFilterCoeff_x(coeffIndex+1, filterIndex) + filterCoeff_x(coeffIndex+1, filterIndex);
    }

    tailDiffFilter = diffFilterCoeff_x(all, seq(0, last/2-1));
    headDiffFilter = diffFilterCoeff_x(all, seq(last/2+1, last));

    initialized = true;
    return true;
}


bool tempFilter::applyCoreFilter(Eigen::MatrixXd& inputSignal, Eigen::MatrixXd& filteredSignal, bool isDiffSignal)
{
    using namespace Eigen;
    using namespace indexing;

    const int innersize = inputSignal.cols() - 2 * (kernelSize/2) + (isDiffSignal ? 1 : 0);

    if (inputSignal.cols() < kernelSize - (isDiffSignal ? 1 : 0))
        return false;

    if (isDiffSignal)
    {
        // Apply the filter on temporal delta signal
        filteredSignal(all, seqN(kernelSize/2, innersize)) = inputSignal(all, seqN(fix<0>, innersize)) * diffFilterCoeff(0)
                                                           + inputSignal(all, seqN(fix<1>, innersize)) * diffFilterCoeff(1)
                                                           + inputSignal(all, seqN(fix<2>, innersize)) * diffFilterCoeff(2)
                                                           + inputSignal(all, seqN(fix<3>, innersize)) * diffFilterCoeff(3)
                                                           + inputSignal(all, seqN(fix<4>, innersize)) * diffFilterCoeff(4)
                                                           + inputSignal(all, seqN(fix<5>, innersize)) * diffFilterCoeff(5)
                                                           + inputSignal(all, seqN(fix<6>, innersize)) * diffFilterCoeff(6)
                                                           + inputSignal(all, seqN(fix<7>, innersize)) * diffFilterCoeff(7);

        // The first and the last frames use specific filters
        filteredSignal(all, seqN(fix<0>, fix<4>)) = inputSignal(all, seqN(fix<0>, fix<8>)) * tailDiffFilter;
        filteredSignal(all, seqN(last-fix<3>, fix<4>)) = inputSignal(all, seqN(last-fix<7>, fix<8>)) * headDiffFilter;
    }
    else
    {
        // Apply the filter
        filteredSignal(all, seqN(kernelSize/2, innersize)) = inputSignal(all, seqN(fix<0>, innersize)) * filterCoeff(0)
                                                           + inputSignal(all, seqN(fix<1>, innersize)) * filterCoeff(1)
                                                           + inputSignal(all, seqN(fix<2>, innersize)) * filterCoeff(2)
                                                           + inputSignal(all, seqN(fix<3>, innersize)) * filterCoeff(3)
                                                           + inputSignal(all, seqN(fix<4>, innersize)) * filterCoeff(4)
                                                           + inputSignal(all, seqN(fix<5>, innersize)) * filterCoeff(5)
                                                           + inputSignal(all, seqN(fix<6>, innersize)) * filterCoeff(6)
                                                           + inputSignal(all, seqN(fix<7>, innersize)) * filterCoeff(7)
                                                           + inputSignal(all, seqN(fix<8>, innersize)) * filterCoeff(8);

        // The first and the last frames use specific filters
        filteredSignal(all, seqN(fix<0>, fix<4>)) = inputSignal(all, seqN(fix<0>, fix<9>)) * tailFilter;
        filteredSignal(all, seqN(last-fix<3>, fix<4>)) = inputSignal(all, seqN(last-fix<8>, fix<9>)) * headFilter;
    }

    return true;
}


Eigen::MatrixXd tempFilter::apply(Eigen::MatrixXd& inputSignal, bool isAngle)
{
    using namespace aliceVision::SO3;
    using namespace Eigen;

    assert(initialized);

    if (inputSignal.cols() < kernelSize)
        return inputSignal;

    MatrixXd filteredSignal(inputSignal.rows(), inputSignal.cols());

    // The filter used for the angles is equivalent to the filter used for the positions
    // but the filter coefficients are designed to work with temporal delta signals
    // This means the filter coefficients (dfc) applied to the temporal delta signal
    // are equivalent to filterCoeff (fc) applied to the same signal (s)
    // i.e. sum( dfc(i) * (s(i+1)-s(i)) ) = sum( fc(i) * s(i) )
    // Delta signals are used for the angles as filter operations over rotation angles are not well-defined
    // and are more accurate for small rotation angles

    if (isAngle)
    {
        MatrixXd diffSignal(3, inputSignal.cols()-1);
        // If rotation angles were easy to deal with:
        // diffSignal = inputSignal(all, seqN(1, inputSignal.cols()-1)) - inputSignal(all, seqN(0, inputSignal.cols()-1));

        // The temporal delta signal is computed using rotations matrices, and then converted into so3
        for (int col = 0; col < inputSignal.cols() - 1; col++)
        {
            AngleAxisd prevAA(inputSignal(0, col), inputSignal(seqN(1,3), col));
            AngleAxisd currAA(inputSignal(0, col+1), inputSignal(seqN(1,3), col+1));

            AngleAxisd diffAA = AngleAxisd(Quaterniond(currAA) * Quaterniond(prevAA).conjugate());
            diffSignal.col(col) = diffAA.angle() * diffAA.axis();
        }

        // The filter is designed to use as input the temporal diff of the input signal,
        // and to output a delta to the input signal
        // i.e. diffSignal = inputSignal_(t+1) - inputSignal_(t)
        //      filteredSignal = inputSignal_(t) + diffFilteredSignal_(t)
        MatrixXd diffFilteredSignal(3, inputSignal.cols());

        // Apply the filter on the temporal delta signal
        applyCoreFilter(diffSignal, diffFilteredSignal, true);

        // If rotation angles were easy to deal with:
        // filteredSignal = inputSignal + diffFilteredSignal;

        for (int col = 0; col < inputSignal.cols(); col++)
        {
            AngleAxisd inputAA = AngleAxisd(inputSignal(0, col), inputSignal(seqN(1,3), col));

            double rotAngle = diffFilteredSignal.col(col).norm();
            AngleAxisd diffFilteredAA(rotAngle, diffFilteredSignal.col(col).normalized());

            AngleAxisd resAA = AngleAxisd(Quaterniond(diffFilteredAA) * Quaterniond(inputAA));
            filteredSignal.col(col) << resAA.angle(), resAA.axis();
        }
    }
    else
    {
        // Apply the filter
        applyCoreFilter(inputSignal, filteredSignal, false);
    }

    return filteredSignal;
}


Eigen::MatrixXd tempFilter::applyMultiscale(Eigen::MatrixXd& inputSignal, const unsigned int scaleFactor, const int iterationCount, bool isAngle, const Eigen::VectorX<bool>& posesMask)
{
    using namespace Eigen;
    using namespace indexing;

    MatrixXd filteredSignal(inputSignal);

    // This filter extends the range of the original filter by applying the filter to the sub-sampled signal

    // The multi-scale filter applies filtering at decreasing scales
    // controlled by the following constant
    const double SCALE_REDUCTION_FACTOR = 1.4;

    // An optional mask can be used to apply the filter just on some poses
    bool useMask = posesMask.cols() != 0;
    MatrixX<bool> mask;

    for (int scaleF = scaleFactor; scaleF >= 1; scaleF = (scaleF > 1) ? round(double(scaleF) / SCALE_REDUCTION_FACTOR) : 0)
    {
        ALICEVISION_LOG_DEBUG(" Filter scale factor : " << scaleF);

        if (inputSignal.cols() < scaleF)
            continue;

        for (int phase = 0; phase < scaleF; phase++)
        {
            MatrixXd scaledSignal(filteredSignal(all, seq(phase, last, scaleF)));

            if (useMask)
                mask = posesMask(seq(phase, last, scaleF)).rowwise().replicate(inputSignal.rows()).transpose();

            for (int iterFilter = 0; iterFilter < iterationCount; iterFilter++)
            {
                scaledSignal = apply(scaledSignal, isAngle);

                if (useMask)
                {
                    // Restore the original poses
                    scaledSignal = mask.select(scaledSignal, inputSignal(all, seq(phase, last, scaleF)));
                }
            }
            filteredSignal(all, seq(phase, last, scaleF)) = scaledSignal;
        }
    }

    return filteredSignal;
}
