// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>

#include "nanoflann.hpp"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <set>
#include <iterator>
#include <iomanip>
#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::image;
using namespace aliceVision::sfmData;
using namespace aliceVision::sfmDataIO;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

static const std::size_t MAX_LEAF_ELEMENTS = 10;

struct LandmarksAdaptator
{
    using Derived = LandmarksAdaptator; //!< In this case the dataset class is myself.
    using T = double;

    /// CRTP helper method
    inline const Derived& derived() const { return *static_cast<const Derived*>(this); }
    /// CRTP helper method
    inline Derived& derived() { return *static_cast<Derived*>(this); }

    const sfmData::Landmarks& _data;
    LandmarksAdaptator(const sfmData::Landmarks& data)
        : _data(data)
    {
    }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return _data.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline T kdtree_get_pt(const size_t idx, int dim) const
    {
        auto it = _data.begin();
        std::advance(it, idx);
        return it->second.X(dim);
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it
    //   again. Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& bb) const
    {
        return false;
    }
};

using KdTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, LandmarksAdaptator>,
    LandmarksAdaptator,
    3, /* dim */
    IndexT
>;

/**
 * A result-set class used when performing a radius based search.
 */
class PixSizeSearch
{
public:
    const double _radius_sq;
    const std::vector<double>& _pixSize;
    const double _pixSize_i;
    bool found = false;

    inline PixSizeSearch(double radius, const std::vector<double>& pixSize, int i)
        : _radius_sq(radius * radius)
        , _pixSize(pixSize)
        , _pixSize_i(pixSize[i])
    {
    }

    inline bool full() const { return found; }

    inline bool addPoint(double dist, IndexT index)
    {
        if(dist < _radius_sq && _pixSize[index] < _pixSize_i)
        {
            found = true;
            return false;
        }
        return true;
    }

    inline double worstDist() const { return _radius_sq; }
};

bool filterLandmarks(SfMData& sfmData, double radiusScale)
{
    mvsUtils::MultiViewParams mp(sfmData, "", "", "", false);
    std::vector<double> landmarksPixSize(sfmData.getLandmarks().size());

    ALICEVISION_LOG_INFO("Computing pixel size: started.");
    #pragma omp parallel for
    for(auto i = 0; i < sfmData.getLandmarks().size(); i++)
    {
        auto landmarkPair = sfmData.getLandmarks().begin();
        std::advance(landmarkPair, i);
        const sfmData::Landmark& landmark = landmarkPair->second;

        // compute landmark pixSize
        double pixSize = 0.;
        int n = 0;
        for(const auto& observationPair : landmark.observations)
        {
            const IndexT viewId = observationPair.first;
            pixSize += mp.getCamPixelSize(Point3d(landmark.X.x(), landmark.X.y(), landmark.X.z()),
                                          mp.getIndexFromViewId(viewId), observationPair.second.scale);
            n++;
        }
        pixSize /= n;

        landmarksPixSize[i] = pixSize;
    }
    ALICEVISION_LOG_INFO("Computing pixel size: done.");

    //// sort landmarks by descending pixSize order
    //std::stable_sort(landmarksPixSize.begin(), landmarksPixSize.end(), std::greater<>{});

    ALICEVISION_LOG_INFO("Build nanoflann KdTree index.");
    LandmarksAdaptator data(sfmData.getLandmarks());
    KdTree tree(3, data, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree.buildIndex();
    ALICEVISION_LOG_INFO("KdTree created for " << sfmData.getLandmarks().size() << " points.");
    std::vector<IndexT> newIdx(sfmData.getLandmarks().size());
    IndexT currentIdx = 0;

    ALICEVISION_LOG_INFO("Identifying landmarks to remove: started.");
    #pragma omp parallel for
    for (auto i = 0; i < sfmData.getLandmarks().size(); i++)
    {
        PixSizeSearch search(landmarksPixSize[i] * radiusScale, landmarksPixSize, i);
        bool found = tree.findNeighbors(search, sfmData.getLandmarks().at(i).X.data(), nanoflann::SearchParams());
        if (!found)
        {
            newIdx[i] = -1;
        }
        else
        {
            newIdx[i] = currentIdx++;
        }
    }
    ALICEVISION_LOG_INFO("Identifying landmarks to remove: done.");

    ALICEVISION_LOG_INFO("Removing landmarks: started.");
    Landmarks filteredLandmarks;
    #pragma omp parallel for
    for (auto i = 0; i < sfmData.getLandmarks().size(); i++)
    {
        if(newIdx[i] != -1)
        {
            filteredLandmarks[newIdx[i]] = sfmData.getLandmarks().at(i);
        }
    }
    sfmData.getLandmarks() = filteredLandmarks;
    ALICEVISION_LOG_INFO("Removing landmarks: done.");

    //// take only best observations
    //observationScores.resize(maxNbObservationsPerLandmark);



    //// replace the observations
    //Observations filteredObservations;
    //for(auto observationScorePair : observationScores)
    //{
    //    filteredObservations[observationScorePair.second] = landmark.observations[observationScorePair.second];
    //}
    //landmark.observations = filteredObservations;
    return true;
}

bool filterObservations(SfMData& sfmData, int maxNbObservationsPerLandmark)
{
    #pragma omp parallel for
    for(auto i = 0; i < sfmData.getLandmarks().size(); i++)
    {
        auto landmarkPair = sfmData.getLandmarks().begin();
        std::advance(landmarkPair, i);
        sfmData::Landmark& landmark = landmarkPair->second;

        // check number of observations
        if(landmark.observations.size() <= maxNbObservationsPerLandmark)
            continue;

        // // check angle between observations
        // if(!checkLandmarkMinObservationAngle(sfmData, landmark, minObservationAngle))

        // compute observation scores

        std::vector<std::pair<double, IndexT> > observationScores;
        observationScores.reserve(landmark.observations.size());

        for(const auto& observationPair : landmark.observations)
        {
            const IndexT viewId = observationPair.first;
            const sfmData::View& view = *(sfmData.getViews().at(viewId));
            const geometry::Pose3 pose = sfmData.getPose(view).getTransform();

            observationScores.push_back(std::pair<double, IndexT>(
                (pose.center() - landmark.X).squaredNorm(),
                viewId
            ));
        }

        // sort observations by ascending score order
        std::stable_sort(observationScores.begin(), observationScores.end());
        // take only best observations
        observationScores.resize(maxNbObservationsPerLandmark);

        // replace the observations
        Observations filteredObservations;
        for(auto observationScorePair : observationScores)
        {
            filteredObservations[observationScorePair.second] = landmark.observations[observationScorePair.second];
        }
        landmark.observations = filteredObservations;
    }
    return true;
}

int aliceVision_main(int argc, char *argv[])
{
    // command-line parameters

    // std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputSfmFilename;
    std::string outputSfmFilename;
    int maxNbObservationsPerLandmark = 5;
    double radiusScale = 2;

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputSfmFilename)->required(),
         "Input SfMData file.")
        ("output,o", po::value<std::string>(&outputSfmFilename)->required(),
         "Output SfMData file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("maxNbObservationsPerLandmark", po::value<int>(&maxNbObservationsPerLandmark)->default_value(maxNbObservationsPerLandmark),
         "Maximum number of allowed observations per landmark.")
        ("radiusScale", po::value<double>(&radiusScale)->default_value(radiusScale),
         "Scale factor applied to pixel size based radius filter applied to landmarks.");

    CmdLine cmdline("AliceVision SfM filtering.");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Create output dir
    {
        auto outDir = fs::path(outputSfmFilename).parent_path().string();
        if (!fs::exists(outDir))
            fs::create_directory(outDir);
    }

    // Read the input SfM scene
    SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, inputSfmFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << inputSfmFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    // filter SfM data
    ALICEVISION_LOG_INFO("Filtering landmarks: started.");
    bool success2 = filterLandmarks(sfmData, radiusScale);
    ALICEVISION_LOG_INFO("Filtering landmarks: done.");
    ALICEVISION_LOG_INFO("Filtering observations: started.");
    bool success1 = filterObservations(sfmData, maxNbObservationsPerLandmark);
    ALICEVISION_LOG_INFO("Filtering observations: done.");

    if(success1)
    {
        sfmDataIO::Save(sfmData, outputSfmFilename, sfmDataIO::ESfMData::ALL);
        return EXIT_SUCCESS;
    }

    return EXIT_FAILURE;
}
