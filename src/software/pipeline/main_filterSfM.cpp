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

static const std::size_t MAX_LEAF_ELEMENTS = 64;

struct LandmarksAdaptator
{
    using Derived = LandmarksAdaptator; //!< In this case the dataset class is myself.
    using T = double;

    /// CRTP helper method
    inline const Derived& derived() const { return *static_cast<const Derived*>(this); }
    /// CRTP helper method
    inline Derived& derived() { return *static_cast<Derived*>(this); }

    const std::vector<Landmark> _data;
    LandmarksAdaptator(const std::vector<Landmark>& data)
        : _data(data)
    {
    }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return _data.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline T kdtree_get_pt(const size_t idx, int dim) const
    {
        return _data[idx].X(dim);
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

    inline PixSizeSearch(double radius, const std::vector<double>& pixSize, size_t i)
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

bool filterLandmarks(SfMData& sfmData, double radiusScale, bool useFeatureScale, int minNbObservationsPerLandmark)
{
    const auto initialNbLandmarks = sfmData.getLandmarks().size();
    std::vector<Landmark> landmarksData(initialNbLandmarks);
    {
        size_t i = 0;
        if (minNbObservationsPerLandmark > 0)
        {
            ALICEVISION_LOG_INFO("Removing landmarks having an insufficient number of observations: started.");
            for(auto& it : sfmData.getLandmarks())
            {
                if(it.second.observations.size() < minNbObservationsPerLandmark)
                    continue;
                landmarksData[i++] = it.second;
            }
            landmarksData.resize(i);
            ALICEVISION_LOG_INFO(
                "Removed " << (initialNbLandmarks - i) <<
                " landmarks out of " << initialNbLandmarks <<
                ", i.e. " << ((initialNbLandmarks - i) * 100.f / initialNbLandmarks) <<
                " % "
            );
            ALICEVISION_LOG_INFO("Removing landmarks having an insufficient number of observations: done.");
        }
        else
        {
            for(auto& it : sfmData.getLandmarks())
            {
                landmarksData[i++] = it.second;
            }
        }
    }

    if (radiusScale <= 0)
    {
        std::vector<std::pair<IndexT, Landmark>> filteredLandmarks(landmarksData.size());
        for(IndexT newIdx = 0; newIdx < landmarksData.size(); newIdx++)
        {
            filteredLandmarks[newIdx] = std::make_pair(newIdx, landmarksData[newIdx]);
        }
        sfmData.getLandmarks() = Landmarks(filteredLandmarks.begin(), filteredLandmarks.end());
        return true;
    }

    mvsUtils::MultiViewParams mp(sfmData, "", "", "", false);
    std::vector<double> landmarksPixSize(landmarksData.size());

    ALICEVISION_LOG_INFO("Computing pixel size: started.");
    #pragma omp parallel for
    for(auto i = 0; i < landmarksData.size(); i++)
    {
        const Landmark& landmark = landmarksData[i];

        // compute landmark pixSize
        double pixSize = 0.;
        int n = 0;
        for(const auto& observationPair : landmark.observations)
        {
            const IndexT viewId = observationPair.first;
            pixSize += mp.getCamPixelSize(
                Point3d(landmark.X.x(), landmark.X.y(), landmark.X.z()),
                mp.getIndexFromViewId(viewId),
                useFeatureScale ? observationPair.second.scale : 1);
            n++;
        }
        pixSize /= n;

        landmarksPixSize[i] = pixSize;
    }
    ALICEVISION_LOG_INFO("Computing pixel size: done.");

    //// sort landmarks by descending pixSize order
    //std::stable_sort(landmarksPixSize.begin(), landmarksPixSize.end(), std::greater<>{});

    ALICEVISION_LOG_INFO("Build nanoflann KdTree index.");
    LandmarksAdaptator data(landmarksData);
    KdTree tree(3, data, nanoflann::KDTreeSingleIndexAdaptorParams(MAX_LEAF_ELEMENTS));
    tree.buildIndex();
    ALICEVISION_LOG_INFO("KdTree created for " << landmarksData.size() << " points.");
    std::vector<bool> toRemove(landmarksData.size(), false);

    ALICEVISION_LOG_INFO("Identifying landmarks to remove based on pixel size: started.");

    size_t nbToRemove = 0;
    #pragma omp parallel for reduction(+:nbToRemove)
    for(auto i = 0; i < landmarksData.size(); i++)
    {
        PixSizeSearch search(landmarksPixSize[i] * radiusScale, landmarksPixSize, i);
        bool found = tree.findNeighbors(search, landmarksData[i].X.data(), nanoflann::SearchParams());
        if (found)
        {
            toRemove[i] = true;
            nbToRemove++;
        }
    }

    ALICEVISION_LOG_INFO(
        "Identified " << nbToRemove <<
        " landmarks to remove out of " << initialNbLandmarks <<
        ", i.e. " << (nbToRemove * 100.f / initialNbLandmarks) <<
        " % "
    );
    ALICEVISION_LOG_INFO("Identifying landmarks to remove: done.");

    ALICEVISION_LOG_INFO("Removing landmarks based on pixel size: started.");
    std::vector<std::pair<IndexT, Landmark>> filteredLandmarks(landmarksData.size() - nbToRemove);
    IndexT newIdx = 0;
    for (size_t i = 0; i < landmarksData.size(); i++)
    {
        if(!toRemove[i])
        {
            filteredLandmarks[newIdx++] = std::make_pair(newIdx, landmarksData[i]);
        }
    }
    sfmData.getLandmarks() = Landmarks(filteredLandmarks.begin(), filteredLandmarks.end());
    ALICEVISION_LOG_INFO("Removing landmarks based on pixel size: done.");

    return true;
}

bool filterObservations(SfMData& sfmData, int maxNbObservationsPerLandmark)
{
    std::vector<Landmark*> landmarksData(sfmData.getLandmarks().size());
    {
        size_t i = 0;
        for(auto& it : sfmData.getLandmarks())
        {
            landmarksData[i++] = &it.second;
        }
    }

    #pragma omp parallel for
    for(auto i = 0; i < landmarksData.size(); i++)
    {
        sfmData::Landmark& landmark = *landmarksData[i];

        // check number of observations
        if(landmark.observations.size() <= maxNbObservationsPerLandmark)
            continue;

        // // check angle between observations
        // if(!checkLandmarkMinObservationAngle(sfmData, landmark, minObservationAngle))

        // compute observation scores

        std::vector<std::pair<double, IndexT>> observationScores;
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
        for(const auto& observationScorePair : observationScores)
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
    int maxNbObservationsPerLandmark = 2;
    int minNbObservationsPerLandmark = 5;
    double radiusScale = 2;
    bool useFeatureScale = true;

    // user optional parameters
    std::vector<std::string> featuresFolders;
    std::vector<std::string> matchesFolders;
    std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);

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
        ("minNbObservationsPerLandmark", po::value<int>(&minNbObservationsPerLandmark)->default_value(minNbObservationsPerLandmark),
         "Minimum number of observations required to keep a landmark.")
        ("radiusScale", po::value<double>(&radiusScale)->default_value(radiusScale),
         "Scale factor applied to pixel size based radius filter applied to landmarks.")
        ("useFeatureScale", po::value<bool>(&useFeatureScale)->default_value(useFeatureScale),
         "If true, use feature scale for computing pixel size. Otherwise, use a scale of 1 pixel.")
        ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken(),
         "Path to folder(s) containing the extracted features.")
        ("matchesFolders,m", po::value<std::vector<std::string>>(&matchesFolders)->multitoken(),
         "Path to folder(s) in which computed matches are stored.")
        ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
        feature::EImageDescriberType_informations().c_str());

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

    if(radiusScale > 0 || minNbObservationsPerLandmark > 0)
    {
        ALICEVISION_LOG_INFO("Filtering landmarks: started.");
        filterLandmarks(sfmData, radiusScale, useFeatureScale, minNbObservationsPerLandmark);
        ALICEVISION_LOG_INFO("Filtering landmarks: done.");
    }
    
    if(maxNbObservationsPerLandmark > 0)
    {
        ALICEVISION_LOG_INFO("Filtering observations: started.");
        filterObservations(sfmData, maxNbObservationsPerLandmark);
        ALICEVISION_LOG_INFO("Filtering observations: done.");
    }


    sfmDataIO::Save(sfmData, outputSfmFilename, sfmDataIO::ESfMData::ALL);
    return EXIT_SUCCESS;

}
