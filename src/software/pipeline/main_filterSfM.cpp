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
    const std::vector<Landmark*> _data_ptr;
    bool usePtr;
    LandmarksAdaptator(const std::vector<Landmark>& data)
        : _data(data)
    {
        usePtr = false;
    }

    LandmarksAdaptator(const std::vector<Landmark*>& data)
        : _data_ptr(data)
    {
        usePtr = true;
    }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return usePtr ? _data_ptr.size() : _data.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline T kdtree_get_pt(const size_t idx, int dim) const
    {
        return usePtr ? _data_ptr[idx]->X(dim) : _data[idx].X(dim);
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
    size_t
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

    inline bool addPoint(double dist, size_t index)
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
        bool found = tree.findNeighbors(search, landmarksData[i].X.data(), nanoflann::SearchParameters());
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
    sfmData.getLandmarks() = std::move(Landmarks(filteredLandmarks.begin(), filteredLandmarks.end()));
    ALICEVISION_LOG_INFO("Removing landmarks based on pixel size: done.");

    return true;
}

bool filterObservations(SfMData& sfmData, int maxNbObservationsPerLandmark, int nbNeighbors, double neighborsInfluence,
                        int nbIterations)
{
    // store in vector for faster access
    std::vector<Landmark*> landmarksData(sfmData.getLandmarks().size());
    {
        size_t i = 0;
        for(auto& it : sfmData.getLandmarks())
        {
            landmarksData[i++] = &it.second;
        }
    }

    // contains the observing view ids for each landmark with their corresponding scores
    std::vector<std::pair<std::vector<IndexT>, std::vector<double>>> viewScoresData(landmarksData.size());

    ALICEVISION_LOG_INFO("Computing initial observation scores based on distance to observing view: started");
#pragma omp parallel for
    for(auto i = 0; i < landmarksData.size(); i++)
    {
        const sfmData::Landmark& landmark = *landmarksData[i];

        // compute observation scores
        const auto& nbObservations = landmark.observations.size();
        auto& [viewIds, viewScores] = viewScoresData[i];
        viewIds.reserve(nbObservations);
        viewScores.reserve(nbObservations);
        // accumulator for normalizing the scores
        double total = 0.;
        for(const auto& observationPair : landmark.observations)
        {
            const IndexT viewId = observationPair.first;
            const sfmData::View& view = *(sfmData.getViews().at(viewId));
            const geometry::Pose3 pose = sfmData.getPose(view).getTransform();

            viewIds.push_back(viewId);
            // score is the inverse of distance to observations
            const auto& v = 1. / (pose.center() - landmark.X).squaredNorm();
            total += v;
            viewScores.push_back(v);
        }

        // normalize view scores
        for(auto j = 0; j < nbObservations; j++)
        {
            viewScores[j] /= total;
        }

        // sort by ascending view id order
        // for consequent faster access

        // indices that sort the view ids
        std::vector<size_t> idx(nbObservations);
        std::iota(idx.begin(), idx.end(), 0);
        std::stable_sort(idx.begin(), idx.end(), [&v = viewIds](size_t i1, size_t i2) { return v[i1] < v[i2]; });
        // apply sorting to both view ids and view scores for correspondance
        auto ids_temp = viewIds;
        auto scores_temp = viewScores;
        for(auto j = 0; j < nbObservations; j++)
        {
            viewIds[j] = ids_temp[idx[j]];
            viewScores[j] = scores_temp[idx[j]];
        }
    }
    ALICEVISION_LOG_INFO("Computing initial observation scores based on distance to observing view: done");

    ALICEVISION_LOG_INFO("Computing landmark neighbors and distance-based weights: started");
    ALICEVISION_LOG_INFO("Build nanoflann KdTree index.");
    LandmarksAdaptator dataAdaptor(landmarksData);
    KdTree tree(3, dataAdaptor, nanoflann::KDTreeSingleIndexAdaptorParams(MAX_LEAF_ELEMENTS));
    tree.buildIndex();
    ALICEVISION_LOG_INFO("KdTree created for " << landmarksData.size() << " points.");
    // note that the landmark is a neighbor to itself with zero distance, hence the +/- 1
    int nbNeighbors_ = std::min(nbNeighbors, static_cast<int>(landmarksData.size() - 1)) + 1;
    // contains the neighbor landmarks ids with their corresponding weights
    std::vector<std::pair<std::vector<size_t>, std::vector<double>>> neighborsData(landmarksData.size());
#pragma omp parallel for
    for(auto i = 0; i < landmarksData.size(); i++)
    {
        const sfmData::Landmark& landmark = *landmarksData[i];
        auto& [indices_, weights_] = neighborsData[i];
        indices_.resize(nbNeighbors_);
        weights_.resize(nbNeighbors_);
        tree.knnSearch(landmark.X.data(), nbNeighbors_, &indices_[0], &weights_[0]);
        // a landmark is a neighbor to itself with zero distance, remove it
        indices_.erase(indices_.begin());
        weights_.erase(weights_.begin());
        // accumulator used for normalisation
        double total = 0.;
        for(auto& w : weights_)
        {
            // weight is the inverse of distance between a landmark and its neighbor
            w = 1. / std::sqrt(w);
            if(std::isinf(w))
                w = std::numeric_limits<double>::max();
            total += w;
        }
        if(std::isinf(total))
            total = std::numeric_limits<double>::max();
        // normalize weights
        for(auto& w : weights_)
        {
            w /= total;
        }
    }
    ALICEVISION_LOG_INFO("Computing landmark neighbors and distance-based weights: done");

    ALICEVISION_LOG_INFO("Propagating neighbors observation scores: started");
    // new view scores at iteration t
    std::vector<std::vector<double>> viewScoresData_t(landmarksData.size());
    for(auto i = 0; i < nbIterations; i++)
    {
        ALICEVISION_LOG_INFO("Iteration " << i << "...");
#pragma omp parallel for
        for(auto id = 0; id < landmarksData.size(); id++)
        {
            const auto& [viewIds, viewScores] = viewScoresData[id];
            auto& viewScores_acc = viewScoresData_t[id];
            // initialize to zero, will first contain the weighted average scores of neighbors
            viewScores_acc.assign(viewScores.size(), 0.);
            // accumulator for normalisation
            double viewScores_total = 0.;
            auto& [indices_, weights_] = neighborsData[id];
            for(auto j = 0; j < nbNeighbors; j++)
            {
                const auto& neighborId = indices_[j];
                const auto& neighborWeight = weights_[j];
                const auto& [viewIds_neighbor, viewScores_neighbor] = viewScoresData[neighborId];
                // loop over common views
                auto viewIds_it = viewIds.begin();
                auto viewIds_neighbor_it = viewIds_neighbor.begin();
                auto viewScores_neighbor_it = viewScores_neighbor.begin();
                auto viewScores_acc_it = viewScores_acc.begin();
                while(viewIds_it != viewIds.end() && viewIds_neighbor_it != viewIds_neighbor.end())
                {
                    if(*viewIds_it < *viewIds_neighbor_it)
                    {
                        ++viewIds_it;
                        ++viewScores_acc_it;
                    }
                    else
                    {
                        // if same view, accumulate weighted scores
                        if(!(*viewIds_neighbor_it < *viewIds_it))
                        {
                            const auto& v = *viewScores_neighbor_it * neighborWeight;
                            (*viewScores_acc_it) += v;
                            viewScores_total += v;
                            ++viewIds_it;
                            ++viewScores_acc_it;
                        }
                        ++viewIds_neighbor_it;
                        ++viewScores_neighbor_it;
                    }
                }
            }
            // if no common views with neighbor landmarks
            if(viewScores_total == 0.)
                continue;
            for(auto j = 0; j < viewScores_acc.size(); j++)
            {
                // normalize score and apply influence factor
                viewScores_acc[j] *= neighborsInfluence / viewScores_total;
                // combine weghted neighbor scores and the landmark's own scores
                viewScores_acc[j] += (1 - neighborsInfluence) * viewScores[j];
            }
        }
        // update scores at end of iteration
        double error = 0.;
#pragma omp parallel for reduction(+:error)
        for(auto id = 0; id < landmarksData.size(); id++)
        {
            double error_j = 0.;
            for(auto j = 0; j < viewScoresData_t[id].size(); j++)
            {
                const auto& v = viewScoresData_t[id][j] - viewScoresData[id].second[j];
                error_j += v * v;
            }
            error_j /= viewScoresData_t[id].size();
            error += error_j;
            viewScoresData[id].second = std::move(viewScoresData_t[id]);
        }
        error /= landmarksData.size();
        ALICEVISION_LOG_INFO("MSE at iteration " << i << ": " << error);
    }
    ALICEVISION_LOG_INFO("Propagating neighbors observation scores: done");

    ALICEVISION_LOG_INFO("Selecting observations with best scores: started");
#pragma omp parallel for
    for(auto i = 0; i < landmarksData.size(); i++)
    {
        sfmData::Landmark& landmark = *landmarksData[i];
        const auto& nbObservations = landmark.observations.size();
        auto& [viewIds, viewScores] = viewScoresData[i];

        // check number of observations
        if(landmark.observations.size() <= maxNbObservationsPerLandmark)
            continue;

        // sort by descending view score order
        std::vector<size_t> idx(nbObservations);
        std::iota(idx.begin(), idx.end(), 0);
        std::stable_sort(idx.begin(), idx.end(), [&v = viewScores](size_t i1, size_t i2) { return v[i1] > v[i2]; });

        // keep only observations with best scores
        Observations filteredObservations;
        for(auto j = 0; j < maxNbObservationsPerLandmark; j++)
        {
            const auto& viewId = viewIds[idx[j]];
            filteredObservations[viewId] = landmark.observations[viewId];
        }
        landmark.observations = std::move(filteredObservations);
    }
    ALICEVISION_LOG_INFO("Selecting observations with best scores: done");

    return true;
}

int aliceVision_main(int argc, char *argv[])
{
    // command-line parameters

    std::string inputSfmFilename;
    std::string outputSfmFilename;
    int maxNbObservationsPerLandmark = 2;
    int minNbObservationsPerLandmark = 5;
    double radiusScale = 2;
    bool useFeatureScale = true;
    int nbNeighbors = 10;
    double neighborsInfluence = 0.5;
    int nbIterations = 5;

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
        ("nbNeighbors", po::value<int>(&nbNeighbors)->default_value(nbNeighbors),
         "Number of neighbor landmarks used in making the decision for best observations.")
        ("neighborsInfluence", po::value<double>(&neighborsInfluence)->default_value(neighborsInfluence),
         "Specifies how much influential the neighbors are in selecting the best observations."
         "Between 0. and 1., the closer to 1., the more influencial the neighborhood is.")
        ("nbIterations", po::value<int>(&nbIterations)->default_value(nbIterations),
         "Number of iterations to propagate neighbors information.")
        ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken(),
         "Path to folder(s) containing the extracted features.")
        ("matchesFolders,m", po::value<std::vector<std::string>>(&matchesFolders)->multitoken(),
         "Path to folder(s) in which computed matches are stored.")
        ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
        feature::EImageDescriberType_informations().c_str());

    CmdLine cmdline("AliceVision SfM filtering."); // TODO add description
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
        filterObservations(sfmData, maxNbObservationsPerLandmark, nbNeighbors, neighborsInfluence, nbIterations);
        ALICEVISION_LOG_INFO("Filtering observations: done.");
    }


    sfmDataIO::Save(sfmData, outputSfmFilename, sfmDataIO::ESfMData::ALL);
    return EXIT_SUCCESS;

}
