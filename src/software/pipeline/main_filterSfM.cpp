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
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

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
using namespace boost::accumulators;

static const std::size_t MAX_LEAF_ELEMENTS = 64;

struct FilterParams
{
    struct FilterLandmarksParams
    {
        bool enabled = true;
        struct FilterLandmarksStep1Params
        {
            bool enabled = true;
            int minNbObservationsPerLandmark = 3;
        } step1;
        struct FilterLandmarksStep2Params
        {
            bool enabled = true;
            int maxNbObservationsPerLandmark = 2;
            int nbNeighbors3D = 10;
        } step2;
        struct FilterLandmarksStep3Params
        {
            bool enabled = true;
            double radiusScale = 2;
            bool useFeatureScale = true;
        } step3;
    } filterLandmarks;
    struct FilterObservations3DParams
    {
        bool enabled = true;
        int maxNbObservationsPerLandmark = 2;
        bool propagationEnabled = true;
        int nbNeighbors3D = 10;
        double neighborsInfluence = 0.5;
        int nbIterations = 5;
        bool dampingEnabled = true;
        double dampingFactor = 0.5;
    } filterObservations3D;
    struct FilterObservations2DParams
    {
        bool enabled = true;
        int nbNeighbors2D = 5;
        float percentile = 0.95f;
        double maskRadiusThreshold = 0.1;
    } filterObservations2D;
};

std::istream& operator>>(std::istream& in, FilterParams& params)
{
    std::string token;
    in >> token;
    std::vector<std::string> splitParams;
    boost::split(splitParams, token, boost::algorithm::is_any_of(":"));
    if(splitParams.size() != 21)
        throw std::invalid_argument("Failed to parse FilterParams from: \n" + token);
    int i = 0;

    params.filterLandmarks.enabled = boost::to_lower_copy(splitParams[i++]) == "true";

    params.filterLandmarks.step1.enabled = boost::to_lower_copy(splitParams[i++]) == "true";
    params.filterLandmarks.step1.minNbObservationsPerLandmark = boost::lexical_cast<int>(splitParams[i++]);

    params.filterLandmarks.step2.enabled = boost::to_lower_copy(splitParams[i++]) == "true";
    params.filterLandmarks.step2.maxNbObservationsPerLandmark = boost::lexical_cast<int>(splitParams[i++]);
    params.filterLandmarks.step2.nbNeighbors3D = boost::lexical_cast<int>(splitParams[i++]);

    params.filterLandmarks.step3.enabled = boost::to_lower_copy(splitParams[i++]) == "true";
    params.filterLandmarks.step3.radiusScale = boost::lexical_cast<double>(splitParams[i++]);
    params.filterLandmarks.step3.useFeatureScale = boost::to_lower_copy(splitParams[i++]) == "true";

    params.filterObservations3D.enabled = boost::to_lower_copy(splitParams[i++]) == "true";
    params.filterObservations3D.maxNbObservationsPerLandmark = boost::lexical_cast<int>(splitParams[i++]);
    params.filterObservations3D.propagationEnabled = boost::to_lower_copy(splitParams[i++]) == "true";
    params.filterObservations3D.nbNeighbors3D = boost::lexical_cast<int>(splitParams[i++]);
    params.filterObservations3D.neighborsInfluence = boost::lexical_cast<double>(splitParams[i++]);
    params.filterObservations3D.nbIterations = boost::lexical_cast<int>(splitParams[i++]);
    params.filterObservations3D.dampingEnabled = boost::to_lower_copy(splitParams[i++]) == "true";
    params.filterObservations3D.dampingFactor = boost::lexical_cast<double>(splitParams[i++]);

    params.filterObservations2D.enabled = boost::to_lower_copy(splitParams[i++]) == "true";
    params.filterObservations2D.nbNeighbors2D = boost::lexical_cast<int>(splitParams[i++]);
    params.filterObservations2D.percentile = boost::lexical_cast<float>(splitParams[i++]);
    params.filterObservations2D.maskRadiusThreshold = boost::lexical_cast<double>(splitParams[i++]);

    return in;
}

// required for linux build
inline std::ostream& operator<<(std::ostream& os, const FilterParams& params)
{
    return os;
}

struct ObservationsAdaptator
{
    using Derived = ObservationsAdaptator; //!< In this case the dataset class is myself.
    using T = double;

    /// CRTP helper method
    inline const Derived& derived() const { return *static_cast<const Derived*>(this); }
    /// CRTP helper method
    inline Derived& derived() { return *static_cast<Derived*>(this); }

    const std::vector<Observation> _data;
    ObservationsAdaptator(const std::vector<Observation>& data)
        : _data(data)
    {
    }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return _data.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline T kdtree_get_pt(const size_t idx, int dim) const { return _data[idx].x(dim); }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it
    //   again. Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& bb) const
    {
        return false;
    }
};

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

using KdTree2D = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, ObservationsAdaptator>,
    ObservationsAdaptator, 2 /* dim */, size_t>;

using KdTree3D = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, LandmarksAdaptator>,
    LandmarksAdaptator, 3 /* dim */,size_t>;

class RadiusKnnSearch
{
public:
    const double _radius_sq;
    const int _nb_neighbors;
    int nb_found = 0;

    inline RadiusKnnSearch(double radius, int k)
        : _radius_sq(radius * radius)
        , _nb_neighbors(k)
    {
    }

    inline bool full() const { return nb_found == _nb_neighbors; }

    inline bool addPoint(double dist, IndexT index)
    {
        if(dist < _radius_sq)
        {
            nb_found++;
            return nb_found < _nb_neighbors;
        }
        return true;
    }

    inline double worstDist() const { return _radius_sq; }
};

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
        // strict comparison because a point in the tree can be a neighbor of itself
        if(dist < _radius_sq && _pixSize[index] < _pixSize_i)
        {
            found = true;
            return false;
        }
        return true;
    }

    inline double worstDist() const { return _radius_sq; }
};

using ObservationsPerView = stl::flat_map<std::size_t, std::pair<std::vector<Observation>, std::vector<Landmark*>>>;

/**
 * @brief Get the landmark observations of camera views
 * with the corresponding landmarks information.
 * @param[in] sfmData: A given SfMData
 * @return Observation information per camera view
 */
ObservationsPerView getObservationsPerViews(SfMData& sfmData)
{
    ObservationsPerView observationsPerView;
    for(auto& landIt : sfmData.getLandmarks())
    {
        for(const auto& obsIt : landIt.second.observations)
        {
            IndexT viewId = obsIt.first;
            auto& landmarksSet = observationsPerView[viewId];
            landmarksSet.first.push_back(obsIt.second);
            landmarksSet.second.push_back(&landIt.second);
        }
    }
    return observationsPerView;
}

bool filterLandmarks(SfMData& sfmData, const FilterParams::FilterLandmarksParams& params)
{
    std::vector<Landmark> landmarksData;
    // Step 1
    {
        const auto& initialNbLandmarks = sfmData.getLandmarks().size();
        landmarksData.resize(initialNbLandmarks);
        size_t i = 0;
        if(params.step1.enabled)
        {
            ALICEVISION_LOG_INFO("Removing landmarks having an insufficient number of observations: started.");
            for(auto& it : sfmData.getLandmarks())
            {
                if(it.second.observations.size() < params.step1.minNbObservationsPerLandmark)
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

    // Step 2
    if(params.step2.enabled)
    {
        ALICEVISION_LOG_INFO("Removing landmarks with dissimilar observations of 3D landmark neighbors: started.");

        ALICEVISION_LOG_INFO("Build nanoflann KdTree index for landmarks in 3D.");
        LandmarksAdaptator data(landmarksData);
        KdTree3D tree(3, data, nanoflann::KDTreeSingleIndexAdaptorParams(MAX_LEAF_ELEMENTS));
        tree.buildIndex();
        ALICEVISION_LOG_INFO("KdTree created for " << landmarksData.size() << " points.");

        ALICEVISION_LOG_INFO("Computing landmarks neighbors: started.");
        // note that the landmark is a neighbor to itself with zero distance, hence the +/- 1
        int nbNeighbors_ = std::min(params.step2.nbNeighbors3D, static_cast<int>(landmarksData.size() - 1)) + 1;
        // contains the observing view ids and neighbors for each landmark
        std::vector<std::pair<std::vector<IndexT>, std::vector<size_t>>> viewData(landmarksData.size());
#pragma omp parallel for
        for(auto i = 0; i < landmarksData.size(); i++)
        {
            const sfmData::Landmark& landmark = landmarksData[i];
            const auto& nbObservations = landmark.observations.size();
            auto& [viewIds, neighbors] = viewData[i];
            viewIds.reserve(nbObservations);
            for(const auto& observationPair : landmark.observations)
            {
                const IndexT viewId = observationPair.first;
                viewIds.push_back(viewId);
            }
            // sort by ascending view id order for consequent faster access
            std::stable_sort(viewIds.begin(), viewIds.end());

            neighbors.resize(nbNeighbors_);
            std::vector<double> weights_(nbNeighbors_);
            tree.knnSearch(landmark.X.data(), nbNeighbors_, &neighbors[0], &weights_[0]);
            // a landmark is a neighbor to itself with zero distance, remove it
            neighbors.erase(neighbors.begin());
        }
        ALICEVISION_LOG_INFO("Computing landmarks neighbors: done.");

        ALICEVISION_LOG_INFO("Identifying landmarks to remove: started.");
        std::vector<bool> toRemove(landmarksData.size(), false);
        size_t nbToRemove = 0;
        const auto& initialNbLandmarks = landmarksData.size();
#pragma omp parallel for reduction(+ : nbToRemove)
        for(auto i = 0; i < landmarksData.size(); i++)
        {
            int score = 0;
            auto& [viewIds, neighbors] = viewData[i];
            for(auto j = 0; j < neighbors.size(); j++)
            {
                int scoreNeighbor = 0;
                const auto& neighborId = neighbors[j];
                const auto& viewIds_neighbor = viewData[neighborId].first;
                // loop over common views
                auto viewIds_it = viewIds.begin();
                auto viewIds_neighbor_it = viewIds_neighbor.begin();
                while(viewIds_it != viewIds.end() && viewIds_neighbor_it != viewIds_neighbor.end())
                {
                    if(*viewIds_it < *viewIds_neighbor_it)
                    {
                        ++viewIds_it;
                    }
                    else
                    {
                        // if same view
                        if(!(*viewIds_neighbor_it < *viewIds_it))
                        {
                            scoreNeighbor++;
                            if(scoreNeighbor == params.step2.maxNbObservationsPerLandmark)
                            {
                                score++;
                                break;
                            }
                            ++viewIds_it;
                        }
                        ++viewIds_neighbor_it;
                    }
                }
            }
            if(score < params.step2.nbNeighbors3D / 2)
            {
                toRemove[i] = true;
                nbToRemove++;
            }
        }
        ALICEVISION_LOG_INFO("Identified " << nbToRemove << " landmarks to remove out of " << initialNbLandmarks
                                           << ", i.e. " << (nbToRemove * 100.f / initialNbLandmarks) << " % ");
        ALICEVISION_LOG_INFO("Identifying landmarks to remove: done.");

        ALICEVISION_LOG_INFO("Removing identified landmarks: started.");
        std::vector<Landmark> landmarksData_filtered(landmarksData.size() - nbToRemove);
        IndexT newIdx = 0;
        for(size_t i = 0; i < landmarksData.size(); i++)
        {
            if(!toRemove[i])
            {
                landmarksData_filtered[newIdx++] = landmarksData[i];
            }
        }
        landmarksData = std::move(landmarksData_filtered);
        ALICEVISION_LOG_INFO("Removing identified landmarks: done.");

        ALICEVISION_LOG_INFO("Removing landmarks with dissimilar observations of 3D landmark neighbors: done.");
    }

    // Step 3
    if(params.step3.enabled)
    {
        ALICEVISION_LOG_INFO("Removing landmarks with worse resolution than neighbors: started.");

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
                pixSize += mp.getCamPixelSize(Point3d(landmark.X.x(), landmark.X.y(), landmark.X.z()),
                                              mp.getIndexFromViewId(viewId),
                                              params.step3.useFeatureScale ? observationPair.second.scale : 1);
                n++;
            }
            pixSize /= n;
            landmarksPixSize[i] = pixSize;
        }
        ALICEVISION_LOG_INFO("Computing pixel size: done.");

        ALICEVISION_LOG_INFO("Identifying landmarks to remove based on pixel size: started.");

        ALICEVISION_LOG_INFO("Build nanoflann KdTree index for landmarks in 3D.");
        LandmarksAdaptator data(landmarksData);
        KdTree3D tree(3, data, nanoflann::KDTreeSingleIndexAdaptorParams(MAX_LEAF_ELEMENTS));
        tree.buildIndex();
        ALICEVISION_LOG_INFO("KdTree created for " << landmarksData.size() << " points.");

        const auto& initialNbLandmarks = landmarksData.size();
        std::vector<bool> toRemove(landmarksData.size(), false);
        size_t nbToRemove = 0;
#pragma omp parallel for reduction(+ : nbToRemove)
        for(auto i = 0; i < landmarksData.size(); i++)
        {
            PixSizeSearch search(landmarksPixSize[i] * params.step3.radiusScale, landmarksPixSize, i);
            bool found = tree.findNeighbors(search, landmarksData[i].X.data(), nanoflann::SearchParameters());
            if(found)
            {
                toRemove[i] = true;
                nbToRemove++;
            }
        }
        ALICEVISION_LOG_INFO("Identified " << nbToRemove << " landmarks to remove out of " << initialNbLandmarks
                                           << ", i.e. " << (nbToRemove * 100.f / initialNbLandmarks) << " % ");
        ALICEVISION_LOG_INFO("Identifying landmarks to remove: done.");

        ALICEVISION_LOG_INFO("Removing identified landmarks: started.");
        std::vector<Landmark> landmarksData_filtered(landmarksData.size() - nbToRemove);
        IndexT newIdx = 0;
        for(size_t i = 0; i < landmarksData.size(); i++)
        {
            if(!toRemove[i])
            {
                landmarksData_filtered[newIdx++] = landmarksData[i];
            }
        }
        landmarksData = std::move(landmarksData_filtered);
        ALICEVISION_LOG_INFO("Removing identified landmarks: done.");

        ALICEVISION_LOG_INFO("Removing landmarks with worse resolution than neighbors: done.");
    }

    // applying modifications to Sfm data
    std::vector<std::pair<IndexT, Landmark>> filteredLandmarks(landmarksData.size());
    for(IndexT newIdx = 0; newIdx < landmarksData.size(); newIdx++)
    {
        filteredLandmarks[newIdx] = std::make_pair(newIdx, landmarksData[newIdx]);
    }
    sfmData.getLandmarks() = std::move(Landmarks(filteredLandmarks.begin(), filteredLandmarks.end()));
    return true;
}

bool filterObservations3D(SfMData& sfmData, const FilterParams::FilterObservations3DParams& params)
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

    if(params.propagationEnabled)
    {
        ALICEVISION_LOG_INFO("Computing landmark neighbors and distance-based weights: started");
        ALICEVISION_LOG_INFO("Build nanoflann KdTree index for landmarks in 3D.");
        LandmarksAdaptator dataAdaptor(landmarksData);
        KdTree3D tree(3, dataAdaptor, nanoflann::KDTreeSingleIndexAdaptorParams(MAX_LEAF_ELEMENTS));
        tree.buildIndex();
        ALICEVISION_LOG_INFO("KdTree created for " << landmarksData.size() << " points.");
        // note that the landmark is a neighbor to itself with zero distance, hence the +/- 1
        int nbNeighbors_ = std::min(params.nbNeighbors3D, static_cast<int>(landmarksData.size() - 1)) + 1;
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
        for(auto i = 0; i < params.nbIterations; i++)
        {
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
                for(auto j = 0; j < params.nbNeighbors3D; j++)
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
                // if common views with neighbor landmarks
                if(viewScores_total != 0.)
                    for(auto j = 0; j < viewScores_acc.size(); j++)
                    {
                        // normalize score and apply influence factor
                        viewScores_acc[j] *= params.neighborsInfluence / viewScores_total;
                        // combine weighted neighbor scores and the landmark's own scores
                        viewScores_acc[j] += (1 - params.neighborsInfluence) * viewScores[j];
                    }

                // dampen scores of non-chosen observations
                if(params.dampingEnabled && viewScores_acc.size() <= params.maxNbObservationsPerLandmark)
                {
                    // sort by descending view score order
                    std::vector<size_t> idx(viewScores_acc.size());
                    std::iota(idx.begin(), idx.end(), 0);
                    std::stable_sort(idx.begin(), idx.end(),
                                     [&v = viewScores_acc](size_t i1, size_t i2) { return v[i1] > v[i2]; });
                    viewScores_total = 1.;
                    for(auto j = params.maxNbObservationsPerLandmark; j < viewScores_acc.size(); j++)
                    {
                        const double& v = params.dampingFactor * viewScores_acc[j];
                        viewScores_total += v - viewScores_acc[j];
                        viewScores_acc[j] = v;
                    }
                    // re-normalize
                    for(auto j = 0; j < viewScores_acc.size(); j++)
                        viewScores_acc[j] /= viewScores_total;
                }
            }
            // Mean Squared Error
            double error = 0.;
            // update scores at end of iteration
#pragma omp parallel for reduction(+ : error)
            for(auto id = 0; id < landmarksData.size(); id++)
            {
                // compute MSE
                {
                    double error_j = 0.;
                    for(auto j = 0; j < viewScoresData_t[id].size(); j++)
                    {
                        const auto& v = viewScoresData_t[id][j] - viewScoresData[id].second[j];
                        error_j += v * v;
                    }
                    error_j /= viewScoresData_t[id].size();
                    error += error_j;
                }
                // update scores
                viewScoresData[id].second = std::move(viewScoresData_t[id]);
            }
            error /= landmarksData.size();
            ALICEVISION_LOG_INFO("MSE at iteration " << i << ": " << error);
        }
        ALICEVISION_LOG_INFO("Propagating neighbors observation scores: done");
    }

    ALICEVISION_LOG_INFO("Selecting observations with best scores: started");
#pragma omp parallel for
    for(auto i = 0; i < landmarksData.size(); i++)
    {
        sfmData::Landmark& landmark = *landmarksData[i];
        const auto& nbObservations = landmark.observations.size();
        auto& [viewIds, viewScores] = viewScoresData[i];

        // check number of observations
        if(landmark.observations.size() <= params.maxNbObservationsPerLandmark)
            continue;

        // sort by descending view score order
        std::vector<size_t> idx(nbObservations);
        std::iota(idx.begin(), idx.end(), 0);
        std::stable_sort(idx.begin(), idx.end(), [&v = viewScores](size_t i1, size_t i2) { return v[i1] > v[i2]; });

        // keep only observations with best scores
        Observations filteredObservations;
        for(auto j = 0; j < params.maxNbObservationsPerLandmark; j++)
        {
            const auto& viewId = viewIds[idx[j]];
            filteredObservations[viewId] = landmark.observations[viewId];
        }
        landmark.observations = std::move(filteredObservations);
    }
    ALICEVISION_LOG_INFO("Selecting observations with best scores: done");

    return true;
}

bool filterObservations2D(SfMData& sfmData, const FilterParams::FilterObservations2DParams& params,
                          HashMap<IndexT, double>& estimatedRadii)
{
    std::set<IndexT> viewIds = sfmData.getValidViews();
    std::vector<double> estimatedRadii_(viewIds.size(), -1.);
    auto observationsPerView = getObservationsPerViews(sfmData);

#pragma omp parallel for
    for(int i = 0; i < viewIds.size(); ++i)
    {
        auto itView = viewIds.begin();
        std::advance(itView, i);
        const IndexT viewId = *itView;
        auto observationsIt = observationsPerView.find(viewId);
        if(observationsIt == observationsPerView.end())
            continue;
        auto& observations = observationsIt->second.first;
        auto& landmarks = observationsIt->second.second;

        // Build nanoflann KdTree index for projected landmarks in 2D
        ObservationsAdaptator data(observations);
        KdTree2D tree(2, data, nanoflann::KDTreeSingleIndexAdaptorParams(MAX_LEAF_ELEMENTS));
        tree.buildIndex();

        // note that the observation is a neighbor to itself with zero distance, hence the +/- 1
        size_t nbNeighbors_ = std::min(static_cast<size_t>(params.nbNeighbors2D), observations.size() - 1) + 1;
        // average neighbors distance for each observation
        std::vector<double> means(observations.size());
        const std::size_t cacheSize = 1000;
        // accumulator for quantile computation
        accumulator_set<double, stats<tag::tail_quantile<right>>> acc(tag::tail<right>::cache_size = cacheSize);
        for(auto j = 0; j < observations.size(); j++)
        {
            // Find neighbors and the corresponding distances
            const auto& obs = observations[j];
            std::vector<size_t> indices_(nbNeighbors_);
            std::vector<double> distances_(nbNeighbors_);
            tree.knnSearch(obs.x.data(), nbNeighbors_, &indices_[0], &distances_[0]);
            // returned distances are L2 -> apply squared root
            std::transform(distances_.begin(), distances_.end(), distances_.begin(),
                           static_cast<double (*)(double)>(std::sqrt));
            // average distance
            const auto& mean = std::accumulate(distances_.begin(), distances_.end(), 0.0) / (nbNeighbors_ - 1);
            means[j] = mean;
            // update accumulator
            acc(mean);
        }
        double mean_max = std::numeric_limits<double>::max();
        // check to avoid exception
        if(params.percentile != 1.f)
            mean_max = quantile(acc, quantile_probability = params.percentile);
        // estimated mask radius is the average of distance means
        // quantile is used to avoid outlier bias
        double radius = quantile(acc, quantile_probability = (1.f - params.percentile) * 0.5f);
        // check if estimated radius is too large
        {
            const View& view = *(sfmData.getViews().at(viewId));
            double radiusMax = params.maskRadiusThreshold * 0.5 * (view.getImage().getWidth() + view.getImage().getHeight());
            if(radius > radiusMax)
                radius = radiusMax;
        }
        estimatedRadii_[i] = radius;

        // filter outlier observations
        std::vector<Observation> filteredObservations;
        std::vector<Landmark*> filteredLandmarks;
        filteredObservations.reserve(observations.size());
        filteredLandmarks.reserve(landmarks.size());
        for(auto j = 0; j < observations.size(); j++)
            if(means[j] < mean_max)
            {
                filteredObservations.push_back(observations[j]);
                filteredLandmarks.push_back(landmarks[j]);
            }
        filteredObservations.shrink_to_fit();
        filteredLandmarks.shrink_to_fit();
        observations = std::move(filteredObservations);
        landmarks = std::move(filteredLandmarks);
    }

    // clear and update landmark observations
    for(auto& landmark : sfmData.getLandmarks())
    {
        landmark.second.observations.clear();
    }
    for(int i = 0; i < viewIds.size(); ++i)
    {
        auto itView = viewIds.begin();
        std::advance(itView, i);
        const IndexT viewId = *itView;

        if(estimatedRadii_[i] != -1.)
            estimatedRadii[viewId] = estimatedRadii_[i];

        const auto& observationsIt = observationsPerView.find(viewId);
        if(observationsIt != observationsPerView.end())
        {
            auto& observations = observationsIt->second.first;
            auto& landmarks = observationsIt->second.second;
            for(int j = 0; j < observations.size(); j++)
            {
                landmarks[j]->observations[viewId] = observations[j];
            }
        }
    }
    // remove landmarks with no remaining observations
    for(auto it = sfmData.getLandmarks().begin(); it != sfmData.getLandmarks().end();)
    {
        if(it->second.observations.size() == 0)
            it = sfmData.getLandmarks().erase(it);
        else
            ++it;
    }

    return true;
}

int aliceVision_main(int argc, char *argv[])
{
    // command-line parameters

    std::string inputSfmFilename;
    std::string outputSfmFilename;

    // user optional parameters
    FilterParams params;
    std::string outputRadiiFilename;
    // required for 2D visualization in meshroom
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
        ("filterParams", po::value<FilterParams>(&params)->default_value(params),
            "Filter parameters devided into 3 successive stages.\n"
            "\n"
            "Filter landmarks parameters:\n"
            " * Enabled: Filter Landmarks over multiple steps.\n"
            " * Step 1 parameters:\n"
            "   * Enabled: Remove landmarks with insufficient observations.\n"
            "   * Min Nb of Observations: Minimum number of observations required to keep a landmark.\n"
            " * Step 2 parameters:\n"
            "   * Enabled: Remove landmarks with dissimilar observations of 3D landmark neighbors.\n"
            "   * Max Nb of Observations: Maximum number of allowed observations per landmark.\n"
            "   * Nb of Neighbors in 3D: Number of neighbor landmarks used in making the decision for best observations.\n"
            " * Step 3 parameters:\n"
            "   * Enabled: Remove landmarks with worse resolution than neighbors.\n"
            "   * Radius Scale: Scale factor applied to pixel size based radius filter applied to landmarks.\n"
            "   * Use Feature Scale: If true, use feature scale for computing pixel size. Otherwise, use a scale of 1 pixel.\n"
            "\n"
            "Filter Observations 3D parameters:\n"
            " * Enabled: Select best observations for observation consistency between 3D neighboring landmarks.\n"
            " * Max Nb of Observations: Maximum number of allowed observations per landmark.\n"
            " * Enable Neighbors Influence: nable propagating neighbors' scores iteratively.\n"
            "   * Nb of Neighbors in 3D: Number of neighbor landmarks used in making the decision for best observations.\n"
            "   * Neighbors Influence: Specifies how much influential the neighbors are in selecting the best observations.\n"
            "     Between 0. and 1., the closer to 1., the more influencial the neighborhood is.\n"
            "   * Nb of Iterations: Number of iterations to propagate neighbors information.\n"
            "   * Enable Damping: Enable additional damping of observations to reject after each iterations.\n"
            "     * Damping Factor: Multiplicative damping factor.\n"
            "\n"
            "Filter Observations 2D parameters:\n"
            " * Enabled: Select best observations for observation consistency between 2D projected neighboring\n"
            "   landmarks per view. Eventually remove landmarks with no remaining observations.\n"
            "   Also estimate depth map mask radius per view based on landmarks.\n"
            " * Nb of Neighbors in 2D: Number of neighbor observations to be considered for the landmarks-based masking.\n"
            " * Percentile: Used as a quantile probability for filtering relatively outlier observations.\n"
            " * Mask Radius Threshold: Percentage of image size to be used as an upper limit for estimated mask radius.")
        ("outputRadiiFile", po::value<std::string>(&outputRadiiFilename)->default_value(outputRadiiFilename),
         "Output Radii file containing the estimated projection radius of observations per view.")
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

    if(params.filterLandmarks.enabled && (params.filterLandmarks.step1.enabled ||
                                          params.filterLandmarks.step2.enabled || params.filterLandmarks.step3.enabled))
    {
        ALICEVISION_LOG_INFO("Filtering landmarks: started.");
        filterLandmarks(sfmData, params.filterLandmarks);
        ALICEVISION_LOG_INFO("Filtering landmarks: done.");
    }
    
    if(params.filterObservations3D.enabled)
    {
        ALICEVISION_LOG_INFO("Filtering observations in 3D: started.");
        filterObservations3D(sfmData, params.filterObservations3D);
        ALICEVISION_LOG_INFO("Filtering observations in 3D: done.");
    }

    if(params.filterObservations2D.enabled)
    {
        HashMap<IndexT, double> estimatedRadii;
        ALICEVISION_LOG_INFO("Filtering observations in 2D: started.");
        filterObservations2D(sfmData, params.filterObservations2D, estimatedRadii);
        ALICEVISION_LOG_INFO("Filtering observations in 2D: done.");

        if(outputRadiiFilename.empty())
            outputRadiiFilename = (fs::path(outputSfmFilename).parent_path() / "radii.txt").string();
        std::ofstream fs(outputRadiiFilename, std::ios::out);
        if(!fs.is_open())
            ALICEVISION_LOG_WARNING("Unable to create the radii file " << outputRadiiFilename);
        else
        {
            for(const auto& radius : estimatedRadii)
                fs << radius.first << "\t" << radius.second << std::endl;
            fs.close();
        }
    }

    sfmDataIO::Save(sfmData, outputSfmFilename, sfmDataIO::ESfMData::ALL);
    return EXIT_SUCCESS;

}
