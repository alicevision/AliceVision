// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/pipeline/expanding/ConnexityGraph.hpp>
#include <aliceVision/sfm/pipeline/expanding/LbaPolicyConnexity.hpp>
#include <aliceVision/camera/Pinhole.hpp>

#include <limits>

#define BOOST_TEST_MODULE lbaPolicyConnexity
#include <boost/test/unit_test.hpp>

using namespace aliceVision;
using namespace aliceVision::sfmData;

namespace {

const int unreachable = std::numeric_limits<int>::max();

// Views 0..count-1, each with its own pose (pose id = view id) and a shared constant intrinsic,
// so that the intrinsic does not link the views in the connexity graph
SfMData makeScene(IndexT count)
{
    SfMData scene;

    auto intrinsic = std::make_shared<camera::Pinhole>(100, 100, 100.0, 100.0, 0.0, 0.0);
    intrinsic->setState(EEstimatorParameterState::CONSTANT);
    scene.getIntrinsics().emplace(0, intrinsic);

    for (IndexT viewId = 0; viewId < count; viewId++)
    {
        scene.getViews().emplace(viewId, std::make_shared<View>("", viewId, 0, viewId, 100, 100));
        scene.getPoses().assign(viewId, CameraPose(geometry::Pose3()));
    }

    return scene;
}

void setLandmark(SfMData& scene, IndexT landmarkId, const std::vector<IndexT>& viewIds)
{
    Landmark landmark(Vec3(0.0, 0.0, 1.0));
    for (const IndexT viewId : viewIds)
    {
        landmark.getObservations().emplace(viewId, Observation(Vec2::Zero(), landmarkId, 1.0));
    }

    scene.getLandmarks()[landmarkId] = landmark;
}

std::vector<int> getDistances(const sfm::ConnexityGraph& graph, IndexT count)
{
    std::vector<int> distances;
    for (IndexT poseId = 0; poseId < count; poseId++)
    {
        distances.push_back(graph.getDistance(poseId));
    }

    return distances;
}

}  // namespace

// The graph is reused between builds, as in the expansion: the pairs counts must follow
// the added, modified and removed landmarks, so that each build gives the same graph as a new one
BOOST_AUTO_TEST_CASE(ConnexityGraph_incrementalUpdate)
{
    const IndexT count = 4;
    SfMData scene = makeScene(count);
    sfm::ConnexityGraph graph;

    auto checkDistances = [&](const std::vector<int>& expected) {
        BOOST_REQUIRE(graph.build(scene, {0}));
        const std::vector<int> distances = getDistances(graph, count);
        BOOST_CHECK_EQUAL_COLLECTIONS(distances.begin(), distances.end(), expected.begin(), expected.end());

        sfm::ConnexityGraph newGraph;
        BOOST_REQUIRE(newGraph.build(scene, {0}));
        const std::vector<int> newDistances = getDistances(newGraph, count);
        BOOST_CHECK_EQUAL_COLLECTIONS(distances.begin(), distances.end(), newDistances.begin(), newDistances.end());
    };

    // Chain 0-1-2-3, the pair (0,1) is seen by two landmarks
    setLandmark(scene, 10, {0, 1});
    setLandmark(scene, 11, {0, 1});
    setLandmark(scene, 12, {1, 2});
    setLandmark(scene, 13, {2, 3});
    checkDistances({0, 1, 2, 3});

    // Removed landmark: (0,1) is still seen by landmark 11
    // Removed landmark: (2,3) is not seen anymore
    // Modified landmark: (1,2) becomes (1,3)
    scene.getLandmarks().erase(10);
    scene.getLandmarks().erase(13);
    setLandmark(scene, 12, {1, 3});
    checkDistances({0, 1, unreachable, 2});

    // New landmark (0,2), and a landmark id seen before comes back
    setLandmark(scene, 14, {0, 2});
    setLandmark(scene, 13, {2, 3});
    checkDistances({0, 1, 1, 2});

    // Nothing changed
    checkDistances({0, 1, 1, 2});
}

// A landmark parameterized in a reference view frame needs the reference pose in the bundle,
// even when the reference view does not observe it
BOOST_AUTO_TEST_CASE(LbaPolicyConnexity_ignoredReferencePose)
{
    // Chain 0-1-2-3 seen from view 0, with a distance limit of 1:
    // poses 0 and 1 are refined, pose 2 is constant, pose 3 is ignored
    SfMData scene = makeScene(4);
    setLandmark(scene, 10, {0, 1});
    setLandmark(scene, 11, {1, 2});
    setLandmark(scene, 12, {2, 3});

    // Same observations, only the reference view differs
    setLandmark(scene, 20, {0, 1});
    scene.getLandmarks().at(20).setReferenceView(scene.getViews().at(0));
    setLandmark(scene, 21, {0, 1});
    scene.getLandmarks().at(21).setReferenceView(scene.getViews().at(3));

    sfm::LbaPolicyConnexity policy;
    policy.setDistanceLimit(1);
    track::TracksHandler tracksHandler;
    BOOST_REQUIRE(policy.build(scene, tracksHandler, {0}));

    BOOST_CHECK(scene.getAbsolutePose(3).getState() == EEstimatorParameterState::IGNORED);
    BOOST_CHECK(scene.getLandmarks().at(20).getState() == EEstimatorParameterState::REFINED);
    BOOST_CHECK(scene.getLandmarks().at(21).getState() == EEstimatorParameterState::IGNORED);
}
