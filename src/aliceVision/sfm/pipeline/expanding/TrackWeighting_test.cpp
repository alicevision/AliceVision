// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE TrackWeighting
#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/sfmDataIO/sceneSample.hpp>
#include <aliceVision/sfmData/ImageSequence.hpp>
#include <aliceVision/sfmData/ImageSet.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfm/pipeline/expanding/TrackWeighting.hpp>


BOOST_AUTO_TEST_CASE(TrackWeighting_alongTrackAndTrackLength)
{
    // Check that weightObservationsAlongTrack stores expected weights in different configurations

    using namespace aliceVision;

    makeRandomOperationsReproducible();
    const double tolerance = 1e-6;

    sfmData::SfMData sfmData;
    sfmDataIO::generateSphereScene(sfmData, 100, 240);

    // Generate new ImageSet and add observations
    size_t imageGroupID2 = 3454613548;
    for (IndexT idPV = 300; idPV <410; idPV++)
    {
        sfmData.getPoses().assign(idPV, sfmData::CameraPose());
        sfmData.getViews().emplace(idPV, std::make_shared<sfmData::View>("", idPV, 0, idPV, 100, 100));
        sfmData::View& view = sfmData.getView(idPV);
        view.setFrameId(idPV);
        view.setImageGroupId(imageGroupID2);
        for (auto& [landmarkId, landmark]: sfmData.getLandmarks())
        {
            landmark.getObservations().emplace(idPV, sfmData::Observation(Eigen::Vector2d(0,0), idPV, 1.));
        }
    }
    auto imageGroupPtr2 = std::make_shared<sfmData::ImageSet>();
    sfmData.getImageGroups().emplace(imageGroupID2, imageGroupPtr2);

    // Generate new ImageSequence and add observations
    size_t imageGroupID3 = 8745461354;
    for (IndexT idPV = 500; idPV <640; idPV++)
    {
        sfmData.getPoses().assign(idPV, sfmData::CameraPose());
        sfmData.getViews().emplace(idPV, std::make_shared<sfmData::View>("", idPV, 0, idPV, 100, 100));
        sfmData.getView(idPV).setFrameId(idPV);
        sfmData.getView(idPV).setImageGroupId(imageGroupID3);
        for (auto& [landmarkId, landmark]: sfmData.getLandmarks())
        {
            landmark.getObservations().emplace(idPV, sfmData::Observation(Eigen::Vector2d(0,0), idPV, 1.));
        }
    }
    auto imageGroupPtr3 = std::make_shared<sfmData::ImageSequence>();
    sfmData.getImageGroups().emplace(imageGroupID3, imageGroupPtr3);

    // landmark with observations on the whole imageSequences and on the whole imageSet
    {
        auto& landmark_0 = sfmData.getLandmarks().at(0);
        BOOST_CHECK_EQUAL(landmark_0.getObservations().size(), 490);
    }
    // landmark with observations on the whole 1st imageSequence except on the first view, no observation on the imageSet,
    // and observations on the whole 2nd imageSequence
    {
        auto& landmark_1 = sfmData.getLandmarks().at(1);
        auto& ldmk_1_observations = landmark_1.getObservations();
        ldmk_1_observations.erase(0);
        for (IndexT idPV = 300; idPV <410; idPV++)
        {
            ldmk_1_observations.erase(idPV);
        }
        BOOST_CHECK_EQUAL(ldmk_1_observations.size(), 379);
    }
    // landmark with observations on the whole 1st imageSequence except on the last view, and no observation on the imageSet
    // and on the 2nd imageSequence
    {
        auto& landmark_2 = sfmData.getLandmarks().at(2);
        auto& ldmk_2_observations = landmark_2.getObservations();
        ldmk_2_observations.erase(239);
        for (IndexT idPV = 300; idPV <410; idPV++)
        {
            ldmk_2_observations.erase(idPV);
        }
        for (IndexT idPV = 500; idPV <640; idPV++)
        {
            ldmk_2_observations.erase(idPV);
        }
        BOOST_CHECK_EQUAL(ldmk_2_observations.size(), 239);
    }
    // landmark with observations on the whole 1st imageSequence except on the first and the last views,
    // observations on the whole imageSet, and one observation on the 2nd imageSequence
    {
        auto& landmark_3 = sfmData.getLandmarks().at(3);
        auto& ldmk_3_observations = landmark_3.getObservations();
        ldmk_3_observations.erase(0);
        ldmk_3_observations.erase(239);
        for (IndexT idPV = 500; idPV <640; idPV = idPV + ((idPV == 560) ?  2 : 1) )
        {
            ldmk_3_observations.erase(idPV);
        }
        BOOST_CHECK_EQUAL(ldmk_3_observations.size(), 349);
    }
    // landmark with observations on the whole 1st imageSequence except on the 2nd view,
    // observations on the whole imageSet, and two observations on the 2nd imageSequence
    {
        auto& landmark_4 = sfmData.getLandmarks().at(4);
        auto& ldmk_4_observations = landmark_4.getObservations();
        ldmk_4_observations.erase(1);
        for (IndexT idPV = 500; idPV <640; idPV = idPV + ((idPV == 560) ?  3 : 1) )
        {
            ldmk_4_observations.erase(idPV);
        }
        BOOST_CHECK_EQUAL(ldmk_4_observations.size(), 351);
    }
    // landmark with observations on the whole 1st imageSequence except on the 3rd view
    // observations on the whole imageSet, and three observations on the 2nd imageSequence
    {
        auto& landmark_5 = sfmData.getLandmarks().at(5);
        auto& ldmk_5_observations = landmark_5.getObservations();
        ldmk_5_observations.erase(2);
        for (IndexT idPV = 500; idPV <640; idPV = idPV + ((idPV == 560) ?  4 : 1) )
        {
            ldmk_5_observations.erase(idPV);
        }
        BOOST_CHECK_EQUAL(ldmk_5_observations.size(), 352);
    }
    // landmark with observations on the whole 1st imageSequence except on the 4th and the last views
    // observations on the whole imageSet, and observations on the whole 2nd imageSequence except on one view
    {
        auto& landmark_6 = sfmData.getLandmarks().at(6);
        auto& ldmk_6_observations = landmark_6.getObservations();
        ldmk_6_observations.erase(3);
        ldmk_6_observations.erase(239);
        ldmk_6_observations.erase(560);
        BOOST_CHECK_EQUAL(ldmk_6_observations.size(), 487);
    }
    // landmark with observations on the whole 1st imageSequence except on the 5th and the 239th views
    // observations on the whole imageSet, and observations on the whole 2nd imageSequence except on two views
    {
        auto& landmark_7 = sfmData.getLandmarks().at(7);
        auto& ldmk_7_observations = landmark_7.getObservations();
        ldmk_7_observations.erase(4);
        ldmk_7_observations.erase(238);
        ldmk_7_observations.erase(560);
        ldmk_7_observations.erase(561);
        BOOST_CHECK_EQUAL(ldmk_7_observations.size(), 486);
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th and the 238th views
    // observations on the whole imageSet, and observations on the whole 2nd imageSequence except on three views
    {
        auto& landmark_8 = sfmData.getLandmarks().at(8);
        auto& ldmk_8_observations = landmark_8.getObservations();
        ldmk_8_observations.erase(5);
        ldmk_8_observations.erase(237);
        ldmk_8_observations.erase(560);
        ldmk_8_observations.erase(561);
        ldmk_8_observations.erase(563);
        BOOST_CHECK_EQUAL(ldmk_8_observations.size(), 485);
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 7th and the 237th views
    // observations on the whole imageSet, and observations on the first and the last views of the 2nd imageSequence
    {
        auto& landmark_9 = sfmData.getLandmarks().at(9);
        auto& ldmk_9_observations = landmark_9.getObservations();
        ldmk_9_observations.erase(5);
        ldmk_9_observations.erase(6);
        ldmk_9_observations.erase(236);
        for (IndexT idPV = 501; idPV < 639; idPV++)
        {
            ldmk_9_observations.erase(idPV);
        }
        BOOST_CHECK_EQUAL(ldmk_9_observations.size(), 349);
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 7th, the 8th and the 236th views
    // observations on the whole imageSet, and observations on the two first and two last views of the 2nd imageSequence
    {
        auto& landmark_10 = sfmData.getLandmarks().at(10);
        auto& ldmk_10_observations = landmark_10.getObservations();
        ldmk_10_observations.erase(5);
        ldmk_10_observations.erase(6);
        ldmk_10_observations.erase(7);
        ldmk_10_observations.erase(235);
        for (IndexT idPV = 502; idPV < 638; idPV++)
        {
            ldmk_10_observations.erase(idPV);
        }
        BOOST_CHECK_EQUAL(ldmk_10_observations.size(), 350);
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 7th, the 120th and the 235th views
    // observations on the whole imageSet, and observations on the three first and three last views of the 2nd imageSequence
    {
        auto& landmark_11 = sfmData.getLandmarks().at(11);
        auto& ldmk_11_observations = landmark_11.getObservations();
        ldmk_11_observations.erase(5);
        ldmk_11_observations.erase(6);
        ldmk_11_observations.erase(119);
        ldmk_11_observations.erase(234);
        for (IndexT idPV = 503; idPV < 637; idPV++)
        {
            ldmk_11_observations.erase(idPV);
        }
        BOOST_CHECK_EQUAL(ldmk_11_observations.size(), 352);
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 60th, the 120th and the 234th views
    // observations on the whole imageSet, and observations on nine views of the 2nd imageSequence
    {
        auto& landmark_12 = sfmData.getLandmarks().at(12);
        auto& ldmk_12_observations = landmark_12.getObservations();
        ldmk_12_observations.erase(5);
        ldmk_12_observations.erase(59);
        ldmk_12_observations.erase(119);
        ldmk_12_observations.erase(233);
        for (IndexT idPV = 500; idPV <640; idPV = idPV + ((idPV == 560) ?  10 : 1) )
        {
            ldmk_12_observations.erase(idPV);
        }
        BOOST_CHECK_EQUAL(ldmk_12_observations.size(), 355);
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 60th, the 61th and the 234th views
    // observations on the whole imageSet, and observations on ten views of the 2nd imageSequence
    {
        auto& landmark_13 = sfmData.getLandmarks().at(13);
        auto& ldmk_13_observations = landmark_13.getObservations();
        ldmk_13_observations.erase(5);
        ldmk_13_observations.erase(59);
        ldmk_13_observations.erase(60);
        ldmk_13_observations.erase(233);
        for (IndexT idPV = 500; idPV <640; idPV = idPV + ((idPV == 560) ?  11 : 1) )
        {
            ldmk_13_observations.erase(idPV);
        }
        BOOST_CHECK_EQUAL(ldmk_13_observations.size(), 356);
    }

    for (auto& [_, landmark] : sfmData.getLandmarks())
    {
        for (auto& [_, obs] : landmark.getObservations())
        {
            BOOST_CHECK_EQUAL(obs.getWeight(), 1.);
        }
    }

    int fadingSize = 0;
    sfm::weightObservationsAlongTrack(sfmData, fadingSize);

    double trackLengthMaxWeight = 0.;
    int trackLengthLowThreshold = 350;
    int trackLengthHighThreshold = 400;
    sfm::weightTracks(sfmData, trackLengthMaxWeight, trackLengthLowThreshold, trackLengthHighThreshold);

    for (auto& [_, landmark] : sfmData.getLandmarks())
    {
        for (auto& [_, obs] : landmark.getObservations())
        {
            BOOST_CHECK_EQUAL(obs.getWeight(), 1.);
        }
    }

    trackLengthMaxWeight = 5.;
    sfm::weightTracks(sfmData, trackLengthMaxWeight, trackLengthLowThreshold, trackLengthHighThreshold);

    for (auto& [_, ldmk_0_obs]: sfmData.getLandmarks().at(0).getObservations())
    {
        BOOST_CHECK_CLOSE(ldmk_0_obs.getWeight(), 1., tolerance);
    }
    for (auto& [_, ldmk_1_obs]: sfmData.getLandmarks().at(1).getObservations())
    {
        BOOST_CHECK_CLOSE(ldmk_1_obs.getWeight(), .664, tolerance);
    }
    for (auto& [_, ldmk_2_obs]: sfmData.getLandmarks().at(2).getObservations())
    {
        BOOST_CHECK_CLOSE(ldmk_2_obs.getWeight(), .200, tolerance);
    }
    for (auto& [_, ldmk_3_obs]: sfmData.getLandmarks().at(3).getObservations())
    {
        BOOST_CHECK_CLOSE(ldmk_3_obs.getWeight(), .200, tolerance);
    }
    for (auto& [_, ldmk_4_obs]: sfmData.getLandmarks().at(4).getObservations())
    {
        BOOST_CHECK_CLOSE(ldmk_4_obs.getWeight(), .216, tolerance);
    }
    for (auto& [_, ldmk_5_obs]: sfmData.getLandmarks().at(5).getObservations())
    {
        BOOST_CHECK_CLOSE(ldmk_5_obs.getWeight(), .232, tolerance);
    }
    for (auto& [_, ldmk_6_obs]: sfmData.getLandmarks().at(6).getObservations())
    {
        BOOST_CHECK_CLOSE(ldmk_6_obs.getWeight(), 1., tolerance);
    }
    for (auto& [_, ldmk_7_obs]: sfmData.getLandmarks().at(7).getObservations())
    {
        BOOST_CHECK_CLOSE(ldmk_7_obs.getWeight(), 1., tolerance);
    }
    for (auto& [_, ldmk_8_obs]: sfmData.getLandmarks().at(8).getObservations())
    {
        BOOST_CHECK_CLOSE(ldmk_8_obs.getWeight(), 1., tolerance);
    }
    for (auto& [_, ldmk_9_obs]: sfmData.getLandmarks().at(9).getObservations())
    {
        BOOST_CHECK_CLOSE(ldmk_9_obs.getWeight(), .200, tolerance);
    }
    for (auto& [_, ldmk_10_obs]: sfmData.getLandmarks().at(10).getObservations())
    {
        BOOST_CHECK_CLOSE(ldmk_10_obs.getWeight(), .200, tolerance);
    }
    for (auto& [_, ldmk_11_obs]: sfmData.getLandmarks().at(11).getObservations())
    {
        BOOST_CHECK_CLOSE(ldmk_11_obs.getWeight(), .232, tolerance);
    }
    for (auto& [_, ldmk_12_obs]: sfmData.getLandmarks().at(12).getObservations())
    {
        BOOST_CHECK_CLOSE(ldmk_12_obs.getWeight(), .280, tolerance);
    }
    for (auto& [_, ldmk_13_obs]: sfmData.getLandmarks().at(13).getObservations())
    {
        BOOST_CHECK_CLOSE(ldmk_13_obs.getWeight(), .296, tolerance);
    }

    sfm::resetObservationWeights(sfmData);

    for (auto& [_, landmark] : sfmData.getLandmarks())
    {
        for (auto& [_, obs] : landmark.getObservations())
        {
            BOOST_CHECK_EQUAL(obs.getWeight(), 1.);
        }
    }

    fadingSize = 5;
    sfm::weightObservationsAlongTrack(sfmData, fadingSize);

    // landmark with observations on the whole imageSequences and on the whole imageSet
    {
        auto& landmark_0 = sfmData.getLandmarks().at(0);

        for (auto& [_, ldmk_0_obs] : landmark_0.getObservations())
        {
            BOOST_CHECK_EQUAL(ldmk_0_obs.getWeight(), 1.);
        }
        // const double landmarkWeight = 1.;
    }
    // landmark with observations on the whole 1st imageSequence except on the first view, no observation on the imageSet,
    // and observations on the whole 2nd imageSequence
    {
        auto& landmark_1 = sfmData.getLandmarks().at(1);
        auto& ldmk_1_observations = landmark_1.getObservations();

        BOOST_CHECK_CLOSE(ldmk_1_observations.at(1).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_1_observations.at(2).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_1_observations.at(3).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_1_observations.at(4).getWeight(), .8, tolerance);
        for (auto& [ldmk_1_id, ldmk_1_obs] : ldmk_1_observations)
        {
            if (ldmk_1_id > 4)
            {
                BOOST_CHECK_EQUAL(ldmk_1_obs.getWeight(), 1.);
            }
        }
        // const double landmarkWeight = .664;
    }
    // landmark with observations on the whole 1st imageSequence except on the last view, and no observation on the imageSet
    // and on the 2nd imageSequence
    {
        auto& landmark_2 = sfmData.getLandmarks().at(2);
        auto& ldmk_2_observations = landmark_2.getObservations();

        for (auto& [ldmk_2_id, ldmk_2_obs] : ldmk_2_observations)
        {
            if (ldmk_2_id < 235)
            {
                BOOST_CHECK_EQUAL(ldmk_2_obs.getWeight(), 1.);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_2_observations.at(235).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_2_observations.at(236).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_2_observations.at(237).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_2_observations.at(238).getWeight(), .2, tolerance);
        // const double landmarkWeight = .2;
    }
    // landmark with observations on the whole 1st imageSequence except on the first and the last views,
    // observations on the whole imageSet, and one observation on the 2nd imageSequence
    {
        auto& landmark_3 = sfmData.getLandmarks().at(3);
        auto& ldmk_3_observations = landmark_3.getObservations();

        BOOST_CHECK_CLOSE(ldmk_3_observations.at(1).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(2).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(3).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(4).getWeight(), .8, tolerance);
        for (auto& [ldmk_3_id, ldmk_3_obs] : ldmk_3_observations)
        {
            if (ldmk_3_id > 4 && ldmk_3_id < 235)
            {
                BOOST_CHECK_EQUAL(ldmk_3_obs.getWeight(), 1.);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(235).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(236).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(237).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(238).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_3_observations.at(561).getWeight(), .2, tolerance);
        // const double landmarkWeight = .2;
    }
    // landmark with observations on the whole 1st imageSequence except on the 2nd view,
    // observations on the whole imageSet, and two observations on the 2nd imageSequence
    {
        auto& landmark_4 = sfmData.getLandmarks().at(4);
        auto& ldmk_4_observations = landmark_4.getObservations();

        BOOST_CHECK_CLOSE(ldmk_4_observations.at(0).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_4_observations.at(2).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_4_observations.at(3).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_4_observations.at(4).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_4_observations.at(5).getWeight(), .8, tolerance);
        for (auto& [ldmk_4_id, ldmk_4_obs] : ldmk_4_observations)
        {
            if (ldmk_4_id > 5 && ldmk_4_id < 240)
            {
                BOOST_CHECK_EQUAL(ldmk_4_obs.getWeight(), 1.);
            }
        }

        BOOST_CHECK_CLOSE(ldmk_4_observations.at(561).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_4_observations.at(562).getWeight(), .2, tolerance);
        // const double landmarkWeight = .216;
    }
    // landmark with observations on the whole 1st imageSequence except on the 3rd view,
    // observations on the whole imageSet, and three observations on the 2nd imageSequence
    {
        auto& landmark_5 = sfmData.getLandmarks().at(5);
        auto& ldmk_5_observations = landmark_5.getObservations();

        BOOST_CHECK_CLOSE(ldmk_5_observations.at(0).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(1).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_5_observations.at(3).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(4).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(5).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(6).getWeight(), .8, tolerance);
        for (auto& [ldmk_5_id, ldmk_5_obs] : ldmk_5_observations)
        {
            if (ldmk_5_id > 6 && ldmk_5_id < 240)
            {
                BOOST_CHECK_EQUAL(ldmk_5_obs.getWeight(), 1.);
            }
        }

        BOOST_CHECK_CLOSE(ldmk_5_observations.at(561).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(562).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(563).getWeight(), .2, tolerance);
        // const double landmarkWeight = .232;
    }
    // landmark with observations on the whole 1st imageSequence except on the 4th and the last views,
    // observations on the whole imageSet, and observations on the whole 2nd imageSequence except on one view
    {
        auto& landmark_6 = sfmData.getLandmarks().at(6);
        auto& ldmk_6_observations = landmark_6.getObservations();

        BOOST_CHECK_CLOSE(ldmk_6_observations.at(0).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(1).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(2).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_6_observations.at(4).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(5).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(6).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(7).getWeight(), .8, tolerance);
        for (auto& [ldmk_6_id, ldmk_6_obs] : ldmk_6_observations)
        {
            if (ldmk_6_id > 7 && ldmk_6_id < 235)
            {
                BOOST_CHECK_EQUAL(ldmk_6_obs.getWeight(), 1.);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(235).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(236).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(237).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(238).getWeight(), .2, tolerance);

        for (auto& [ldmk_6_id, ldmk_6_obs] : ldmk_6_observations)
        {
            if (ldmk_6_id > 240 && ldmk_6_id < 556)
            {
                BOOST_CHECK_EQUAL(ldmk_6_obs.getWeight(), 1.);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(556).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(557).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(558).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(559).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_6_observations.at(561).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(562).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(563).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(564).getWeight(), .8, tolerance);
        for (auto& [ldmk_6_id, ldmk_6_obs] : ldmk_6_observations)
        {
            if (ldmk_6_id > 564)
            {
                BOOST_CHECK_EQUAL(ldmk_6_obs.getWeight(), 1.);
            }
        }
        // const double landmarkWeight = 1.;
    }
    // landmark with observations on the whole 1st imageSequence except on the 5th and the 239th views
    // observations on the whole imageSet, and observations on the whole 2nd imageSequence except on two views
    {
        auto& landmark_7 = sfmData.getLandmarks().at(7);
        auto& ldmk_7_observations = landmark_7.getObservations();

        BOOST_CHECK_CLOSE(ldmk_7_observations.at(0).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(1).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(2).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(3).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_7_observations.at(5).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(6).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(7).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(8).getWeight(), .8, tolerance);
        for (auto& [ldmk_7_id, ldmk_7_obs] : ldmk_7_observations)
        {
            if (ldmk_7_id > 8 && ldmk_7_id < 234)
            {
                BOOST_CHECK_EQUAL(ldmk_7_obs.getWeight(), 1.);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(234).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(235).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(236).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(237).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_7_observations.at(239).getWeight(), .2, tolerance);

        for (auto& [ldmk_7_id, ldmk_7_obs] : ldmk_7_observations)
        {
            if (ldmk_7_id > 240 && ldmk_7_id < 556)
            {
                BOOST_CHECK_EQUAL(ldmk_7_obs.getWeight(), 1.);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(556).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(557).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(558).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(559).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_7_observations.at(562).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(563).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(564).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(565).getWeight(), .8, tolerance);
        for (auto& [ldmk_7_id, ldmk_7_obs] : ldmk_7_observations)
        {
            if (ldmk_7_id > 565)
            {
                BOOST_CHECK_EQUAL(ldmk_7_obs.getWeight(), 1.);
            }
        }
        // const double landmarkWeight = 1.;
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th and the 238th views
    // observations on the whole imageSet, and observations on the whole 2nd imageSequence except on three views
    {
        auto& landmark_8 = sfmData.getLandmarks().at(8);
        auto& ldmk_8_observations = landmark_8.getObservations();

        BOOST_CHECK_CLOSE(ldmk_8_observations.at(0).getWeight(), 1., tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(1).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(2).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(3).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(4).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_8_observations.at(6).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(7).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(8).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(9).getWeight(), .8, tolerance);
        for (auto& [ldmk_8_id, ldmk_8_obs] : ldmk_8_observations)
        {
            if (ldmk_8_id > 9 && ldmk_8_id < 233)
            {
                BOOST_CHECK_EQUAL(ldmk_8_obs.getWeight(), 1.);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(233).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(234).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(235).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(236).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_8_observations.at(238).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(239).getWeight(), .4, tolerance);

        for (auto& [ldmk_8_id, ldmk_8_obs] : ldmk_8_observations)
        {
            if (ldmk_8_id > 240 && ldmk_8_id < 556)
            {
                BOOST_CHECK_EQUAL(ldmk_8_obs.getWeight(), 1.);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(556).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(557).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(558).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(559).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_8_observations.at(562).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_8_observations.at(564).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(565).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(566).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(567).getWeight(), .8, tolerance);
        for (auto& [ldmk_8_id, ldmk_8_obs] : ldmk_8_observations)
        {
            if (ldmk_8_id > 567)
            {
                BOOST_CHECK_EQUAL(ldmk_8_obs.getWeight(), 1.);
            }
        }
        // const double landmarkWeight = 1.;
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 7th and the 237th views
    // observations on the whole imageSet, and observations on the first and the last views of the 2nd imageSequence
    {
        auto& landmark_9 = sfmData.getLandmarks().at(9);
        auto& ldmk_9_observations = landmark_9.getObservations();

        BOOST_CHECK_CLOSE(ldmk_9_observations.at(0).getWeight(), 1., tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(1).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(2).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(3).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(4).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_9_observations.at(7).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(8).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(9).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(10).getWeight(), .8, tolerance);
        for (auto& [ldmk_9_id, ldmk_9_obs] : ldmk_9_observations)
        {
            if (ldmk_9_id > 10 && ldmk_9_id < 232)
            {
                BOOST_CHECK_EQUAL(ldmk_9_obs.getWeight(), 1.);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(232).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(233).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(234).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(235).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_9_observations.at(237).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(238).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(239).getWeight(), .6, tolerance);

        BOOST_CHECK_CLOSE(ldmk_9_observations.at(500).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_9_observations.at(639).getWeight(), .2, tolerance);
        // const double landmarkWeight = .200;
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 7th, the 8th and the 236th views
    // observations on the whole imageSet, and observations on the two first and two last views of the 2nd imageSequence
    {
        auto& landmark_10 = sfmData.getLandmarks().at(10);
        auto& ldmk_10_observations = landmark_10.getObservations();

        BOOST_CHECK_CLOSE(ldmk_10_observations.at(0).getWeight(), 1., tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(1).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(2).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(3).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(4).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_10_observations.at(8).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(9).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(10).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(11).getWeight(), .8, tolerance);
        for (auto& [ldmk_10_id, ldmk_10_obs] : ldmk_10_observations)
        {
            if (ldmk_10_id > 11 && ldmk_10_id < 231)
            {
                BOOST_CHECK_EQUAL(ldmk_10_obs.getWeight(), 1.);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(231).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(232).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(233).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(234).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_10_observations.at(236).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(237).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(238).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(239).getWeight(), .8, tolerance);

        BOOST_CHECK_CLOSE(ldmk_10_observations.at(500).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(501).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_10_observations.at(638).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(639).getWeight(), .4, tolerance);
        // const double landmarkWeight = .200;
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 7th, the 120th and the 235th views
    // observations on the whole imageSet, and observations on the three first and three last views of the 2nd imageSequence
    {
        auto& landmark_11 = sfmData.getLandmarks().at(11);
        auto& ldmk_11_observations = landmark_11.getObservations();

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(0).getWeight(), 1., tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(1).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(2).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(3).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(4).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(7).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(8).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(9).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(10).getWeight(), .8, tolerance);
        for (auto& [ldmk_11_id, ldmk_11_obs] : ldmk_11_observations)
        {
            if (ldmk_11_id > 10 && ldmk_11_id < 115)
            {
                BOOST_CHECK_EQUAL(ldmk_11_obs.getWeight(), 1.);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(115).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(116).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(117).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(118).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(120).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(121).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(122).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(123).getWeight(), .8, tolerance);
        for (auto& [ldmk_11_id, ldmk_11_obs] : ldmk_11_observations)
        {
            if (ldmk_11_id > 123 && ldmk_11_id < 230)
            {
                BOOST_CHECK_EQUAL(ldmk_11_obs.getWeight(), 1.);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(230).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(231).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(232).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(233).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(235).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(236).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(237).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(238).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(239).getWeight(), 1., tolerance);

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(500).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(501).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(502).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(637).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(638).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(639).getWeight(), .6, tolerance);
        // const double landmarkWeight = .232;
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 60th, the 120th and the 234th views
    // observations on the whole imageSet, and observations on nine views of the 2nd imageSequence
    {
        auto& landmark_12 = sfmData.getLandmarks().at(12);
        auto& ldmk_12_observations = landmark_12.getObservations();

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(0).getWeight(), 1., tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(1).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(2).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(3).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(4).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(6).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(7).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(8).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(9).getWeight(), .8, tolerance);
        for (auto& [ldmk_12_id, ldmk_12_obs] : ldmk_12_observations)
        {
            if (ldmk_12_id > 9 && ldmk_12_id < 55)
            {
                BOOST_CHECK_EQUAL(ldmk_12_obs.getWeight(), 1.);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(55).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(56).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(57).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(58).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(60).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(61).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(62).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(63).getWeight(), .8, tolerance);
        for (auto& [ldmk_12_id, ldmk_12_obs] : ldmk_12_observations)
        {
            if (ldmk_12_id > 63 && ldmk_12_id < 115)
            {
                BOOST_CHECK_EQUAL(ldmk_12_obs.getWeight(), 1.);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(115).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(116).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(117).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(118).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(120).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(121).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(122).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(123).getWeight(), .8, tolerance);
        for (auto& [ldmk_12_id, ldmk_12_obs] : ldmk_12_observations)
        {
            if (ldmk_12_id > 123 && ldmk_12_id < 229)
            {
                BOOST_CHECK_EQUAL(ldmk_12_obs.getWeight(), 1.);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(229).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(230).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(231).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(232).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(234).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(235).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(236).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(237).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(238).getWeight(), 1., tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(239).getWeight(), 1., tolerance);

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(561).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(562).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(563).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(564).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(565).getWeight(), 1., tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(566).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(567).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(568).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(569).getWeight(), .2, tolerance);
        // const double landmarkWeight = .280;
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 60th, the 61th and the 234th views
    // observations on the whole imageSet, and observations on ten views of the 2nd imageSequence
    {
        auto& landmark_13 = sfmData.getLandmarks().at(13);
        auto& ldmk_13_observations = landmark_13.getObservations();


        BOOST_CHECK_CLOSE(ldmk_13_observations.at(0).getWeight(), 1., tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(1).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(2).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(3).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(4).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_13_observations.at(6).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(7).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(8).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(9).getWeight(), .8, tolerance);
        for (auto& [ldmk_13_id, ldmk_13_obs] : ldmk_13_observations)
        {
            if (ldmk_13_id > 9 && ldmk_13_id < 55)
            {
                BOOST_CHECK_EQUAL(ldmk_13_obs.getWeight(), 1.);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(55).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(56).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(57).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(58).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_13_observations.at(61).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(62).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(63).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(64).getWeight(), .8, tolerance);
        for (auto& [ldmk_13_id, ldmk_13_obs] : ldmk_13_observations)
        {
            if (ldmk_13_id > 64 && ldmk_13_id < 229)
            {
                BOOST_CHECK_EQUAL(ldmk_13_obs.getWeight(), 1.);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(229).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(230).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(231).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(232).getWeight(), .2, tolerance);

        BOOST_CHECK_CLOSE(ldmk_13_observations.at(234).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(235).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(236).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(237).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(238).getWeight(), 1., tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(239).getWeight(), 1., tolerance);

        BOOST_CHECK_CLOSE(ldmk_13_observations.at(561).getWeight(), .2, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(562).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(563).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(564).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(565).getWeight(), 1., tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(566).getWeight(), 1., tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(567).getWeight(), .8, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(568).getWeight(), .6, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(569).getWeight(), .4, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(570).getWeight(), .2, tolerance);
        // const double landmarkWeight = .296;
    }


    sfm::resetObservationWeights(sfmData);
    sfm::weightObservationsAlongTrack(sfmData, fadingSize);
    sfm::weightTracks(sfmData, trackLengthMaxWeight, trackLengthLowThreshold, trackLengthHighThreshold);


    // landmark with observations on the whole imageSequences and on the whole imageSet
    {
        auto& landmark_0 = sfmData.getLandmarks().at(0);
        const double landmarkWeight = 1.;

        for (auto& [_, ldmk_0_obs] : landmark_0.getObservations())
        {
            BOOST_CHECK_CLOSE(ldmk_0_obs.getWeight(), landmarkWeight, tolerance);
        }
    }
    // landmark with observations on the whole 1st imageSequence except on the first view, no observation on the imageSet,
    // and observations on the whole 2nd imageSequence
    {
        auto& landmark_1 = sfmData.getLandmarks().at(1);
        auto& ldmk_1_observations = landmark_1.getObservations();
        const double landmarkWeight = .664;

        BOOST_CHECK_CLOSE(ldmk_1_observations.at(1).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_1_observations.at(2).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_1_observations.at(3).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_1_observations.at(4).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_1_id, ldmk_1_obs] : ldmk_1_observations)
        {
            if (ldmk_1_id > 4)
            {
                BOOST_CHECK_CLOSE(ldmk_1_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
    }
    // landmark with observations on the whole 1st imageSequence except on the last view, and no observation on the imageSet
    // and on the 2nd imageSequence
    {
        auto& landmark_2 = sfmData.getLandmarks().at(2);
        auto& ldmk_2_observations = landmark_2.getObservations();
        const double landmarkWeight = .2;

        for (auto& [ldmk_2_id, ldmk_2_obs] : ldmk_2_observations)
        {
            if (ldmk_2_id < 235)
            {
                BOOST_CHECK_CLOSE(ldmk_2_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_2_observations.at(235).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_2_observations.at(236).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_2_observations.at(237).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_2_observations.at(238).getWeight(), .2 * landmarkWeight, tolerance);
    }
    // landmark with observations on the whole 1st imageSequence except on the first and the last views,
    // observations on the whole imageSet, and one observation on the 2nd imageSequence
    {
        auto& landmark_3 = sfmData.getLandmarks().at(3);
        auto& ldmk_3_observations = landmark_3.getObservations();
        const double landmarkWeight = .2;

        BOOST_CHECK_CLOSE(ldmk_3_observations.at(1).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(2).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(3).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(4).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_3_id, ldmk_3_obs] : ldmk_3_observations)
        {
            if (ldmk_3_id > 4 && ldmk_3_id < 235)
            {
                BOOST_CHECK_CLOSE(ldmk_3_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(235).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(236).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(237).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(238).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_3_observations.at(561).getWeight(), .2 * landmarkWeight, tolerance);
    }
    // landmark with observations on the whole 1st imageSequence except on the 2nd view,
    // observations on the whole imageSet, and two observations on the 2nd imageSequence
    {
        auto& landmark_4 = sfmData.getLandmarks().at(4);
        auto& ldmk_4_observations = landmark_4.getObservations();
        const double landmarkWeight = .216;

        BOOST_CHECK_CLOSE(ldmk_4_observations.at(0).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_4_observations.at(2).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_4_observations.at(3).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_4_observations.at(4).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_4_observations.at(5).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_4_id, ldmk_4_obs] : ldmk_4_observations)
        {
            if (ldmk_4_id > 5 && ldmk_4_id < 240)
            {
                BOOST_CHECK_CLOSE(ldmk_4_obs.getWeight(), landmarkWeight, tolerance);
            }
        }

        BOOST_CHECK_CLOSE(ldmk_4_observations.at(561).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_4_observations.at(562).getWeight(), .2 * landmarkWeight, tolerance);
    }
    // landmark with observations on the whole 1st imageSequence except on the 3rd view,
    // observations on the whole imageSet, and three observations on the 2nd imageSequence
    {
        auto& landmark_5 = sfmData.getLandmarks().at(5);
        auto& ldmk_5_observations = landmark_5.getObservations();
        const double landmarkWeight = .232;

        BOOST_CHECK_CLOSE(ldmk_5_observations.at(0).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(1).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_5_observations.at(3).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(4).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(5).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(6).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_5_id, ldmk_5_obs] : ldmk_5_observations)
        {
            if (ldmk_5_id > 6 && ldmk_5_id < 240)
            {
                BOOST_CHECK_CLOSE(ldmk_5_obs.getWeight(), landmarkWeight, tolerance);
            }
        }

        BOOST_CHECK_CLOSE(ldmk_5_observations.at(561).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(562).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(563).getWeight(), .2 * landmarkWeight, tolerance);
    }
    // landmark with observations on the whole 1st imageSequence except on the 4th and the last views,
    // observations on the whole imageSet, and observations on the whole 2nd imageSequence except on one view
    {
        auto& landmark_6 = sfmData.getLandmarks().at(6);
        auto& ldmk_6_observations = landmark_6.getObservations();
        const double landmarkWeight = 1.;

        BOOST_CHECK_CLOSE(ldmk_6_observations.at(0).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(1).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(2).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_6_observations.at(4).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(5).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(6).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(7).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_6_id, ldmk_6_obs] : ldmk_6_observations)
        {
            if (ldmk_6_id > 7 && ldmk_6_id < 235)
            {
                BOOST_CHECK_CLOSE(ldmk_6_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(235).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(236).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(237).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(238).getWeight(), .2 * landmarkWeight, tolerance);

        for (auto& [ldmk_6_id, ldmk_6_obs] : ldmk_6_observations)
        {
            if (ldmk_6_id > 240 && ldmk_6_id < 556)
            {
                BOOST_CHECK_CLOSE(ldmk_6_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(556).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(557).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(558).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(559).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_6_observations.at(561).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(562).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(563).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(564).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_6_id, ldmk_6_obs] : ldmk_6_observations)
        {
            if (ldmk_6_id > 564)
            {
                BOOST_CHECK_CLOSE(ldmk_6_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
    }
    // landmark with observations on the whole 1st imageSequence except on the 5th and the 239th views
    // observations on the whole imageSet, and observations on the whole 2nd imageSequence except on two views
    {
        auto& landmark_7 = sfmData.getLandmarks().at(7);
        auto& ldmk_7_observations = landmark_7.getObservations();
        const double landmarkWeight = 1.;

        BOOST_CHECK_CLOSE(ldmk_7_observations.at(0).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(1).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(2).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(3).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_7_observations.at(5).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(6).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(7).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(8).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_7_id, ldmk_7_obs] : ldmk_7_observations)
        {
            if (ldmk_7_id > 8 && ldmk_7_id < 234)
            {
                BOOST_CHECK_CLOSE(ldmk_7_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(234).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(235).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(236).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(237).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_7_observations.at(239).getWeight(), .2 * landmarkWeight, tolerance);

        for (auto& [ldmk_7_id, ldmk_7_obs] : ldmk_7_observations)
        {
            if (ldmk_7_id > 240 && ldmk_7_id < 556)
            {
                BOOST_CHECK_CLOSE(ldmk_7_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(556).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(557).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(558).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(559).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_7_observations.at(562).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(563).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(564).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(565).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_7_id, ldmk_7_obs] : ldmk_7_observations)
        {
            if (ldmk_7_id > 565)
            {
                BOOST_CHECK_CLOSE(ldmk_7_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th and the 238th views
    // observations on the whole imageSet, and observations on the whole 2nd imageSequence except on three views
    {
        auto& landmark_8 = sfmData.getLandmarks().at(8);
        auto& ldmk_8_observations = landmark_8.getObservations();
        const double landmarkWeight = 1.;

        BOOST_CHECK_CLOSE(ldmk_8_observations.at(0).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(1).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(2).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(3).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(4).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_8_observations.at(6).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(7).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(8).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(9).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_8_id, ldmk_8_obs] : ldmk_8_observations)
        {
            if (ldmk_8_id > 9 && ldmk_8_id < 233)
            {
                BOOST_CHECK_CLOSE(ldmk_8_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(233).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(234).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(235).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(236).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_8_observations.at(238).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(239).getWeight(), .4 * landmarkWeight, tolerance);

        for (auto& [ldmk_8_id, ldmk_8_obs] : ldmk_8_observations)
        {
            if (ldmk_8_id > 240 && ldmk_8_id < 556)
            {
                BOOST_CHECK_CLOSE(ldmk_8_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(556).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(557).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(558).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(559).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_8_observations.at(562).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_8_observations.at(564).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(565).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(566).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(567).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_8_id, ldmk_8_obs] : ldmk_8_observations)
        {
            if (ldmk_8_id > 567)
            {
                BOOST_CHECK_CLOSE(ldmk_8_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 7th and the 237th views
    // observations on the whole imageSet, and observations on the first and the last views of the 2nd imageSequence
    {
        auto& landmark_9 = sfmData.getLandmarks().at(9);
        auto& ldmk_9_observations = landmark_9.getObservations();
        const double landmarkWeight = .200;

        BOOST_CHECK_CLOSE(ldmk_9_observations.at(0).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(1).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(2).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(3).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(4).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_9_observations.at(7).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(8).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(9).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(10).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_9_id, ldmk_9_obs] : ldmk_9_observations)
        {
            if (ldmk_9_id > 10 && ldmk_9_id < 232)
            {
                BOOST_CHECK_CLOSE(ldmk_9_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(232).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(233).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(234).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(235).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_9_observations.at(237).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(238).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(239).getWeight(), .6 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_9_observations.at(500).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_9_observations.at(639).getWeight(), .2 * landmarkWeight, tolerance);
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 7th, the 8th and the 236th views
    // observations on the whole imageSet, and observations on the two first and two last views of the 2nd imageSequence
    {
        auto& landmark_10 = sfmData.getLandmarks().at(10);
        auto& ldmk_10_observations = landmark_10.getObservations();
        const double landmarkWeight = .200;

        BOOST_CHECK_CLOSE(ldmk_10_observations.at(0).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(1).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(2).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(3).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(4).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_10_observations.at(8).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(9).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(10).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(11).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_10_id, ldmk_10_obs] : ldmk_10_observations)
        {
            if (ldmk_10_id > 11 && ldmk_10_id < 231)
            {
                BOOST_CHECK_CLOSE(ldmk_10_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(231).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(232).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(233).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(234).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_10_observations.at(236).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(237).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(238).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(239).getWeight(), .8 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_10_observations.at(500).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(501).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_10_observations.at(638).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(639).getWeight(), .4 * landmarkWeight, tolerance);
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 7th, the 120th and the 235th views
    // observations on the whole imageSet, and observations on the three first and three last views of the 2nd imageSequence
    {
        auto& landmark_11 = sfmData.getLandmarks().at(11);
        auto& ldmk_11_observations = landmark_11.getObservations();
        const double landmarkWeight = .232;

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(0).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(1).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(2).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(3).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(4).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(7).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(8).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(9).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(10).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_11_id, ldmk_11_obs] : ldmk_11_observations)
        {
            if (ldmk_11_id > 10 && ldmk_11_id < 115)
            {
                BOOST_CHECK_CLOSE(ldmk_11_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(115).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(116).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(117).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(118).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(120).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(121).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(122).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(123).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_11_id, ldmk_11_obs] : ldmk_11_observations)
        {
            if (ldmk_11_id > 123 && ldmk_11_id < 230)
            {
                BOOST_CHECK_CLOSE(ldmk_11_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(230).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(231).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(232).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(233).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(235).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(236).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(237).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(238).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(239).getWeight(), 1. * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(500).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(501).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(502).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(637).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(638).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(639).getWeight(), .6 * landmarkWeight, tolerance);
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 60th, the 120th and the 234th views
    // observations on the whole imageSet, and observations on nine views of the 2nd imageSequence
    {
        auto& landmark_12 = sfmData.getLandmarks().at(12);
        auto& ldmk_12_observations = landmark_12.getObservations();
        const double landmarkWeight = .280;

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(0).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(1).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(2).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(3).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(4).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(6).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(7).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(8).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(9).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_12_id, ldmk_12_obs] : ldmk_12_observations)
        {
            if (ldmk_12_id > 9 && ldmk_12_id < 55)
            {
                BOOST_CHECK_CLOSE(ldmk_12_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(55).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(56).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(57).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(58).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(60).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(61).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(62).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(63).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_12_id, ldmk_12_obs] : ldmk_12_observations)
        {
            if (ldmk_12_id > 63 && ldmk_12_id < 115)
            {
                BOOST_CHECK_CLOSE(ldmk_12_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(115).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(116).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(117).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(118).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(120).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(121).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(122).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(123).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_12_id, ldmk_12_obs] : ldmk_12_observations)
        {
            if (ldmk_12_id > 123 && ldmk_12_id < 229)
            {
                BOOST_CHECK_CLOSE(ldmk_12_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(229).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(230).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(231).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(232).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(234).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(235).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(236).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(237).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(238).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(239).getWeight(), 1. * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(561).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(562).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(563).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(564).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(565).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(566).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(567).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(568).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(569).getWeight(), .2 * landmarkWeight, tolerance);
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 60th, the 61th and the 234th views
    // observations on the whole imageSet, and observations on ten views of the 2nd imageSequence
    {
        auto& landmark_13 = sfmData.getLandmarks().at(13);
        auto& ldmk_13_observations = landmark_13.getObservations();
        const double landmarkWeight = .296;

        BOOST_CHECK_CLOSE(ldmk_13_observations.at(0).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(1).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(2).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(3).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(4).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_13_observations.at(6).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(7).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(8).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(9).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_13_id, ldmk_13_obs] : ldmk_13_observations)
        {
            if (ldmk_13_id > 9 && ldmk_13_id < 55)
            {
                BOOST_CHECK_CLOSE(ldmk_13_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(55).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(56).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(57).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(58).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_13_observations.at(61).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(62).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(63).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(64).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_13_id, ldmk_13_obs] : ldmk_13_observations)
        {
            if (ldmk_13_id > 64 && ldmk_13_id < 229)
            {
                BOOST_CHECK_CLOSE(ldmk_13_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(229).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(230).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(231).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(232).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_13_observations.at(234).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(235).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(236).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(237).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(238).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(239).getWeight(), 1. * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_13_observations.at(561).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(562).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(563).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(564).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(565).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(566).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(567).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(568).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(569).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(570).getWeight(), .2 * landmarkWeight, tolerance);
    }

    // Check that the weights are correctly computed in the other order
    // (to be sure the weights are updated and not simply overwritten)
    sfm::resetObservationWeights(sfmData);
    sfm::weightTracks(sfmData, trackLengthMaxWeight, trackLengthLowThreshold, trackLengthHighThreshold);
    sfm::weightObservationsAlongTrack(sfmData, fadingSize);

    // landmark with observations on the whole imageSequences and on the whole imageSet
    {
        auto& landmark_0 = sfmData.getLandmarks().at(0);
        const double landmarkWeight = 1.;

        for (auto& [_, ldmk_0_obs] : landmark_0.getObservations())
        {
            BOOST_CHECK_CLOSE(ldmk_0_obs.getWeight(), landmarkWeight, tolerance);
        }
    }
    // landmark with observations on the whole 1st imageSequence except on the first view, no observation on the imageSet,
    // and observations on the whole 2nd imageSequence
    {
        auto& landmark_1 = sfmData.getLandmarks().at(1);
        auto& ldmk_1_observations = landmark_1.getObservations();
        const double landmarkWeight = .664;

        BOOST_CHECK_CLOSE(ldmk_1_observations.at(1).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_1_observations.at(2).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_1_observations.at(3).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_1_observations.at(4).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_1_id, ldmk_1_obs] : ldmk_1_observations)
        {
            if (ldmk_1_id > 4)
            {
                BOOST_CHECK_CLOSE(ldmk_1_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
    }
    // landmark with observations on the whole 1st imageSequence except on the last view, and no observation on the imageSet
    // and on the 2nd imageSequence
    {
        auto& landmark_2 = sfmData.getLandmarks().at(2);
        auto& ldmk_2_observations = landmark_2.getObservations();
        const double landmarkWeight = .2;

        for (auto& [ldmk_2_id, ldmk_2_obs] : ldmk_2_observations)
        {
            if (ldmk_2_id < 235)
            {
                BOOST_CHECK_CLOSE(ldmk_2_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_2_observations.at(235).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_2_observations.at(236).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_2_observations.at(237).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_2_observations.at(238).getWeight(), .2 * landmarkWeight, tolerance);
    }
    // landmark with observations on the whole 1st imageSequence except on the first and the last views,
    // observations on the whole imageSet, and one observation on the 2nd imageSequence
    {
        auto& landmark_3 = sfmData.getLandmarks().at(3);
        auto& ldmk_3_observations = landmark_3.getObservations();
        const double landmarkWeight = .2;

        BOOST_CHECK_CLOSE(ldmk_3_observations.at(1).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(2).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(3).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(4).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_3_id, ldmk_3_obs] : ldmk_3_observations)
        {
            if (ldmk_3_id > 4 && ldmk_3_id < 235)
            {
                BOOST_CHECK_CLOSE(ldmk_3_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(235).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(236).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(237).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_3_observations.at(238).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_3_observations.at(561).getWeight(), .2 * landmarkWeight, tolerance);
    }
    // landmark with observations on the whole 1st imageSequence except on the 2nd view,
    // observations on the whole imageSet, and two observations on the 2nd imageSequence
    {
        auto& landmark_4 = sfmData.getLandmarks().at(4);
        auto& ldmk_4_observations = landmark_4.getObservations();
        const double landmarkWeight = .216;

        BOOST_CHECK_CLOSE(ldmk_4_observations.at(0).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_4_observations.at(2).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_4_observations.at(3).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_4_observations.at(4).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_4_observations.at(5).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_4_id, ldmk_4_obs] : ldmk_4_observations)
        {
            if (ldmk_4_id > 5 && ldmk_4_id < 240)
            {
                BOOST_CHECK_CLOSE(ldmk_4_obs.getWeight(), landmarkWeight, tolerance);
            }
        }

        BOOST_CHECK_CLOSE(ldmk_4_observations.at(561).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_4_observations.at(562).getWeight(), .2 * landmarkWeight, tolerance);
    }
    // landmark with observations on the whole 1st imageSequence except on the 3rd view,
    // observations on the whole imageSet, and three observations on the 2nd imageSequence
    {
        auto& landmark_5 = sfmData.getLandmarks().at(5);
        auto& ldmk_5_observations = landmark_5.getObservations();
        const double landmarkWeight = .232;

        BOOST_CHECK_CLOSE(ldmk_5_observations.at(0).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(1).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_5_observations.at(3).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(4).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(5).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(6).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_5_id, ldmk_5_obs] : ldmk_5_observations)
        {
            if (ldmk_5_id > 6 && ldmk_5_id < 240)
            {
                BOOST_CHECK_CLOSE(ldmk_5_obs.getWeight(), landmarkWeight, tolerance);
            }
        }

        BOOST_CHECK_CLOSE(ldmk_5_observations.at(561).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(562).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_5_observations.at(563).getWeight(), .2 * landmarkWeight, tolerance);
    }
    // landmark with observations on the whole 1st imageSequence except on the 4th and the last views,
    // observations on the whole imageSet, and observations on the whole 2nd imageSequence except on one view
    {
        auto& landmark_6 = sfmData.getLandmarks().at(6);
        auto& ldmk_6_observations = landmark_6.getObservations();
        const double landmarkWeight = 1.;

        BOOST_CHECK_CLOSE(ldmk_6_observations.at(0).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(1).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(2).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_6_observations.at(4).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(5).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(6).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(7).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_6_id, ldmk_6_obs] : ldmk_6_observations)
        {
            if (ldmk_6_id > 7 && ldmk_6_id < 235)
            {
                BOOST_CHECK_CLOSE(ldmk_6_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(235).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(236).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(237).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(238).getWeight(), .2 * landmarkWeight, tolerance);

        for (auto& [ldmk_6_id, ldmk_6_obs] : ldmk_6_observations)
        {
            if (ldmk_6_id > 240 && ldmk_6_id < 556)
            {
                BOOST_CHECK_CLOSE(ldmk_6_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(556).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(557).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(558).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(559).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_6_observations.at(561).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(562).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(563).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_6_observations.at(564).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_6_id, ldmk_6_obs] : ldmk_6_observations)
        {
            if (ldmk_6_id > 564)
            {
                BOOST_CHECK_CLOSE(ldmk_6_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
    }
    // landmark with observations on the whole 1st imageSequence except on the 5th and the 239th views
    // observations on the whole imageSet, and observations on the whole 2nd imageSequence except on two views
    {
        auto& landmark_7 = sfmData.getLandmarks().at(7);
        auto& ldmk_7_observations = landmark_7.getObservations();
        const double landmarkWeight = 1.;

        BOOST_CHECK_CLOSE(ldmk_7_observations.at(0).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(1).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(2).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(3).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_7_observations.at(5).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(6).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(7).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(8).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_7_id, ldmk_7_obs] : ldmk_7_observations)
        {
            if (ldmk_7_id > 8 && ldmk_7_id < 234)
            {
                BOOST_CHECK_CLOSE(ldmk_7_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(234).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(235).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(236).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(237).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_7_observations.at(239).getWeight(), .2 * landmarkWeight, tolerance);

        for (auto& [ldmk_7_id, ldmk_7_obs] : ldmk_7_observations)
        {
            if (ldmk_7_id > 240 && ldmk_7_id < 556)
            {
                BOOST_CHECK_CLOSE(ldmk_7_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(556).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(557).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(558).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(559).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_7_observations.at(562).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(563).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(564).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_7_observations.at(565).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_7_id, ldmk_7_obs] : ldmk_7_observations)
        {
            if (ldmk_7_id > 565)
            {
                BOOST_CHECK_CLOSE(ldmk_7_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th and the 238th views
    // observations on the whole imageSet, and observations on the whole 2nd imageSequence except on three views
    {
        auto& landmark_8 = sfmData.getLandmarks().at(8);
        auto& ldmk_8_observations = landmark_8.getObservations();
        const double landmarkWeight = 1.;

        BOOST_CHECK_CLOSE(ldmk_8_observations.at(0).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(1).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(2).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(3).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(4).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_8_observations.at(6).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(7).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(8).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(9).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_8_id, ldmk_8_obs] : ldmk_8_observations)
        {
            if (ldmk_8_id > 9 && ldmk_8_id < 233)
            {
                BOOST_CHECK_CLOSE(ldmk_8_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(233).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(234).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(235).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(236).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_8_observations.at(238).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(239).getWeight(), .4 * landmarkWeight, tolerance);

        for (auto& [ldmk_8_id, ldmk_8_obs] : ldmk_8_observations)
        {
            if (ldmk_8_id > 240 && ldmk_8_id < 556)
            {
                BOOST_CHECK_CLOSE(ldmk_8_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(556).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(557).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(558).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(559).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_8_observations.at(562).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_8_observations.at(564).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(565).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(566).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_8_observations.at(567).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_8_id, ldmk_8_obs] : ldmk_8_observations)
        {
            if (ldmk_8_id > 567)
            {
                BOOST_CHECK_CLOSE(ldmk_8_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 7th and the 237th views
    // observations on the whole imageSet, and observations on the first and the last views of the 2nd imageSequence
    {
        auto& landmark_9 = sfmData.getLandmarks().at(9);
        auto& ldmk_9_observations = landmark_9.getObservations();
        const double landmarkWeight = .200;

        BOOST_CHECK_CLOSE(ldmk_9_observations.at(0).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(1).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(2).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(3).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(4).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_9_observations.at(7).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(8).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(9).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(10).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_9_id, ldmk_9_obs] : ldmk_9_observations)
        {
            if (ldmk_9_id > 10 && ldmk_9_id < 232)
            {
                BOOST_CHECK_CLOSE(ldmk_9_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(232).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(233).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(234).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(235).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_9_observations.at(237).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(238).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_9_observations.at(239).getWeight(), .6 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_9_observations.at(500).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_9_observations.at(639).getWeight(), .2 * landmarkWeight, tolerance);
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 7th, the 8th and the 236th views
    // observations on the whole imageSet, and observations on the two first and two last views of the 2nd imageSequence
    {
        auto& landmark_10 = sfmData.getLandmarks().at(10);
        auto& ldmk_10_observations = landmark_10.getObservations();
        const double landmarkWeight = .200;

        BOOST_CHECK_CLOSE(ldmk_10_observations.at(0).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(1).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(2).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(3).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(4).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_10_observations.at(8).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(9).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(10).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(11).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_10_id, ldmk_10_obs] : ldmk_10_observations)
        {
            if (ldmk_10_id > 11 && ldmk_10_id < 231)
            {
                BOOST_CHECK_CLOSE(ldmk_10_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(231).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(232).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(233).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(234).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_10_observations.at(236).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(237).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(238).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(239).getWeight(), .8 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_10_observations.at(500).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(501).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_10_observations.at(638).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_10_observations.at(639).getWeight(), .4 * landmarkWeight, tolerance);
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 7th, the 120th and the 235th views
    // observations on the whole imageSet, and observations on the three first and three last views of the 2nd imageSequence
    {
        auto& landmark_11 = sfmData.getLandmarks().at(11);
        auto& ldmk_11_observations = landmark_11.getObservations();
        const double landmarkWeight = .232;

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(0).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(1).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(2).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(3).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(4).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(7).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(8).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(9).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(10).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_11_id, ldmk_11_obs] : ldmk_11_observations)
        {
            if (ldmk_11_id > 10 && ldmk_11_id < 115)
            {
                BOOST_CHECK_CLOSE(ldmk_11_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(115).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(116).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(117).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(118).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(120).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(121).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(122).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(123).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_11_id, ldmk_11_obs] : ldmk_11_observations)
        {
            if (ldmk_11_id > 123 && ldmk_11_id < 230)
            {
                BOOST_CHECK_CLOSE(ldmk_11_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(230).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(231).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(232).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(233).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(235).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(236).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(237).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(238).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(239).getWeight(), 1. * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(500).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(501).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(502).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_11_observations.at(637).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(638).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_11_observations.at(639).getWeight(), .6 * landmarkWeight, tolerance);
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 60th, the 120th and the 234th views
    // observations on the whole imageSet, and observations on nine views of the 2nd imageSequence
    {
        auto& landmark_12 = sfmData.getLandmarks().at(12);
        auto& ldmk_12_observations = landmark_12.getObservations();
        const double landmarkWeight = .280;

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(0).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(1).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(2).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(3).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(4).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(6).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(7).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(8).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(9).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_12_id, ldmk_12_obs] : ldmk_12_observations)
        {
            if (ldmk_12_id > 9 && ldmk_12_id < 55)
            {
                BOOST_CHECK_CLOSE(ldmk_12_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(55).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(56).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(57).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(58).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(60).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(61).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(62).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(63).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_12_id, ldmk_12_obs] : ldmk_12_observations)
        {
            if (ldmk_12_id > 63 && ldmk_12_id < 115)
            {
                BOOST_CHECK_CLOSE(ldmk_12_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(115).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(116).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(117).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(118).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(120).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(121).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(122).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(123).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_12_id, ldmk_12_obs] : ldmk_12_observations)
        {
            if (ldmk_12_id > 123 && ldmk_12_id < 229)
            {
                BOOST_CHECK_CLOSE(ldmk_12_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(229).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(230).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(231).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(232).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(234).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(235).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(236).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(237).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(238).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(239).getWeight(), 1. * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_12_observations.at(561).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(562).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(563).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(564).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(565).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(566).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(567).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(568).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_12_observations.at(569).getWeight(), .2 * landmarkWeight, tolerance);
    }
    // landmark with observations on the whole 1st imageSequence except on the 6th, the 60th, the 61th and the 234th views
    // observations on the whole imageSet, and observations on ten views of the 2nd imageSequence
    {
        auto& landmark_13 = sfmData.getLandmarks().at(13);
        auto& ldmk_13_observations = landmark_13.getObservations();
        const double landmarkWeight = .296;

        BOOST_CHECK_CLOSE(ldmk_13_observations.at(0).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(1).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(2).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(3).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(4).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_13_observations.at(6).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(7).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(8).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(9).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_13_id, ldmk_13_obs] : ldmk_13_observations)
        {
            if (ldmk_13_id > 9 && ldmk_13_id < 55)
            {
                BOOST_CHECK_CLOSE(ldmk_13_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(55).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(56).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(57).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(58).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_13_observations.at(61).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(62).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(63).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(64).getWeight(), .8 * landmarkWeight, tolerance);
        for (auto& [ldmk_13_id, ldmk_13_obs] : ldmk_13_observations)
        {
            if (ldmk_13_id > 64 && ldmk_13_id < 229)
            {
                BOOST_CHECK_CLOSE(ldmk_13_obs.getWeight(), landmarkWeight, tolerance);
            }
        }
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(229).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(230).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(231).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(232).getWeight(), .2 * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_13_observations.at(234).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(235).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(236).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(237).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(238).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(239).getWeight(), 1. * landmarkWeight, tolerance);

        BOOST_CHECK_CLOSE(ldmk_13_observations.at(561).getWeight(), .2 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(562).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(563).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(564).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(565).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(566).getWeight(), 1. * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(567).getWeight(), .8 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(568).getWeight(), .6 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(569).getWeight(), .4 * landmarkWeight, tolerance);
        BOOST_CHECK_CLOSE(ldmk_13_observations.at(570).getWeight(), .2 * landmarkWeight, tolerance);
    }
}
