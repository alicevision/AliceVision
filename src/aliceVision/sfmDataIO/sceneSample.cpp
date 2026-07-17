// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "sceneSample.hpp"
#include <aliceVision/stl/hash.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/sfmData/ImageSequence.hpp>

namespace aliceVision {
namespace sfmDataIO {

ESceneType ESceneType_stringToEnum(const std::string& SScene)
{
    std::string scene = SScene;
    std::transform(scene.begin(), scene.end(), scene.begin(), ::tolower);

    if (scene == "cube")
    {
        return ESceneType::SCENE_CUBE;
    }
    if (scene == "sphere")
    {
        return ESceneType::SCENE_SPHERE;
    }
    throw std::invalid_argument("Invalid scene type: " + scene);
}

std::string ESceneType_enumToString(const ESceneType EScene)
{
    if (EScene == ESceneType::SCENE_CUBE)
    {
        return "cube";
    }
    if (EScene == ESceneType::SCENE_SPHERE)
    {
        return "sphere";
    }
    throw std::invalid_argument("Unrecognized ESceneType: " + std::to_string(int(EScene)));
}

std::ostream& operator<<(std::ostream& os, ESceneType p) { return os << ESceneType_enumToString(p); }

std::istream& operator>>(std::istream& in, ESceneType& p)
{
    std::string token(std::istreambuf_iterator<char>(in), {});
    p = ESceneType_stringToEnum(token);
    return in;
}


void generateSampleScene(sfmData::SfMData& output, ESceneType scene)
{
    // Cleanup sfmData
    output.clear();

    if (scene == ESceneType::SCENE_CUBE)
    {
        generateCubeScene(output);
    }
    else if (scene == ESceneType::SCENE_SPHERE)
    {
        generateSphereScene(output, 1000, 240);
    }
    else
    {
        throw std::out_of_range("Invalid ESceneType");
    }
}


void generateCubeScene(sfmData::SfMData& output)
{
    // Generate points on a cube
    IndexT idpt = 0;
    for (int x = -10; x <= 10; ++x)
    {
        for (int y = -10; y <= 10; ++y)
        {
            for (int z = -10; z <= 10; ++z)
            {
                output.getLandmarks().emplace(idpt, sfmData::Landmark(Vec3(x, y, z), feature::EImageDescriberType::UNKNOWN));
                ++idpt;
            }
        }
    }

    const int w = 4092;
    const int h = 2048;
    const double focalLengthPixX = 1000.0;
    const double focalLengthPixY = 2000.0;
    const double offsetX = -26;
    const double offsetY = 16;
    output.getIntrinsics().emplace(
      0, camera::createPinhole(camera::DISTORTION_NONE, camera::UNDISTORTION_NONE, w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY));
    output.getIntrinsics().emplace(
      1,
      camera::createPinhole(camera::DISTORTION_RADIALK3, camera::UNDISTORTION_NONE, w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY, {0.1, 0.05, -0.001}));

    // Generate poses on another cube
    IndexT idpose = 0;
    IndexT idview = 0;
    for (int x = -1; x <= 1; ++x)
    {
        for (int y = -1; y <= 1; ++y)
        {
            for (int z = -1; z <= 1; ++z)
            {
                const Eigen::Vector3d thetau(x, y, z);
                const Eigen::AngleAxis<double> aa(thetau.norm(), thetau.normalized());

                geometry::Pose3 pose(aa.toRotationMatrix(), Vec3(x, y, z));

                output.getPoses().assign(idpose, sfmData::CameraPose(pose));

                for (const auto itIntrinsic : output.getIntrinsics())
                {
                    output.getViews().emplace(idview, std::make_shared<sfmData::View>("", idview, itIntrinsic.first, idpose, w, h));
                    ++idview;
                }

                ++idpose;
            }
        }
    }
}

void generateSphereScene(sfmData::SfMData& output, int pointsNb, int posesNb)
{
    // Generate random points on a sphere

    size_t sphereSceneHash = 4246135487;

    const int w = 4092;
    const int h = 2048;
    const double focalLengthPixX = 10000.0;
    const double focalLengthPixY = 20000.0;
    output.getIntrinsics().emplace(
      0, camera::createPinhole(camera::DISTORTION_NONE, camera::UNDISTORTION_NONE, w, h, focalLengthPixX, focalLengthPixY, 0, 0));

    size_t imageGroupID = sphereSceneHash;
    stl::hash_combine(imageGroupID, pointsNb);
    stl::hash_combine(imageGroupID, posesNb);

    // Generate poses on a circle
    for (IndexT idPV = 0; idPV < posesNb; idPV++)
    {
        double angle2d = (double(idPV) * 2. * M_PI) / posesNb;
        Eigen::AngleAxis<double> aa(angle2d + .5 * M_PI, Eigen::Vector3d::UnitY());
        Eigen::Matrix3d rotation = aa.toRotationMatrix();
        Eigen::Vector3d position(std::cos(angle2d), 0., std::sin(angle2d));
        geometry::Pose3 pose(rotation, 10. * position);
        output.getPoses().assign(idPV, sfmData::CameraPose(pose));
        output.getViews().emplace(idPV, std::make_shared<sfmData::View>("", idPV, 0, idPV, w, h));
        output.getView(idPV).setFrameId(idPV);
        output.getView(idPV).setImageGroupId(imageGroupID);
    }

    auto imageGroupPtr = std::make_shared<sfmData::ImageSequence>();
    output.getImageGroups().emplace(imageGroupID, imageGroupPtr);

    // Generate random points on a sphere and corresponding observations in each view
    const double arbitraryScale = 1.0;
    for (int landmarkId = 0; landmarkId < pointsNb; landmarkId++)
    {
        Eigen::Vector3d point3D = Eigen::Vector3d::Random();
        point3D = point3D / point3D.norm();

        sfmData::Landmark landmark(point3D, feature::EImageDescriberType::UNKNOWN);
        for (int viewId = 0; viewId < posesNb; viewId++)
        {
            const sfmData::View& view = output.getView(viewId);
            const geometry::Pose3 pose = output.getPose(view).getTransform();
            const camera::IntrinsicBase& intrinsic = output.getIntrinsic(0);
            const Eigen::Vector2d pt = intrinsic.transformProject(pose, landmark.getX().homogeneous(), true);
            landmark.getObservations().emplace(viewId, sfmData::Observation(pt, landmarkId, arbitraryScale));
        }
        output.getLandmarks().emplace(landmarkId, landmark);
    }
}

}  // namespace sfmDataIO
}  // namespace aliceVision
