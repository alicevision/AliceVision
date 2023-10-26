// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "sceneSample.hpp"

namespace aliceVision {
namespace sfmDataIO {

void generateSampleScene(sfmData::SfMData& output)
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
      0, camera::createPinhole(camera::EINTRINSIC::PINHOLE_CAMERA, w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY));
    output.getIntrinsics().emplace(
      1,
      camera::createPinhole(
        camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL3, w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY, {0.1, 0.05, -0.001}));

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

                output.getPoses().emplace(idpose, geometry::Pose3(aa.toRotationMatrix(), Vec3(x, y, z)));

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

}  // namespace sfmDataIO
}  // namespace aliceVision
