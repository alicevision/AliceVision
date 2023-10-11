// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "plyIO.hpp"

#include <fstream>

namespace aliceVision {
namespace sfmDataIO {

bool savePLY(const sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag)
{
    const bool b_structure = (partFlag & STRUCTURE) == STRUCTURE;
    const bool b_extrinsics = (partFlag & EXTRINSICS) == EXTRINSICS;

    if (!(b_structure || b_extrinsics))
        return false;

    bool b_binary = false;
    {
        auto position = filename.find(".bin.");
        if (position != std::string::npos)
            b_binary = true;
    }

    // Create the stream and check it is ok
    std::ofstream stream(filename);
    if (!stream.is_open())
        return false;

    bool bOk = false;
    {
        // Count how many views having valid poses:
        IndexT view_with_pose_count = 0;
        if (b_extrinsics)
        {
            for (const auto& view : sfmData.getViews())
            {
                view_with_pose_count += sfmData.isPoseAndIntrinsicDefined(view.second.get());
            }
        }

        stream << "ply" << '\n'
               << "format " << (b_binary ? "binary_little_endian" : "ascii") << " 1.0 " << '\n'
               << "element vertex "
               // Vertex count: (#landmark + #view_with_valid_pose)
               << ((b_structure ? sfmData.getLandmarks().size() : 0) + view_with_pose_count) << '\n'
               << "property float x" << '\n'
               << "property float y" << '\n'
               << "property float z" << '\n'
               << "property uchar red" << '\n'
               << "property uchar green" << '\n'
               << "property uchar blue" << '\n'
               << "end_header" << std::endl;

        if (b_extrinsics)
        {
            for (const auto& view : sfmData.getViews())
            {
                if (sfmData.isPoseAndIntrinsicDefined(view.second.get()))
                {
                    const geometry::Pose3 pose = sfmData.getPose(*(view.second.get())).getTransform();

                    if (b_binary)
                    {
                        auto point = pose.center().transpose();
                        float values[3] = {point.x(), point.y(), point.z()};
                        stream.write(reinterpret_cast<const char*>(&values), sizeof(float) * 3);

                        uint8_t color[3] = {0, 255, 0};
                        stream.write(reinterpret_cast<const char*>(&color), 3);
                    }
                    else
                    {
                        stream << pose.center().transpose() << " 0 255 0"
                               << "\n";
                    }
                }
            }
        }

        if (b_structure)
        {
            const sfmData::Landmarks& landmarks = sfmData.getLandmarks();

            for (sfmData::Landmarks::const_iterator iterLandmarks = landmarks.begin(); iterLandmarks != landmarks.end(); ++iterLandmarks)
            {
                if (b_binary)
                {
                    const auto& point = iterLandmarks->second.X.transpose();
                    float values[3] = {point.x(), point.y(), point.z()};
                    stream.write(reinterpret_cast<const char*>(&values), sizeof(float) * 3);

                    const auto& rgb = iterLandmarks->second.rgb;
                    uint8_t color[3] = {rgb.r(), rgb.g(), rgb.b()};
                    stream.write(reinterpret_cast<const char*>(&color), sizeof(uint8_t) * 3);
                }
                else
                {
                    stream << iterLandmarks->second.X.transpose() << " " << (int)iterLandmarks->second.rgb.r() << " "
                           << (int)iterLandmarks->second.rgb.g() << " " << (int)iterLandmarks->second.rgb.b() << "\n";
                }
            }
        }
        stream.flush();
        bOk = stream.good();
        stream.close();
    }
    return bOk;
}

}  // namespace sfmDataIO
}  // namespace aliceVision
