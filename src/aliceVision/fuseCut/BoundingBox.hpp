// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <Eigen/Dense>

namespace aliceVision {
namespace fuseCut {

/**
 * Oriented Bounding box structure
 * A bounding box is considered to be a centered cube of length 2
 * Which is rotated, scaled and translated
*/
struct BoundingBox
{
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
    Eigen::Vector3d scale = Eigen::Vector3d::Zero();

    /**
     * Is the bounding box initialized ?
     * @return true if values avec been set correctly
    */
    inline bool isInitialized() const 
    { 
        return scale(0) != 0.0; 
    }

    /**
     * Return the transformation matrix SE(3)
     * @return the SE(3) matrix
    */
    Eigen::Matrix4d modelMatrix() const
    {
        // Compute the translation matrix
        Eigen::Matrix4d translateMat = Eigen::Matrix4d::Identity();
        translateMat.col(3).head<3>() << translation.x(), translation.y(), translation.z();

        // Compute the rotation matrix from quaternion made with Euler angles in that order: ZXY (same as Qt algorithm)
        Eigen::Matrix4d rotateMat = Eigen::Matrix4d::Identity();

        {
            double pitch = rotation.x() * M_PI / 180;
            double yaw = rotation.y() * M_PI / 180;
            double roll = rotation.z() * M_PI / 180;

            pitch *= 0.5;
            yaw *= 0.5;
            roll *= 0.5;

            const double cy = std::cos(yaw);
            const double sy = std::sin(yaw);
            const double cr = std::cos(roll);
            const double sr = std::sin(roll);
            const double cp = std::cos(pitch);
            const double sp = std::sin(pitch);
            const double cycr = cy * cr;
            const double sysr = sy * sr;

            const double w = cycr * cp + sysr * sp;
            const double x = cycr * sp + sysr * cp;
            const double y = sy * cr * cp - cy * sr * sp;
            const double z = cy * sr * cp - sy * cr * sp;

            Eigen::Quaterniond quaternion(w, x, y, z);
            rotateMat.block<3, 3>(0, 0) = quaternion.matrix();
        }

        // Compute the scale matrix
        Eigen::Matrix4d scaleMat = Eigen::Matrix4d::Identity();
        scaleMat.diagonal().head<3>() << scale.x(), scale.y(), scale.z();

        // Model matrix
        Eigen::Matrix4d modelMat = translateMat * rotateMat * scaleMat;

        return modelMat;
    }

    /**
     * Return an hexahedron which is the corners of the boudning box
     * To replace ....
     * 
    */
    void toHexahedron(Point3d* hexah) const
    {
        Eigen::Matrix4d modelMat = modelMatrix();

        // Retrieve the eight vertices of the bounding box
        // Based on VoxelsGrid::getHexah implementation
        Eigen::Vector4d origin = Eigen::Vector4d(-1, -1, -1, 1);
        Eigen::Vector4d vvx = Eigen::Vector4d(2, 0, 0, 0);
        Eigen::Vector4d vvy = Eigen::Vector4d(0, 2, 0, 0);
        Eigen::Vector4d vvz = Eigen::Vector4d(0, 0, 2, 0);

        Eigen::Vector4d vertex0 = modelMat * origin;
        Eigen::Vector4d vertex1 = modelMat * (origin + vvx);
        Eigen::Vector4d vertex2 = modelMat * (origin + vvx + vvy);
        Eigen::Vector4d vertex3 = modelMat * (origin + vvy);
        Eigen::Vector4d vertex4 = modelMat * (origin + vvz);
        Eigen::Vector4d vertex5 = modelMat * (origin + vvz + vvx);
        Eigen::Vector4d vertex6 = modelMat * (origin + vvz + vvx + vvy);
        Eigen::Vector4d vertex7 = modelMat * (origin + vvz + vvy);

        // Apply those eight vertices to the hexah
        hexah[0] = Point3d(vertex0.x(), vertex0.y(), vertex0.z());
        hexah[1] = Point3d(vertex1.x(), vertex1.y(), vertex1.z());
        hexah[2] = Point3d(vertex2.x(), vertex2.y(), vertex2.z());
        hexah[3] = Point3d(vertex3.x(), vertex3.y(), vertex3.z());
        hexah[4] = Point3d(vertex4.x(), vertex4.y(), vertex4.z());
        hexah[5] = Point3d(vertex5.x(), vertex5.y(), vertex5.z());
        hexah[6] = Point3d(vertex6.x(), vertex6.y(), vertex6.z());
        hexah[7] = Point3d(vertex7.x(), vertex7.y(), vertex7.z());
    }

    static BoundingBox fromHexahedron(const Point3d* hexah)
    {
        BoundingBox bbox;

        // Compute the scale
        bbox.scale(0) = (hexah[0] - hexah[1]).size() / 2.;
        bbox.scale(1) = (hexah[0] - hexah[3]).size() / 2.;
        bbox.scale(2) = (hexah[0] - hexah[4]).size() / 2.;

        // Compute the translation
        Point3d cg(0., 0., 0.);
        for (int i = 0; i < 8; i++)
        {
            cg += hexah[i];
        }
        cg /= 8.;

        bbox.translation(0) = cg.x;
        bbox.translation(1) = cg.y;
        bbox.translation(2) = cg.z;

        // Compute the rotation matrix
        Eigen::Matrix3d rotateMat = Eigen::Matrix3d::Identity();
        Point3d cx = ((hexah[1] + hexah[2] + hexah[5] + hexah[6]) / 4. - cg).normalize();
        Point3d cy = ((hexah[3] + hexah[2] + hexah[7] + hexah[6]) / 4. - cg).normalize();
        Point3d cz = ((hexah[7] + hexah[4] + hexah[5] + hexah[6]) / 4. - cg).normalize();
        rotateMat.col(0).head<3>() << cx.x, cx.y, cx.z;
        rotateMat.col(1).head<3>() << cy.x, cy.y, cy.z;
        rotateMat.col(2).head<3>() << cz.x, cz.y, cz.z;

        // Euler rotation angles
        Eigen::Vector3d ea = rotateMat.eulerAngles(1, 0, 2) * 180. / M_PI;
        bbox.rotation(0) = ea(1);
        bbox.rotation(1) = ea(0);
        bbox.rotation(2) = ea(2);

        return bbox;
    }
};

/**
 * Istream overloading for parameters reading
 * @param in the input stream
 * @param out_bbox the created box
*/
inline std::istream& operator>>(std::istream& in, BoundingBox& out_bbox)
{
    std::string token(std::istreambuf_iterator<char>(in), {});

    std::vector<std::string> dataStr;
    boost::split(dataStr, token, boost::is_any_of(","));
    if (dataStr.size() != 9)
    {
        throw std::runtime_error("Invalid number of values for bounding box.");
    }

    std::vector<double> data;
    data.reserve(9);
    for (const std::string& elt : dataStr)
    {
        data.push_back(boost::lexical_cast<double>(elt));
    }

    out_bbox.translation << data[0], data[1], data[2];
    out_bbox.rotation << data[3], data[4], data[5];
    out_bbox.scale << data[6], data[7], data[8];

    return in;
}

}
}