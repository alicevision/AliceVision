// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "lightingEstimation.hpp"
#include "augmentedNormals.hpp"

#include <Eigen/Dense>

#include <iostream>


namespace aliceVision {
namespace lightingEstimation {

/**
 * @brief Evaluate albedo and normal product for one channel
 */
void albedoNormalsProduct(MatrixXf& rhoTimesN, const MatrixXf& albedoChannel, const image::Image<AugmentedNormal>& augmentedNormals)
{
    for (int i = 0; i < augmentedNormals.size(); ++i)
    {
        rhoTimesN(i, 0) = albedoChannel(i) * augmentedNormals(i).nx();
        rhoTimesN(i, 1) = albedoChannel(i) * augmentedNormals(i).ny();
        rhoTimesN(i, 2) = albedoChannel(i) * augmentedNormals(i).nz();
        rhoTimesN(i, 3) = albedoChannel(i) * augmentedNormals(i).nambiant();
        rhoTimesN(i, 4) = albedoChannel(i) * augmentedNormals(i).nx_ny();
        rhoTimesN(i, 5) = albedoChannel(i) * augmentedNormals(i).nx_nz();
        rhoTimesN(i, 6) = albedoChannel(i) * augmentedNormals(i).ny_nz();
        rhoTimesN(i, 7) = albedoChannel(i) * augmentedNormals(i).nx2_ny2();
        rhoTimesN(i, 8) = albedoChannel(i) * augmentedNormals(i).nz2();
    }
}

/**
 * @brief Resolve lighting estimation problem for one channel
 */
 
void estimateLigthingOneChannel(Eigen::Matrix<float, 9, 1>& lighting, const MatrixXf& albedoChannel, const MatrixXf& pictureChannel, const image::Image<AugmentedNormal>& augNormals)
{
    const auto nbPoints = augNormals.size();

    MatrixXf rhoTimesN(nbPoints, 9);
    albedoNormalsProduct(rhoTimesN, albedoChannel, augNormals);

    lighting = rhoTimesN.colPivHouseholderQr().solve(pictureChannel);
}

void estimateLigthing(LightingVector& lighting, const image::Image<image::RGBfColor>& albedo, const image::Image<image::RGBfColor>& picture, const image::Image<image::RGBfColor>& normals)
{
    using namespace Eigen;

    // Map albedo, image
    const std::size_t nbPixels = albedo.Width() * albedo.Height();

    Map<MatrixXf, 0, InnerStride<3>> albedoR((float*)&albedo.data()->r(), nbPixels, 1);
    Map<MatrixXf, 0, InnerStride<3>> albedoG((float*)&albedo.data()->g(), nbPixels, 1);
    Map<MatrixXf, 0, InnerStride<3>> albedoB((float*)&albedo.data()->b(), nbPixels, 1);

    Map<MatrixXf, 0, InnerStride<3>> pictureR((float*)&picture.data()->r(), nbPixels, 1);
    Map<MatrixXf, 0, InnerStride<3>> pictureG((float*)&picture.data()->g(), nbPixels, 1);
    Map<MatrixXf, 0, InnerStride<3>> pictureB((float*)&picture.data()->b(), nbPixels, 1);

    Eigen::Matrix<float, 9, 1> lightingR;
    Eigen::Matrix<float, 9, 1> lightingG;
    Eigen::Matrix<float, 9, 1> lightingB;

    // Augmented normales
    image::Image<AugmentedNormal> augNormals(normals.cast<AugmentedNormal>());

    // EstimateLightingOneChannel
    estimateLigthingOneChannel(lightingR, albedoR, pictureR, augNormals);
    estimateLigthingOneChannel(lightingG, albedoG, pictureG, augNormals);
    estimateLigthingOneChannel(lightingB, albedoB, pictureB, augNormals);

    // lighting vectors fusion
    lighting.col(0) = lightingR;
    lighting.col(1) = lightingG;
    lighting.col(2) = lightingB;
}


}
}
