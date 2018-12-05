#include "lightingEstimation.hpp"
#include "augmentedNormals.hpp"

#include <Eigen/Dense>

#include <iostream>


namespace aliceVision {
namespace inverseRendering {


void albedoNormalsProduct(MatrixXf& rhoTimesN, const MatrixXf& albedoChannel, const AugmentedNormals& augmentedNormals)
{
    for (int i = 0; i < augmentedNormals.ambiant.size(); ++i)
    {
        rhoTimesN(i, 1) = albedoChannel(i) * augmentedNormals.nx(i);
        rhoTimesN(i, 2) = albedoChannel(i) * augmentedNormals.ny(i);
        rhoTimesN(i, 3) = albedoChannel(i) * augmentedNormals.nz(i);
        rhoTimesN(i, 4) = albedoChannel(i) * augmentedNormals.ambiant(i);
        rhoTimesN(i, 5) = albedoChannel(i) * augmentedNormals.nx_ny(i);
        rhoTimesN(i, 6) = albedoChannel(i) * augmentedNormals.nx_nz(i);
        rhoTimesN(i, 7) = albedoChannel(i) * augmentedNormals.ny_nz(i);
        rhoTimesN(i, 8) = albedoChannel(i) * augmentedNormals.nx2_ny2(i);
        rhoTimesN(i, 9) = albedoChannel(i) * augmentedNormals.nz2(i);
    }
}

void estimateLigthingOneChannel(Eigen::Matrix<float, 9, 1>& lighting, const MatrixXf& albedoChannel, const MatrixXf& pictureChannel, const AugmentedNormals& augNormals)
{
    int nbPoints = augNormals.ambiant.size();

    MatrixXf rhoTimesN(nbPoints, 9);
    albedoNormalsProduct(rhoTimesN, albedoChannel, augNormals);

    lighting = rhoTimesN.colPivHouseholderQr().solve(pictureChannel);
}

void estimateLigthing(LightingVector& lighting, const Image<RGBfColor>& albedo, const Image<RGBfColor>& picture, const Image<RGBfColor>& normals)
{
    using namespace Eigen;

    // Map albedo, image
    std::size_t nbPixels = albedo.Width() * albedo.Height();

    Map<MatrixXf, 0, OuterStride<3>> albedoR((float*)&albedo.data()->r(), nbPixels, 1);
    Map<MatrixXf, 0, OuterStride<3>> albedoG((float*)&albedo.data()->g(), nbPixels, 1);
    Map<MatrixXf, 0, OuterStride<3>> albedoB((float*)&albedo.data()->b(), nbPixels, 1);

    Map<MatrixXf, 0, OuterStride<3>> pictureR((float*)&picture.data()->r(), nbPixels, 1);
    Map<MatrixXf, 0, OuterStride<3>> pictureG((float*)&picture.data()->g(), nbPixels, 1);
    Map<MatrixXf, 0, OuterStride<3>> pictureB((float*)&picture.data()->b(), nbPixels, 1);

    Eigen::Matrix<float, 9, 1> lightingR;
    Eigen::Matrix<float, 9, 1> lightingG;
    Eigen::Matrix<float, 9, 1> lightingB;

    // Augmented normales
    AugmentedNormals augNormals;
    getAugmentedNormals(augNormals, normals);

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
