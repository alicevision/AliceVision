// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ellipseGeometry.hpp"
#include <aliceVision/photometricStereo/photometricDataIO.hpp>
#include <aliceVision/image/io.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <limits>

namespace aliceVision {
namespace lightingEstimation {

#define PI 3.14159265

void quadraticFromEllipseParameters(const std::array<float, 5>& ellipseParameters,
                                    Eigen::Matrix3f& Q)
{
    float phi = ellipseParameters[0]*PI/180.0;
    float c_x = ellipseParameters[1];
    float c_y = ellipseParameters[2];
    float b = ellipseParameters[3];
    float a = ellipseParameters[4];

    float A = a*a*sin(phi)*sin(phi) + b*b*cos(phi)*cos(phi);
    float B = 2*(b*b-a*a)*sin(phi)*cos(phi);
    float C = a*a*cos(phi)*cos(phi) + b*b*sin(phi)*sin(phi);
    float D = -2*A*c_x - B*c_y;
    float E = -B*c_x - 2*C*c_y;
    float F = A*c_x*c_x + C*c_y*c_y - a*a*b*b + B*c_x*c_y;

    Q << A, B/2, D/2,
         B/2, C, E/2,
         D/2, E/2, F;

    Q /= Q.norm();
}

int findUniqueIndex(const std::vector<int>& vec) {

    std::unordered_map<int, int> countMap;

    // Compter le nombre d'occurrences de chaque élément dans le vecteur
    for (int num : vec) {
        countMap[num]++;
    }

    int toReturn = -1;
    // Trouver l'élément avec une seule occurrence
    for (size_t i = 0; i < vec.size(); ++i) {
        if (countMap[vec[i]] == 1) {
            toReturn = i;
        }
    }

    return toReturn;
}

void estimateSphereCenter(const std::array<float, 5>& ellipseParameters,
                          const float sphereRadius,
                          const Eigen::Matrix3f& K,
                          std::array<float, 3>& sphereCenter)
{
    Eigen::Matrix3f Q;
    quadraticFromEllipseParameters(ellipseParameters, Q);

    Eigen::Matrix3f M = K.transpose()*Q*K;
    Eigen::EigenSolver<Eigen::MatrixXf> es(M);

    Eigen::Vector3f eigval = es.eigenvalues().real();

    std::vector<int> eigvalSign;

    for (int i = 0; i < 3; ++i) {
        eigvalSign.push_back((eigval[i] > 0) ? 1 : -1);
    }
   
    int index = findUniqueIndex(eigvalSign);
    float uniqueEigval = eigval[index];

    float dist = sqrt(1 - ((eigval[0] + eigval[1] + eigval[2] - uniqueEigval)/2) / uniqueEigval);

    Eigen::Vector3f eigvect = es.eigenvectors().col(index).real();
    float sign = eigvect[2] > 0 ? 1 : -1;

    float norm = eigvect.norm();
    float C_factor = sphereRadius*dist*sign/norm;
    Eigen::Vector3f C = C_factor*eigvect;
    sphereCenter[0]=C[0];
    sphereCenter[1]=C[1];
    sphereCenter[2]=C[2];
}

void sphereRayIntersection(const Eigen::Vector3f& direction,
                           const std::array<float, 3>& sphereCenter,
                           const float sphereRadius,
                           float& delta,
                           Eigen::Vector3f& normal)
{
    float a = direction.dot(direction);

    Eigen::Vector3f spCenter;
    spCenter << sphereCenter[0], sphereCenter[1], sphereCenter[2];

    float b = -2*direction.dot(spCenter);
    float c = spCenter.dot(spCenter) - sphereRadius*sphereRadius;

    delta = b*b - 4*a*c;

    float factor;

    if (delta >= 0) {
        factor = (-b - sqrt(delta))/(2*a);
        normal = (direction*factor - spCenter)/sphereRadius;
    }
    else {
        delta = -1;
    }
}

void estimateSphereNormals(const std::array<float, 3>& sphereCenter,
                           const float sphereRadius,
                           const Eigen::Matrix3f& K,
                           image::Image<image::RGBfColor>& normals,
                           image::Image<float>& newMask) 
{
    Eigen::Matrix3f invK = K.inverse();

    for (int i = 0; i < normals.rows(); ++i) {
        for (int j = 0; j < normals.cols(); ++j) {
            // Get homogeneous coordinates of the pixel :
            Eigen::Vector3f coordinates = Eigen::Vector3f(j, i, 1.0f);

            // Get the direction of the ray :
            Eigen::Vector3f direction = invK*coordinates;

            // Estimate the interception of the ray with the sphere :
            float delta = 0.0;
            Eigen::Vector3f normal = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

            sphereRayIntersection(direction, sphereCenter, sphereRadius, delta, normal);

            // If the ray intercepts the sphere we add this pixel to the new mask :
            if(delta > 0)
            {
                newMask(i, j) = 1.0;
                normals(i, j) = image::RGBfColor(normal[0], normal[1], normal[2]);
            }
            else
            {
                newMask(i, j) = 0.0;
                normals(i, j) = image::RGBfColor(0.0, 0.0, 0.0);
            }
        }
    }
}

void getRealNormalOnSphere(const cv::Mat& maskCV,
                       const Eigen::Matrix3f& K,
                       const float sphereRadius,
                       image::Image<image::RGBfColor>& normals,
                       image::Image<float>& newMask)
{

    // Apply a threshold to the image
    int thresholdValue = 150;
    cv::Mat thresh;
    cv::threshold(maskCV, thresh, thresholdValue, 255, 0);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Fit the ellipse to the first contour
    std::vector<cv::Point> cnt = contours[0];
    cv::RotatedRect ellipse = cv::fitEllipse(cnt);
    float angle = ellipse.angle;
    cv::Size2f size = ellipse.size;
    cv::Point2f center = ellipse.center;

    // Ellipse is converted as five-parameter array
    std::array<float, 5> ellipseParameters;

    ellipseParameters[0] = angle;
    ellipseParameters[1] = center.x;
    ellipseParameters[2] = center.y;
    ellipseParameters[3] = size.width/2;
    ellipseParameters[4] = size.height/2;

    std::array<float, 3> sphereCenter;
    estimateSphereCenter(ellipseParameters, sphereRadius, K, sphereCenter);
    estimateSphereNormals(sphereCenter, sphereRadius, K, normals, newMask);
}

void getEllipseMaskFromSphereParameters(const std::array<float, 3>& sphereParam, const Eigen::Matrix3f& K, std::array<float, 5>& ellipseParameters, cv::Mat maskCV)
{

    // Distance between center of image and center of disk :
    float imX = K(0, 2);
    float imY = K(1, 2);
    float f = K(0, 0);
    float delta = sqrt((sphereParam[0])*(sphereParam[0]) + (sphereParam[1])*(sphereParam[1]));
    // sphere params = x,y, radius
    // ellipse params  = angle, x, y, semi minor axe, semi major axe

    // Ellipse from sphere parameters
    // semi minor axe = radius;
    // semi major axe = formula from paper
    // main direction = disc center to picture center
    // ellipse center = disc center

    float radians =  atan((sphereParam[0]) / (sphereParam[1]));
    ellipseParameters[0] = radians * (180.0/3.141592653589793238463);

    ellipseParameters[1] = sphereParam[0];
    ellipseParameters[2] = sphereParam[1];
    ellipseParameters[3] = sphereParam[2];


    // a² = b² ((distance between image center and disc center)² + f² + b²)/(f² + b²)
    ellipseParameters[4] = sqrt((ellipseParameters[3]*ellipseParameters[3]) * (delta * delta + f*f + ellipseParameters[3]*ellipseParameters[3])/(f*f + ellipseParameters[3]*ellipseParameters[3]));
}

} // namespace lightingEstimation
} // namespace aliceVision