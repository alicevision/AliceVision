// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2014 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/feature/feature.hpp>
#include <aliceVision/feature/sift/ImageDescriber_SIFT.hpp>
#include <aliceVision/matching/RegionsMatcher.hpp>
#include <aliceVision/multiview/essential.hpp>
#include <aliceVision/robustEstimation/ACRansac.hpp>
#include <aliceVision/robustEstimation/conditioning.hpp>
#include <aliceVision/multiview/AngularRadianErrorKernel.hpp>

#include "sphericalCam.hpp"

#include <software/utils/sfmHelper/sfmPlyHelper.hpp>

#include <dependencies/vectorGraphics/svgDrawer.hpp>

#include <string>
#include <iostream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::image;
using namespace aliceVision::matching;
using namespace aliceVision::robustEstimation;
using namespace svg;

int main()
{
    std::mt19937 randomNumberGenerator;
    std::cout << "Compute the relative pose between two spherical image."
              << "\nUse an Acontrario robust estimation based on angular errors." << std::endl;

    const std::string sInputDir = std::string(THIS_SOURCE_DIR);
    const std::string jpg_filenameL = sInputDir + "/SponzaLion000.jpg";

    Image<unsigned char> imageL;
    readImage(jpg_filenameL, imageL, image::EImageColorSpace::NO_CONVERSION);

    const std::string jpg_filenameR = sInputDir + "/SponzaLion001.jpg";

    Image<unsigned char> imageR;
    readImage(jpg_filenameR, imageR, image::EImageColorSpace::NO_CONVERSION);

    //--
    // Detect regions thanks to an image_describer
    //--
    using namespace aliceVision::feature;
    SiftParams siftParams;
    siftParams._firstOctave = -1;
    std::unique_ptr<ImageDescriber> image_describer(new ImageDescriber_SIFT(siftParams));
    std::map<IndexT, std::unique_ptr<feature::Regions> > regions_perImage;
    image_describer->describe(imageL, regions_perImage[0]);
    image_describer->describe(imageR, regions_perImage[1]);

    const SIFT_Regions* regionsL = dynamic_cast<SIFT_Regions*>(regions_perImage.at(0).get());
    const SIFT_Regions* regionsR = dynamic_cast<SIFT_Regions*>(regions_perImage.at(1).get());

    const PointFeatures
        featsL = regions_perImage.at(0)->GetRegionsPositions(),
        featsR = regions_perImage.at(1)->GetRegionsPositions();

    std::cout << "Left image SIFT count: " << featsL.size() << std::endl;
    std::cout << "Right image SIFT count: "<< featsR.size() << std::endl;

    // Show both images side by side
    {
        Image<unsigned char> concat;
        ConcatH(imageL, imageR, concat);
        std::string out_filename = "01_concat.jpg";
        writeImage(out_filename, concat,
                   image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION));
    }

    //- Draw features on the two image (side by side)
    {
        Image<unsigned char> concat;
        ConcatH(imageL, imageR, concat);

        //-- Draw features :
        for (size_t i=0; i < featsL.size(); ++i ) {
            const PointFeature point = regionsL->Features()[i];
            DrawCircle(point.x(), point.y(), point.scale(), 255, &concat);
        }
        for (size_t i=0; i < featsR.size(); ++i ) {
            const PointFeature point = regionsR->Features()[i];
            DrawCircle(point.x()+imageL.Width(), point.y(), point.scale(), 255, &concat);
        }
        std::string out_filename = "02_features.jpg";
        writeImage(out_filename, concat,
                   image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION));
    }

    std::vector<IndMatch> vec_PutativeMatches;
    //-- Perform matching -> find Nearest neighbor, filtered with Distance ratio
    {
        // Find corresponding points
        matching::DistanceRatioMatch(
            randomNumberGenerator,
            0.8, matching::ANN_L2,
            *regions_perImage.at(0).get(),
            *regions_perImage.at(1).get(),
            vec_PutativeMatches);

        IndMatchDecorator<float> matchDeduplicator(vec_PutativeMatches, featsL, featsR);
        matchDeduplicator.getDeduplicated(vec_PutativeMatches);

        // Draw correspondences after Nearest Neighbor ratio filter
        svgDrawer svgStream(imageL.Width() + imageR.Width(), std::max(imageL.Height(), imageR.Height()));
        svgStream.drawImage(jpg_filenameL, imageL.Width(), imageL.Height());
        svgStream.drawImage(jpg_filenameR, imageR.Width(), imageR.Height(), imageL.Width());
        for (size_t i = 0; i < vec_PutativeMatches.size(); ++i) {
            //Get back linked feature, draw a circle and link them by a line
            const PointFeature L = regionsL->Features()[vec_PutativeMatches[i]._i];
            const PointFeature R = regionsR->Features()[vec_PutativeMatches[i]._j];
            svgStream.drawLine(L.x(), L.y(), R.x()+imageL.Width(), R.y(), svgStyle().stroke("green", 2.0));
            svgStream.drawCircle(L.x(), L.y(), L.scale(), svgStyle().stroke("yellow", 2.0));
            svgStream.drawCircle(R.x()+imageL.Width(), R.y(), R.scale(),svgStyle().stroke("yellow", 2.0));
        }
        std::string out_filename = "03_siftMatches.svg";
        std::ofstream svgFile(out_filename.c_str());
        svgFile << svgStream.closeSvgFile().str();
        svgFile.close();
    }

    // Essential geometry filtering of putative matches
    {
        //A. get back interest point and send it to the robust estimation framework
        Mat xL(2, vec_PutativeMatches.size());
        Mat xR(2, vec_PutativeMatches.size());

        for (size_t k = 0; k < vec_PutativeMatches.size(); ++k)  {
        const PointFeature & imaL = featsL[vec_PutativeMatches[k]._i];
        const PointFeature & imaR = featsR[vec_PutativeMatches[k]._j];
        xL.col(k) = imaL.coords().cast<double>();
        xR.col(k) = imaR.coords().cast<double>();
        }

        //-- Convert planar to spherical coordinates
        Mat xL_spherical(3,vec_PutativeMatches.size()), xR_spherical(3,vec_PutativeMatches.size());
        spherical_cam::planarToSpherical(xL, imageL.Width(), imageL.Height(), xL_spherical);
        spherical_cam::planarToSpherical(xR, imageR.Width(), imageR.Height(), xR_spherical);

        //-- Essential matrix robust estimation from spherical bearing vectors
        {
            std::vector<size_t> vec_inliers;

            // Use the 8 point solver in order to estimate E
            typedef aliceVision::spherical_cam::EssentialKernel_spherical Kernel;

            // Define the AContrario angular error adaptor
            typedef aliceVision::multiview::AngularRadianErrorKernel<
                aliceVision::spherical_cam::EightPointRelativePoseSolver,
                aliceVision::spherical_cam::AngularError,
                robustEstimation::Mat3Model>
                KernelType;

            KernelType kernel(xL_spherical, xR_spherical);

            // Robust estimation of the Essential matrix and it's precision
            robustEstimation::Mat3Model E;
            const double precision = std::numeric_limits<double>::infinity();
            const std::pair<double,double> ACRansacOut = ACRANSAC(kernel, randomNumberGenerator, vec_inliers, 1024, &E, precision);
            const double & threshold = ACRansacOut.first;
            const double & NFA = ACRansacOut.second;

            std::cout << "\n Angular threshold found: " << radianToDegree(threshold) << "(Degree)"<<std::endl;
            std::cout << "\n #Putatives/#inliers : " << xL_spherical.cols() << "/" << vec_inliers.size() << "\n" << std::endl;

            if (vec_inliers.size() > 120)
            {
                // If an essential matrix have been found
                // Extract R|t
                //  - From the 4 possible solutions extracted from E keep the best
                //  - (check cheirality of correspondence once triangulation is done)

                // Accumulator to find the best solution
                std::vector<size_t> f(4, 0);

                std::vector<robustEstimation::Mat3Model> Es;  // Essential,
                std::vector<Mat3> Rs;                  // Rotation matrix.
                std::vector<Vec3> ts;                  // Translation matrix.

                Es.push_back(E);

                // Recover best rotation and translation from E.
                motionFromEssential(E.getMatrix(), &Rs, &ts);

                //-> Test the 4 solutions will all the point
                Mat34 P1;
                P_from_KRt(Mat3::Identity(), Mat3::Identity(), Vec3::Zero(), &P1);
                std::vector< std::vector<size_t> > vec_newInliers(4);
                std::vector< std::vector<Vec3> > vec_3D(4);

                for (int kk = 0; kk < 4; ++kk) {
                    const Mat3 &R2 = Rs[kk];
                    const Vec3 &t2 = ts[kk];
                    Mat34 P2;
                    P_from_KRt(Mat3::Identity(), R2, t2, &P2);

                    //-- For each inlier:
                    //   - triangulate
                    //   - check chierality
                    for (size_t k = 0; k < vec_inliers.size(); ++k) {
                        const Vec3 & x1_ = xL_spherical.col(vec_inliers[k]);
                        const Vec3 & x2_ = xR_spherical.col(vec_inliers[k]);

                        //Triangulate
                        Vec3 X;
                        aliceVision::spherical_cam::TriangulateDLT(P1, x1_, P2, x2_, &X);

                        //Check positivity of the depth (sign of the dot product)
                        const Vec3 Mc = R2 * X + t2;
                        if (x2_.dot(Mc) > 0 && x1_.dot(X) > 0) {
                            ++f[kk];
                            vec_newInliers[kk].push_back(vec_inliers[k]);
                            vec_3D[kk].push_back(X);
                        }
                    }
                }
                std::cout << std::endl << "estimate_Rt_fromE" << std::endl;
                std::cout << " #points in front of both cameras for each solution: "
                          << f[0] << " " << f[1] << " " << f[2] << " " << f[3] << std::endl;

                std::vector<size_t>::iterator iter = max_element(f.begin(), f.end());
                if(*iter != 0)  {
                    const size_t index = std::distance(f.begin(),iter);
                    if (f[index] < 120) {
                        std::cout << "Found solution have too few 3D points." << std::endl;
                    }
                    else {
                        std::cout << "Export found 3D scene in current folder." << std::endl;
                        vec_inliers.clear();
                        vec_inliers = vec_newInliers[index];
                        std::ostringstream os;
                        os << "./" << "relativePose_Spherical"<< ".ply";
                        plyHelper::exportToPly(vec_3D[index], os.str());
                    }
                }
            }
        }
    }
    return EXIT_SUCCESS;
}

