// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/ProgressDisplay.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/utils/filesIO.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <filesystem>
#include <vector>
#include <set>
#include <iterator>
#include <iomanip>
#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::image;
using namespace aliceVision::sfmData;
using namespace aliceVision::sfmDataIO;

namespace po = boost::program_options;
namespace fs = std::filesystem;

template<class ImageT, class MaskFuncT>
void process(const std::string& dstColorImage,
             const IntrinsicBase* cam,
             const oiio::ParamValueList& metadata,
             const std::string& srcImage,
             bool evCorrection,
             float exposureCompensation,
             MaskFuncT&& maskFunc)
{
    ImageT image, image_ud;
    readImage(srcImage, image, image::EImageColorSpace::LINEAR);

    // exposure correction
    if (evCorrection)
    {
        for (int pix = 0; pix < image.width() * image.height(); ++pix)
        {
            image(pix)[0] *= exposureCompensation;
            image(pix)[1] *= exposureCompensation;
            image(pix)[2] *= exposureCompensation;
        }
    }

    // mask
    maskFunc(image);

    // undistort
    if (cam->isValid() && cam->hasDistortion())
    {
        // undistort the image and save it
        using Pix = typename ImageT::Tpixel;
        Pix pixZero(Pix::Zero());
        UndistortImage(image, cam, image_ud, pixZero);
        writeImage(dstColorImage, image_ud, image::ImageWriteOptions(), metadata);
    }
    else
    {
        writeImage(dstColorImage, image, image::ImageWriteOptions(), metadata);
    }
}

using ObservationsPerView = stl::flat_map<std::size_t, std::vector<const Observation*>>;

/**
 * @brief Get the landmark observations of camera views.
 * @param[in] sfmData: A given SfMData
 * @return Observations per camera view
 */
ObservationsPerView getObservationsPerViews(const SfMData& sfmData)
{
    ObservationsPerView observationsPerView;
    for(auto& landIt : sfmData.getLandmarks())
    {
        for (const auto& obsIt : landIt.second.getObservations())
        {
            IndexT viewId = obsIt.first;
            auto& observationsSet = observationsPerView[viewId];
            observationsSet.push_back(&obsIt.second);
        }
    }
    return observationsPerView;
}

bool prepareDenseScene(const SfMData& sfmData,
                       const std::vector<std::string>& imagesFolders,
                       const std::vector<std::string>& masksFolders,
                       const std::string& maskExtension,
                       int beginIndex,
                       int endIndex,
                       const std::string& outFolder,
                       image::EImageFileType outputFileType,
                       bool saveMetadata,
                       bool saveMatricesFiles,
                       bool evCorrection,
                       float landmarksMaskScale,
                       std::string inputRadiiFilename)
{
    // defined view Ids
    std::set<IndexT> viewIds;

    sfmData::Views::const_iterator itViewBegin = sfmData.getViews().begin();
    sfmData::Views::const_iterator itViewEnd = sfmData.getViews().end();

    if (endIndex > 0)
    {
        itViewEnd = itViewBegin;
        std::advance(itViewEnd, endIndex);
    }

    std::advance(itViewBegin, (beginIndex < 0) ? 0 : beginIndex);

    // export valid views as projective cameras
    for (auto it = itViewBegin; it != itViewEnd; ++it)
    {
        const View* view = it->second.get();
        if (!sfmData.isPoseAndIntrinsicDefined(view))
            continue;
        viewIds.insert(view->getViewId());
    }

    if ((outputFileType != image::EImageFileType::EXR) && saveMetadata)
        ALICEVISION_LOG_WARNING("Cannot save informations in images metadata.\n"
                                "Choose '.exr' file type if you want AliceVision custom metadata");

    // export data
    auto progressDisplay = system::createConsoleProgressDisplay(viewIds.size(), std::cout, "Exporting Scene Undistorted Images\n");

    // for exposure correction
    const double medianCameraExposure = sfmData.getMedianCameraExposureSetting().getExposure();
    ALICEVISION_LOG_INFO("Median Camera Exposure: " << medianCameraExposure << ", Median EV: " << std::log2(1.0 / medianCameraExposure));

    bool doMaskLandmarks = landmarksMaskScale > 0.f;
    ObservationsPerView observationsPerView;
    std::map<IndexT, double> estimatedRadii;
    if (doMaskLandmarks)
    {
        observationsPerView = std::move(getObservationsPerViews(sfmData));

        if (!inputRadiiFilename.empty())
        {
            std::stringstream stream;
            std::string line;
            IndexT viewId;
            double radius;

            std::fstream fs(inputRadiiFilename, std::ios::in);
            if(!fs.is_open())
            {
                ALICEVISION_LOG_WARNING("Unable to open the radii file " << inputRadiiFilename
                                                                         << "\nDefaulting to using image size.");
            }
            else
            {
                while(!fs.eof())
                {
                    std::getline(fs, line);
                    stream.clear();
                    stream.str(line);
                    stream >> viewId;
                    stream >> radius;
                    estimatedRadii[viewId] = radius;
                }
                fs.close();
            }
        }

    }

#pragma omp parallel for
    for (int i = 0; i < viewIds.size(); ++i)
    {
        auto itView = viewIds.begin();
        std::advance(itView, i);

        const IndexT viewId = *itView;
        const View* view = sfmData.getViews().at(viewId).get();

        Intrinsics::const_iterator iterIntrinsic = sfmData.getIntrinsics().find(view->getIntrinsicId());

        // we have a valid view with a corresponding camera & pose
        const std::string baseFilename = std::to_string(viewId);

        // get metadata from source image to be sure we get all metadata. We don't use the metadatas from the Views inside the SfMData to avoid type
        // conversion problems with string maps.
        std::string srcImage = view->getImage().getImagePath();
        oiio::ParamValueList metadata = image::readImageMetadata(srcImage);

        // export camera
        if (saveMetadata || saveMatricesFiles)
        {
            // get camera pose / projection
            const Pose3 pose = sfmData.getPose(*view).getTransform();

            std::shared_ptr<camera::IntrinsicBase> cam = iterIntrinsic->second;
            std::shared_ptr<camera::Pinhole> camPinHole = std::dynamic_pointer_cast<camera::Pinhole>(cam);
            if (!camPinHole)
            {
                ALICEVISION_LOG_ERROR("Camera is not pinhole in filter");
                continue;
            }

            Mat34 P = camPinHole->getProjectiveEquivalent(pose);

            // get camera intrinsics matrices
            const Mat3 K = dynamic_cast<const Pinhole*>(sfmData.getIntrinsicPtr(view->getIntrinsicId()))->K();
            const Mat3& R = pose.rotation();
            const Vec3& t = pose.translation();

            if (saveMatricesFiles)
            {
                std::ofstream fileP((fs::path(outFolder) / (baseFilename + "_P.txt")).string());
                fileP << std::setprecision(10) << P(0, 0) << " " << P(0, 1) << " " << P(0, 2) << " " << P(0, 3) << "\n"
                      << P(1, 0) << " " << P(1, 1) << " " << P(1, 2) << " " << P(1, 3) << "\n"
                      << P(2, 0) << " " << P(2, 1) << " " << P(2, 2) << " " << P(2, 3) << "\n";
                fileP.close();

                std::ofstream fileKRt((fs::path(outFolder) / (baseFilename + "_KRt.txt")).string());
                fileKRt << std::setprecision(10) << K(0, 0) << " " << K(0, 1) << " " << K(0, 2) << "\n"
                        << K(1, 0) << " " << K(1, 1) << " " << K(1, 2) << "\n"
                        << K(2, 0) << " " << K(2, 1) << " " << K(2, 2) << "\n"
                        << "\n"
                        << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << "\n"
                        << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << "\n"
                        << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << "\n"
                        << "\n"
                        << t(0) << " " << t(1) << " " << t(2) << "\n";
                fileKRt.close();
            }

            if (saveMetadata)
            {
                // convert to 44 matix
                Mat4 projectionMatrix;
                projectionMatrix << P(0, 0), P(0, 1), P(0, 2), P(0, 3), P(1, 0), P(1, 1), P(1, 2), P(1, 3), P(2, 0), P(2, 1), P(2, 2), P(2, 3), 0, 0,
                  0, 1;

                // convert matrices to rowMajor
                std::vector<double> vP(projectionMatrix.size());
                std::vector<double> vK(K.size());
                std::vector<double> vR(R.size());

                typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMatrixXd;
                Eigen::Map<RowMatrixXd>(vP.data(), projectionMatrix.rows(), projectionMatrix.cols()) = projectionMatrix;
                Eigen::Map<RowMatrixXd>(vK.data(), K.rows(), K.cols()) = K;
                Eigen::Map<RowMatrixXd>(vR.data(), R.rows(), R.cols()) = R;

                // add metadata
                metadata.push_back(oiio::ParamValue("AliceVision:downscale", 1));
                metadata.push_back(oiio::ParamValue("AliceVision:P", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX44), 1, vP.data()));
                metadata.push_back(oiio::ParamValue("AliceVision:K", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX33), 1, vK.data()));
                metadata.push_back(oiio::ParamValue("AliceVision:R", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX33), 1, vR.data()));
                metadata.push_back(oiio::ParamValue("AliceVision:t", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::VEC3), 1, t.data()));
            }
        }

        // export undistort image
        {
            if (!imagesFolders.empty())
            {
                std::vector<std::string> paths = sfmDataIO::viewPathsFromFolders(*view, imagesFolders);

                // if path was not found
                if (paths.empty())
                {
                    throw std::runtime_error("Cannot find view '" + std::to_string(view->getViewId()) + "' image file in given folder(s)");
                }
                else if (paths.size() > 1)
                {
                    throw std::runtime_error("Ambiguous case: Multiple source image files found in given folder(s) for the view '" +
                                             std::to_string(view->getViewId()) + "'.");
                }

                srcImage = paths[0];
            }
            const std::string dstColorImage =
              (fs::path(outFolder) / (baseFilename + "." + image::EImageFileType_enumToString(outputFileType))).string();
            const IntrinsicBase* cam = iterIntrinsic->second.get();

            // add exposure values to images metadata
            const double cameraExposure = view->getImage().getCameraExposureSetting().getExposure();
            const double ev = std::log2(1.0 / cameraExposure);
            const float exposureCompensation = float(medianCameraExposure / cameraExposure);
            metadata.push_back(oiio::ParamValue("AliceVision:EV", float(ev)));
            metadata.push_back(oiio::ParamValue("AliceVision:EVComp", exposureCompensation));

            if (evCorrection)
            {
                ALICEVISION_LOG_INFO("image " << viewId << ", exposure: " << cameraExposure << ", Ev " << ev
                                              << " Ev compensation: " + std::to_string(exposureCompensation));
            }

            image::Image<unsigned char> maskLandmarks;
            if(doMaskLandmarks)
            {
                // for the T camera, image alpha should be at least 0.4f * 255 (masking)
                maskLandmarks =
                    image::Image<unsigned char>(view->getImage().getWidth(), view->getImage().getHeight(), true, 127);
                double radius;
                const auto& it = estimatedRadii.find(viewId);
                if(it != estimatedRadii.end())
                    radius = it->second;
                else
                    radius = 0.5 * (view->getImage().getWidth() + view->getImage().getHeight());
                int r = (int)(landmarksMaskScale * radius);
                const auto& observationsIt = observationsPerView.find(viewId);
                if(observationsIt != observationsPerView.end())
                {
                    const auto& observations = observationsIt->second;
                    int j = 0;
                    for(const auto& observation : observations)
                    {
                        const auto& obs = *observation;
                        for (int y = std::max(obs.getCoordinates().y() - r, 0.);
                             y <= std::min(obs.getCoordinates().y() + r, (double)maskLandmarks.height() - 1);
                             y++)
                        {
                            for (int x = std::max(obs.getCoordinates().x() - r, 0.);
                                 x <= std::min(obs.getCoordinates().x() + r, (double)maskLandmarks.width() - 1);
                                 x++)
                            {
                                maskLandmarks(y, x) = std::numeric_limits<unsigned char>::max();
                            }
                        }
                        j++;
                    }
                }
            }

            image::Image<unsigned char> mask;
            bool maskLoaded = false;
            if(tryLoadMask(&mask, masksFolders, viewId, srcImage, maskExtension))
                maskLoaded = true;
            process<Image<RGBAfColor>>(
                dstColorImage, cam, metadata, srcImage, evCorrection, exposureCompensation,
                [&maskLoaded, &mask, &maskLandmarks, &doMaskLandmarks](Image<RGBAfColor>& image)
                {
                    if(maskLoaded && (image.width() * image.height() != mask.width() * mask.height()))
                    {
                        ALICEVISION_LOG_WARNING("Invalid image mask size: mask is ignored.");
                        return;
                    }

                    for(int pix = 0; pix < image.width() * image.height(); ++pix)
                    {
                        image(pix).a() = (maskLoaded && mask(pix) == 0) ? 0.f : (doMaskLandmarks && maskLandmarks(pix) == 127) ? .5f : 1.f;
                    }
                });
        }

        ++progressDisplay;
    }

    return true;
}

int aliceVision_main(int argc, char* argv[])
{
    // command-line parameters

    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmDataFilename;
    std::string outFolder;
    std::string outImageFileTypeName = image::EImageFileType_enumToString(image::EImageFileType::EXR);
    std::vector<std::string> imagesFolders;
    std::vector<std::string> masksFolders;
    std::string maskExtension = "png";
    int rangeStart = -1;
    int rangeSize = 1;
    bool saveMetadata = true;
    bool saveMatricesTxtFiles = false;
    bool evCorrection = false;
    std::string inputRadiiFilename;
    float landmarksMaskScale = 0.f;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&outFolder)->required(),
         "Output folder.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("imagesFolders",  po::value<std::vector<std::string>>(&imagesFolders)->multitoken(),
         "Use images from specific folder(s) instead of those specify in the SfMData file.\n"
         "Filename should be the same or the image UID.")
        ("masksFolders", po::value<std::vector<std::string>>(&masksFolders)->multitoken(),
         "Use masks from specific folder(s).\n"
         "Filename should be the same or the image UID.")
        ("maskExtension", po::value<std::string>(&maskExtension)->default_value(maskExtension),
         "File extension of the masks to use.")
        ("outputFileType", po::value<std::string>(&outImageFileTypeName)->default_value(outImageFileTypeName),
         image::EImageFileType_informations().c_str())
        ("saveMetadata", po::value<bool>(&saveMetadata)->default_value(saveMetadata),
         "Save projections and intrinsics information in images metadata.")
        ("saveMatricesTxtFiles", po::value<bool>(&saveMatricesTxtFiles)->default_value(saveMatricesTxtFiles),
         "Save projections and intrinsics information in text files.")
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
         "Range image index start.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
         "Range size.")
        ("evCorrection", po::value<bool>(&evCorrection)->default_value(evCorrection),
         "Correct exposure value.")
        ("landmarksMaskScale", po::value<float>(&landmarksMaskScale)->default_value(landmarksMaskScale),
         "Scale of the projection of landmarks to mask images for depth computation.\n"
         "If 0, masking using landmarks will not be used.\n"
         "Otherwise, it's used to scale the projection radius \n"
         "(either specified by `inputRadiiFile` or by image size if the former is not given).")
        ("inputRadiiFile", po::value<std::string>(&inputRadiiFilename)->default_value(inputRadiiFilename),
         "Input Radii file containing the estimated projection radius of landmarks per view. \n"
         "If not specified, image size will be used to specify the radius.");
    // clang-format on
    CmdLine cmdline("AliceVision prepareDenseScene");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set output file type
    image::EImageFileType outputFileType = image::EImageFileType_stringToEnum(outImageFileTypeName);

    // Create output dir
    if (!utils::exists(outFolder))
        fs::create_directory(outFolder);

    // Read the input SfM scene
    SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    int rangeEnd = sfmData.getViews().size();

    // set range
    if (rangeStart != -1)
    {
        if (rangeStart < 0 || rangeSize < 0)
        {
            ALICEVISION_LOG_ERROR("Range is incorrect");
            return EXIT_FAILURE;
        }

        if (rangeStart + rangeSize > sfmData.getViews().size())
            rangeSize = sfmData.getViews().size() - rangeStart;

        rangeEnd = rangeStart + rangeSize;

        if (rangeSize <= 0)
        {
            ALICEVISION_LOG_WARNING("Nothing to compute.");
            return EXIT_SUCCESS;
        }
    }
    else
    {
        rangeStart = 0;
    }

    // export
    if (prepareDenseScene(sfmData,
                          imagesFolders,
                          masksFolders,
                          maskExtension,
                          rangeStart,
                          rangeEnd,
                          outFolder,
                          outputFileType,
                          saveMetadata,
                          saveMatricesTxtFiles,
                          evCorrection,
                          landmarksMaskScale,
                          inputRadiiFilename))
        return EXIT_SUCCESS;

    return EXIT_FAILURE;
}
