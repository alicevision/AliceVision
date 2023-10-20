// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "AlembicExporter.hpp"
#include <aliceVision/version.hpp>

#include <Alembic/AbcGeom/All.h>
#include <Alembic/AbcCoreOgawa/All.h>
#include <Alembic/Abc/OObject.h>

#include <boost/filesystem.hpp>

#include <numeric>

namespace fs = boost::filesystem;

namespace aliceVision {
namespace sfmDataIO {

using namespace Alembic::Abc;
using namespace Alembic::AbcGeom;

struct AlembicExporter::DataImpl
{
    explicit DataImpl(const std::string& filename)
      : _archive(Alembic::AbcCoreOgawa::WriteArchive(), filename),
        _topObj(_archive, Alembic::Abc::kTop)
    {
        // create MVG hierarchy
        _mvgRoot = Alembic::AbcGeom::OXform(_topObj, "mvgRoot");
        _mvgCameras = Alembic::AbcGeom::OXform(_mvgRoot, "mvgCameras");
        _mvgCamerasUndefined = Alembic::AbcGeom::OXform(_mvgRoot, "mvgCamerasUndefined");
        _mvgCloud = Alembic::AbcGeom::OXform(_mvgRoot, "mvgCloud");
        _mvgPointCloud = Alembic::AbcGeom::OXform(_mvgCloud, "mvgPointCloud");

        // add version as custom property
        const std::vector<::uint32_t> abcVersion = {
          ALICEVISION_SFMDATAIO_VERSION_MAJOR, ALICEVISION_SFMDATAIO_VERSION_MINOR, ALICEVISION_SFMDATAIO_VERSION_REVISION};
        const std::vector<::uint32_t> aliceVisionVersion = {ALICEVISION_VERSION_MAJOR, ALICEVISION_VERSION_MINOR, ALICEVISION_VERSION_REVISION};

        auto userProps = _mvgRoot.getProperties();

        OUInt32ArrayProperty(userProps, "mvg_ABC_version").set(abcVersion);
        OUInt32ArrayProperty(userProps, "mvg_aliceVision_version").set(aliceVisionVersion);

        // hide mvgCamerasUndefined
        Alembic::AbcGeom::CreateVisibilityProperty(_mvgCamerasUndefined, 0).set(Alembic::AbcGeom::kVisibilityHidden);
    }

    /**
     * @brief Add a camera
     * @param[in] name The camera identifier
     * @param[in] view The corresponding view
     * @param[in] pose The camera pose (nullptr if undefined)
     * @param[in] intrinsic The camera intrinsic (nullptr if undefined)
     * @param[in,out] parent The Alembic parent node
     */
    void addCamera(const std::string& name,
                   const sfmData::View& view,
                   const sfmData::CameraPose* pose = nullptr,
                   std::shared_ptr<camera::IntrinsicBase> intrinsic = nullptr,
                   const Vec6* uncertainty = nullptr,
                   Alembic::Abc::OObject* parent = nullptr);

    Alembic::Abc::OArchive _archive;
    Alembic::Abc::OObject _topObj;
    Alembic::AbcGeom::OXform _mvgRoot;
    Alembic::AbcGeom::OXform _mvgCameras;
    Alembic::AbcGeom::OXform _mvgCamerasUndefined;
    Alembic::AbcGeom::OXform _mvgCloud;
    Alembic::AbcGeom::OXform _mvgPointCloud;
    Alembic::AbcGeom::OXform _xform;
    Alembic::AbcGeom::OCamera _camObj;
    Alembic::AbcGeom::OUInt32ArrayProperty _propSensorSize_pix;
    Alembic::AbcGeom::OStringProperty _imagePlane;
    Alembic::AbcGeom::OUInt32Property _propViewId;
    Alembic::AbcGeom::OUInt32Property _propIntrinsicId;
    Alembic::AbcGeom::OStringProperty _mvgIntrinsicType;
    Alembic::AbcGeom::ODoubleArrayProperty _mvgIntrinsicParams;
};

void AlembicExporter::DataImpl::addCamera(const std::string& name,
                                          const sfmData::View& view,
                                          const sfmData::CameraPose* pose,
                                          std::shared_ptr<camera::IntrinsicBase> intrinsic,
                                          const Vec6* uncertainty,
                                          Alembic::Abc::OObject* parent)
{
    if (parent == nullptr)
        parent = &_mvgCameras;

    std::stringstream ssLabel;
    ssLabel << "camxform_" << std::setfill('0') << std::setw(5) << view.getResectionId() << "_" << view.getPoseId();
    ssLabel << "_" << name << "_" << view.getViewId();

    Alembic::AbcGeom::OXform xform(*parent, ssLabel.str());
    OCamera camObj(xform, "camera_" + ssLabel.str());

    auto userProps = camObj.getSchema().getUserProperties();

    XformSample xformsample;

    // set camera pose
    if (pose != nullptr)
    {
        OBoolProperty(userProps, "mvg_poseLocked").set(pose->isLocked());

        // Convert from computer vision convention to computer graphics (opengl-like)
        Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
        M(1, 1) = -1;
        M(2, 2) = -1;

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = pose->getTransform().rotation();
        T.block<3, 1>(0, 3) = pose->getTransform().translation();

        Eigen::Matrix4d T2 = (M * T * M).inverse();

        // compensate translation with rotation
        // build transform matrix
        Abc::M44d xformMatrix;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                xformMatrix[j][i] = T2(i, j);
            }
        }

        xformsample.setMatrix(xformMatrix);
    }

    xform.getSchema().set(xformsample);

    // set view custom properties
    if (!view.getImage().getImagePath().empty())
        OStringProperty(userProps, "mvg_imagePath").set(view.getImage().getImagePath());

    OUInt32Property(userProps, "mvg_viewId").set(view.getViewId());
    OUInt32Property(userProps, "mvg_poseId").set(view.getPoseId());
    OUInt32Property(userProps, "mvg_intrinsicId").set(view.getIntrinsicId());
    OUInt32Property(userProps, "mvg_resectionId").set(view.getResectionId());

    if (view.isPartOfRig())
    {
        OUInt32Property(userProps, "mvg_rigId").set(view.getRigId());
        OUInt32Property(userProps, "mvg_subPoseId").set(view.getSubPoseId());
    }

    if (view.getFrameId() != UndefinedIndexT)
        OUInt32Property(userProps, "mvg_frameId").set(view.getFrameId());

    if (view.isPoseIndependant() == false)
        OBoolProperty(userProps, "mvg_poseIndependant").set(view.isPoseIndependant());

    // set view metadata
    {
        std::vector<std::string> rawMetadata(view.getImage().getMetadata().size() * 2);
        auto it = view.getImage().getMetadata().cbegin();

        for (std::size_t i = 0; i < rawMetadata.size(); i += 2)
        {
            rawMetadata.at(i) = it->first;
            rawMetadata.at(i + 1) = it->second;
            std::advance(it, 1);
        }
        OStringArrayProperty(userProps, "mvg_metadata").set(rawMetadata);
    }

    // Export ancestors
    OUInt32ArrayProperty(userProps, "mvg_ancestorsParams").set(view.getAncestors());

    // Export ancestor Images
    std::vector<std::string> v_path;
    std::vector<unsigned int> v_width;
    std::vector<unsigned int> v_height;
    std::vector<unsigned int> v_metadataSize;
    std::vector<std::string> ancestorImagesRawMetadata;

    for (auto ancestorImage : view.getAncestorImages())
    {
        v_path.push_back(ancestorImage->getImagePath());
        v_width.push_back(ancestorImage->getWidth());
        v_height.push_back(ancestorImage->getHeight());
        v_metadataSize.push_back(ancestorImage->getMetadata().size());

        auto it = ancestorImage->getMetadata().cbegin();
        for (std::size_t i = 0; i < v_metadataSize.back(); i++)
        {
            ancestorImagesRawMetadata.push_back(it->first);
            ancestorImagesRawMetadata.push_back(it->second);

            std::advance(it, 1);
        }
    }

    OStringArrayProperty(userProps, "mvg_ancestorImagesPath").set(v_path);
    OUInt32ArrayProperty(userProps, "mvg_ancestorImagesWidth").set(v_width);
    OUInt32ArrayProperty(userProps, "mvg_ancestorImagesHeight").set(v_height);
    OUInt32ArrayProperty(userProps, "mvg_ancestorImagesMetadataSize").set(v_metadataSize);
    OStringArrayProperty(userProps, "mvg_ancestorImagesRawMetadata").set(ancestorImagesRawMetadata);

    // set intrinsic properties
    std::shared_ptr<camera::IntrinsicScaleOffsetDisto> intrinsicCasted = std::dynamic_pointer_cast<camera::IntrinsicScaleOffsetDisto>(intrinsic);
    if (intrinsicCasted)
    {
        CameraSample camSample;

        // Take the max of the image size to handle the case where the image is in portrait mode
        const float imgWidth = intrinsicCasted->w();
        const float imgHeight = intrinsicCasted->h();
        const float sensorWidth = intrinsicCasted->sensorWidth();
        const float sensorHeight = intrinsicCasted->sensorHeight();
        const float sensorWidth_pix = std::max(imgWidth, imgHeight);
        const float focalLengthX_pix = static_cast<const float>(intrinsicCasted->getScale()(0));
        const float focalLengthY_pix = static_cast<const float>(intrinsicCasted->getScale()(1));
        const float focalLength_mm = sensorWidth * focalLengthX_pix / sensorWidth_pix;
        const float squeeze = focalLengthX_pix / focalLengthY_pix;
        const float pix2mm = sensorWidth / sensorWidth_pix;

        // aliceVision: origin is (top,left) corner and orientation is (bottom,right)
        // ABC: origin is centered and orientation is (up,right)
        // Following values are in cm, hence the 0.1 multiplier
        const float haperture_cm = static_cast<const float>(0.1 * imgWidth * pix2mm);
        const float vaperture_cm = static_cast<const float>(0.1 * imgHeight * pix2mm);

        camSample.setFocalLength(focalLength_mm);
        camSample.setHorizontalAperture(haperture_cm);
        camSample.setVerticalAperture(vaperture_cm);
        camSample.setLensSqueezeRatio(squeeze);

        // Add sensor width (largest image side) in pixels as custom property
        std::vector<::uint32_t> sensorSize_pix = {intrinsicCasted->w(), intrinsicCasted->h()};
        std::vector<double> sensorSize_mm = {sensorWidth, sensorHeight};

        double initialFocalLength =
          (intrinsicCasted->getInitialScale().x() > 0) ? (intrinsicCasted->getInitialScale().x() * sensorWidth / double(intrinsicCasted->w())) : -1;

        OUInt32ArrayProperty(userProps, "mvg_sensorSizePix").set(sensorSize_pix);
        ODoubleArrayProperty(userProps, "mvg_sensorSizeMm").set(sensorSize_mm);
        OStringProperty(userProps, "mvg_intrinsicType").set(intrinsicCasted->getTypeStr());
        OStringProperty(userProps, "mvg_intrinsicInitializationMode").set(camera::EInitMode_enumToString(intrinsicCasted->getInitializationMode()));
        ODoubleProperty(userProps, "mvg_initialFocalLength").set(initialFocalLength);
        OBoolProperty(userProps, "mvg_intrinsicLocked").set(intrinsicCasted->isLocked());
        OBoolProperty(userProps, "mvg_intrinsicPixelRatioLocked").set(intrinsicCasted->isRatioLocked());
        OStringProperty(userProps, "mvg_intrinsicDistortionInitializationMode")
          .set(camera::EInitMode_enumToString(intrinsicCasted->getDistortionInitializationMode()));

        // Intrinsic parameters
        {
            Vec2 scale = intrinsicCasted->getScale();
            Vec2 offset = intrinsicCasted->getOffset();
            std::vector<double> params = {scale(0), scale(1), offset(0), offset(1)};
            ODoubleArrayProperty(userProps, "mvg_intrinsicParams").set(params);
        }
        // Distortion parameters
        std::shared_ptr<camera::Distortion> distortion = intrinsicCasted->getDistortion();
        if (distortion)
        {
            ODoubleArrayProperty(userProps, "mvg_distortionParams").set(distortion->getParameters());
        }
        // Undistortion parameters and offset
        std::shared_ptr<camera::Undistortion> undistortion = intrinsicCasted->getUndistortion();
        if (undistortion)
        {
            ODoubleArrayProperty(userProps, "mvg_undistortionParams").set(undistortion->getParameters());
            ODoubleProperty(userProps, "mvg_undistortionOffsetX").set(undistortion->getOffset().x());
            ODoubleProperty(userProps, "mvg_undistortionOffsetY").set(undistortion->getOffset().y());
        }

        camObj.getSchema().set(camSample);
    }

    std::shared_ptr<camera::Equidistant> intrinsicEquiCasted = std::dynamic_pointer_cast<camera::Equidistant>(intrinsic);
    if (intrinsicEquiCasted)
    {
        ODoubleProperty(userProps, "mvg_fisheyeCircleCenterX").set(intrinsicEquiCasted->getCircleCenterX());
        ODoubleProperty(userProps, "mvg_fisheyeCircleCenterY").set(intrinsicEquiCasted->getCircleCenterY());
        ODoubleProperty(userProps, "mvg_fisheyeCircleRadius").set(intrinsicEquiCasted->getCircleRadius());
    }

    if (uncertainty)
    {
        std::vector<double> uncertaintyParams(uncertainty->data(), uncertainty->data() + 6);
        ODoubleArrayProperty mvg_uncertaintyParams(userProps, "mvg_uncertaintyEigenValues");
        mvg_uncertaintyParams.set(uncertaintyParams);
    }

    if (pose == nullptr || intrinsicCasted == nullptr)
    {
        // hide camera
        Alembic::AbcGeom::CreateVisibilityProperty(xform, 0).set(Alembic::AbcGeom::kVisibilityHidden);
    }
}

AlembicExporter::AlembicExporter(const std::string& filename)
  : _dataImpl(new DataImpl(filename))
{}

AlembicExporter::~AlembicExporter() = default;

std::string AlembicExporter::getFilename() const { return _dataImpl->_archive.getName(); }

void AlembicExporter::addSfM(const sfmData::SfMData& sfmData, ESfMData flagsPart)
{
    OCompoundProperty userProps = _dataImpl->_mvgRoot.getProperties();

    OStringArrayProperty(userProps, "mvg_featuresFolders").set(sfmData.getRelativeFeaturesFolders());
    OStringArrayProperty(userProps, "mvg_matchesFolders").set(sfmData.getRelativeMatchesFolders());

    if (flagsPart & ESfMData::STRUCTURE)
    {
        const sfmData::LandmarksUncertainty noUncertainty;

        addLandmarks(sfmData.getLandmarks(),
                     (flagsPart & ESfMData::LANDMARKS_UNCERTAINTY) ? sfmData._landmarksUncertainty : noUncertainty,
                     ((flagsPart & ESfMData::OBSERVATIONS || flagsPart & ESfMData::OBSERVATIONS_WITH_FEATURES)),
                     (flagsPart & ESfMData::OBSERVATIONS_WITH_FEATURES));
    }

    if (flagsPart & ESfMData::VIEWS || flagsPart & ESfMData::EXTRINSICS)
    {
        std::map<IndexT, std::map<IndexT, std::vector<IndexT>>> rigsViewIds;  // map<rigId,map<poseId,viewId>>

        // save all single views
        for (const auto& viewPair : sfmData.getViews())
        {
            const sfmData::View& view = *(viewPair.second);

            if (view.isPartOfRig() && !view.isPoseIndependant())
            {
                // save rigId, poseId, viewId in a temporary structure, will process later
                rigsViewIds[view.getRigId()][view.getPoseId()].push_back(view.getViewId());
                continue;
            }
            addSfMSingleCamera(sfmData, view, flagsPart);
        }

        // save rigs views
        for (const auto& rigPair : rigsViewIds)
        {
            for (const auto& poseViewIds : rigPair.second)
                addSfMCameraRig(sfmData, rigPair.first, poseViewIds.second, flagsPart);  // add one camera rig per rig pose
        }
    }
}

void AlembicExporter::addSfMSingleCamera(const sfmData::SfMData& sfmData, const sfmData::View& view, ESfMData flagsPart)
{
    const std::string name = fs::path(view.getImage().getImagePath()).stem().string();
    const sfmData::CameraPose* pose =
      ((flagsPart & ESfMData::EXTRINSICS) && sfmData.existsPose(view)) ? &(sfmData.getPoses().at(view.getPoseId())) : nullptr;
    const std::shared_ptr<camera::IntrinsicBase> intrinsic =
      (flagsPart & ESfMData::INTRINSICS) ? sfmData.getIntrinsicsharedPtr(view.getIntrinsicId()) : nullptr;

    if (sfmData.isPoseAndIntrinsicDefined(&view) && (flagsPart & ESfMData::EXTRINSICS))
        _dataImpl->addCamera(name, view, pose, intrinsic, nullptr, &_dataImpl->_mvgCameras);
    else
        _dataImpl->addCamera(name, view, pose, intrinsic, nullptr, &_dataImpl->_mvgCamerasUndefined);
}

void AlembicExporter::addSfMCameraRig(const sfmData::SfMData& sfmData, IndexT rigId, const std::vector<IndexT>& viewIds, ESfMData flagsPart)
{
    const sfmData::Rig& rig = sfmData.getRigs().at(rigId);
    const std::size_t nbSubPoses = rig.getNbSubPoses();

    const sfmData::View& firstView = *(sfmData.getViews().at(viewIds.front()));

    XformSample xformsample;
    const IndexT rigPoseId = firstView.getPoseId();
    bool rigPoseLocked = false;

    if (sfmData.getPoses().find(rigPoseId) != sfmData.getPoses().end())
    {
        // rig pose
        const sfmData::CameraPose& rigPose = sfmData.getAbsolutePose(rigPoseId);
        const geometry::Pose3& rigTransform = rigPose.getTransform();
        rigPoseLocked = rigPose.isLocked();

        Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
        M(1, 1) = -1;
        M(2, 2) = -1;

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = rigPose.getTransform().rotation();
        T.block<3, 1>(0, 3) = rigPose.getTransform().translation();

        Eigen::Matrix4d T2 = (M * T * M).inverse();

        // compensate translation with rotation
        // build transform matrix
        Abc::M44d xformMatrix;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                xformMatrix[j][i] = T2(i, j);
            }
        }

        xformsample.setMatrix(xformMatrix);
    }

    std::stringstream ssLabel;
    ssLabel << "rigxform_" << std::setfill('0') << std::setw(5) << rigId << "_" << rigPoseId;

    std::map<bool, Alembic::AbcGeom::OXform> rigObj;
    for (const IndexT viewId : viewIds)
    {
        const sfmData::View& view = *(sfmData.getViews().at(viewId));
        const sfmData::RigSubPose& rigSubPose = rig.getSubPose(view.getSubPoseId());
        const bool isReconstructed = (rigSubPose.status != sfmData::ERigSubPoseStatus::UNINITIALIZED);
        const std::string name = fs::path(view.getImage().getImagePath()).stem().string();
        const std::shared_ptr<camera::IntrinsicBase> intrinsic =
          (flagsPart & ESfMData::INTRINSICS) ? sfmData.getIntrinsicsharedPtr(view.getIntrinsicId()) : nullptr;
        std::unique_ptr<sfmData::CameraPose> subPosePtr;

        if (isReconstructed && (flagsPart & ESfMData::EXTRINSICS))
        {
            subPosePtr = std::unique_ptr<sfmData::CameraPose>(new sfmData::CameraPose(rigSubPose.pose));
        }

        Alembic::Abc::OObject& parent = isReconstructed ? _dataImpl->_mvgCameras : _dataImpl->_mvgCamerasUndefined;

        if (rigObj.find(isReconstructed) == rigObj.end())
        {
            // The first time we declare a view, we have to create a RIG entry.
            // The RIG entry will be different if the view is reconstructed or not.
            rigObj[isReconstructed] = Alembic::AbcGeom::OXform(parent, ssLabel.str());
            auto schema = rigObj.at(isReconstructed).getSchema();
            schema.set(xformsample);
            {
                auto userProps = schema.getUserProperties();
                OUInt32Property(userProps, "mvg_rigId").set(rigId);
                OUInt32Property(userProps, "mvg_poseId").set(rigPoseId);
                OUInt16Property(userProps, "mvg_nbSubPoses").set(nbSubPoses);
                OBoolProperty(userProps, "mvg_rigPoseLocked").set(rigPoseLocked);
            }
        }
        _dataImpl->addCamera(name, view, subPosePtr.get(), intrinsic, nullptr, &(rigObj.at(isReconstructed)));
    }
}

void AlembicExporter::addLandmarks(const sfmData::Landmarks& landmarks,
                                   const sfmData::LandmarksUncertainty& landmarksUncertainty,
                                   bool withVisibility,
                                   bool withFeatures)
{
    if (landmarks.empty())
        return;

    // Fill vector with the values taken from AliceVision
    std::vector<V3f> positions;
    std::vector<Imath::C3f> colors;
    std::vector<Alembic::Util::uint32_t> descTypes;
    positions.reserve(landmarks.size());
    descTypes.reserve(landmarks.size());

    // For all the 3d points in the hash_map
    for (const auto& landmark : landmarks)
    {
        const Vec3& pt = landmark.second.X;
        const image::RGBColor& color = landmark.second.rgb;
        // convert position from computer vision convention to computer graphics (opengl-like)
        positions.emplace_back(pt[0], -pt[1], -pt[2]);
        colors.emplace_back(color.r() / 255.f, color.g() / 255.f, color.b() / 255.f);
        descTypes.emplace_back(static_cast<Alembic::Util::uint8_t>(landmark.second.descType));
    }

    std::vector<Alembic::Util::uint64_t> ids(positions.size());
    std::iota(begin(ids), end(ids), 0);

    OPoints partsOut(_dataImpl->_mvgPointCloud, "particleShape1");
    OPointsSchema& pSchema = partsOut.getSchema();

    OPointsSchema::Sample psamp(std::move(V3fArraySample(positions)), std::move(UInt64ArraySample(ids)));
    pSchema.set(psamp);

    OCompoundProperty arbGeom = pSchema.getArbGeomParams();

    C3fArraySample cval_samp(&colors[0], colors.size());
    OC3fGeomParam::Sample color_samp(cval_samp, kVertexScope);

    OC3fGeomParam rgbOut(arbGeom, "color", false, kVertexScope, 1);
    rgbOut.set(color_samp);

    OCompoundProperty userProps = pSchema.getUserProperties();

    OUInt32ArrayProperty(userProps, "mvg_describerType").set(descTypes);

    if (withVisibility)
    {
        std::vector<::uint32_t> visibilitySize;
        visibilitySize.reserve(positions.size());
        for (const auto& landmark : landmarks)
        {
            visibilitySize.emplace_back(landmark.second.observations.size());
        }
        std::size_t nbObservations = std::accumulate(visibilitySize.begin(), visibilitySize.end(), 0);

        // Use std::vector<::uint32_t> and std::vector<float> instead of std::vector<V2i> and std::vector<V2f>
        // Because Maya don't import them correctly
        std::vector<::uint32_t> visibilityViewId;
        std::vector<::uint32_t> visibilityFeatId;
        visibilityViewId.reserve(nbObservations);

        std::vector<float> featPos2d;
        std::vector<float> featScale;
        if (withFeatures)
        {
            featPos2d.reserve(nbObservations * 2);
            visibilityFeatId.reserve(nbObservations);
            featScale.reserve(nbObservations);
        }

        for (const auto& landmark : landmarks)
        {
            const sfmData::Observations& observations = landmark.second.observations;
            for (const auto& vObs : observations)
            {
                const sfmData::Observation& obs = vObs.second;

                // viewId
                visibilityViewId.emplace_back(vObs.first);

                if (withFeatures)
                {
                    // featureId
                    visibilityFeatId.emplace_back(obs.id_feat);

                    // feature 2D position (x, y))
                    featPos2d.emplace_back(obs.x[0]);
                    featPos2d.emplace_back(obs.x[1]);

                    featScale.emplace_back(obs.scale);
                }
            }
        }

        OUInt32ArrayProperty(userProps, "mvg_visibilitySize").set(visibilitySize);
        OUInt32ArrayProperty(userProps, "mvg_visibilityViewId").set(visibilityViewId);

        if (withFeatures)
        {
            OUInt32ArrayProperty(userProps, "mvg_visibilityFeatId").set(visibilityFeatId);
            OFloatArrayProperty(userProps, "mvg_visibilityFeatPos").set(featPos2d);  // feature position (x,y)
            OFloatArrayProperty(userProps, "mvg_visibilityFeatScale").set(featScale);
        }
    }
    if (!landmarksUncertainty.empty())
    {
        std::vector<V3d> uncertainties;

        std::size_t indexLandmark = 0;
        for (sfmData::Landmarks::const_iterator itLandmark = landmarks.begin(); itLandmark != landmarks.end(); ++itLandmark, ++indexLandmark)
        {
            const IndexT idLandmark = itLandmark->first;
            const Vec3& u = landmarksUncertainty.at(idLandmark);
            uncertainties.emplace_back(u[0], u[1], u[2]);
        }
        // Uncertainty eigen values (x,y,z)
        OV3dArrayProperty propUncertainty(userProps, "mvg_uncertaintyEigenValues");
        propUncertainty.set(uncertainties);
    }
}

void AlembicExporter::addCamera(const std::string& name,
                                const sfmData::View& view,
                                const sfmData::CameraPose* pose,
                                std::shared_ptr<camera::IntrinsicBase> intrinsic,
                                const Vec6* uncertainty)
{
    _dataImpl->addCamera(name, view, pose, intrinsic, uncertainty);
}

void AlembicExporter::initAnimatedCamera(const std::string& cameraName, std::size_t startFrame)
{
    // Sample the time in order to have one keyframe every frame
    // nb: it HAS TO be attached to EACH keyframed properties
    TimeSamplingPtr tsp(new TimeSampling(1.0 / 24.0, startFrame / 24.0));

    // Create the camera transform object
    std::stringstream ss;
    ss << cameraName;
    _dataImpl->_xform = Alembic::AbcGeom::OXform(_dataImpl->_mvgCameras, "animxform_" + ss.str());
    _dataImpl->_xform.getSchema().setTimeSampling(tsp);

    // Create the camera parameters object (intrinsics & custom properties)
    _dataImpl->_camObj = OCamera(_dataImpl->_xform, "animcam_" + ss.str());
    _dataImpl->_camObj.getSchema().setTimeSampling(tsp);

    // Add the custom properties
    auto userProps = _dataImpl->_camObj.getSchema().getUserProperties();
    // Sensor size
    _dataImpl->_propSensorSize_pix = OUInt32ArrayProperty(userProps, "mvg_sensorSizePix");
    _dataImpl->_propSensorSize_pix.setTimeSampling(tsp);
    // Image path
    _dataImpl->_imagePlane = OStringProperty(userProps, "mvg_imagePath");
    _dataImpl->_imagePlane.setTimeSampling(tsp);
    // View id
    _dataImpl->_propViewId = OUInt32Property(userProps, "mvg_viewId");
    _dataImpl->_propViewId.setTimeSampling(tsp);
    // Intrinsic id
    _dataImpl->_propIntrinsicId = OUInt32Property(userProps, "mvg_intrinsicId");
    _dataImpl->_propIntrinsicId.setTimeSampling(tsp);
    // Intrinsic type (ex: PINHOLE_CAMERA_RADIAL3)
    _dataImpl->_mvgIntrinsicType = OStringProperty(userProps, "mvg_intrinsicType");
    _dataImpl->_mvgIntrinsicType.setTimeSampling(tsp);
    // Intrinsic parameters
    _dataImpl->_mvgIntrinsicParams = ODoubleArrayProperty(userProps, "mvg_intrinsicParams");
    _dataImpl->_mvgIntrinsicParams.setTimeSampling(tsp);
}

void AlembicExporter::addCameraKeyframe(const geometry::Pose3& pose,
                                        const camera::Pinhole* cam,
                                        const std::string& imagePath,
                                        IndexT viewId,
                                        IndexT intrinsicId,
                                        float sensorWidthMM)
{
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M(1, 1) = -1;
    M(2, 2) = -1;

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = pose.rotation();
    T.block<3, 1>(0, 3) = pose.translation();

    Eigen::Matrix4d T2 = (M * T * M).inverse();

    // compensate translation with rotation
    // build transform matrix
    Abc::M44d xformMatrix;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            xformMatrix[j][i] = T2(i, j);
        }
    }

    // Create the XformSample
    XformSample xformsample;
    xformsample.setMatrix(xformMatrix);

    // Attach it to the schema of the OXform
    _dataImpl->_xform.getSchema().set(xformsample);

    // Camera intrinsic parameters
    CameraSample camSample;

    // Take the max of the image size to handle the case where the image is in portrait mode
    const float imgWidth = cam->w();
    const float imgHeight = cam->h();
    const float sensorWidth_pix = std::max(imgWidth, imgHeight);
    const float focalLength_pix = static_cast<const float>(cam->getScale()(0));
    const float focalLength_mm = sensorWidthMM * focalLength_pix / sensorWidth_pix;
    const float pix2mm = sensorWidthMM / sensorWidth_pix;

    // aliceVision: origin is (top,left) corner and orientation is (bottom,right)
    // ABC: origin is centered and orientation is (up,right)
    // Following values are in cm, hence the 0.1 multiplier
    const float haperture_cm = static_cast<const float>(0.1 * imgWidth * pix2mm);
    const float vaperture_cm = static_cast<const float>(0.1 * imgHeight * pix2mm);

    camSample.setFocalLength(focalLength_mm);
    camSample.setHorizontalAperture(haperture_cm);
    camSample.setVerticalAperture(vaperture_cm);

    // Add sensor size in pixels as custom property
    std::vector<::uint32_t> sensorSize_pix = {cam->w(), cam->h()};
    _dataImpl->_propSensorSize_pix.set(sensorSize_pix);

    // Set custom attributes
    // Image path
    _dataImpl->_imagePlane.set(imagePath);

    // View id
    _dataImpl->_propViewId.set(viewId);
    // Intrinsic id
    _dataImpl->_propIntrinsicId.set(intrinsicId);
    // Intrinsic type
    _dataImpl->_mvgIntrinsicType.set(cam->getTypeStr());
    // Intrinsic parameters
    std::vector<double> intrinsicParams = cam->getParams();
    _dataImpl->_mvgIntrinsicParams.set(intrinsicParams);

    // Attach intrinsic parameters to camera object
    _dataImpl->_camObj.getSchema().set(camSample);
}

void AlembicExporter::jumpKeyframe(const std::string& imagePath)
{
    if (_dataImpl->_xform.getSchema().getNumSamples() == 0)
    {
        camera::Pinhole default_intrinsic;
        this->addCameraKeyframe(geometry::Pose3(), &default_intrinsic, imagePath, 0, 0);
    }
    else
    {
        _dataImpl->_xform.getSchema().setFromPrevious();
        _dataImpl->_camObj.getSchema().setFromPrevious();
    }
}

}  // namespace sfmDataIO
}  // namespace aliceVision
