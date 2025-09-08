// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "cerealIO.hpp"

#include <iostream>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/vector.hpp>

template <class T>
struct ArchiveWithFlag : public T
{
    using Base = T;
    using T::operator();

    template <class... Args>
    ArchiveWithFlag(const aliceVision::sfmDataIO::ESfMData & params, Args &&... args) : parts(params), T(std::forward<Args>(args)...)
    {

    }

    bool hasPart(const aliceVision::sfmDataIO::ESfMData & value)
    {
        return (parts & value) == value;
    }

    aliceVision::sfmDataIO::ESfMData parts;
};

template <class T>
struct FilteredContainer
{
    FilteredContainer(T & value, aliceVision::sfmDataIO::ESfMData filter, bool forceLoad = true)
    :  toSave(value), filterValue(filter), forcedLoad(forceLoad)
    {

    }

    template<class Archive>
    void serialize(Archive& ar) 
    {

        bool hide = false;
        auto * ptr = dynamic_cast<ArchiveWithFlag<Archive>*>(&ar);
        if (ptr)
        {
            if (!ptr->hasPart(filterValue))
            {
                hide = true;
            }
        }

        if (hide)
        {
            if constexpr (Archive::is_loading::value) 
            {
                //Load to dummy
                if (forcedLoad)
                {
                    T buf;
                    ar(buf);
                }
            }
            else
            {
                ar(cereal::make_size_tag(0));
            }
            
            return;
        }

        ar(toSave);
    }

    bool forcedLoad;
    T & toSave;
    aliceVision::sfmDataIO::ESfMData filterValue;
};

namespace cereal
{

template<class Archive, class Key, class T>
void serialize(Archive& ar, std::pair<Key, T>& pair) {
    ar(pair.first, pair.second);
}

template<class Archive, typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
void serialize(Archive& ar, Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& matrix) {
    int rows = matrix.rows();
    int cols = matrix.cols();
    
    ar(cereal::make_nvp("rows", rows));
    ar(cereal::make_nvp("cols", cols));
    
    // Resize matrix if loading
    if (rows != matrix.rows() || cols != matrix.cols()) {
        matrix.resize(rows, cols);
    }
    
    // Serialize the data
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            ar(matrix(i, j));
        }
    }
}

// Serialize Eigen::Vector (specialization)
template<class Archive, typename Scalar, int Size>
void serialize(Archive& ar, Eigen::Matrix<Scalar, Size, 1>& vector) {
    int size = vector.size();
    ar(cereal::make_nvp("size", size));
    
    if (size != vector.size()) {
        vector.resize(size);
    }
    
    for (int i = 0; i < size; ++i) {
        ar(vector(i));
    }
}

template<class Archive>
void serialize(Archive& ar, Eigen::Vector2d& vec) {
    ar(cereal::make_nvp("x", vec.x()));
    ar(cereal::make_nvp("y", vec.y()));
}

template<class Archive>
void serialize(Archive & archive, aliceVision::sfmData::Observation & obs)
{ 
    if constexpr (Archive::is_loading::value) 
    {
        double scale;
        aliceVision::IndexT id;

        archive(scale, id, obs.getCoordinates());

        obs.setScale(scale);
        obs.setFeatureId(id);
    }
    else 
    {
        archive(obs.getScale(), obs.getFeatureId(), obs.getCoordinates()); 
    }
}

template<class Archive>
void serialize(Archive& ar, aliceVision::sfmData::Landmark & landmark) {
    ar(cereal::make_nvp("coordinates", landmark.X));
    ar(cereal::make_nvp("descType", landmark.descType));
    ar(cereal::make_nvp("color", landmark.rgb));
    ar(cereal::make_nvp("state", landmark.state));
    ar(cereal::make_nvp("observations", landmark.getObservations()));
}

template<class Archive>
void serialize(Archive& ar, aliceVision::geometry::Pose3 & pose) {
    ar(pose.getHomogeneousMutable());
}

template<class Archive>
void serialize(Archive& ar, aliceVision::sfmData::CameraPose & cp) {

    if constexpr (Archive::is_loading::value) 
    {
        bool isRotationOnly, isLocked, isRemovable;
        aliceVision::EEstimatorParameterState state;
        aliceVision::geometry::Pose3 pose;

        ar(cereal::make_nvp("isRotationOnly", isRotationOnly));
        ar(cereal::make_nvp("isLocked", isLocked));
        ar(cereal::make_nvp("isRemovable", isRemovable));
        ar(cereal::make_nvp("state", state));
        ar(cereal::make_nvp("transform", pose));

        cp.setRotationOnly(isRotationOnly);
        cp.setRemovable(isRemovable);
        if (isLocked)
        {
            cp.lock();
        }
        else 
        {
            cp.unlock();
        }
    
        cp.setState(state);
        cp.setTransform(pose);
    }
    else 
    {
        ar(cereal::make_nvp("isRotationOnly", cp.isRotationOnly()));
        ar(cereal::make_nvp("isLocked", cp.isLocked()));
        ar(cereal::make_nvp("isRemovable", cp.isRemovable()));
        ar(cereal::make_nvp("state", cp.getState()));
        ar(cereal::make_nvp("transform", cp.getTransform()));
    }
}


template<class Archive>
void serialize(Archive& ar, aliceVision::sfmData::View & view) 
{
    if constexpr (Archive::is_loading::value) 
    {
        aliceVision::IndexT viewId, intrinsicId, poseId, rigId, subPoseId, frameId, resectionId;
        bool isPoseIndependant;
        std::shared_ptr<aliceVision::sfmData::ImageInfo> imageInfo;
        std::vector<aliceVision::IndexT> ancestors;

        ar(cereal::make_nvp("viewId", viewId));
        ar(cereal::make_nvp("intrinsicId", intrinsicId));
        ar(cereal::make_nvp("poseId", poseId));
        ar(cereal::make_nvp("rigId", rigId));
        ar(cereal::make_nvp("subPoseId", subPoseId));
        ar(cereal::make_nvp("frameId", frameId));
        ar(cereal::make_nvp("resectionId", resectionId));
        ar(cereal::make_nvp("isPoseIndependant", isPoseIndependant));
        ar(cereal::make_nvp("imageInfo", imageInfo));
        ar(cereal::make_nvp("ancestors", ancestors));

        view.setViewId(viewId);
        view.setIntrinsicId(intrinsicId);
        view.setPoseId(poseId);
        view.setRigAndSubPoseId(rigId, subPoseId);
        view.setFrameId(frameId);
        view.setResectionId(resectionId);
        view.setIndependantPose(isPoseIndependant);
        view.setImageInfo(imageInfo);

        for (const auto item : ancestors) 
        {
            view.addAncestor(item);
        }
    }
    else 
    {
        ar(cereal::make_nvp("viewId", view.getViewId()));
        ar(cereal::make_nvp("intrinsicId", view.getIntrinsicId()));
        ar(cereal::make_nvp("poseId", view.getPoseId()));
        ar(cereal::make_nvp("rigId", view.getRigId()));
        ar(cereal::make_nvp("subPoseId", view.getSubPoseId()));
        ar(cereal::make_nvp("frameId", view.getFrameId()));
        ar(cereal::make_nvp("resectionId", view.getResectionId()));
        ar(cereal::make_nvp("isPoseIndependant", view.isPoseIndependant()));
        ar(cereal::make_nvp("imageInfo", view.getImageInfo()));
        ar(cereal::make_nvp("ancestors", view.getAncestors()));
    }
}

template<class Archive>
void serialize(Archive& ar, aliceVision::sfmData::ImageInfo & imageInfo) 
{
    if constexpr (Archive::is_loading::value) 
    {
        std::string path;
        std::size_t width;
        std::size_t height;
        std::map<std::string, std::string> metadata;

        ar(cereal::make_nvp("imagePath", path));
        ar(cereal::make_nvp("width", width));
        ar(cereal::make_nvp("height", height));
        ar(cereal::make_nvp("metadata", metadata));

        imageInfo.setImagePath(path);
        imageInfo.setWidth(width);
        imageInfo.setHeight(height);
        imageInfo.setMetadata(metadata);
    }
    else 
    {
        ar(cereal::make_nvp("imagePath", imageInfo.getImagePath()));
        ar(cereal::make_nvp("width", imageInfo.getWidth()));
        ar(cereal::make_nvp("height", imageInfo.getHeight()));
        ar(cereal::make_nvp("metadata", imageInfo.getMetadata()));
    }
}



template<class Archive>
void serialize(Archive& ar, aliceVision::sfmData::RigSubPose & rsp) {
    ar(cereal::make_nvp("status", rsp.status));
    ar(cereal::make_nvp("pose", rsp.pose));
}

template<class Archive>
void serialize(Archive& ar, aliceVision::sfmData::Rig & rig) {
    ar(cereal::make_nvp("subposes", rig.getSubPoses()));
}

template<class Archive>
void serialize(Archive& ar, aliceVision::sfmData::Constraint2D & constraint) {
    ar(cereal::make_nvp("descType", constraint.descType));
    ar(cereal::make_nvp("viewFirst", constraint.ViewFirst));
    ar(cereal::make_nvp("viewSecond", constraint.ViewSecond));
    ar(cereal::make_nvp("observationFirst", constraint.ObservationFirst));
    ar(cereal::make_nvp("observationSecond", constraint.ObservationSecond));
}

template<class Archive>
void serialize(Archive& ar, aliceVision::sfmData::ConstraintPoint & constraint) {
    ar(cereal::make_nvp("landmarkId", constraint.landmarkId));
    ar(cereal::make_nvp("normal", constraint.normal));
    ar(cereal::make_nvp("point", constraint.point));
}

template<class Archive>
void serialize(Archive& ar, aliceVision::sfmData::RotationPrior & prior) {
    ar(cereal::make_nvp("viewFirst", prior.ViewFirst));
    ar(cereal::make_nvp("viewSecond", prior.ViewSecond));
    ar(cereal::make_nvp("second_R_first", prior._second_R_first));
}

template<class Archive>
void serialize(Archive& ar, aliceVision::sfmData::SurveyPoint & spoint) {
    ar(cereal::make_nvp("point3d", spoint.point3d));
    ar(cereal::make_nvp("survey", spoint.survey));
}


template<class Archive>
void serialize(Archive& ar, aliceVision::camera::Distortion & distortion) 
{
    if constexpr (Archive::is_loading::value) 
    {
        std::vector<double> distortionParams;
        bool isLocked;

        ar(cereal::make_nvp("params", distortionParams));
        ar(cereal::make_nvp("locked", isLocked));

        distortion.setParameters(distortionParams);
        distortion.setLocked(isLocked);
    }
    else 
    {
        ar(cereal::make_nvp("params", distortion.getParameters()));
        ar(cereal::make_nvp("locked", distortion.isLocked()));
    }
}

template<class Archive>
void serialize(Archive& ar, aliceVision::camera::Undistortion & undistortion) 
{
    if constexpr (Archive::is_loading::value) 
    {
        int width, height;
        double pixelAspectRatio;
        bool isDesqueezed;
        std::vector<double> undistortionParams;
        bool isLocked;

        ar(cereal::make_nvp("width", width));
        ar(cereal::make_nvp("height", height));
        ar(cereal::make_nvp("pixelAspectRation", pixelAspectRatio));
        ar(cereal::make_nvp("isDesqueezed", isDesqueezed));
        ar(cereal::make_nvp("params", undistortionParams));
        ar(cereal::make_nvp("locked", isLocked));

        undistortion.setSize(width, height);
        undistortion.setPixelAspectRatio(pixelAspectRatio);
        undistortion.setDesqueezed(isDesqueezed);
        undistortion.setParameters(undistortionParams);
        undistortion.setLocked(isLocked);
    }
    else 
    {
        ar(cereal::make_nvp("width", undistortion.getSize().x()));
        ar(cereal::make_nvp("height", undistortion.getSize().y()));
        ar(cereal::make_nvp("pixelAspectRation", undistortion.getPixelAspectRatio()));
        ar(cereal::make_nvp("isDesqueezed", undistortion.isDesqueezed()));
        ar(cereal::make_nvp("params", undistortion.getParameters()));
        ar(cereal::make_nvp("locked", undistortion.isLocked()));
    }
}


template<class Archive>
void serialize(Archive& ar, aliceVision::camera::IntrinsicBase & camera) 
{
    if constexpr (Archive::is_loading::value) 
    {
        aliceVision::camera::EInitMode initializationMode;
        bool locked;
        aliceVision::EEstimatorParameterState state;
        unsigned int width, height;
        double sensorWidth, sensorHeight;
        std::string serialNumber;

        ar(cereal::make_nvp("initializationMode", initializationMode));
        ar(cereal::make_nvp("locked", locked));
        ar(cereal::make_nvp("width", width));
        ar(cereal::make_nvp("height", height));
        ar(cereal::make_nvp("sensorWidth", sensorWidth));
        ar(cereal::make_nvp("sensorHeight", sensorHeight));
        ar(cereal::make_nvp("serialNumber", serialNumber));

        camera.setInitializationMode(initializationMode);
        camera.setWidth(width);
        camera.setHeight(height);
        camera.setSensorWidth(sensorWidth);
        camera.setSensorHeight(sensorHeight);
        camera.setSerialNumber(serialNumber);
        
        if (locked)
        {
            camera.lock();
        }
        else 
        {
            camera.unlock();
        }
    }
    else 
    {
        ar(cereal::make_nvp("initializationMode", camera.getInitializationMode()));
        ar(cereal::make_nvp("locked", camera.isLocked()));
        ar(cereal::make_nvp("width", camera.w()));
        ar(cereal::make_nvp("height", camera.h()));
        ar(cereal::make_nvp("sensorWidth", camera.sensorWidth()));
        ar(cereal::make_nvp("sensorHeight", camera.sensorHeight()));
        ar(cereal::make_nvp("serialNumber", camera.serialNumber()));
    }
}

template<class Archive>
void serialize(Archive& ar, aliceVision::camera::IntrinsicScaleOffset & camera) 
{
    ar(cereal::base_class<aliceVision::camera::IntrinsicBase>(&camera));

    if constexpr (Archive::is_loading::value) 
    {
        aliceVision::Vec2 scale, initialScale, offset;
        bool ratioLocked, offsetLocked, scaleLocked;

        ar(cereal::make_nvp("scale", scale));
        ar(cereal::make_nvp("initialScale", initialScale));
        ar(cereal::make_nvp("offset", offset));
        ar(cereal::make_nvp("ratioLocked", ratioLocked));
        ar(cereal::make_nvp("offsetLocked", offsetLocked));
        ar(cereal::make_nvp("scaleLocked", scaleLocked));

        camera.setScale(scale);
        camera.setInitialScale(initialScale);
        camera.setOffset(offset);
        camera.setRatioLocked(ratioLocked);
        camera.setOffsetLocked(offsetLocked);
        camera.setScaleLocked(scaleLocked);
    }
    else 
    {
        ar(cereal::make_nvp("scale", camera.getScale()));
        ar(cereal::make_nvp("initialScale", camera.getInitialScale()));
        ar(cereal::make_nvp("offset", camera.getOffset()));
        ar(cereal::make_nvp("ratioLocked", camera.isRatioLocked()));
        ar(cereal::make_nvp("offsetLocked", camera.isOffsetLocked()));
        ar(cereal::make_nvp("scaleLocked", camera.isScaleLocked()));
    }
    
}

template<class Archive>
void serialize(Archive& ar, aliceVision::camera::IntrinsicScaleOffsetDisto & camera) 
{
    ar(cereal::base_class<aliceVision::camera::IntrinsicScaleOffset>(&camera));

    if constexpr (Archive::is_loading::value) 
    {
        aliceVision::camera::EInitMode mode;

        ar(cereal::make_nvp("distortionInitializationMode", mode));
        ar(cereal::make_nvp("distortion", camera.getDistortion()));
        ar(cereal::make_nvp("undistortion", camera.getUndistortion()));

        camera.setDistortionInitializationMode(mode);
    }
    else 
    {
        ar(cereal::make_nvp("distortionInitializationMode", camera.getDistortionInitializationMode()));
        ar(cereal::make_nvp("distortion", camera.getDistortion()));
        ar(cereal::make_nvp("undistortion", camera.getUndistortion()));
    }
}

template<class Archive>
void serialize(Archive& ar, aliceVision::camera::Pinhole & camera) 
{
    ar(cereal::base_class<aliceVision::camera::IntrinsicScaleOffsetDisto>(&camera));
}

template<class Archive>
void serialize(Archive& ar, aliceVision::camera::Equidistant & camera) 
{
    ar(cereal::base_class<aliceVision::camera::IntrinsicScaleOffsetDisto>(&camera));
}

template<class Archive>
void serialize(Archive& ar, aliceVision::camera::Equirectangular & camera) 
{
    ar(cereal::base_class<aliceVision::camera::IntrinsicScaleOffsetDisto>(&camera));
}

}

namespace aliceVision {
namespace sfmDataIO {

bool saveCerealJSON(const sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag)
{
    try 
    {
        // Create directory if it doesn't exist
        std::filesystem::path path(filename);
        if (path.has_parent_path()) 
        {
            std::filesystem::create_directories(path.parent_path());
        }
        
        std::ofstream os(filename);
        if (!os.is_open()) 
        {
            throw std::runtime_error("Cannot open file: " + filename);
        }
        
        
        ArchiveWithFlag<cereal::JSONOutputArchive> oarchive(partFlag, os, cereal::JSONOutputArchive::Options::Default());
        oarchive(cereal::make_nvp("landmarks", FilteredContainer(sfmData.getLandmarks(), sfmDataIO::STRUCTURE)));
        oarchive(cereal::make_nvp("intrinsics", FilteredContainer(sfmData.getIntrinsics(), sfmDataIO::INTRINSICS)));
        oarchive(cereal::make_nvp("views", FilteredContainer(sfmData.getViews(), sfmDataIO::VIEWS)));
        oarchive(cereal::make_nvp("ancestors", FilteredContainer(sfmData.getAncestors(), sfmDataIO::ANCESTORS)));
        oarchive(cereal::make_nvp("poses", FilteredContainer(sfmData.getPoses(), sfmDataIO::EXTRINSICS)));
        oarchive(cereal::make_nvp("rigs", FilteredContainer(sfmData.getRigs(), sfmDataIO::EXTRINSICS)));
        oarchive(cereal::make_nvp("surveyPoints", FilteredContainer(sfmData.getSurveyPoints(), sfmDataIO::SURVEYS)));
        oarchive(cereal::make_nvp("featuresFolders", sfmData.getFeaturesFolders()));
        oarchive(cereal::make_nvp("matchesFolders", sfmData.getMatchesFolders()));
    }
    catch(const cereal::Exception & e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
    catch (const std::exception& e) 
    {
        return false;
    }

    return true;
}

bool loadCerealJSON(sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag)
{    
    try 
    {
        std::ifstream is(filename);
        if (!is.is_open()) 
        {
            throw std::runtime_error("Cannot open file: " + filename);
        }
        
        ArchiveWithFlag<cereal::JSONInputArchive> iarchive(partFlag, is);
        
        iarchive(cereal::make_nvp("landmarks", FilteredContainer(sfmData.getLandmarks(), sfmDataIO::STRUCTURE)));
        iarchive(cereal::make_nvp("intrinsics", FilteredContainer(sfmData.getIntrinsics(), sfmDataIO::INTRINSICS)));
        iarchive(cereal::make_nvp("views", FilteredContainer(sfmData.getViews(), sfmDataIO::VIEWS)));
        iarchive(cereal::make_nvp("ancestors", FilteredContainer(sfmData.getAncestors(), sfmDataIO::ANCESTORS)));
        iarchive(cereal::make_nvp("poses", FilteredContainer(sfmData.getPoses(), sfmDataIO::EXTRINSICS)));
        iarchive(cereal::make_nvp("rigs", FilteredContainer(sfmData.getRigs(), sfmDataIO::EXTRINSICS)));
        iarchive(cereal::make_nvp("surveyPoints", FilteredContainer(sfmData.getSurveyPoints(), sfmDataIO::SURVEYS)));
        iarchive(cereal::make_nvp("featuresFolders", sfmData.getFeaturesFolders()));
        iarchive(cereal::make_nvp("matchesFolders", sfmData.getMatchesFolders()));
    }
    catch(const cereal::Exception & e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
    catch (const std::exception& e) 
    {
        return false;
    }

    return true;
}

bool saveCerealBinary(const sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag)
{
    try 
    {
        // Create directory if it doesn't exist
        std::filesystem::path path(filename);
        if (path.has_parent_path()) 
        {
            std::filesystem::create_directories(path.parent_path());
        }
        
        std::ofstream os(filename, std::ios::binary);
        if (!os.is_open()) 
        {
            throw std::runtime_error("Cannot open file: " + filename);
        }
        
        
        ArchiveWithFlag<cereal::PortableBinaryOutputArchive> oarchive(partFlag, os);
        
        oarchive(cereal::make_nvp("landmarks", FilteredContainer(sfmData.getLandmarks(), sfmDataIO::STRUCTURE)));
        oarchive(cereal::make_nvp("intrinsics", FilteredContainer(sfmData.getIntrinsics(), sfmDataIO::INTRINSICS)));
        oarchive(cereal::make_nvp("views", FilteredContainer(sfmData.getViews(), sfmDataIO::VIEWS)));
        oarchive(cereal::make_nvp("ancestors", FilteredContainer(sfmData.getAncestors(), sfmDataIO::ANCESTORS)));
        oarchive(cereal::make_nvp("poses", FilteredContainer(sfmData.getPoses(), sfmDataIO::EXTRINSICS)));
        oarchive(cereal::make_nvp("rigs", FilteredContainer(sfmData.getRigs(), sfmDataIO::EXTRINSICS)));
        oarchive(cereal::make_nvp("surveyPoints", FilteredContainer(sfmData.getSurveyPoints(), sfmDataIO::SURVEYS)));
        oarchive(cereal::make_nvp("featuresFolders", sfmData.getFeaturesFolders()));
        oarchive(cereal::make_nvp("matchesFolders", sfmData.getMatchesFolders()));
    }
    catch(const cereal::Exception & e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
    catch (const std::exception& e) 
    {
        return false;
    }

    return true;
}

bool loadCerealBinary(sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag)
{    
    try 
    {
        std::ifstream is(filename, std::ios::binary);
        if (!is.is_open()) 
        {
            throw std::runtime_error("Cannot open file: " + filename);
        }
        
        ArchiveWithFlag<cereal::PortableBinaryInputArchive> iarchive(partFlag, is);

        iarchive(cereal::make_nvp("landmarks", FilteredContainer(sfmData.getLandmarks(), sfmDataIO::STRUCTURE)));
        iarchive(cereal::make_nvp("intrinsics", FilteredContainer(sfmData.getIntrinsics(), sfmDataIO::INTRINSICS)));
        iarchive(cereal::make_nvp("views", FilteredContainer(sfmData.getViews(), sfmDataIO::VIEWS)));
        iarchive(cereal::make_nvp("ancestors", FilteredContainer(sfmData.getAncestors(), sfmDataIO::ANCESTORS)));
        iarchive(cereal::make_nvp("poses", FilteredContainer(sfmData.getPoses(), sfmDataIO::EXTRINSICS)));
        iarchive(cereal::make_nvp("rigs", FilteredContainer(sfmData.getRigs(), sfmDataIO::EXTRINSICS)));
        iarchive(cereal::make_nvp("surveyPoints", FilteredContainer(sfmData.getSurveyPoints(), sfmDataIO::SURVEYS)));
        iarchive(cereal::make_nvp("featuresFolders", sfmData.getFeaturesFolders()));
        iarchive(cereal::make_nvp("matchesFolders", sfmData.getMatchesFolders()));
    }
    catch(const cereal::Exception & e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
    catch (const std::exception& e) 
    {
        return false;
    }

    return true;
}

}  // namespace sfmDataIO
}  // namespace aliceVision



CEREAL_REGISTER_TYPE(aliceVision::camera::IntrinsicBase)
CEREAL_REGISTER_TYPE(aliceVision::camera::IntrinsicScaleOffset)
CEREAL_REGISTER_TYPE(aliceVision::camera::IntrinsicScaleOffsetDisto)
CEREAL_REGISTER_TYPE(aliceVision::camera::Pinhole)
CEREAL_REGISTER_TYPE(aliceVision::camera::Equirectangular)
CEREAL_REGISTER_TYPE(aliceVision::camera::Equidistant)

CEREAL_REGISTER_POLYMORPHIC_RELATION(aliceVision::camera::IntrinsicBase, aliceVision::camera::IntrinsicScaleOffset)
CEREAL_REGISTER_POLYMORPHIC_RELATION(aliceVision::camera::IntrinsicScaleOffset, aliceVision::camera::IntrinsicScaleOffsetDisto)
CEREAL_REGISTER_POLYMORPHIC_RELATION(aliceVision::camera::IntrinsicScaleOffsetDisto, aliceVision::camera::Pinhole)
CEREAL_REGISTER_POLYMORPHIC_RELATION(aliceVision::camera::IntrinsicScaleOffsetDisto, aliceVision::camera::Equirectangular)
CEREAL_REGISTER_POLYMORPHIC_RELATION(aliceVision::camera::IntrinsicScaleOffsetDisto, aliceVision::camera::Equidistant)

CEREAL_REGISTER_DYNAMIC_INIT(cerealIO)
