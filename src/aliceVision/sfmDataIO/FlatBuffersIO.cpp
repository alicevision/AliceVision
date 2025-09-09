// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.


#include <aliceVision/sfmDataIO/FlatBuffersIO.hpp>
#include <Root_generated.h>
#include <aliceVision/geometry/lie.hpp>
#include <aliceVision/sfmDataIO/MappedFile.hpp>
#include <fstream>
#include <optional>

#include <aliceVision/sfmDataIO/FlatBuffersIO/Eigen.hpp>
#include <aliceVision/sfmDataIO/FlatBuffersIO/View.hpp>
#include <aliceVision/sfmDataIO/FlatBuffersIO/Rig.hpp>
#include <aliceVision/sfmDataIO/FlatBuffersIO/CameraPose.hpp>
#include <aliceVision/sfmDataIO/FlatBuffersIO/Intrinsics.hpp>
#include <aliceVision/sfmDataIO/FlatBuffersIO/Landmark.hpp>
#include <aliceVision/sfmDataIO/FlatBuffersIO/SurveyPoint.hpp>
#include <aliceVision/sfmDataIO/FlatBuffersIO/Std.hpp>



namespace aliceVision {
namespace sfmDataIO {

template <class T>
const T * optionalPtr(const std::optional<T> & opt)
{
    if (opt.has_value())
    {
        return &opt.value();
    }

    return nullptr;
}

flatbuffers::Offset<AliceVisionIO::V1::SfmData> Pack(flatbuffers::FlatBufferBuilder & builder, const sfmData::SfMData & obj, ESfMData partFlag)
{
    using namespace AliceVisionIO;
    using namespace flatbuffers;

    Offset<V1::SfmData> ret;

    // save flags
    const bool saveViews = (partFlag & VIEWS) == VIEWS;
    const bool saveAncestors = (partFlag & ANCESTORS) == ANCESTORS;
    const bool saveIntrinsics = (partFlag & INTRINSICS) == INTRINSICS;
    const bool saveExtrinsics = (partFlag & EXTRINSICS) == EXTRINSICS;
    const bool saveStructure = (partFlag & STRUCTURE) == STRUCTURE;
    const bool saveSurveys = (partFlag & SURVEYS) == SURVEYS;
    const bool saveFeatures = (partFlag & OBSERVATIONS_WITH_FEATURES) == OBSERVATIONS_WITH_FEATURES;
    const bool saveObservations = saveFeatures || ((partFlag & OBSERVATIONS) == OBSERVATIONS);

    auto features = Pack<String>(builder, obj.getRelativeFeaturesFolders());
    auto matches = Pack<String>(builder, obj.getRelativeMatchesFolders());


    std::optional<std::vector<Offset<AliceVisionIO::V1::View>>> packedViews;
    std::optional<std::vector<Offset<AliceVisionIO::V1::Image>>> packedImages;
    std::optional<std::vector<Offset<AliceVisionIO::V1::CameraPose>>> packedPoses;
    std::optional<std::vector<Offset<AliceVisionIO::V1::Landmark>>> packedLandmarks;
    std::optional<std::vector<AliceVisionIO::V1::Observation>> packedObservations;
    std::optional<std::vector<Offset<AliceVisionIO::V1::Distortion>>> packedDistortions;
    std::optional<std::vector<Offset<AliceVisionIO::V1::Undistortion>>> packedUndistortions;
    std::optional<std::vector<Offset<AliceVisionIO::V1::Intrinsic>>> packedIntrinsics;
    std::optional<std::vector<Offset<AliceVisionIO::V1::Rig>>> packedRigs;
    std::optional<std::vector<Offset<AliceVisionIO::V1::SurveyPoint>>> packedSurveys;

    if (saveExtrinsics)
    {
        packedRigs = Pack<V1::Rig>(builder, obj.getRigs());
        packedPoses = Pack<V1::CameraPose>(builder, obj.getPoses());
    }

    if (saveIntrinsics)
    {
        packedIntrinsics = Pack<V1::Intrinsic>(builder, obj.getIntrinsics());
        packedDistortions = Pack<V1::Distortion, camera::Distortion>(builder, getDistortions(obj.getIntrinsics()));
        packedUndistortions = Pack<V1::Undistortion>(builder, getUndistortions(obj.getIntrinsics()));
    }

    if (saveStructure)
    {
        packedLandmarks = Pack<V1::Landmark>(builder, obj.getLandmarks());

        if (saveObservations)
        {
            packedObservations = Pack(builder, getObservations(obj.getLandmarks()));
        }
    }

    if (saveViews)
    {
        packedViews = Pack<V1::View>(builder, obj.getViews());
        packedImages = Pack<V1::Image>(builder, getImagesInfos(obj.getViews()));
    }

    if (saveSurveys)
    {
        packedSurveys = Pack<V1::SurveyPoint>(builder, obj.getSurveyPoints());
    }

    return V1::CreateSfmDataDirect(builder,
                                   &features,
                                   &matches,
                                   optionalPtr(packedViews),
                                   optionalPtr(packedImages),
                                   optionalPtr(packedPoses),
                                   optionalPtr(packedLandmarks),
                                   optionalPtr(packedObservations),
                                   optionalPtr(packedDistortions),
                                   optionalPtr(packedUndistortions),
                                   optionalPtr(packedIntrinsics),
                                   optionalPtr(packedRigs),
                                   optionalPtr(packedSurveys));
}

void Unpack(sfmData::SfMData & sfmData, const AliceVisionIO::V1::SfmData * packedSfmData, ESfMData partFlag)
{
    if (!packedSfmData)
    {
        return;
    }

    const bool loadViews = (partFlag & VIEWS) == VIEWS;
    const bool loadAncestors = (partFlag & ANCESTORS) == ANCESTORS;
    const bool loadIntrinsics = (partFlag & INTRINSICS) == INTRINSICS;
    const bool loadExtrinsics = (partFlag & EXTRINSICS) == EXTRINSICS;
    const bool loadStructure = (partFlag & STRUCTURE) == STRUCTURE;
    const bool loadSurveys = (partFlag & SURVEYS) == SURVEYS;
    const bool loadFeatures = (partFlag & OBSERVATIONS_WITH_FEATURES) == OBSERVATIONS_WITH_FEATURES;
    const bool loadObservations = loadFeatures || ((partFlag & OBSERVATIONS) == OBSERVATIONS);

    sfmData.setFeaturesFolders(Unpack(packedSfmData->features_folders()));
    sfmData.setMatchesFolders(Unpack(packedSfmData->matches_folders()));

    if (loadViews)
    {
        Unpack(sfmData.getViews(), packedSfmData->views());

        std::map<IndexT, sfmData::ImageInfo::sptr> imageInfos;
        Unpack(imageInfos, packedSfmData->images());

        /*Assign image to view*/
        for (auto & [idView, image] : imageInfos)
        {
            sfmData.getView(idView).setImageInfo(image);
        }
    }

    if (loadExtrinsics)
    {
        Unpack(sfmData.getPoses(), packedSfmData->poses());
        Unpack(sfmData.getRigs(), packedSfmData->rigs());
    }
    
    if (loadIntrinsics)
    {
        Unpack(sfmData.getIntrinsics(), packedSfmData->intrinsics());
        sfmData::SharedPtrMap<camera::Distortion> distortions;
        Unpack(distortions, packedSfmData->distortions());

        // Assign distortion to view
        for (auto & [idIntrinsic, distortion] : distortions)
        {
            auto intrinsic = sfmData.getIntrinsics().at(idIntrinsic);
            auto isod = std::dynamic_pointer_cast<camera::IntrinsicScaleOffsetDisto>(intrinsic);
            isod->setDistortionObject(distortion);
        }

        sfmData::SharedPtrMap<camera::Undistortion> undistortions;
        Unpack(undistortions, packedSfmData->undistortions());

        //Assign distortion to view
        for (auto & [idIntrinsic, undistortion] : undistortions)
        {
            auto intrinsic = sfmData.getIntrinsics().at(idIntrinsic);
            auto isod = std::dynamic_pointer_cast<camera::IntrinsicScaleOffsetDisto>(intrinsic);
            isod->setUndistortionObject(undistortion);
        }
    }
    
    if (loadStructure)
    {
        Unpack(sfmData.getLandmarks(), packedSfmData->landmarks());

        if (loadObservations)
        {            
            std::map<aliceVision::Pair, sfmData::Observation> observations;
            Unpack(observations, packedSfmData->observations());
    
            //Assign observations
            sfmData::Landmarks & landmarks = sfmData.getLandmarks();
            for (const auto & [pair, observationToAdd] : observations)
            {
                auto & observations = landmarks.at(pair.first).getObservations();
                observations.emplace(pair.second, observationToAdd);
            }

        }

        
    }

    if (loadSurveys)
    {
        Unpack(sfmData.getSurveyPoints(), packedSfmData->surveys());
    }
}

bool saveFlatBuffers(const sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag)
{
    //Open binary stream on disk
    std::ofstream fs(filename, std::ios::out | std::ios::binary);
    if (!fs.is_open())
    {
        ALICEVISION_LOG_ERROR("Unable to create the file " << filename);
        return false;
    }

    // Create the FlatBuffer
    flatbuffers::FlatBufferBuilder builder(1024);

    // Version
    AliceVisionIO::V1::Version version(ALICEVISION_VERSION_MAJOR,
                                     ALICEVISION_VERSION_MINOR,
                                     ALICEVISION_SFMDATAIO_VERSION_REVISION);

    // Pack the sfmData
    auto packedRoot = AliceVisionIO::V1::CreateRoot(builder, 
                                            &version,
                                            Pack(builder, sfmData, partFlag));

    builder.Finish(packedRoot, "ASFM");

    // Grab the buffers
    uint8_t* buf = builder.GetBufferPointer();
    int size = builder.GetSize();
    
    // Write the binary data
    fs.write(reinterpret_cast<const char*>(buf), size);
    fs.close();

    return true;
}

bool loadFlatBuffers(sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag)
{
    MemoryMappedFile input;

    if (!input.open(filename))
    {
        ALICEVISION_LOG_ERROR("Unable to open the file " << filename);
        return false;
    }

    if (!flatbuffers::BufferHasIdentifier(input.data<uint8_t>(), "ASFM"))
    {
        ALICEVISION_LOG_ERROR("Incorrect file header in " << filename);
        return false;
    }

    const AliceVisionIO::V1::Root * root = AliceVisionIO::V1::GetRoot(input.data());
    
    Unpack(sfmData, root->sfm_data(), partFlag);

    return true;
}

}
}