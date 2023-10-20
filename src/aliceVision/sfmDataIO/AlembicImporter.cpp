// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "AlembicImporter.hpp"

#include <Alembic/AbcGeom/All.h>
#include <Alembic/AbcCoreFactory/All.h>
#include <Alembic/AbcCoreOgawa/All.h>

#include <aliceVision/version.hpp>

namespace aliceVision {
namespace sfmDataIO {

using namespace Alembic::Abc;
using namespace Alembic::AbcGeom;

template<class AbcArrayProperty, typename T>
void getAbcArrayProp(ICompoundProperty& userProps, const std::string& id, index_t sampleFrame, T& outputArray)
{
    typedef typename AbcArrayProperty::sample_ptr_type sample_ptr_type;

    AbcArrayProperty prop(userProps, id);
    sample_ptr_type sample;
    prop.get(sample, ISampleSelector(sampleFrame));
    outputArray.assign(sample->get(), sample->get() + sample->size());
}

void getAbcArrayProp_uint(ICompoundProperty& userProps, const std::string& id, index_t sampleFrame, std::vector<unsigned int>& outputArray)
{
    try
    {
        getAbcArrayProp<Alembic::Abc::IUInt32ArrayProperty>(userProps, id, sampleFrame, outputArray);
    }
    catch (Alembic::Util::Exception&)
    {
        getAbcArrayProp<Alembic::Abc::IInt32ArrayProperty>(userProps, id, sampleFrame, outputArray);
    }
}

/**
 * @brief Retrieve an Abc property.
 *         Maya convert everything into arrays. So here we made a trick
 *         to retrieve the element directly or the first element
 *         if it's an array.
 * @param userProps
 * @param id
 * @param sampleFrame
 * @return value
 */
template<class AbcProperty>
typename AbcProperty::traits_type::value_type getAbcProp(ICompoundProperty& userProps,
                                                         const Alembic::Abc::PropertyHeader& propHeader,
                                                         const std::string& id,
                                                         index_t sampleFrame)
{
    typedef typename AbcProperty::traits_type traits_type;
    typedef typename traits_type::value_type value_type;
    typedef typename Alembic::Abc::ITypedArrayProperty<traits_type> array_type;
    typedef typename array_type::sample_ptr_type array_sample_ptr_type;

    // Maya transforms everything into arrays
    if (propHeader.isArray())
    {
        Alembic::Abc::ITypedArrayProperty<traits_type> prop(userProps, id);
        array_sample_ptr_type sample;
        prop.get(sample, ISampleSelector(sampleFrame));
        return (*sample)[0];
    }
    else
    {
        value_type v;
        AbcProperty prop(userProps, id);
        prop.get(v, ISampleSelector(sampleFrame));
        return v;
    }
}

std::size_t getAbcProp_uint(ICompoundProperty& userProps, const Alembic::Abc::PropertyHeader& propHeader, const std::string& id, index_t sampleFrame)
{
    try
    {
        return getAbcProp<Alembic::Abc::IUInt32Property>(userProps, propHeader, id, sampleFrame);
    }
    catch (Alembic::Util::Exception&)
    {
        return getAbcProp<Alembic::Abc::IInt32Property>(userProps, propHeader, id, sampleFrame);
    }
}
template<class ABCSCHEMA>
inline ICompoundProperty getAbcUserProperties(ABCSCHEMA& schema)
{
    ICompoundProperty userProps = schema.getUserProperties();
    if (userProps && userProps.getNumProperties() != 0)
        return userProps;

    // Maya always use ArbGeomParams instead of user properties.
    return schema.getArbGeomParams();
}

struct AV_UInt32ArraySamplePtr
{
    bool _isUnsigned = true;
    UInt32ArraySamplePtr _v_uint;
    Int32ArraySamplePtr _v_int;

    AV_UInt32ArraySamplePtr() = default;
    template<typename T>
    AV_UInt32ArraySamplePtr(const T& userProps, const char* s)
    {
        read(userProps, s);
    }

    template<typename T>
    void read(const T& userProps, const char* s)
    {
        try
        {
            IUInt32ArrayProperty propVisibilitySize_uint(userProps, s);
            propVisibilitySize_uint.get(_v_uint);
        }
        catch (Alembic::Util::Exception&)
        {
            _isUnsigned = false;
            IInt32ArrayProperty propVisibilitySize_int(userProps, s);
            propVisibilitySize_int.get(_v_int);
        }
    }

    std::size_t size() const { return _isUnsigned ? _v_uint->size() : _v_int->size(); }

    std::size_t operator[](const std::size_t& i) { return _isUnsigned ? (*_v_uint)[i] : (*_v_int)[i]; }

    void reset()
    {
        if (_isUnsigned)
        {
            _v_uint->reset();
        }
        else
        {
            _v_int->reset();
        }
    }
    operator bool() const { return _isUnsigned ? bool(_v_uint) : bool(_v_int); }
};

bool readPointCloud(const Version& abcVersion, IObject iObj, M44d mat, sfmData::SfMData& sfmdata, ESfMData flags_part)
{
    using namespace aliceVision::geometry;

    IPoints points(iObj, kWrapExisting);
    IPointsSchema& ms = points.getSchema();
    P3fArraySamplePtr positions = ms.getValue().getPositions();

    ICompoundProperty userProps = getAbcUserProperties(ms);
    ICompoundProperty arbGeom = ms.getArbGeomParams();

    C3fArraySamplePtr sampleColors;
    if (arbGeom && arbGeom.getPropertyHeader("color"))
    {
        IC3fArrayProperty propColor(arbGeom, "color");
        propColor.get(sampleColors);
        if (sampleColors->size() != positions->size())
        {
            ALICEVISION_LOG_WARNING("[Alembic Importer] Colors will be ignored. Color vector size: "
                                    << sampleColors->size() << ", positions vector size: " << positions->size());
            sampleColors.reset();
        }
    }

    AV_UInt32ArraySamplePtr sampleDescs;
    if (userProps && userProps.getPropertyHeader("mvg_describerType"))
    {
        sampleDescs.read(userProps, "mvg_describerType");
        if (sampleDescs.size() != positions->size())
        {
            ALICEVISION_LOG_WARNING("[Alembic Importer] Describer type will be ignored. describerType vector size: "
                                    << sampleDescs.size() << ", positions vector size: " << positions->size());
            sampleDescs.reset();
        }
    }

    // Number of points before adding the Alembic data
    const std::size_t nbPointsInit = sfmdata.getLandmarks().size();
    for (std::size_t point3d_i = 0; point3d_i < positions->size(); ++point3d_i)
    {
        const P3fArraySamplePtr::element_type::value_type& pos_i = positions->get()[point3d_i];

        sfmData::Landmark& landmark = sfmdata.getLandmarks()[nbPointsInit + point3d_i];

        if (abcVersion < Version(1, 2, 3))
        {
            landmark = sfmData::Landmark(Vec3(pos_i.x, pos_i.y, pos_i.z), feature::EImageDescriberType::UNKNOWN);
        }
        else
        {
            // convert from computer graphics convention (opengl-like) to computer vision
            landmark = sfmData::Landmark(Vec3(pos_i.x, -pos_i.y, -pos_i.z), feature::EImageDescriberType::UNKNOWN);
        }

        if (sampleColors)
        {
            const P3fArraySamplePtr::element_type::value_type& color_i = sampleColors->get()[point3d_i];
            landmark.rgb = image::RGBColor(static_cast<unsigned char>(color_i[0] * 255.0f),
                                           static_cast<unsigned char>(color_i[1] * 255.0f),
                                           static_cast<unsigned char>(color_i[2] * 255.0f));
        }

        if (sampleDescs)
        {
            const std::size_t descType_i = sampleDescs[point3d_i];
            landmark.descType = static_cast<feature::EImageDescriberType>(descType_i);
        }
    }

    // for compatibility with files generated with a previous version
    if (userProps && userProps.getPropertyHeader("mvg_visibilitySize") && userProps.getPropertyHeader("mvg_visibilityIds") &&
        userProps.getPropertyHeader("mvg_visibilityFeatPos") &&
        (flags_part & ESfMData::OBSERVATIONS || flags_part & ESfMData::OBSERVATIONS_WITH_FEATURES))
    {
        AV_UInt32ArraySamplePtr sampleVisibilitySize(userProps, "mvg_visibilitySize");
        AV_UInt32ArraySamplePtr sampleVisibilityIds(userProps, "mvg_visibilityIds");

        FloatArraySamplePtr sampleFeatPos2d;
        IFloatArrayProperty propFeatPos2d(userProps, "mvg_visibilityFeatPos");
        propFeatPos2d.get(sampleFeatPos2d);

        if (positions->size() != sampleVisibilitySize.size())
        {
            ALICEVISION_LOG_ERROR("Alembic Error: number of observations per 3D point should be identical to the number of 2D features.\n"
                                  "# observations per 3D point: "
                                  << sampleVisibilitySize.size()
                                  << ".\n"
                                     "# 3D points: "
                                  << positions->size() << ".");
            return false;
        }
        if (sampleVisibilityIds.size() != sampleFeatPos2d->size())
        {
            ALICEVISION_LOG_ERROR("Alembic Error: visibility Ids and features 2D pos should have the same size.\n"
                                  "# visibility Ids: "
                                  << sampleVisibilityIds.size()
                                  << ".\n"
                                     "# features 2D pos: "
                                  << sampleFeatPos2d->size() << ".");
            return false;
        }

        std::size_t obsGlobal_i = 0;
        for (std::size_t point3d_i = 0; point3d_i < positions->size(); ++point3d_i)
        {
            sfmData::Landmark& landmark = sfmdata.getLandmarks()[nbPointsInit + point3d_i];
            // Number of observation for this 3d point
            const std::size_t visibilitySize = sampleVisibilitySize[point3d_i];

            for (std::size_t obs_i = 0; obs_i < visibilitySize * 2; obs_i += 2, obsGlobal_i += 2)
            {
                const int viewID = sampleVisibilityIds[obsGlobal_i];
                const int featID = sampleVisibilityIds[obsGlobal_i + 1];
                sfmData::Observation& observations = landmark.observations[viewID];
                observations.id_feat = featID;

                const float posX = (*sampleFeatPos2d)[obsGlobal_i];
                const float posY = (*sampleFeatPos2d)[obsGlobal_i + 1];
                observations.x[0] = posX;
                observations.x[1] = posY;
            }
        }
    }

    if (userProps && userProps.getPropertyHeader("mvg_visibilitySize") && userProps.getPropertyHeader("mvg_visibilityViewId") &&
        (flags_part & ESfMData::OBSERVATIONS || flags_part & ESfMData::OBSERVATIONS_WITH_FEATURES))
    {
        AV_UInt32ArraySamplePtr sampleVisibilitySize(userProps, "mvg_visibilitySize");
        AV_UInt32ArraySamplePtr sampleVisibilityViewId(userProps, "mvg_visibilityViewId");

        if (positions->size() != sampleVisibilitySize.size())
        {
            ALICEVISION_LOG_ERROR("Alembic Error: number of observations per 3D point should be identical to the number of 2D features.\n"
                                  "# observations per 3D point: "
                                  << sampleVisibilitySize.size()
                                  << ".\n"
                                     "# 3D points: "
                                  << positions->size() << ".");
            return false;
        }

        AV_UInt32ArraySamplePtr sampleVisibilityFeatId;
        FloatArraySamplePtr sampleVisibilityFeatPos;
        FloatArraySamplePtr sampleVisibilityFeatScale;

        if (userProps.getPropertyHeader("mvg_visibilityFeatId") && userProps.getPropertyHeader("mvg_visibilityFeatPos") &&
            flags_part & ESfMData::OBSERVATIONS_WITH_FEATURES)
        {
            sampleVisibilityFeatId.read(userProps, "mvg_visibilityFeatId");

            IFloatArrayProperty propVisibilityFeatPos(userProps, "mvg_visibilityFeatPos");
            propVisibilityFeatPos.get(sampleVisibilityFeatPos);

            if (userProps && userProps.getPropertyHeader("mvg_visibilityFeatScale"))
            {
                IFloatArrayProperty propVisibilityFeatScale(userProps, "mvg_visibilityFeatScale");
                propVisibilityFeatScale.get(sampleVisibilityFeatScale);
            }

            if (sampleVisibilityViewId.size() != sampleVisibilityFeatId.size() ||
                2 * sampleVisibilityViewId.size() != sampleVisibilityFeatPos->size())
            {
                ALICEVISION_LOG_ERROR("Alembic Error: visibility Ids and features id / 2D pos should have the same size.\n"
                                      "# view Ids: "
                                      << sampleVisibilityViewId.size()
                                      << ".\n"
                                         "# features id: "
                                      << sampleVisibilityFeatId.size()
                                      << ".\n"
                                         "# features 2D pos: "
                                      << sampleVisibilityFeatPos->size() << ".");
                return false;
            }
        }
        else
        {
            ALICEVISION_LOG_WARNING("Alembic LOAD: NO OBSERVATIONS_WITH_FEATURES: "
                                    << ", mvg_visibilityFeatId: " << long(userProps.getPropertyHeader("mvg_visibilityFeatId"))
                                    << ", mvg_visibilityFeatPos: " << long(userProps.getPropertyHeader("mvg_visibilityFeatPos"))
                                    << ", OBSERVATIONS_WITH_FEATURES flag: " << bool(flags_part & ESfMData::OBSERVATIONS_WITH_FEATURES));
        }

        const bool hasFeatures = bool(sampleVisibilityFeatId) && (sampleVisibilityFeatId.size() > 0);

        std::size_t obsGlobalIndex = 0;
        for (std::size_t point3d_i = 0; point3d_i < positions->size(); ++point3d_i)
        {
            const int landmarkId = nbPointsInit + point3d_i;
            sfmData::Landmark& landmark = sfmdata.getLandmarks()[landmarkId];

            // Number of observation for this 3d point
            const std::size_t visibilitySize = sampleVisibilitySize[point3d_i];

            for (std::size_t obs_i = 0; obs_i < visibilitySize; ++obs_i, ++obsGlobalIndex)
            {
                const int viewId = sampleVisibilityViewId[obsGlobalIndex];

                if (hasFeatures)
                {
                    const std::size_t featId = sampleVisibilityFeatId[obsGlobalIndex];
                    sfmData::Observation& observation = landmark.observations[viewId];
                    observation.id_feat = featId;

                    const float posX = (*sampleVisibilityFeatPos)[2 * obsGlobalIndex];
                    const float posY = (*sampleVisibilityFeatPos)[2 * obsGlobalIndex + 1];
                    observation.x[0] = posX;
                    observation.x[1] = posY;

                    // for compatibility with previous version without scale
                    if (sampleVisibilityFeatScale)
                    {
                        observation.scale = (*sampleVisibilityFeatScale)[obsGlobalIndex];
                    }
                }
                else
                {
                    landmark.observations[viewId] = sfmData::Observation();
                }
            }
        }
    }

    return true;
}

bool readCamera(const Version& abcVersion,
                const ICamera& camera,
                const M44d& mat,
                sfmData::SfMData& sfmData,
                ESfMData flagsPart,
                const index_t sampleFrame = 0,
                bool isReconstructed = true)
{
    using namespace aliceVision::geometry;
    using namespace aliceVision::camera;

    ICameraSchema cs = camera.getSchema();
    CameraSample camSample;
    if (sampleFrame == 0)
        camSample = cs.getValue();
    else
        camSample = cs.getValue(ISampleSelector(sampleFrame));

    // Check if we have an associated image plane
    ICompoundProperty userProps = getAbcUserProperties(cs);
    std::string imagePath;
    std::vector<unsigned int> sensorSize_pix = {0, 0};
    std::vector<double> sensorSize_mm = {0, 0};
    std::string mvg_intrinsicType = EINTRINSIC_enumToString(EINTRINSIC::PINHOLE_CAMERA);
    std::string mvg_intrinsicInitializationMode = EInitMode_enumToString(EInitMode::NONE);
    std::string mvg_intrinsicDistortionInitializationMode = EInitMode_enumToString(EInitMode::NONE);
    std::vector<double> mvg_intrinsicParams;
    std::vector<IndexT> mvg_ancestorsParams;
    Vec2 initialFocalLengthPix = {-1, -1};
    double fisheyeCenterX = 0.0;
    double fisheyeCenterY = 0.0;
    double fisheyeRadius = 1.0;
    std::vector<std::string> rawMetadata;
    IndexT viewId = sfmData.getViews().size();
    IndexT poseId = sfmData.getViews().size();
    IndexT intrinsicId = sfmData.getIntrinsics().size();
    IndexT rigId = UndefinedIndexT;
    IndexT subPoseId = UndefinedIndexT;
    IndexT frameId = UndefinedIndexT;
    IndexT resectionId = UndefinedIndexT;
    bool intrinsicLocked = false;
    bool poseLocked = false;
    bool poseIndependant = true;
    bool lockRatio = true;
    std::vector<double> distortionParams;
    std::vector<double> undistortionParams;
    Vec2 undistortionOffset = {0, 0};
    std::vector<std::string> ancestorImagesPath;
    std::vector<unsigned int> ancestorImagesWidth;
    std::vector<unsigned int> ancestorImagesHeight;
    std::vector<unsigned int> ancestorImagesMetadataSize;
    std::vector<std::string> ancestorImagesRawMetadata;

    if (userProps)
    {
        if ((flagsPart & ESfMData::VIEWS) || (flagsPart & ESfMData::INTRINSICS) || (flagsPart & ESfMData::EXTRINSICS))
        {
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_imagePath"))
                imagePath = getAbcProp<Alembic::Abc::IStringProperty>(userProps, *propHeader, "mvg_imagePath", sampleFrame);

            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_viewId"))
            {
                viewId = getAbcProp_uint(userProps, *propHeader, "mvg_viewId", sampleFrame);
            }
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_poseId"))
            {
                poseId = getAbcProp_uint(userProps, *propHeader, "mvg_poseId", sampleFrame);
            }
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_intrinsicId"))
            {
                intrinsicId = getAbcProp_uint(userProps, *propHeader, "mvg_intrinsicId", sampleFrame);
            }
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_rigId"))
            {
                rigId = getAbcProp_uint(userProps, *propHeader, "mvg_rigId", sampleFrame);
            }
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_subPoseId"))
            {
                subPoseId = getAbcProp_uint(userProps, *propHeader, "mvg_subPoseId", sampleFrame);
            }
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_frameId"))
            {
                frameId = getAbcProp_uint(userProps, *propHeader, "mvg_frameId", sampleFrame);
            }
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_resectionId"))
            {
                resectionId = getAbcProp_uint(userProps, *propHeader, "mvg_resectionId", sampleFrame);
            }
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_intrinsicLocked"))
            {
                intrinsicLocked = getAbcProp<Alembic::Abc::IBoolProperty>(userProps, *propHeader, "mvg_intrinsicLocked", sampleFrame);
            }
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_intrinsicPixelRatioLocked"))
            {
                lockRatio = getAbcProp<Alembic::Abc::IBoolProperty>(userProps, *propHeader, "mvg_intrinsicPixelRatioLocked", sampleFrame);
            }
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_poseLocked"))
            {
                poseLocked = getAbcProp<Alembic::Abc::IBoolProperty>(userProps, *propHeader, "mvg_poseLocked", sampleFrame);
            }
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_poseIndependant"))
            {
                poseIndependant = getAbcProp<Alembic::Abc::IBoolProperty>(userProps, *propHeader, "mvg_poseIndependant", sampleFrame);
            }
            if (userProps.getPropertyHeader("mvg_metadata"))
            {
                getAbcArrayProp<Alembic::Abc::IStringArrayProperty>(userProps, "mvg_metadata", sampleFrame, rawMetadata);
                if (rawMetadata.size() % 2 != 0)
                {
                    ALICEVISION_THROW_ERROR("[Alembic] 'metadata' property is supposed to be key/values. Number of values is " +
                                            std::to_string(rawMetadata.size()) + ".");
                }
            }
            if (userProps.getPropertyHeader("mvg_sensorSizePix"))
            {
                getAbcArrayProp_uint(userProps, "mvg_sensorSizePix", sampleFrame, sensorSize_pix);
                if (sensorSize_pix.size() != 2)
                {
                    ALICEVISION_THROW_ERROR("[Alembic] 'sensorSizePix' property is supposed to be 2 values. Number of values is " +
                                            std::to_string(sensorSize_pix.size()) + ".");
                }
            }
            if (userProps.getPropertyHeader("mvg_sensorSizeMm"))
            {
                getAbcArrayProp<Alembic::Abc::IDoubleArrayProperty>(userProps, "mvg_sensorSizeMm", sampleFrame, sensorSize_mm);
                if (sensorSize_mm.size() != 2)
                {
                    ALICEVISION_THROW_ERROR("[Alembic] 'sensorSizeMm' property is supposed to be 2 values. Number of values is " +
                                            std::to_string(sensorSize_mm.size()) + ".");
                }
            }
            else
            {
                sensorSize_mm = {24.0, 36.0};
            }
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_intrinsicType"))
            {
                mvg_intrinsicType = getAbcProp<Alembic::Abc::IStringProperty>(userProps, *propHeader, "mvg_intrinsicType", sampleFrame);
            }
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_intrinsicInitializationMode"))
            {
                mvg_intrinsicInitializationMode =
                  getAbcProp<Alembic::Abc::IStringProperty>(userProps, *propHeader, "mvg_intrinsicInitializationMode", sampleFrame);
            }
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_intrinsicDistortionInitializationMode"))
            {
                mvg_intrinsicDistortionInitializationMode =
                  getAbcProp<Alembic::Abc::IStringProperty>(userProps, *propHeader, "mvg_intrinsicDistortionInitializationMode", sampleFrame);
            }
            // For compatibility with versions < 1.2 (value was in pixels)
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_initialFocalLengthPix"))
            {
                initialFocalLengthPix(0) =
                  getAbcProp<Alembic::Abc::IDoubleProperty>(userProps, *propHeader, "mvg_initialFocalLengthPix", sampleFrame);
            }
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_initialFocalLength"))
            {
                initialFocalLengthPix(0) = (sensorSize_pix.at(0) / sensorSize_mm[0]) *
                                           getAbcProp<Alembic::Abc::IDoubleProperty>(userProps, *propHeader, "mvg_initialFocalLength", sampleFrame);
            }
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_fisheyeCircleCenterX"))
            {
                fisheyeCenterX = getAbcProp<Alembic::Abc::IDoubleProperty>(userProps, *propHeader, "mvg_fisheyeCircleCenterX", sampleFrame);
            }
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_fisheyeCircleCenterY"))
            {
                fisheyeCenterY = getAbcProp<Alembic::Abc::IDoubleProperty>(userProps, *propHeader, "mvg_fisheyeCircleCenterY", sampleFrame);
            }
            if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_fisheyeCircleRadius"))
            {
                fisheyeRadius = getAbcProp<Alembic::Abc::IDoubleProperty>(userProps, *propHeader, "mvg_fisheyeCircleRadius", sampleFrame);
            }
            if (userProps.getPropertyHeader("mvg_intrinsicParams"))
            {
                Alembic::Abc::IDoubleArrayProperty prop(userProps, "mvg_intrinsicParams");
                Alembic::Abc::IDoubleArrayProperty::sample_ptr_type sample;
                prop.get(sample, ISampleSelector(sampleFrame));
                mvg_intrinsicParams.assign(sample->get(), sample->get() + sample->size());
            }
            if (userProps.getPropertyHeader("mvg_ancestorsParams"))
            {
                Alembic::Abc::IUInt32ArrayProperty prop(userProps, "mvg_ancestorsParams");
                Alembic::Abc::IUInt32ArrayProperty::sample_ptr_type sample;
                prop.get(sample, ISampleSelector(sampleFrame));
                mvg_ancestorsParams.assign(sample->get(), sample->get() + sample->size());
            }
            if (userProps.getPropertyHeader("mvg_distortionParams"))
            {
                // Distortion parameters
                Alembic::Abc::IDoubleArrayProperty prop(userProps, "mvg_distortionParams");
                Alembic::Abc::IDoubleArrayProperty::sample_ptr_type sample;
                prop.get(sample, ISampleSelector(sampleFrame));
                distortionParams.assign(sample->get(), sample->get() + sample->size());
            }
            if (userProps.getPropertyHeader("mvg_undistortionParams"))
            {
                // Undistortion parameters
                Alembic::Abc::IDoubleArrayProperty prop(userProps, "mvg_undistortionParams");
                Alembic::Abc::IDoubleArrayProperty::sample_ptr_type sample;
                prop.get(sample, ISampleSelector(sampleFrame));
                undistortionParams.assign(sample->get(), sample->get() + sample->size());
                // Undistortion offset
                if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_undistortionOffsetX"))
                {
                    undistortionOffset(0) = getAbcProp<Alembic::Abc::IDoubleProperty>(userProps, *propHeader, "mvg_undistortionOffsetX", sampleFrame);
                }
                if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_undistortionOffsetY"))
                {
                    undistortionOffset(1) = getAbcProp<Alembic::Abc::IDoubleProperty>(userProps, *propHeader, "mvg_undistortionOffsetY", sampleFrame);
                }
            }
            if (userProps.getPropertyHeader("mvg_ancestorImagesPath"))
            {
                getAbcArrayProp<Alembic::Abc::IStringArrayProperty>(userProps, "mvg_ancestorImagesPath", sampleFrame, ancestorImagesPath);
            }
            if (userProps.getPropertyHeader("mvg_ancestorImagesWidth"))
            {
                getAbcArrayProp<Alembic::Abc::IUInt32ArrayProperty>(userProps, "mvg_ancestorImagesWidth", sampleFrame, ancestorImagesWidth);
            }
            if (userProps.getPropertyHeader("mvg_ancestorImagesHeight"))
            {
                getAbcArrayProp<Alembic::Abc::IUInt32ArrayProperty>(userProps, "mvg_ancestorImagesHeight", sampleFrame, ancestorImagesHeight);
            }
            if (userProps.getPropertyHeader("mvg_ancestorImagesMetadataSize"))
            {
                getAbcArrayProp<Alembic::Abc::IUInt32ArrayProperty>(
                  userProps, "mvg_ancestorImagesMetadataSize", sampleFrame, ancestorImagesMetadataSize);
            }
            if (userProps.getPropertyHeader("mvg_ancestorImagesRawMetadata"))
            {
                getAbcArrayProp<Alembic::Abc::IStringArrayProperty>(
                  userProps, "mvg_ancestorImagesRawMetadata", sampleFrame, ancestorImagesRawMetadata);
                if (ancestorImagesRawMetadata.size() % 2 != 0)
                {
                    ALICEVISION_THROW_ERROR("[Alembic] 'metadata' property is supposed to be key/values. Number of values is " +
                                            std::to_string(ancestorImagesRawMetadata.size()) + ".");
                }
            }
        }
    }

    if (flagsPart & ESfMData::INTRINSICS)
    {
        // get known values from alembic
        // const float haperture_cm = camSample.getHorizontalAperture();
        // const float vaperture_cm = camSample.getVerticalAperture();
        // compute other needed values
        // const float sensorWidth_mm = std::max(vaperture_cm, haperture_cm) * 10.0;
        // const float mm2pix = sensorSize_pix.at(0) / sensorWidth_mm;
        // imgWidth = haperture_cm * 10.0 * mm2pix;
        // imgHeight = vaperture_cm * 10.0 * mm2pix;

        // create intrinsic parameters object
        std::shared_ptr<camera::IntrinsicBase> intrinsic = createIntrinsic(
          /*intrinsic type*/ EINTRINSIC_stringToEnum(mvg_intrinsicType),
          /*width*/ sensorSize_pix.at(0),
          /*height*/ sensorSize_pix.at(1));

        intrinsic->setSensorWidth(sensorSize_mm.at(0));
        intrinsic->setSensorHeight(sensorSize_mm.at(1));
        intrinsic->importFromParams(mvg_intrinsicParams, abcVersion);
        intrinsic->setInitializationMode(EInitMode_stringToEnum(mvg_intrinsicInitializationMode));
        intrinsic->setDistortionInitializationMode(EInitMode_stringToEnum(mvg_intrinsicDistortionInitializationMode));

        std::shared_ptr<camera::IntrinsicScaleOffsetDisto> intrinsicCasted = std::dynamic_pointer_cast<camera::IntrinsicScaleOffsetDisto>(intrinsic);
        if (intrinsicCasted)
        {
            // fy_pix = fx_pix * fy/fx
            initialFocalLengthPix(1) =
              (initialFocalLengthPix(0) > 0) ? initialFocalLengthPix(0) * mvg_intrinsicParams[1] / mvg_intrinsicParams[0] : -1;
            intrinsicCasted->setInitialScale(initialFocalLengthPix);
            intrinsicCasted->setRatioLocked(lockRatio);
            std::shared_ptr<camera::Distortion> distortion = intrinsicCasted->getDistortion();
            if (distortion)
            {
                distortion->setParameters(distortionParams);
            }
            std::shared_ptr<camera::Undistortion> undistortion = intrinsicCasted->getUndistortion();
            if (undistortion)
            {
                undistortion->setParameters(undistortionParams);
                undistortion->setOffset(undistortionOffset);
            }
        }

        std::shared_ptr<camera::Equidistant> casted = std::dynamic_pointer_cast<camera::Equidistant>(intrinsic);
        if (casted)
        {
            casted->setCircleCenterX(fisheyeCenterX);
            casted->setCircleCenterY(fisheyeCenterY);
            casted->setCircleRadius(fisheyeRadius);
        }

        if (intrinsicLocked)
            intrinsic->lock();
        else
            intrinsic->unlock();

        sfmData.getIntrinsics().emplace(intrinsicId, intrinsic);
    }

    // add imported data to the SfMData container TODO use UID
    // this view is incomplete if no flag VIEWS
    std::shared_ptr<sfmData::View> view =
      std::make_shared<sfmData::View>(imagePath, viewId, intrinsicId, poseId, sensorSize_pix.at(0), sensorSize_pix.at(1), rigId, subPoseId);
    if (flagsPart & ESfMData::VIEWS)
    {
        view->setResectionId(resectionId);
        view->setFrameId(frameId);
        view->setIndependantPose(poseIndependant);

        // set metadata
        for (std::size_t i = 0; i < rawMetadata.size(); i += 2)
        {
            view->getImage().addMetadata(rawMetadata.at(i), rawMetadata.at(i + 1));
        }

        // set ancestor viewIds
        for (IndexT val : mvg_ancestorsParams)
        {
            view->addAncestor(val);
        }

        // set ancestor images
        size_t mIndexStart = 0;
        for (std::size_t i = 0; i < ancestorImagesPath.size(); i++)
        {
            std::map<std::string, std::string> metadata;
            size_t mIndexEnd = mIndexStart + ancestorImagesMetadataSize[i];
            for (size_t mIndex = mIndexStart; mIndex < mIndexEnd; mIndex++)
            {
                metadata.insert(
                  std::pair<std::string, std::string>(ancestorImagesRawMetadata[2 * mIndex], ancestorImagesRawMetadata[2 * mIndex + 1]));
            }
            mIndexStart = mIndexEnd;
            view->addAncestorImage(std::make_shared<sfmData::ImageInfo>(
              sfmData::ImageInfo(ancestorImagesPath[i], ancestorImagesWidth[i], ancestorImagesHeight[i], metadata)));
        }

        sfmData.getViews().emplace(viewId, view);
    }

    if ((flagsPart & ESfMData::EXTRINSICS) && isReconstructed)
    {
        Mat4 T = Mat4::Identity();
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                T(i, j) = mat[j][i];
            }
        }

        // convert from computer graphics convention (opengl-like) to computer vision
        Mat4 M = Mat4::Identity();
        M(1, 1) = -1.0;
        M(2, 2) = -1.0;

        Mat4 T2;
        if (abcVersion < Version(1, 2, 3))
        {
            T2 = (T * M).inverse();
        }
        else
        {
            T2 = (M * T * M).inverse();
        }

        Pose3 pose(T2);

        if (view->isPartOfRig() && !view->isPoseIndependant())
        {
            sfmData::Rig& rig = sfmData.getRigs()[view->getRigId()];
            std::vector<sfmData::RigSubPose>& sp = rig.getSubPoses();
            if (view->getSubPoseId() >= sp.size())
                sp.resize(view->getSubPoseId() + 1);
            sfmData::RigSubPose& subPose = rig.getSubPose(view->getSubPoseId());
            if (subPose.status == sfmData::ERigSubPoseStatus::UNINITIALIZED)
            {
                subPose.status = sfmData::ERigSubPoseStatus::ESTIMATED;
                subPose.pose = pose;
            }
        }
        else
        {
            sfmData.setPose(*view, sfmData::CameraPose(pose, poseLocked));
        }
    }

    return true;
}

bool readXform(const Version& abcVersion, IXform& xform, M44d& mat, sfmData::SfMData& sfmData, ESfMData flagsPart, bool isReconstructed = true)
{
    using namespace aliceVision::geometry;
    using namespace aliceVision::camera;

    IXformSchema schema = xform.getSchema();
    XformSample xsample;

    schema.get(xsample);

    // If we have an animated camera we handle it with the xform here
    if (xform.getSchema().getNumSamples() != 1)
    {
        ALICEVISION_LOG_DEBUG(xform.getSchema().getNumSamples() << " samples found in this animated xform.");
        for (index_t frame = 0; frame < xform.getSchema().getNumSamples(); ++frame)
        {
            xform.getSchema().get(xsample, ISampleSelector(frame));
            readCamera(abcVersion, ICamera(xform.getChild(0), kWrapExisting), mat * xsample.getMatrix(), sfmData, flagsPart, frame, isReconstructed);
        }
        return true;
    }

    mat *= xsample.getMatrix();

    ICompoundProperty userProps = getAbcUserProperties(schema);

    // Check if it is a rig node
    IndexT rigId = UndefinedIndexT;
    IndexT poseId = UndefinedIndexT;
    std::size_t nbSubPoses = 0;
    bool rigPoseLocked = false;

    if (userProps)
    {
        if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_rigId"))
        {
            try
            {
                rigId = getAbcProp<Alembic::Abc::IUInt32Property>(userProps, *propHeader, "mvg_rigId", 0);
            }
            catch (Alembic::Util::Exception&)
            {
                rigId = getAbcProp<Alembic::Abc::IInt32Property>(userProps, *propHeader, "mvg_rigId", 0);
            }
        }

        if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_poseId"))
        {
            try
            {
                poseId = getAbcProp<Alembic::Abc::IUInt32Property>(userProps, *propHeader, "mvg_poseId", 0);
            }
            catch (Alembic::Util::Exception&)
            {
                poseId = getAbcProp<Alembic::Abc::IInt32Property>(userProps, *propHeader, "mvg_poseId", 0);
            }
        }

        if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_nbSubPoses"))
        {
            try
            {
                nbSubPoses = getAbcProp<Alembic::Abc::IUInt16Property>(userProps, *propHeader, "mvg_nbSubPoses", 0);
            }
            catch (Alembic::Util::Exception&)
            {
                nbSubPoses = getAbcProp<Alembic::Abc::IInt16Property>(userProps, *propHeader, "mvg_nbSubPoses", 0);
            }
        }

        if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_rigPoseLocked"))
        {
            rigPoseLocked = getAbcProp<Alembic::Abc::IBoolProperty>(userProps, *propHeader, "mvg_rigPoseLocked", 0);
        }
    }

    if ((rigId == UndefinedIndexT) && (poseId == UndefinedIndexT))
    {
        return true;  // not a rig
    }

    if ((flagsPart & ESfMData::EXTRINSICS) && isReconstructed)
    {
        Mat4 T = Mat4::Identity();
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                T(i, j) = mat[j][i];
            }
        }

        Mat4 M = Mat4::Identity();
        M(1, 1) = -1.0;
        M(2, 2) = -1.0;

        Mat4 T2;
        if (abcVersion < Version(1, 2, 3))
        {
            T2 = T.inverse();
        }
        else
        {
            T2 = (M * T * M).inverse();
        }

        Pose3 pose(T2);

        if (sfmData.getPoses().find(poseId) == sfmData.getPoses().end())
        {
            sfmData.getPoses().emplace(poseId, sfmData::CameraPose(pose, rigPoseLocked));
        }
    }

    if ((rigId != UndefinedIndexT) && sfmData.getRigs().find(rigId) == sfmData.getRigs().end())
        sfmData.getRigs().emplace(rigId, sfmData::Rig(nbSubPoses));

    mat.makeIdentity();
    return true;
}

// Top down read of 3d objects
void visitObject(const Version& abcVersion, IObject iObj, M44d mat, sfmData::SfMData& sfmdata, ESfMData flagsPart, bool isReconstructed = true)
{
    // ALICEVISION_LOG_DEBUG("ABC visit: " << iObj.getFullName());
    if (iObj.getName() == "mvgCamerasUndefined")
        isReconstructed = false;

    const MetaData& md = iObj.getMetaData();
    if (IPoints::matches(md) && (flagsPart & ESfMData::STRUCTURE))
    {
        readPointCloud(abcVersion, iObj, mat, sfmdata, flagsPart);
    }
    else if (IXform::matches(md))
    {
        IXform xform(iObj, kWrapExisting);
        readXform(abcVersion, xform, mat, sfmdata, flagsPart, isReconstructed);
    }
    else if (ICamera::matches(md) && ((flagsPart & ESfMData::VIEWS) || (flagsPart & ESfMData::INTRINSICS) || (flagsPart & ESfMData::EXTRINSICS)))
    {
        ICamera check_cam(iObj, kWrapExisting);
        // If it's not an animated camera we add it here
        if (check_cam.getSchema().getNumSamples() == 1)
        {
            readCamera(abcVersion, check_cam, mat, sfmdata, flagsPart, 0, isReconstructed);
        }
    }

    // Recurse
    for (std::size_t i = 0; i < iObj.getNumChildren(); i++)
    {
        visitObject(abcVersion, iObj.getChild(i), mat, sfmdata, flagsPart, isReconstructed);
    }
}

struct AlembicImporter::DataImpl
{
    DataImpl(const std::string& filename)
    {
        Alembic::AbcCoreFactory::IFactory factory;
        Alembic::AbcCoreFactory::IFactory::CoreType coreType;
        Abc::IArchive archive = factory.getArchive(filename, coreType);

        if (!archive.valid())
            throw std::runtime_error("Can't open '" + filename + "' : Alembic file is not valid.");

        _rootEntity = archive.getTop();
        _filename = filename;
    }

    IObject _rootEntity;
    std::string _filename;
};

AlembicImporter::AlembicImporter(const std::string& filename) { _dataImpl.reset(new DataImpl(filename)); }

AlembicImporter::~AlembicImporter() {}

void AlembicImporter::populateSfM(sfmData::SfMData& sfmdata, ESfMData flagsPart)
{
    const index_t sampleFrame = 0;
    IObject rootObj = _dataImpl->_rootEntity.getChild("mvgRoot");
    ICompoundProperty userProps = rootObj.getProperties();

    // set SfMData folder absolute path
    sfmdata.setAbsolutePath(_dataImpl->_filename);

    std::vector<::uint32_t> vecAbcVersion = {0, 0, 0};

    if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_ABC_version"))
    {
        getAbcArrayProp_uint(userProps, "mvg_ABC_version", sampleFrame, vecAbcVersion);
    }
    // Old versions were using only major,minor
    if (vecAbcVersion.size() < 3)
    {
        vecAbcVersion.resize(3, 0);
    }

    Version abcVersion(vecAbcVersion[0], vecAbcVersion[1], vecAbcVersion[2]);

    if (userProps.getPropertyHeader("mvg_featuresFolders"))
    {
        std::vector<std::string> featuresFolders;
        getAbcArrayProp<Alembic::Abc::IStringArrayProperty>(userProps, "mvg_featuresFolders", sampleFrame, featuresFolders);
        sfmdata.setFeaturesFolders(featuresFolders);
    }

    if (userProps.getPropertyHeader("mvg_matchesFolders"))
    {
        std::vector<std::string> matchesFolders;
        getAbcArrayProp<Alembic::Abc::IStringArrayProperty>(userProps, "mvg_matchesFolders", sampleFrame, matchesFolders);
        sfmdata.setMatchesFolders(matchesFolders);
    }

    // keep compatibility with single folder for feature and matching
    if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_featureFolder"))
    {
        const std::string featuresFolder = getAbcProp<Alembic::Abc::IStringProperty>(userProps, *propHeader, "mvg_featureFolder", sampleFrame);
        sfmdata.addFeaturesFolder(featuresFolder);
    }

    if (const Alembic::Abc::PropertyHeader* propHeader = userProps.getPropertyHeader("mvg_matchingFolder"))
    {
        const std::string matchesFolder = getAbcProp<Alembic::Abc::IStringProperty>(userProps, *propHeader, "mvg_matchingFolder", sampleFrame);
        sfmdata.addMatchesFolder(matchesFolder);
    }

    // TODO : handle the case where the archive wasn't correctly opened
    M44d xformMat;
    visitObject(abcVersion, _dataImpl->_rootEntity, xformMat, sfmdata, flagsPart);

    // TODO: fusion of common intrinsics
}

}  // namespace sfmDataIO
}  // namespace aliceVision
