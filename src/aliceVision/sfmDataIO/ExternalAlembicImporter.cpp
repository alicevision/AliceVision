// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmDataIO/ExternalAlembicImporter.hpp>

#include <aliceVision/version.hpp>
#include <aliceVision/system/Logger.hpp>

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/image/io.hpp>

#include <list>
#include <sstream>
#include <iomanip>

namespace aliceVision {
namespace sfmDataIO {

ExternalAlembicImporter::ExternalAlembicImporter(const std::string& filename, double framerate, unsigned int imageWidth)
{
    Alembic::AbcCoreFactory::IFactory factory;
    Alembic::AbcCoreFactory::IFactory::CoreType coreType;
    Alembic::Abc::IArchive archive = factory.getArchive(filename, coreType);

    if (!archive.valid())
    {
        throw std::runtime_error("Can't open '" + filename + "' : Alembic file is not valid.");
    }

    _rootEntity = archive.getTop();
    _filename = filename;
    _framerate = framerate;
    _imageWidth = imageWidth;
}

bool ExternalAlembicImporter::populateSfM(sfmData::SfMData& sfmdata)
{
    std::vector<Alembic::AbcGeom::ICamera> cameras;

    // Go through the hierarchy and find ICamera objects
    populateCameras(cameras, _rootEntity);

    ALICEVISION_LOG_INFO("Found " << cameras.size() << " different camera(s).");

    //Loop over all cameras
    for (auto cam : cameras)
    {
        // Get all time samples
        std::set<Alembic::Abc::chrono_t> samplesXform;
        if (!collectXformTimes(samplesXform, cam.getParent()))
        {
            return false;
        }

        // Get camera time samples
        std::set<Alembic::Abc::chrono_t> samplesCameras;
        if (!collectCameraTimes(samplesCameras, cam))
        {
            return false;
        }

        // Union of all sets
        std::set<Alembic::Abc::chrono_t> samples;
        samples = samplesCameras;
        samples.insert(samplesXform.begin(), samplesXform.end());

        ALICEVISION_LOG_INFO("Found " << samples.size() << " samples for camera.");

        // Add all poses
        for (const auto & sample : samples)
        {
            Imath::M44d mat = computeTransform(cam.getParent(), sample);
            
            addPose(sfmdata, sample, mat);
            addCamera(sfmdata, sample, cam);
            addView(sfmdata, sample);
        }        
    }

    return true;
}

void ExternalAlembicImporter::addPose(sfmData::SfMData& sfmdata, Alembic::Abc::chrono_t sample, const Imath::M44d & mat)
{
    const size_t frame = std::round(sample * _framerate);

    Mat4 vision_T_gl = Mat4::Identity();
    vision_T_gl(1, 1) = -1.0;
    vision_T_gl(2, 2) = -1.0;

    // Convert to Eigen, transposing 
    // (look like ABC transform is a left to right operation)

    Mat4 world_T_camera = Mat4::Identity();
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            world_T_camera(i, j) = mat[j][i];
        }
    }

    // world_T_camera is in GL coordinates (Y up, Z back)
    // we want camera_T_world = world_T_camera^-1 in vision coordinates (Y down, Z front)
    Mat4 camera_T_world = (vision_T_gl * world_T_camera * vision_T_gl).inverse();

    // Assign pose to camera
    geometry::Pose3 pose(camera_T_world);
    sfmdata.getPoses().assign(frame, sfmData::CameraPose(pose));
}

void ExternalAlembicImporter::addCamera(sfmData::SfMData& sfmdata, Alembic::Abc::chrono_t sample, Alembic::AbcGeom::ICamera iCamera)
{
    Alembic::Abc::ISampleSelector ss(sample);
    Alembic::AbcGeom::ICameraSchema cs = iCamera.getSchema();
    Alembic::AbcGeom::CameraSample camSample = cs.getValue(ss);

    size_t frame = std::round(sample * _framerate);

    const int w = _imageWidth;
    const double cmToMm = 10.0;
    double happ = camSample.getHorizontalAperture() * cmToMm;
    double vapp = camSample.getVerticalAperture() * cmToMm;

    const double dw = static_cast<double>(w);
    const double dh = dw * vapp / happ;
    const int h = static_cast<int>(std::round(dh));


    auto cam = camera::createPinhole(
                                    camera::EDISTORTION::DISTORTION_NONE,
                                    camera::EUNDISTORTION::UNDISTORTION_NONE,
                                    w, h,
                                    1.0, 1.0,
                                    0.0, 0.0
                                    );
    
    

    cam->setSensorWidth(happ);
    cam->setSensorHeight(vapp);
    cam->setFocalLength(camSample.getFocalLength(), 1.0, false);

    Vec2 offset;
    const double mmToPix = dw / happ;
    offset.x() = camSample.getHorizontalFilmOffset() * cmToMm * mmToPix;
    offset.y() = - camSample.getVerticalFilmOffset() * cmToMm * mmToPix;
    cam->setOffset(offset);

    sfmdata.getIntrinsics().emplace(frame, cam);
}

void ExternalAlembicImporter::addView(sfmData::SfMData& sfmdata, Alembic::Abc::chrono_t sample)
{
    const size_t frame = std::round(sample * _framerate);

    const auto & intrinsics = sfmdata.getIntrinsics();
    auto it = intrinsics.find(frame);
    if (it == intrinsics.end())
    {
        // should not happen
        ALICEVISION_LOG_ERROR("Intrinsic not found.");
        return;
    }

    const auto & intrinsic = it->second;
    
    std::stringstream filename;
    filename << std::setfill('0') << std::setw(12) << frame << ".exr";

    sfmData::View view(filename.str(), frame, frame, frame, intrinsic->w(), intrinsic->h());
    view.setFrameId(frame);

    sfmdata.getViews().assign(frame, view);
}

Imath::M44d ExternalAlembicImporter::computeTransform(Alembic::Abc::IObject iObj, double time)
{
    Imath::M44d worldMatrix;
    worldMatrix.makeIdentity();
    
    Alembic::Abc::ISampleSelector ss(time);
    Alembic::Abc::IObject current = iObj;
    
    // Walk up the hierarchy
    while (current.valid()) 
    {
        const Alembic::Abc::MetaData& md = current.getMetaData();
        
        // If this is an IXform, accumulate its transform
        if (Alembic::AbcGeom::IXform::matches(md)) {

            Alembic::AbcGeom::IXform xform(current, Alembic::Abc::kWrapExisting);
            Alembic::AbcGeom::XformSample sample;
            xform.getSchema().get(sample, ss);
            
            // Multiply in parent-first order
            worldMatrix = sample.getMatrix() * worldMatrix;
        }
        
        // Move to parent
        current = current.getParent();
    }
    
    return worldMatrix;
}

void ExternalAlembicImporter::populateCameras(std::vector<Alembic::AbcGeom::ICamera> & cameras, Alembic::Abc::IObject iObj)
{
    cameras.clear();

    std::list<Alembic::Abc::IObject> stack;
    stack.push_back(iObj);

    while (!stack.empty())
    {
        Alembic::Abc::IObject currentObject = stack.back();
        stack.pop_back();

        const Alembic::Abc::MetaData& md = currentObject.getMetaData();

        if (Alembic::AbcGeom::ICamera::matches(md))
        {
            Alembic::AbcGeom::ICamera camera(currentObject, Alembic::Abc::kWrapExisting);
            cameras.push_back(camera);
        }

        for (std::size_t i = 0; i < currentObject.getNumChildren(); i++)
        {
            Alembic::Abc::IObject child = currentObject.getChild(i);
            if (child.valid())
            {
                stack.push_back(child);
            }
        }
    }   
}

bool ExternalAlembicImporter::collectXformTimes(std::set<Alembic::Abc::chrono_t> & samples, Alembic::AbcGeom::IObject iObject)
{
    Alembic::Abc::IObject current = iObject;

    samples.clear();
    
    // Walk up the hierarchy
    while (current.valid()) 
    {
        const Alembic::Abc::MetaData& md = current.getMetaData();
        
        // If this is an IXform, accumulate its transform
        if (Alembic::AbcGeom::IXform::matches(md)) {

            Alembic::AbcGeom::IXform xform(current, Alembic::Abc::kWrapExisting);
            Alembic::AbcGeom::IXformSchema schema = xform.getSchema();           
            Alembic::Abc::TimeSamplingPtr timeSampling = schema.getTimeSampling();

            for (Alembic::Abc::index_t frame = 0; frame < schema.getNumSamples(); ++frame)
            {
                Alembic::Abc::chrono_t time = timeSampling->getSampleTime(frame);

                // Check that the sample is a frame
                double framePos = time * _framerate;
                double frameCheck = std::round(framePos) - framePos;
                if (std::abs(frameCheck) > 1e-6)
                {
                    ALICEVISION_LOG_ERROR("The framerate seems off (non integer frame).");
                    return false;
                }

                samples.insert(time);
            }
        }
        
        // Move to parent
        current = current.getParent();
    }

    return true;
}

bool ExternalAlembicImporter::collectCameraTimes(std::set<Alembic::Abc::chrono_t> & samples, Alembic::AbcGeom::ICamera iCamera)
{
    Alembic::AbcGeom::ICameraSchema schema = iCamera.getSchema();           
    Alembic::Abc::TimeSamplingPtr timeSampling = schema.getTimeSampling();

    for (Alembic::Abc::index_t frame = 0; frame < schema.getNumSamples(); ++frame)
    {
        Alembic::Abc::chrono_t time = timeSampling->getSampleTime(frame);

        // Check that the sample is a frame
        double framePos = time * _framerate;
        double frameCheck = std::round(framePos) - framePos;
        if (std::abs(frameCheck) > 1e-6)
        {
            ALICEVISION_LOG_ERROR("The framerate seems off (non integer frame).");
            return false;
        }

        samples.insert(time);
    }
        

    return true;
}


}  // namespace sfmDataIO
}  // namespace aliceVision
