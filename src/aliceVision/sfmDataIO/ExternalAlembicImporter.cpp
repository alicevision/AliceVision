// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmDataIO/ExternalAlembicImporter.hpp>

#include <aliceVision/version.hpp>
#include <aliceVision/system/Logger.hpp>

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/image/io.hpp>

namespace aliceVision {
namespace sfmDataIO {

ExternalAlembicImporter::ExternalAlembicImporter(const std::string& filename)
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
}

void ExternalAlembicImporter::populateSfM(sfmData::SfMData& sfmdata, const std::vector<std::string> & files)
{
    Alembic::Abc::M44d identity;
    visitObject(_rootEntity, identity, sfmdata, files);
}

void ExternalAlembicImporter::visitObject(Alembic::Abc::IObject iObj, const Alembic::Abc::M44d & mat, sfmData::SfMData& sfmdata, const std::vector<std::string> & files)
{
    
    const Alembic::Abc::MetaData& md = iObj.getMetaData();
    
    if (Alembic::AbcGeom::IXform::matches(md))
    {
        Alembic::AbcGeom::IXform xform(iObj, Alembic::Abc::kWrapExisting);
        Alembic::AbcGeom::IXformSchema schema = xform.getSchema();
            
        Alembic::AbcGeom::XformSample xsample;

        if (schema.getNumSamples() == 1)
        {
            ALICEVISION_THROW_ERROR("Non implemented path.");
        }
        
        if (schema.getNumSamples() != files.size())
        {
            ALICEVISION_THROW_ERROR("Incompatible number of files wrt abc samples.")
        }

        Mat4 M = Mat4::Identity();
        M(1, 1) = -1.0;
        M(2, 2) = -1.0;

        for (Alembic::Abc::index_t frame = 0; frame < xform.getSchema().getNumSamples(); ++frame)
        {
            xform.getSchema().get(xsample,  Alembic::Abc::ISampleSelector(frame));
            const auto & currentMat = xsample.getMatrix();
            const auto newMat = mat * xsample.getMatrix();

            Mat4 T = Mat4::Identity();
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    T(i, j) = newMat[j][i];
                }
            }

            Mat4 T2 = (M * T * M).inverse();
            geometry::Pose3 pose(T2);

            Alembic::AbcGeom::ICamera camera(xform.getChild(0),  Alembic::Abc::kWrapExisting);
            Alembic::AbcGeom::ICameraSchema cs = camera.getSchema();
            Alembic::AbcGeom::CameraSample camSample;
            camSample = cs.getValue(Alembic::Abc::ISampleSelector(frame));

            /*std::cout << " ---- " << std::endl;
            std::cout << camSample.getFocalLength() << std::endl;
            std::cout << camSample.getHorizontalAperture() << std::endl;
            std::cout << camSample.getVerticalAperture() << std::endl;
            std::cout << camSample.getLensSqueezeRatio() << std::endl;
            std::cout << camSample.getHorizontalFilmOffset() << std::endl;
            std::cout << camSample.getVerticalFilmOffset() << std::endl;*/

            int w = 0;
            int h = 0;
            image::readImageMetadata(files[frame], w, h);

            auto cam = camera::createPinhole(
                                            camera::EDISTORTION::DISTORTION_NONE,
                                            camera::EUNDISTORTION::UNDISTORTION_NONE,
                                            w, h,
                                            1.0, 1.0,
                                            0.0, 0.0
                                            );
            
            const double dw = static_cast<double>(w);
            const double dh = static_cast<double>(h);
            double happ = camSample.getHorizontalAperture();
            double vapp = camSample.getVerticalAperture();
            const double sensorWidthPix = std::max(dw, dh);

            cam->setSensorWidth(happ / (dw * 0.1 / sensorWidthPix));
            cam->setSensorHeight(vapp / (dh * 0.1 / sensorWidthPix));
            cam->setFocalLength(camSample.getFocalLength(), 1.0, false);

            sfmdata.getIntrinsics().emplace(frame, cam);
            sfmdata.getPoses()[frame] = sfmData::CameraPose(pose);
            sfmdata.getViews().emplace(frame, std::make_shared<sfmData::View>(files[frame], frame, frame, frame, w, h));
        }
    }

    // Recurse
    for (std::size_t i = 0; i < iObj.getNumChildren(); i++)
    {
        visitObject(iObj.getChild(i), mat, sfmdata, files);
    }
}

}  // namespace sfmDataIO
}  // namespace aliceVision
