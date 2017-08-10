/* 
 * File:   Alembic_Exporter.cpp
 * Author: sgaspari
 * 
 * Created on September 24, 2015, 3:57 PM
 */

#include "AlembicExporter.hpp"

#include <Alembic/AbcGeom/All.h>
#include <Alembic/AbcCoreHDF5/All.h>

#include "openMVG/sfm/sfm_view_metadata.hpp"
#include "openMVG/version.hpp"

#include <numeric>

namespace openMVG {
namespace sfm {


using namespace Alembic::Abc;
namespace AbcG = Alembic::AbcGeom;
using namespace AbcG;

struct AlembicExporter::DataImpl
{
  DataImpl(const std::string &filename)
  : _archive(Alembic::AbcCoreHDF5::WriteArchive(), filename)
  , _topObj(_archive, Alembic::Abc::kTop)
  {
    // Create MVG hierarchy
    _mvgRoot = Alembic::Abc::OObject(_topObj, "mvgRoot");
    _mvgCameras = Alembic::Abc::OObject(_mvgRoot, "mvgCameras");
    _mvgCloud = Alembic::Abc::OObject(_mvgRoot, "mvgCloud");
    _mvgPointCloud = Alembic::Abc::OObject(_mvgCloud, "mvgPointCloud");

    // Add version as custom property
    auto userProps = _mvgRoot.getProperties();
    OUInt32ArrayProperty propAbcVersion(userProps, "mvg_ABC_version");
    OUInt32ArrayProperty propOpenMVGVersion(userProps, "mvg_openMVG_version");
    const std::vector<::uint32_t> abcVersion = {1, 1};
    propAbcVersion.set(abcVersion);
    const std::vector<::uint32_t> openMVGVersion = {OPENMVG_VERSION_MAJOR, OPENMVG_VERSION_MINOR, OPENMVG_VERSION_REVISION};
    propOpenMVGVersion.set(openMVGVersion);
  }
  
  Alembic::Abc::OArchive _archive;
  Alembic::Abc::OObject _topObj;
  Alembic::Abc::OObject _mvgRoot;
  Alembic::Abc::OObject _mvgCameras;
  Alembic::Abc::OObject _mvgCloud;
  Alembic::Abc::OObject _mvgPointCloud;
  Alembic::AbcGeom::OXform _xform;
  Alembic::AbcGeom::OCamera _camObj;
  Alembic::AbcGeom::OUInt32ArrayProperty _propSensorSize_pix;
  Alembic::AbcGeom::OStringProperty _imagePlane;
  Alembic::AbcGeom::OUInt32Property _propViewId;
  Alembic::AbcGeom::OUInt32Property _propIntrinsicId;
  Alembic::AbcGeom::OStringProperty _mvgIntrinsicType;
  Alembic::AbcGeom::ODoubleArrayProperty _mvgIntrinsicParams;
};


AlembicExporter::AlembicExporter(const std::string &filename)
: _data(new DataImpl(filename))
{ }

AlembicExporter::~AlembicExporter()
{
}

void AlembicExporter::addPoints(const sfm::Landmarks &landmarks, bool withVisibility)
{
  if(landmarks.empty())
    return;

  // Fill vector with the values taken from OpenMVG 
  std::vector<V3f> positions;
  std::vector<Imath::C3f> colors;
  std::vector<Alembic::Util::uint32_t> descTypes;
  positions.reserve(landmarks.size());
  descTypes.reserve(landmarks.size());

  // For all the 3d points in the hash_map
  for(const auto landmark : landmarks)
  {
    const openMVG::Vec3& pt = landmark.second.X;
    const openMVG::image::RGBColor& color = landmark.second.rgb;
    positions.emplace_back(pt[0], pt[1], pt[2]);
    colors.emplace_back(color.r()/255.f, color.g()/255.f, color.b()/255.f);
    descTypes.emplace_back(static_cast<Alembic::Util::uint8_t>(landmark.second.descType));
  }

  std::vector<Alembic::Util::uint64_t> ids(positions.size());
  std::iota(begin(ids), end(ids), 0);

  OPoints partsOut(_data->_mvgPointCloud, "particleShape1");
  OPointsSchema &pSchema = partsOut.getSchema();

  OPointsSchema::Sample psamp(std::move(V3fArraySample(positions)), std::move(UInt64ArraySample(ids)));
  pSchema.set(psamp);

  OCompoundProperty arbGeom = pSchema.getArbGeomParams();

  C3fArraySample cval_samp(&colors[0], colors.size());
  OC3fGeomParam::Sample color_samp(cval_samp, kVertexScope);

  OC3fGeomParam rgbOut(arbGeom, "color", false, kVertexScope, 1);
  rgbOut.set(color_samp);

  OCompoundProperty userProps = pSchema.getUserProperties();

  OUInt32ArrayProperty descTypeOut(userProps, "mvg_describerType");
  descTypeOut.set(descTypes);

  if(withVisibility)
  {
    std::vector<::uint32_t> visibilitySize;
    visibilitySize.reserve(positions.size());
    for(const auto landmark : landmarks)
    {
      visibilitySize.emplace_back(landmark.second.observations.size());
    }
    std::size_t nbObservations = std::accumulate(visibilitySize.begin(), visibilitySize.end(), 0);
    
    // Use std::vector<::uint32_t> and std::vector<float> instead of std::vector<V2i> and std::vector<V2f>
    // Because Maya don't import them correctly
    std::vector<::uint32_t> visibilityIds;
    visibilityIds.reserve(nbObservations*2);
    std::vector<float>featPos2d;
    featPos2d.reserve(nbObservations*2);

    for(sfm::Landmarks::const_iterator itLandmark = landmarks.cbegin(), itLandmarkEnd = landmarks.cend();
       itLandmark != itLandmarkEnd; ++itLandmark)
    {
      const sfm::Observations& observations = itLandmark->second.observations;
      for(const auto vObs: observations )
      {
        const sfm::Observation& obs = vObs.second;
        // (View ID, Feature ID)
        visibilityIds.emplace_back(vObs.first);
        visibilityIds.emplace_back(obs.id_feat);
        // Feature 2D position (x, y))
        featPos2d.emplace_back(obs.x[0]);
        featPos2d.emplace_back(obs.x[1]);
      }
    }

    OUInt32ArrayProperty propVisibilitySize( userProps, "mvg_visibilitySize" );
    propVisibilitySize.set(visibilitySize);

    // (viewID, featID)
    OUInt32ArrayProperty propVisibilityIds( userProps, "mvg_visibilityIds" );
    propVisibilityIds.set(visibilityIds);

    // Feature position (x,y)
    OFloatArrayProperty propFeatPos2d( userProps, "mvg_visibilityFeatPos" );
    propFeatPos2d.set(featPos2d);
  }
}


void AlembicExporter::appendCameraRig(IndexT rigId,
                                      IndexT rigPoseId,
                                      const std::vector<View>& views,
                                      const std::vector<std::string>& viewsImagePaths,
                                      const std::vector<cameras::Pinhole_Intrinsic*>& intrinsics,
                                      const geometry::Pose3& rigPose,
                                      const std::vector<RigSubPose> subPoses)
{
  assert(views.size() == viewsImagePaths.size());
  assert(subPoses.size() == intrinsics.size());

  const std::size_t nbSubPoses = views.size();

  // rig pose
  const openMVG::Mat3& R = rigPose.rotation();
  const openMVG::Vec3& center = rigPose.center();

  // Compensate translation with rotation
  // Build transform matrix
  Abc::M44d xformMatrix;
  xformMatrix[0][0] = R(0, 0);
  xformMatrix[0][1] = R(0, 1);
  xformMatrix[0][2] = R(0, 2);
  xformMatrix[1][0] = R(1, 0);
  xformMatrix[1][1] = R(1, 1);
  xformMatrix[1][2] = R(1, 2);
  xformMatrix[2][0] = R(2, 0);
  xformMatrix[2][1] = R(2, 1);
  xformMatrix[2][2] = R(2, 2);
  xformMatrix[3][0] = center(0);
  xformMatrix[3][1] = center(1);
  xformMatrix[3][2] = center(2);
  xformMatrix[3][3] = 1.0;

  XformSample xformsample;
  xformsample.setMatrix(xformMatrix);

  std::stringstream ss;
  ss << std::setfill('0') << std::setw(5) << rigId << "_" << rigPoseId;

  Alembic::AbcGeom::OXform xform(_data->_mvgCameras, "rigxform_" + ss.str());
  xform.getSchema().set(xformsample);

  auto userProps = xform.getSchema().getUserProperties();

  OUInt32Property propRigId(userProps, "mvg_rigId");
  propRigId.set(rigId);

  OUInt32Property propPoseId(userProps, "mvg_poseId");
  propPoseId.set(rigPoseId);

  OUInt16Property propNbSubPoses(userProps, "mvg_nbSubPoses");
  propNbSubPoses.set(nbSubPoses);

  for(std::size_t i = 0; i < nbSubPoses; ++i)
  {
    appendCamera(stlplus::basename_part(viewsImagePaths.at(i)),
                 views.at(i),
                 viewsImagePaths.at(i),
                 intrinsics.at(i),
                 subPoses.at(i).pose,
                 xform);
  }
}

void AlembicExporter::appendCamera(const std::string& cameraName,
                                   const View& view,
                                   const std::string& viewImagePath,
                                   const cameras::Pinhole_Intrinsic* intrinsic,
                                   const geometry::Pose3& pose)
{
  appendCamera(cameraName,
               view,
               viewImagePath,
               intrinsic,
               pose,
               _data->_mvgCameras);
}

void AlembicExporter::appendCamera(const std::string& cameraName,
                                   const View& view,
                                   const std::string& viewImagePath,
                                   const cameras::Pinhole_Intrinsic* intrinsic,
                                   const geometry::Pose3& pose,
                                   Alembic::Abc::OObject& parent)
{
  // Use a common sensor width if we don't have this information.
  // We chose a full frame 24x36 camera
  float sensorWidth_mm = 36.0;

  const sfm::View_Metadata* viewMetadata = dynamic_cast<const sfm::View_Metadata*>(&view);
  if(viewMetadata)
  {
    static const std::string kSensorWidth("sensor_width");
    if(viewMetadata->metadata.find(kSensorWidth) != viewMetadata->metadata.end())
      sensorWidth_mm = std::stof(viewMetadata->metadata.at(kSensorWidth));
  }

  // pose
  const openMVG::Mat3 R = pose.rotation();
  const openMVG::Vec3 center = pose.center();

  // Compensate translation with rotation
  // Build transform matrix
  Abc::M44d xformMatrix;
  xformMatrix[0][0] = R(0, 0);
  xformMatrix[0][1] = R(0, 1);
  xformMatrix[0][2] = R(0, 2);
  xformMatrix[1][0] = R(1, 0);
  xformMatrix[1][1] = R(1, 1);
  xformMatrix[1][2] = R(1, 2);
  xformMatrix[2][0] = R(2, 0);
  xformMatrix[2][1] = R(2, 1);
  xformMatrix[2][2] = R(2, 2);
  xformMatrix[3][0] = center(0);
  xformMatrix[3][1] = center(1);
  xformMatrix[3][2] = center(2);
  xformMatrix[3][3] = 1.0;

  // Correct camera orientation for alembic
  M44d scale;   //  by default this is an identity matrix
  scale[0][0] = 1;
  scale[1][1] = -1;
  scale[2][2] = -1;

  xformMatrix = scale * xformMatrix;

  XformSample xformsample;
  xformsample.setMatrix(xformMatrix);

  std::stringstream ss;
  ss << std::setfill('0') << std::setw(5) << view.getPoseId();
  ss << "_" << cameraName << "_" << view.getViewId();

  Alembic::AbcGeom::OXform xform(parent, "camxform_" + ss.str());
  xform.getSchema().set(xformsample);

  // Camera intrinsic parameters
  OCamera camObj(xform, "camera_" + ss.str());
  auto userProps = camObj.getSchema().getUserProperties();
  CameraSample camSample;

  // Take the max of the image size to handle the case where the image is in portrait mode 
  const float imgWidth = intrinsic->w();
  const float imgHeight = intrinsic->h();
  const float sensorWidth_pix = std::max(imgWidth, imgHeight);
  const float sensorHeight_pix = std::min(imgWidth, imgHeight);
  //const float imgRatio = sensorHeight_pix / sensorWidth_pix;
  const float focalLength_pix = intrinsic->focal();
  //const float sensorHeight_mm = sensorWidth_mm * imgRatio;
  const float focalLength_mm = sensorWidth_mm * focalLength_pix / sensorWidth_pix;
  const float pix2mm = sensorWidth_mm / sensorWidth_pix;

  // openMVG: origin is (top,left) corner and orientation is (bottom,right)
  // ABC: origin is centered and orientation is (up,right)
  // Following values are in cm, hence the 0.1 multiplier
  const float haperture_cm = 0.1 * imgWidth * pix2mm;
  const float vaperture_cm = 0.1 * imgHeight * pix2mm;

  camSample.setFocalLength(focalLength_mm);
  camSample.setHorizontalAperture(haperture_cm);
  camSample.setVerticalAperture(vaperture_cm);
  
  // Add sensor width (largest image side) in pixels as custom property
  OUInt32ArrayProperty propSensorSize_pix(userProps, "mvg_sensorSizePix");
  std::vector<::uint32_t> sensorSize_pix = {::uint32_t(sensorWidth_pix), ::uint32_t(sensorHeight_pix)};
  propSensorSize_pix.set(sensorSize_pix);

  // Add viewImagePath as custom property
  if(!viewImagePath.empty())
  {
    // Set camera image plane 
    OStringProperty imagePlane(userProps, "mvg_imagePath");
    imagePlane.set(viewImagePath.c_str());
  }

  OUInt32Property propViewId(userProps, "mvg_viewId");
  propViewId.set(view.getViewId());

  OUInt32Property propPoseId(userProps, "mvg_poseId");
  propPoseId.set(view.getPoseId());

  OUInt32Property propIntrinsicId(userProps, "mvg_intrinsicId");
  propIntrinsicId.set(view.getIntrinsicId());
  
  OStringProperty mvg_intrinsicType(userProps, "mvg_intrinsicType");
  mvg_intrinsicType.set(intrinsic->getTypeStr());
  
  std::vector<double> intrinsicParams = intrinsic->getParams();
  ODoubleArrayProperty mvg_intrinsicParams(userProps, "mvg_intrinsicParams");
  mvg_intrinsicParams.set(intrinsicParams);

  OUInt32Property propRigId(userProps, "mvg_rigId");
  propRigId.set(view.getRigId());
  
  OUInt32Property propSubPoseId(userProps, "mvg_subPoseId");
  propSubPoseId.set(view.getSubPoseId());

  camObj.getSchema().set(camSample);
}

void AlembicExporter::initAnimatedCamera(const std::string& cameraName)
{
  // Sample the time in order to have one keyframe every frame
  // nb: it HAS TO be attached to EACH keyframed properties
  TimeSamplingPtr tsp( new TimeSampling(1.0 / 24.0, 1.0 / 24.0) );
  
  // Create the camera transform object
  std::stringstream ss;
  ss << cameraName;
  _data->_xform = Alembic::AbcGeom::OXform(_data->_mvgCameras, "animxform_" + ss.str());
  _data->_xform.getSchema().setTimeSampling(tsp);
  
  // Create the camera parameters object (intrinsics & custom properties)
  _data->_camObj = OCamera(_data->_xform, "animcam_" + ss.str());
  _data->_camObj.getSchema().setTimeSampling(tsp);
  
  // Add the custom properties
  auto userProps = _data->_camObj.getSchema().getUserProperties();
  // Sensor size
  _data->_propSensorSize_pix = OUInt32ArrayProperty(userProps, "mvg_sensorSizePix", tsp);
  // Image path
  _data->_imagePlane = OStringProperty(userProps, "mvg_imagePath", tsp);
  // View id
  _data->_propViewId = OUInt32Property(userProps, "mvg_viewId", tsp);
  // Intrinsic id
  _data->_propIntrinsicId = OUInt32Property(userProps, "mvg_intrinsicId", tsp);
  // Intrinsic type (ex: PINHOLE_CAMERA_RADIAL3)
  _data->_mvgIntrinsicType = OStringProperty(userProps, "mvg_intrinsicType", tsp);
  // Intrinsic parameters
  _data->_mvgIntrinsicParams = ODoubleArrayProperty(userProps, "mvg_intrinsicParams", tsp);
}

void AlembicExporter::addCameraKeyframe(const geometry::Pose3 &pose,
                                          const cameras::Pinhole_Intrinsic *cam,
                                          const std::string &imagePath,
                                          const IndexT id_view,
                                          const IndexT id_intrinsic,
                                          const float sensorWidth_mm)
{
  const openMVG::Mat3 R = pose.rotation();
  const openMVG::Vec3 center = pose.center();
  // POSE
  // Compensate translation with rotation
  // Build transform matrix
  Abc::M44d xformMatrix;
  xformMatrix[0][0] = R(0, 0);
  xformMatrix[0][1] = R(0, 1);
  xformMatrix[0][2] = R(0, 2);
  xformMatrix[1][0] = R(1, 0);
  xformMatrix[1][1] = R(1, 1);
  xformMatrix[1][2] = R(1, 2);
  xformMatrix[2][0] = R(2, 0);
  xformMatrix[2][1] = R(2, 1);
  xformMatrix[2][2] = R(2, 2);
  xformMatrix[3][0] = center(0);
  xformMatrix[3][1] = center(1);
  xformMatrix[3][2] = center(2);
  xformMatrix[3][3] = 1.0;

  // Correct camera orientation for alembic
  M44d scale;
  scale[0][0] = 1;
  scale[1][1] = -1;
  scale[2][2] = -1;
  xformMatrix = scale*xformMatrix;

  // Create the XformSample
  XformSample xformsample;
  xformsample.setMatrix(xformMatrix);
  
  // Attach it to the schema of the OXform
  _data->_xform.getSchema().set(xformsample);
  
  // Camera intrinsic parameters
  CameraSample camSample;

  // Take the max of the image size to handle the case where the image is in portrait mode 
  const float imgWidth = cam->w();
  const float imgHeight = cam->h();
  const float sensorWidth_pix = std::max(imgWidth, imgHeight);
  const float sensorHeight_pix = std::min(imgWidth, imgHeight);
  //const float imgRatio = sensorHeight_pix / sensorWidth_pix;
  const float focalLength_pix = cam->focal();
  //const float sensorHeight_mm = sensorWidth_mm * imgRatio;
  const float focalLength_mm = sensorWidth_mm * focalLength_pix / sensorWidth_pix;
  const float pix2mm = sensorWidth_mm / sensorWidth_pix;

  // openMVG: origin is (top,left) corner and orientation is (bottom,right)
  // ABC: origin is centered and orientation is (up,right)
  // Following values are in cm, hence the 0.1 multiplier
  const float haperture_cm = 0.1 * imgWidth * pix2mm;
  const float vaperture_cm = 0.1 * imgHeight * pix2mm;

  camSample.setFocalLength(focalLength_mm);
  camSample.setHorizontalAperture(haperture_cm);
  camSample.setVerticalAperture(vaperture_cm);
  
  // Add sensor width (largest image side) in pixels as custom property
  std::vector<::uint32_t> sensorSize_pix = {::uint32_t(sensorWidth_pix), ::uint32_t(sensorHeight_pix)};
  _data->_propSensorSize_pix.set(sensorSize_pix);
  
  // Set custom attributes
  // Image path
  _data->_imagePlane.set(imagePath);

  // View id
  _data->_propViewId.set(id_view);
  // Intrinsic id
  _data->_propIntrinsicId.set(id_intrinsic);
  // Intrinsic type
  _data->_mvgIntrinsicType.set(cam->getTypeStr());
  // Intrinsic parameters
  std::vector<double> intrinsicParams = cam->getParams();
  _data->_mvgIntrinsicParams.set(intrinsicParams);
  
  // Attach intrinsic parameters to camera object
  _data->_camObj.getSchema().set(camSample);
}

void AlembicExporter::jumpKeyframe(const std::string &imagePath)
{
  if(_data->_xform.getSchema().getNumSamples() == 0)
  {
    cameras::Pinhole_Intrinsic default_intrinsic;
    this->addCameraKeyframe(geometry::Pose3(), &default_intrinsic, imagePath, 0, 0);
  }
  else
  {
    _data->_xform.getSchema().setFromPrevious();
    _data->_camObj.getSchema().setFromPrevious();
  }
}

void AlembicExporter::add(const sfm::SfM_Data& sfmData, sfm::ESfM_Data flags_part)
{
  auto userProps = _data->_mvgRoot.getProperties();

  OStringProperty propFeatureFolder(userProps, "mvg_featureFolder");
  propFeatureFolder.set(sfmData.getFeatureFolder());

  OStringProperty propMatchingFolder(userProps, "mvg_matchingFolder");
  propMatchingFolder.set(sfmData.getMatchingFolder());

  if(flags_part & sfm::ESfM_Data::VIEWS || flags_part & sfm::ESfM_Data::EXTRINSICS)
  {
    std::map<IndexT, std::map<IndexT, std::vector<View>>> viewRigs;

    for(const auto it : sfmData.GetViews())
    {
      const sfm::View* view = it.second.get();
      openMVG::cameras::Pinhole_Intrinsic* intrinsic = nullptr;
      geometry::Pose3 pose;

      if(sfmData.IsPoseAndIntrinsicDefined(view))
      {
        // Rig
        if(view->isPartOfRig())
        {
          const Rig& rig = sfmData.getRig(*view);

          auto& currViewRig = viewRigs[view->getRigId()];

          if(currViewRig.find(view->getPoseId()) == currViewRig.end())
          {
            currViewRig[view->getPoseId()].resize(rig.getNbSubPoses());
          }

          currViewRig[view->getPoseId()][view->getSubPoseId()] = *view;

          continue;
        }

        // OpenMVG single Camera
        intrinsic = dynamic_cast<openMVG::cameras::Pinhole_Intrinsic*>(sfmData.GetIntrinsics().at(view->getIntrinsicId()).get());
        pose = sfmData.getPose(*view);
      }
      else
      {
        // If there is no intrinsic/pose defined, skip camera
        continue;
      }

      const std::string viewImagePath = stlplus::create_filespec(sfmData.s_root_path, view->s_Img_path);
      
      // TODO: store full metadata
      appendCamera(stlplus::basename_part(view->s_Img_path),
                   *view,
                   viewImagePath,
                   intrinsic,
                   pose);
    }

    for(const auto rigIt : viewRigs)
    {
      const IndexT rigId = rigIt.first;

      for(const auto& rigPoseIt : rigIt.second)
      {
        const std::vector<View> views = rigPoseIt.second;
        const Rig& rig = sfmData.getRig(views.front());
        const IndexT rigPoseId = rigPoseIt.first;

        std::vector<std::string> viewsImagePaths;
        std::vector<cameras::Pinhole_Intrinsic*> intrinsics;

        for(std::size_t i = 0; i < rig.getNbSubPoses(); ++i)
        {
          const View& view = views.at(i);

          viewsImagePaths.push_back(stlplus::create_filespec(sfmData.s_root_path, view.s_Img_path));
          intrinsics.push_back(dynamic_cast<openMVG::cameras::Pinhole_Intrinsic*>(sfmData.GetIntrinsics().at(view.getIntrinsicId()).get()));
        }

         appendCameraRig(rigId,
                         rigPoseId,
                         views,
                         viewsImagePaths,
                         intrinsics,
                         sfmData.getPose(views.front()),
                         rig.getSubPoses());
      }
    }
  }
  if(flags_part & sfm::ESfM_Data::STRUCTURE)
  {
    addPoints(sfmData.GetLandmarks(), (flags_part & sfm::ESfM_Data::OBSERVATIONS));
  }
}


std::string AlembicExporter::getFilename()
{
  return _data->_archive.getName();
}

} //namespace sfm
} //namespace openMVG
