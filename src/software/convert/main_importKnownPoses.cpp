// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/utils/regexFilter.hpp>

#include <boost/program_options.hpp>
#include <boost/system/error_code.hpp>
#include <boost/filesystem.hpp>

#include <algorithm>
#include <string>
#include <regex>

#include <iostream>
#include <list>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace json = boost::property_tree;
namespace fs = boost::filesystem;


struct XMPData
{
    std::vector<double> rotation;
    std::vector<double> position;
    std::vector<double> distortionCoefficients;
    std::string distortionModel;
    double focalLength35mm = 0.0;
    int skew = 0;
    double aspectRatio = 1.0;
    double principalPointU = 0.0;
    double principalPointV = 0.0;
    std::string calibrationPrior = "DEFAULT";
    int calibrationGroup = 0;
    int distortionGroup = 0;
    int inTexturing = 0;
    int inMeshing = 0;
};


XMPData read_xmp(const std::string& xmpFilepath, std::string knownPosesFilePath, std::string stem, fs::directory_entry pathIt)
{
    XMPData xmp;
    const fs::path path = pathIt.path();
    if(!is_regular_file(path))
        ALICEVISION_THROW_ERROR("Path isn't a regulat file: " << path);
    std::string extension = path.extension().string();
    boost::to_lower(extension);
    if(extension != ".xmp")
    {
       ALICEVISION_THROW_ERROR("Unknown extension: " << extension);
    }

    json::ptree tree;
    read_xml(knownPosesFilePath + '/' + stem + ".xmp", tree);

    xmp.distortionModel = tree.get<std::string>("x:xmpmeta.rdf:RDF.rdf:Description.<xmlattr>.xcr:DistortionModel", "DEFAULT");
    xmp.focalLength35mm = tree.get<double>("x:xmpmeta.rdf:RDF.rdf:Description.<xmlattr>.xcr:FocalLength35mm", 0.0);
    xmp.skew = tree.get<int>("x:xmpmeta.rdf:RDF.rdf:Description.<xmlattr>.xcr:Skew", 0);
    xmp.aspectRatio = tree.get<double>("x:xmpmeta.rdf:RDF.rdf:Description.<xmlattr>.xcr:AspectRatio", 1.0);
    xmp.principalPointU = tree.get<float>("x:xmpmeta.rdf:RDF.rdf:Description.<xmlattr>.xcr:PrincipalPointU", 0);
    xmp.principalPointV = tree.get<float>("x:xmpmeta.rdf:RDF.rdf:Description.<xmlattr>.xcr:PrincipalPointV", 0);
    xmp.calibrationPrior = tree.get<std::string>("x:xmpmeta.rdf:RDF.rdf:Description.<xmlattr>.xcr:CalibrationPrior", "DEFAULT");
    xmp.calibrationGroup = tree.get<int>("x:xmpmeta.rdf:RDF.rdf:Description.<xmlattr>.xcr:CalibrationGroup", 0);
    xmp.distortionGroup = tree.get<int>("x:xmpmeta.rdf:RDF.rdf:Description.<xmlattr>.xcr:DistortionGroup", 0);
    xmp.inTexturing = tree.get<int>("x:xmpmeta.rdf:RDF.rdf:Description.<xmlattr>.xcr:InTexturing", 0);
    xmp.inMeshing = tree.get<int>("x:xmpmeta.rdf:RDF.rdf:Description.<xmlattr>.xcr:InMeshing", 0);

    std::string rotationStr = tree.get<std::string>("x:xmpmeta.rdf:RDF.rdf:Description.xcr:Rotation", "");
    std::vector<std::string> rotationStrings;
    boost::split(rotationStrings, rotationStr, boost::is_any_of(" \t"), boost::token_compress_on);
    ALICEVISION_LOG_TRACE("stem: " << stem);
    ALICEVISION_LOG_TRACE("rotation: " << rotationStrings);

    for(std::string rot_val : rotationStrings)
    {
        xmp.rotation.push_back(std::stod(rot_val));
    }

    std::string positionStr = tree.get<std::string>("x:xmpmeta.rdf:RDF.rdf:Description.xcr:Position", "");
    std::vector<std::string> positionStrings;
    if(!positionStr.empty())
    {
        boost::split(positionStrings, positionStr, boost::is_any_of(" \t"), boost::token_compress_on);
        ALICEVISION_LOG_TRACE("position: " << positionStrings);
    }
    else
    {
        positionStr = tree.get<std::string>("x:xmpmeta.rdf:RDF.rdf:Description.<xmlattr>.xcr:Position", "");
        boost::split(positionStrings, positionStr, boost::is_any_of(" \t"), boost::token_compress_on);
    }

    for(std::string pos_val : positionStrings)
    {
        xmp.position.push_back(std::stod(pos_val));
    }
    std::string distortionStr =
        tree.get<std::string>("x:xmpmeta.rdf:RDF.rdf:Description.xcr:DistortionCoeficients", "");
    std::vector<std::string> distortionStrings;
    boost::split(distortionStrings, distortionStr, boost::is_any_of(" \t"), boost::token_compress_on);
    ALICEVISION_LOG_TRACE("distortion: " << distortionStrings);

    for(std::string disto_val : distortionStrings)
    {
        xmp.distortionCoefficients.push_back(std::stod(disto_val));
    }
    return xmp;
}

// import from a SfMData format to another
int aliceVision_main(int argc, char **argv)
{
  // command-line parameters
  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string knownPosesFilePath;
  std::string sfmDataFilePath;
  std::string outputFilename;

  sfmData::SfMData sfmData;

  po::options_description allParams("AliceVision importKnownPoses");

  // enter the parameter
  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
      ("knownPosesData", po::value<std::string>(&knownPosesFilePath)->required(), "Input path to a json file or a folder containing an XMP file per image.")
      ("sfmData", po::value<std::string>(&sfmDataFilePath)->required(), "SfmData filepath.")
      ("output,o", po::value<std::string>(&outputFilename)->required(), "Output sfmData filepath.");

  po::options_description logParams("Log parameters");
  logParams.add_options()("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
                          "verbosity level (fatal,  error, warning, info, debug, trace).");

  allParams.add(requiredParams).add(logParams);

  po::variables_map vm;

  try
  {
      po::store(po::parse_command_line(argc, argv, allParams), vm);

      if(vm.count("help") || (argc == 1))
      {
          ALICEVISION_COUT(allParams);
          return EXIT_SUCCESS;
      }

      po::notify(vm);
  }
  catch(boost::program_options::required_option& e)
  {
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << allParams);
      return EXIT_FAILURE;
  }
  catch(boost::program_options::error& e)
  {
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << allParams);
      return EXIT_FAILURE;
  }

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  // Loading the sfmData to modify it
  if(!sfmDataIO::Load(sfmData, sfmDataFilePath, sfmDataIO::ESfMData::ALL))
  {
      ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilePath << "' cannot be read.");
      return EXIT_FAILURE;
  }
  if(sfmData.getViews().empty())
  {
      ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilePath << "' is empty.");
      return EXIT_FAILURE;
  }

  std::map<std::string, IndexT> viewIdPerStem;
  for(const auto viewIt : sfmData.getViews())
  {
    const std::string stem = fs::path(viewIt.second->getImagePath()).stem().string();
    viewIdPerStem[stem] = viewIt.first;
  }
  fs::path knownPosesPath(knownPosesFilePath);
  if(fs::is_directory(knownPosesPath))
  {
      try
      {
          for (const auto& pathIt : fs::directory_iterator(knownPosesPath))
          {
              const std::string stem = pathIt.path().stem().string();
              if (viewIdPerStem.count(stem) == 0) 
              {
                  continue;
              }

              const XMPData xmp = read_xmp(pathIt.path().string(), knownPosesFilePath, stem, pathIt);

              const IndexT viewId = viewIdPerStem[stem];
              sfmData::View& view = sfmData.getView(viewId);
              sfmData::CameraPose& pose = sfmData.getPoses()[view.getPoseId()];

              std::shared_ptr<camera::IntrinsicBase> intrinsicBase = sfmData.getIntrinsicsharedPtr(view.getIntrinsicId());

              Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rot(xmp.rotation.data());

              Vec3 pos_vec(xmp.position[0], xmp.position[1], xmp.position[2]);
              Vec3 translation = - rot * pos_vec;

              Eigen::Matrix4d T;
              T.setIdentity();
              T.block<3, 3>(0, 0) = rot;
              T.block<3, 1>(0, 3) = translation;

              Eigen::Matrix4d av_T_cr = Eigen::Matrix4d::Zero();
              av_T_cr(0, 0) = 1.0;
              av_T_cr(1, 2) = -1.0;
              av_T_cr(2, 1) = 1.0;
              av_T_cr(3, 3) = 1.0;

              T = T * av_T_cr;
              Eigen::Matrix3d R = T.block<3, 3>(0, 0);
              translation = T.block<3, 1>(0, 3);
              pos_vec = -R.transpose() * translation;

              geometry::Pose3 pos3(R, pos_vec);
              pose.setTransform(pos3);

              std::shared_ptr<camera::IntrinsicsScaleOffsetDisto> intrinsic = std::dynamic_pointer_cast<camera::IntrinsicsScaleOffsetDisto>(intrinsicBase);
              if (intrinsic == nullptr)
              {
                  ALICEVISION_THROW_ERROR("Invalid intrinsic");
              }

              const double imageRatio = static_cast<double>(view.getWidth()) / static_cast<double>(view.getHeight());
              const double sensorWidth = intrinsic->sensorWidth();
              const double maxSize = std::max(view.getWidth(), view.getHeight());
              const double focalLengthmm = (sensorWidth * xmp.focalLength35mm) / 36.0;
              const double focalLengthPix = maxSize * focalLengthmm / sensorWidth;
              const double offsetX = (double(view.getWidth()) * 0.5) + (xmp.principalPointU *  maxSize);
              const double offsetY = (double(view.getHeight()) * 0.5) + (xmp.principalPointV *  maxSize);

              intrinsic->setScale(focalLengthPix, focalLengthPix);
              intrinsic->setOffset(offsetX, offsetY);

              std::cout << focalLengthPix << std::endl;

              if(xmp.distortionModel == "brown3t2")
              {
                std::shared_ptr<camera::PinholeBrownT2> camera = std::dynamic_pointer_cast<camera::PinholeBrownT2>(intrinsic);
                if (camera == nullptr)
                {
                    camera = std::make_shared<camera::PinholeBrownT2>();
                    camera->copyFrom(*intrinsic);
                    sfmData.getIntrinsics().at(view.getIntrinsicId()) = camera;
                }

                if(xmp.distortionCoefficients.size() == 6)
                {
                    std::vector<double> distortionCoefficients;

                    distortionCoefficients.push_back(xmp.distortionCoefficients[0]);
                    distortionCoefficients.push_back(xmp.distortionCoefficients[1]);
                    distortionCoefficients.push_back(xmp.distortionCoefficients[2]);
                    // Skip element at index 3 as it is empty
                    distortionCoefficients.push_back(xmp.distortionCoefficients[5]);
                    distortionCoefficients.push_back(xmp.distortionCoefficients[4]);
                    camera->setDistortionParams(distortionCoefficients); // vector of 5 elements (r1, r2, r3, t1, t2)
                }
                else
                {
                    ALICEVISION_THROW_ERROR("Error in xmp file: " << stem << " the distortion coefficient doesn't have the right size.");
                }
              }
              else if(xmp.distortionModel == "brown3")
              {
                std::shared_ptr<camera::PinholeBrownT2> camera = std::dynamic_pointer_cast<camera::PinholeBrownT2>(intrinsic);
                if (camera == nullptr)
                {
                    camera = std::make_shared<camera::PinholeBrownT2>();
                    camera->copyFrom(*intrinsic);
                    sfmData.getIntrinsics().at(view.getIntrinsicId()) = camera;
                }

                if(xmp.distortionCoefficients.size() == 3)
                {
                    std::vector<double> distortionCoefficients = xmp.distortionCoefficients;
                    camera->setDistortionParams(distortionCoefficients); // vector of 5 elements (r1, r2, r3)
                }
                else
                {
                    ALICEVISION_THROW_ERROR("Error in xmp file: " << stem << " the distortion coefficient doesn't have the right size.");
                }
              }
              else
              {
                  ALICEVISION_THROW_ERROR("Unsupported distortion model: " << xmp.distortionModel);
              }
          }
      }
      catch(boost::program_options::error& e)
      {
          ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
          return EXIT_FAILURE;
      }

  }
  else if(is_regular_file(knownPosesPath))
  {
      std::string extension = knownPosesPath.extension().string();
      boost::to_lower(extension);
      if(extension == ".json")
      {
          std::ifstream jsonFile(knownPosesFilePath);
          if(!jsonFile)
          {
              ALICEVISION_LOG_ERROR("Error opening file: " << knownPosesFilePath);
              return EXIT_FAILURE;
          }

          std::string line;
          size_t count = 0;
          std::vector<std::pair<size_t, int>> records;
          std::vector<std::pair<IndexT, IndexT>> frameIdToPoseId;

          // Here we are making a vector that associate a frameId to a PoseId so we can access each easier
          for(const auto& view : sfmData.getViews())
          {
              frameIdToPoseId.emplace_back(view.second->getFrameId(), view.second->getPoseId());
          }
          std::sort(frameIdToPoseId.begin(), frameIdToPoseId.end());

          // ensure there is no duplicated frameId
          auto it = std::adjacent_find(frameIdToPoseId.begin(), frameIdToPoseId.end(),
                                       [](const auto& a, const auto& b) { return a.first == b.first; });
          if(it != frameIdToPoseId.end())
          {
              ALICEVISION_THROW_ERROR("Duplicated frameId in sfmData: " << sfmDataFilePath << ", frameID: " << it->first);
          }
          // This is where we start to read our json line by line
          while(getline(jsonFile, line))
          {
              std::stringstream linestream(line);
              std::vector<double> up;
              std::vector<double> forward;
              std::vector<double> pose;
              json::ptree pt;

              // We put each line in a stringstream because that is what boost's parser needs.
              // The parser turns the json into a property tree.
              json::json_parser::read_json(linestream, pt);
              const int sensor = pt.get<int>("sensorwidth", 0);
              const double fov = pt.get<double>("xFovDegrees", 0);
              const long timestamp = pt.get<long>("tstamp", 0);
              const int frmcnt = pt.get<int>("frmcnt", 0);
              const double expoff = pt.get<double>("exposureOff", 0);
              const double expdur = pt.get<double>("exposureDur", 0);
              // These arguments are lists so we need to loop to store them properly
              for(json::ptree::value_type& up_val : pt.get_child("up"))
              {
                  std::string value = up_val.second.data();
                  up.push_back(std::stof(value));
              }
              for(json::ptree::value_type& for_val : pt.get_child("forward"))
              {
                  std::string value = for_val.second.data();
                  forward.push_back(std::stof(value));
              }
              for(json::ptree::value_type& pose_val : pt.get_child("pose"))
              {
                  std::string value = pose_val.second.data();
                  pose.push_back(std::stof(value));
              }
              // We use records to indexify our frame count this way we know which frame started the list, this will be our offset
              records.emplace_back(count, frmcnt);

              // Without surprise we store our vector pose to get our position
              Vec3 pos_vec(pose[0], pose[1], pose[2]);
              Mat3 rot;
              // And we need those two vectors to calculate the rotation matrix
              Vec3 up_vec(up[0], up[1], up[2]);
              Vec3 forward_vec(forward[0], forward[1], forward[2]);

              rot.row(0) = up_vec.cross(forward_vec);
              rot.row(1) = -up_vec;
              rot.row(2) = forward_vec;
              // we store this new information into a pose3
              geometry::Pose3 pos3(rot, pos_vec);

              // And we set this pose into the sfmData using our frameId (which corresponds to the count) to set a new pos3 transform
              IndexT sfmPoseId = frameIdToPoseId[count].second;
              sfmData.getPoses()[sfmPoseId].setTransform(pos3);
              count++;
              // We repeat this to each line of the file which contains a json
          }
      }
      else if(extension == ".ma")
      {
          std::ifstream file(knownPosesPath.string());

          std::string line;
          std::string name;
          bool hasName = false;
          bool hasPosition = false;
          bool hasRotation = false;
          double pos[3] = {0.0, 0.0, 0.0};
          double rot[3] = {0.0, 0.0, 0.0};

          while (std::getline(file, line))
          {
            std::regex regex("[^\\s\\t;]+");
            std::vector<std::string> words;

            for (auto it = std::sregex_iterator(line.begin(), line.end(), regex); it != std::sregex_iterator(); it++)
            {
                std::string tok = it->str();
                tok.erase(std::remove(tok.begin(), tok.end(), '\"'), tok.end());
                words.push_back(tok);
            }

            if (words.empty())
                continue;

            if (words[0] == "createNode")
            {
                if (words.size() == 4)
                {
                    name = words[3];
                    hasName = true;
                    hasPosition = false;
                    hasRotation = false;
                }
            }

            if (words[0] == "setAttr")
            {
                if (words[1] == ".translate")
                {
                    if (hasName && (!hasPosition))
                    {
                        hasPosition = true;
                        pos[0] = std::stod(words[4]);
                        pos[1] = std::stod(words[5]);
                        pos[2] = std::stod(words[6]);
                    }
                }

                if (words[1] == ".rotate")
                {
                    if (hasName && (!hasRotation))
                    {
                        hasRotation = true;
                        rot[0] = std::stod(words[4]);
                        rot[1] = std::stod(words[5]);
                        rot[2] = std::stod(words[6]);
                    }
                }
            }

            if (hasName && hasRotation && hasPosition)
            {
                if (viewIdPerStem.count(name) == 0) 
                {
                    continue;
                }

                const IndexT viewId = viewIdPerStem[name];
                sfmData::View& view = sfmData.getView(viewId);
                sfmData::CameraPose& pose = sfmData.getPoses()[view.getPoseId()];

                Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
                const Eigen::AngleAxis<double> MX(degreeToRadian(rot[0]), Eigen::Vector3d::UnitX());
                const Eigen::AngleAxis<double> MY(degreeToRadian(rot[1]), Eigen::Vector3d::UnitY());
                const Eigen::AngleAxis<double> MZ(degreeToRadian(rot[2]), Eigen::Vector3d::UnitZ());
                R = MZ * MY * MX;
                
                Eigen::Vector3d position;
                position(0) = pos[0];
                position(1) = pos[1];
                position(2) = pos[2];

                Vec3 translation = - R * position;

                Eigen::Matrix3d alice_R_maya = Eigen::Matrix3d::Identity();

                alice_R_maya(0, 0) = 1.0;
                alice_R_maya(1, 1) = -1.0;
                alice_R_maya(2, 2) = -1.0;
                position = position;

                R =  R * alice_R_maya;

                geometry::Pose3 pose3(R.transpose(), position);
                pose.setTransform(pose3);
                ALICEVISION_LOG_TRACE("Read maya: " << name);

                hasName = false;
                hasRotation = false;
                hasPosition = false;
            }
          }
      }
  }

  // export the SfMData scene in the expected format
  if(!sfmDataIO::Save(sfmData, outputFilename, sfmDataIO::ESfMData::ALL))
  {
      ALICEVISION_LOG_ERROR("An error occured while trying to save '" << outputFilename << "'");
      return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
