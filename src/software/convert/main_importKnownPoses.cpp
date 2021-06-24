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
    double focalLength = 0.0;
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
        ALICEVISION_THROW_ERROR("Unknown extension: " << extension);

    json::ptree tree;
    read_xml(knownPosesFilePath + '/' + stem + ".xmp", tree);

    xmp.distortionModel =
        tree.get<std::string>("x:xmpmeta.rdf:RDF.rdf:Description.<xmlattr>.xcr:DistortionModel", "DEFAULT");
    xmp.focalLength = tree.get<double>("x:xmpmeta.rdf:RDF.rdf:Description.<xmlattr>.xcr:FocalLength", 0.0);
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

  if(fs::is_directory(knownPosesFilePath))
  {
      try
      {
          std::map<std::string, IndexT> viewIdPerStem;
          for(const auto viewIt : sfmData.getViews())
          {
              const std::string stem = fs::path(viewIt.second->getImagePath()).stem().string();
              viewIdPerStem[stem] = viewIt.first;
          }

          for(const auto& pathIt : fs::directory_iterator(knownPosesFilePath))
          {
              const std::string stem = pathIt.path().stem().string();
              const XMPData xmp = read_xmp(pathIt.path().string(), knownPosesFilePath, stem, pathIt);

              const IndexT viewId = viewIdPerStem[stem];
              aliceVision::sfmData::View& view = sfmData.getView(viewId);
              aliceVision::sfmData::CameraPose& pose = sfmData.getPoses()[view.getPoseId()];

              camera::IntrinsicBase* intrinsicBase = sfmData.getIntrinsicPtr(view.getIntrinsicId());

              Mat3 rot;
              Vec3 row_one(xmp.rotation[0], xmp.rotation[1], xmp.rotation[2]);
              Vec3 row_two(xmp.rotation[3], xmp.rotation[4], xmp.rotation[5]);
              Vec3 row_three(xmp.rotation[6], xmp.rotation[7], xmp.rotation[8]);
              rot.row(0) = row_one;
              rot.row(1) = row_two;
              rot.row(2) = row_three;
              Vec3 pos_vec(xmp.position[0], xmp.position[1], xmp.position[2]);
              
              aliceVision::geometry::Pose3 pos3(rot, pos_vec);
              pose.setTransform(pos3);

              aliceVision::camera::IntrinsicsScaleOffsetDisto* intrinsic =
                  dynamic_cast<aliceVision::camera::IntrinsicsScaleOffsetDisto*>(intrinsicBase);

              if(xmp.distortionModel == "brown3t2")
              {
                  if(intrinsic->getType() != camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL3)//camera::EINTRINSIC::PINHOLE_CAMERA_BROWN)
                  {
                      ALICEVISION_THROW_ERROR("Error in: " << stem << "'s instinsics...");
                  }
                  else
                  {
                      if(xmp.distortionCoefficients.size() == 6)
                      {
                          // Element 4 is useless and needs to be ignored.
                          std::vector<double> distortionCoefficients = xmp.distortionCoefficients;
                          distortionCoefficients.erase(distortionCoefficients.begin() + 5);
                          distortionCoefficients.erase(distortionCoefficients.begin() + 4);
                          distortionCoefficients.erase(distortionCoefficients.begin() + 3);

                          // IntrinsicsScaleOffsetDisto::setDistortionParams: wrong number of distortion parameters (expected: 3, given:5).
                          intrinsic->setDistortionParams(
                              distortionCoefficients); // vector of 5 elements (r1, r2, r3, t1, t2)
                      }
                      else
                          ALICEVISION_THROW_ERROR(
                              "Error in xmp file: " << stem << " the distortion coefficient doesn't have the right size.");
                  }
              }
              else if(xmp.distortionModel == "brown3")
              {
                  if(intrinsic->getType() != camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL3)
                  {
                      ALICEVISION_THROW_ERROR("Error in: " << stem << "'s instinsics.");
                  }
                  else
                  {
                      if(xmp.distortionCoefficients.size() == 3)
                      {
                          intrinsic->setDistortionParams(
                              xmp.distortionCoefficients); // vector of 3 elements (r1, r2, r3)
                      }
                      else
                          ALICEVISION_THROW_ERROR(
                              "Error in xmp file: " << stem << " the distortion coefficient doesn't have the right size.");
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
      // export the SfMData scene in the expected format
      if(!sfmDataIO::Save(sfmData, outputFilename, sfmDataIO::ESfMData::ALL))
      {
          ALICEVISION_LOG_ERROR("An error occured while trying to save '" << outputFilename << "'");
          return EXIT_FAILURE;
      }
      return EXIT_SUCCESS;
  }
  else if(is_regular_file(fs::path(knownPosesFilePath)))
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
        try
        {
            // This is where we start to read our json line by line
            while(getline(jsonFile, line))
            {
                std::stringstream linestream(line);
                int sensor;
                float fov;
                long timestamp;
                int frmcnt;
                float expoff;
                float expdur;
                std::vector<float> up;
                std::vector<float> forward;
                std::vector<float> pose;
                json::ptree pt;

                // We put each line in a stringstream because that is what boost's parser needs
                // THe parser turns the json into a property tree in which we access the informations and store them
                json::json_parser::read_json(linestream, pt);
                sensor = pt.get<int>("sensorwidth", 0);
                fov = pt.get<float>("xFovDegrees", 0);
                timestamp = pt.get<long>("tstamp", 0);
                frmcnt = pt.get<int>("frmcnt", 0);
                expoff = pt.get<float>("exposureOff", 0);
                expdur = pt.get<float>("exposureDur", 0);
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
                aliceVision::geometry::Pose3 pos3(rot, pos_vec);

                // And we set this pose into the sfmData using our frameId (which corresponds to the count) to set a new pos3 transform
                IndexT sfmPoseId = frameIdToPoseId[count].second;
                sfmData.getPoses()[sfmPoseId].setTransform(pos3);
                count++;
                // We repeat this to each line of the file which contains a json
            }
        }
        catch(boost::program_options::error& e)
        {
            ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
            return EXIT_FAILURE;
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
