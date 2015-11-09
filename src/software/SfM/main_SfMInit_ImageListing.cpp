// Copyright (c) 2012, 2013, 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
#include "openMVG/exif/exif_IO_EasyExif.hpp"

#include "openMVG/exif/sensor_width_database/ParseDatabase.hpp"

#include "openMVG/image/image.hpp"
#include "openMVG/stl/split.hpp"

#include "openMVG/sfm/sfm.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::exif;
using namespace openMVG::image;
using namespace openMVG::sfm;

/// Check that Kmatrix is a string like "f;0;ppx;0;f;ppy;0;0;1"
/// With f,ppx,ppy as valid numerical value
bool checkIntrinsicStringValidity(const std::string & Kmatrix, double & focal, double & ppx, double & ppy)
{
  std::vector<std::string> vec_str;
  stl::split(Kmatrix, ";", vec_str);
  if (vec_str.size() != 9)  {
    std::cerr << "\n Missing ';' character" << std::endl;
    return false;
  }
  // Check that all K matrix value are valid numbers
  for (size_t i = 0; i < vec_str.size(); ++i) {
    double readvalue = 0.0;
    std::stringstream ss;
    ss.str(vec_str[i]);
    if (! (ss >> readvalue) )  {
      std::cerr << "\n Used an invalid not a number character" << std::endl;
      return false;
    }
    if (i==0) focal = readvalue;
    if (i==2) ppx = readvalue;
    if (i==5) ppy = readvalue;
  }
  return true;
}

/// Recursively list all files from a folder with a specific extension
void listFiles(const std::string& folderOrFile, const std::vector<std::string>& extensions, std::vector<std::string>& outFiles)
{
  if(stlplus::is_file(folderOrFile))
  {
    std::string fileExtension = stlplus::extension_part(folderOrFile);
    std::transform(fileExtension.begin(), fileExtension.end(), fileExtension.begin(), ::tolower);
    for(const std::string& extension: extensions)
    {
      if(fileExtension == extension)
      {
        outFiles.push_back(folderOrFile);
        return;
      }
    }
  }
  else if(stlplus::is_folder(folderOrFile))
  {
    // list all files of the folder
    const std::vector<std::string> allFiles = stlplus::folder_all(folderOrFile);
    for(const std::string& item: allFiles)
    {
      const std::string itemPath = stlplus::create_filespec(folderOrFile, item);
      listFiles(itemPath, extensions, outFiles);
    }
  }
}

/// Retrieve resources path from a json file
/// Need a "resource" variable in the json
bool retrieveResources(const std::string& jsonFile, std::vector<std::string>& vec_imagePaths)
{
  if(!stlplus::file_exists(jsonFile))
  {
    std::cerr << "File \"" << jsonFile << "\" does not exists." << std::endl;
    return false;
  }

  // Read file
  std::ifstream jsonStream(jsonFile.c_str(), std::ifstream::binary);
  if(jsonStream)
  {
    // get length of file:
    jsonStream.seekg (0, jsonStream.end);
    const int length = jsonStream.tellg();
    jsonStream.seekg (0, jsonStream.beg);
    // read data as a block:
    std::string jsonString;
    jsonString.resize(length);
    jsonStream.read(&jsonString[0], length);
    jsonStream.close();
    if(!jsonStream)
    {
      std::cerr << "Error when reading file." << std::endl;
      return false;
    }
    // Parse json
    rapidjson::Document document;
    document.Parse<0>(&jsonString[0]);
    if(!document.IsObject())
    {
      std::cerr << "File \"" << jsonFile << "\" is not in json format." << std::endl;
      return false;
    }
    if(!document.HasMember("resources"))
    {
      std::cerr << "No member \"resources\" in json file" << std::endl;
      return false;
    }
    rapidjson::Value& resourcesValue = document["resources"];
    for(rapidjson::Value::ConstValueIterator it = resourcesValue.Begin(); it != resourcesValue.End(); it++)
      vec_imagePaths.push_back(it->GetString());

    return true;
  }
}

//
// Create the description of an input image dataset for OpenMVG toolsuite
// - Export a SfM_Data file with View & Intrinsic data
//
int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string sImageDir;
  std::string sJsonFile;
  std::string sfileDatabase;
  std::string sOutputDir;
  std::string sKmatrix;

  int i_User_camera_model = PINHOLE_CAMERA_RADIAL3;

  bool b_Group_camera_model = true;
  bool b_use_UID = false;

  double focal_pixels = -1.0;

  cmd.add( make_option('i', sImageDir, "imageDirectory") );
  cmd.add( make_option('j', sJsonFile, "jsonFile") );
  cmd.add( make_option('d', sfileDatabase, "sensorWidthDatabase") );
  cmd.add( make_option('o', sOutputDir, "outputDirectory") );
  cmd.add( make_option('f', focal_pixels, "focal") );
  cmd.add( make_option('k', sKmatrix, "intrinsics") );
  cmd.add( make_option('c', i_User_camera_model, "camera_model") );
  cmd.add( make_option('g', b_Group_camera_model, "group_camera_model") );
  cmd.add( make_option('u', b_use_UID, "use_UID") );

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const std::string& s)
  {
      std::cerr << "Usage: " << argv[0] << '\n'
      << "[-i|--imageDirectory]\n"
      << "[-j|--jsonFile]\n"
      << "[-d|--sensorWidthDatabase]\n"
      << "[-o|--outputDirectory]\n"
      << "[-f|--focal] (pixels)\n"
      << "[-k|--intrinsics] Kmatrix: \"f;0;ppx;0;f;ppy;0;0;1\"\n"
      << "[-c|--camera_model] Camera model type:\n"
      << "\t 1: Pinhole\n"
      << "\t 2: Pinhole radial 1\n"
      << "\t 3: Pinhole radial 3 (default)\n"
      << "\t 4: Pinhole brown 2\n"
      << "[-g|--group_camera_model]\n"
      << "\t 0-> each view have it's own camera intrinsic parameters,\n"
      << "\t 1-> (default) view can share some camera intrinsic parameters\n"
      << "[-u|--use_UID] Generate a UID (unique identifier) for each view. By default, the key is the image index.\n"
      << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  std::cout << " You called : " <<std::endl
            << argv[0] << std::endl
            << "--imageDirectory " << sImageDir << std::endl
            << "--jsonFile " << sJsonFile << std::endl
            << "--sensorWidthDatabase " << sfileDatabase << std::endl
            << "--outputDirectory " << sOutputDir << std::endl
            << "--focal " << focal_pixels << std::endl
            << "--intrinsics " << sKmatrix << std::endl
            << "--camera_model " << i_User_camera_model << std::endl
            << "--group_camera_model " << b_Group_camera_model << std::endl;

  // Expected properties for each image
  double width = -1, height = -1, focal = -1, ppx = -1,  ppy = -1;

  const EINTRINSIC e_User_camera_model = EINTRINSIC(i_User_camera_model);

  if(!sImageDir.empty() && !sJsonFile.empty())
  {
    std::cerr << "\nCannot combine -i and -j options" << std::endl;
    return EXIT_FAILURE;
  }
  if ( !sImageDir.empty() && !stlplus::folder_exists( sImageDir ) )
  {
    std::cerr << "\nThe input directory doesn't exist" << std::endl;
    return EXIT_FAILURE;
  }
  if (sOutputDir.empty())
  {
    std::cerr << "\nInvalid output directory" << std::endl;
    return EXIT_FAILURE;
  }

  if ( !stlplus::folder_exists( sOutputDir ) )
  {
    if ( !stlplus::folder_create( sOutputDir ))
    {
      std::cerr << "\nCannot create output directory" << std::endl;
      return EXIT_FAILURE;
    }
  }

  if (sKmatrix.size() > 0 &&
    !checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy) )
  {
    std::cerr << "\nInvalid K matrix input" << std::endl;
    return EXIT_FAILURE;
  }

  if (sKmatrix.size() > 0 && focal_pixels != -1.0)
  {
    std::cerr << "\nCannot combine -f and -k options" << std::endl;
    return EXIT_FAILURE;
  }

  std::vector<Datasheet> vec_database;
  if (!sfileDatabase.empty())
  {
    if ( !parseDatabase( sfileDatabase, vec_database ) )
    {
      std::cerr
       << "\nInvalid input database: " << sfileDatabase
       << ", please specify a valid file." << std::endl;
      return EXIT_FAILURE;
    }
  }

  // Retrieve image paths
  std::vector<std::string> vec_images;
  const std::vector<std::string> supportedExtensions{ "jpg", "jpeg" };

  if (!sJsonFile.empty())
  {
    // Retrieve resources from json file
    std::vector<std::string> vec_resources;
    if (!retrieveResources(sJsonFile, vec_resources))
      return EXIT_FAILURE;
    // Retrieve images from resources
    for(const std::string& item: vec_resources)
    {
      listFiles(item, supportedExtensions, vec_images);
    }
    std::sort(vec_images.begin(), vec_images.end());
  }
  if(!sImageDir.empty())
  {
    vec_images = stlplus::folder_files( sImageDir );
  }
  std::sort(vec_images.begin(), vec_images.end());

  // Configure an empty scene with Views and their corresponding cameras
  SfM_Data sfm_data;
  sfm_data.s_root_path = "";
  if(!sImageDir.empty())
    sfm_data.s_root_path = sImageDir; // Setup main image root_path
  Views & views = sfm_data.views;
  Intrinsics & intrinsics = sfm_data.intrinsics;

  C_Progress_display my_progress_bar( vec_images.size(),
      std::cout, "\n- Image listing -\n" );
  std::ostringstream error_report_stream;
  for ( std::vector<std::string>::const_iterator iter_image = vec_images.begin();
    iter_image != vec_images.end();
    ++iter_image, ++my_progress_bar )
  {
    // Read meta data to fill camera parameter (w,h,focal,ppx,ppy) fields.
    width = height = ppx = ppy = focal = -1.0;

    const std::string imageFilename = *iter_image;
    std::string imageAbsFilepath;
    if(!sJsonFile.empty())
      imageAbsFilepath = imageFilename;
    else if(!sImageDir.empty())
      imageAbsFilepath = stlplus::create_filespec( sImageDir, imageFilename );

    // Test if the image format is supported:
    if (openMVG::image::GetFormat(imageAbsFilepath.c_str()) == openMVG::image::Unknown)
    {
      error_report_stream
          << stlplus::filename_part(imageAbsFilepath) << ": Unkown image file format." << "\n";
      continue; // image cannot be opened
    }

    ImageHeader imgHeader;
    if (!openMVG::image::ReadImageHeader(imageAbsFilepath.c_str(), &imgHeader))
      continue; // image cannot be read

    width = imgHeader.width;
    height = imgHeader.height;
    ppx = width / 2.0;
    ppy = height / 2.0;

    Exif_IO_EasyExif exifReader;
    exifReader.open( imageAbsFilepath );

    const bool bHaveValidExifMetadata =
      exifReader.doesHaveExifInfo()
      && !exifReader.getBrand().empty()
      && !exifReader.getModel().empty();

    // Consider the case where the focal is provided manually
    if ( !bHaveValidExifMetadata || focal_pixels != -1)
    {
      if (sKmatrix.size() > 0) // Known user calibration K matrix
      {
        if (!checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy))
          focal = -1.0;
      }
      else // User provided focal length value
        if (focal_pixels != -1 )
          focal = focal_pixels;
    }
    else // If image contains meta data
    {
      const std::string sCamName = exifReader.getBrand();
      const std::string sCamModel = exifReader.getModel();

      // Handle case where focal length is equal to 0
      if (exifReader.getFocal() == 0.0f)
      {
        error_report_stream
          << stlplus::basename_part(imageAbsFilepath) << ": Focal length is missing." << "\n";
        focal = -1.0;
      }
      else
      // Create the image entry in the list file
      {
        Datasheet datasheet;
        if ( getInfo( sCamName, sCamModel, vec_database, datasheet ))
        {
          // The camera model was found in the database so we can compute it's approximated focal length
          const double ccdw = datasheet._sensorSize;
          focal = std::max ( width, height ) * exifReader.getFocal() / ccdw;
        }
        else
        {
          error_report_stream
            << stlplus::basename_part(imageAbsFilepath) << ": Camera \""
            << sCamName << "\" model \"" << sCamModel << "\" doesn't exist in the database" << "\n"
            << "Please consider add your camera model and sensor width in the database." << "\n";
        }
      }
    }

    // Build intrinsic parameter related to the view
    std::shared_ptr<IntrinsicBase> intrinsic (NULL);

    if (focal > 0 && ppx > 0 && ppy > 0 && width > 0 && height > 0)
    {
      // Create the desired camera type
      switch(e_User_camera_model)
      {
        case PINHOLE_CAMERA:
          intrinsic = std::make_shared<Pinhole_Intrinsic>
            (width, height, focal, ppx, ppy);
        break;
        case PINHOLE_CAMERA_RADIAL1:
          intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K1>
            (width, height, focal, ppx, ppy, 0.0); // setup no distortion as initial guess
        break;
        case PINHOLE_CAMERA_RADIAL3:
          intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K3>
            (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0);  // setup no distortion as initial guess
        break;
        case PINHOLE_CAMERA_BROWN:
          intrinsic =std::make_shared<Pinhole_Intrinsic_Brown_T2>
            (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0, 0.0, 0.0); // setup no distortion as initial guess
        break;
        default:
          std::cerr << "Error: unknown camera model: " << (int) e_User_camera_model << std::endl;
          return EXIT_FAILURE;
      }
    }
    else
    {
      std::cerr << "Error: No instrinsics for \"" << imageFilename << "\".\n"
        << "focal: " << focal << "\n"
        << "ppx,ppy: " << ppx << ", " << ppy << "\n"
        << "width,height: " << width << ", " << height
        << std::endl;
    }

    IndexT id_view = views.size();
    if( b_use_UID )
    {
      const std::size_t uid = computeUID(exifReader, imageFilename);
      id_view = (IndexT)uid;
    }

    // Build the view corresponding to the image
    View v(imageFilename, id_view, views.size(), views.size(), width, height);

    // Add intrinsic related to the image (if any)
    if (intrinsic == NULL)
    {
      //Since the view have invalid intrinsic data
      // (export the view, with an invalid intrinsic field value)
      v.id_intrinsic = UndefinedIndexT;
    }
    else
    {
      // Add the defined intrinsic to the sfm_container
      intrinsics[v.id_intrinsic] = intrinsic;
    }

    // Add the view to the sfm_container
    views[v.id_view] = std::make_shared<View>(v);
  }

  // Display saved warning & error messages if any.
  if (!error_report_stream.str().empty())
  {
    std::cerr
      << "\nWarning & Error messages:" << std::endl
      << error_report_stream.str() << std::endl;
  }

  // Group camera that share common properties if desired (leads to more faster & stable BA).
  if (b_Group_camera_model)
  {
    GroupSharedIntrinsics(sfm_data);
  }

  // Store SfM_Data views & intrinsic data
  if (!Save(
    sfm_data,
    stlplus::create_filespec( sOutputDir, "sfm_data.json" ).c_str(),
    ESfM_Data(VIEWS|INTRINSICS)))
  {
    return EXIT_FAILURE;
  }

  std::cout << std::endl
    << "SfMInit_ImageListing report:\n"
    << "listed #File(s): " << vec_images.size() << "\n"
    << "usable #File(s) listed in sfm_data: " << sfm_data.GetViews().size() << std::endl;

  return EXIT_SUCCESS;
}
