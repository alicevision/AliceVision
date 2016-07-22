
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/image/image.hpp"
#include "openMVG/features/features.hpp"
#include "openMVG/matching/regions_matcher.hpp"

#include "nonFree/sift/SIFT_describer.hpp"
#include "nonFree/sift/SIFT_OPENCV_Image_describer.hpp"
//#include "nonFree/sift/SIFT_popSIFT_describer.hpp"
#include "nonFree/sift/SIFT_float_describer.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include "third_party/vectorGraphics/svgDrawer.hpp"
#include "third_party/cmdLine/cmdLine.h"

#include "boost/filesystem.hpp"

#include <string>
#include <iostream>

using namespace openMVG;
using namespace openMVG::image;
using namespace openMVG::matching;
using namespace svg;
using namespace std;

namespace bfs = boost::filesystem;

#define SINGLE_IMG

int main(int argc, char **argv)
{
  CmdLine cmd;
  //--
  // Command line parameters
  std::string filenameL = stlplus::folder_up(string(THIS_SOURCE_DIR))
    + "imageData/StanfordMobileVisualSearch/Ace_40p_gray.pgm";
  std::string outputPath = "./";
  std::string sImage_Describer_Method = "SIFT";
  
  
  cmd.add( make_option('i', filenameL, "input_image") );
  cmd.add( make_option('o', outputPath, "output_path") );
  cmd.add( make_option('d', sImage_Describer_Method, "describer_method") );
  
    // Command line parsing
  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const std::string& s) {
      std::cerr << "Usage: " << argv[0] << '\n'
      << "[-i|--input_image]\n"
      << "[-o|--output_path]\n"
      << "[-d|--describer_method]\n"
      << "   POP: popSIFT,\n"
      << "   VL: VLFeat \n"
      << "   OCV: OpenCV  (default).\n"
      << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }
  //--

  Image<RGBColor> image;
  std::cout << "Processing image: " << filenameL << std::endl;
  
   Image<unsigned char> imageL;
   ReadImage(filenameL.c_str(), &imageL);
   
#ifndef SINGLE_IMG
  string filenameR = stlplus::folder_up(string(THIS_SOURCE_DIR))
    + "/imageData/StanfordMobileVisualSearch/Ace_1.png";
  Image<unsigned char> imageR;
  ReadImage(filenameR.c_str(), &imageR);
#endif

  //--
  // Detect regions thanks to an image_describer
  //--
  
  float scaleRectif = 1.0;
  
  using namespace openMVG::features;
  std::unique_ptr<Image_describer> image_describer;
  
  //if (sImage_Describer_Method == "POP")
  //{
  //  image_describer.reset(new SIFT_popSIFT_describer);
  //  scaleRectif = 0.5f;
  //}else 
  if (sImage_Describer_Method == "VL")
  {
      image_describer.reset(new SIFT_Image_describer);
      
  }else if (sImage_Describer_Method == "OCV")
  {
      image_describer.reset(new SIFT_OPENCV_Image_describer);
      scaleRectif = 0.5f;
  }
  
  // Output all
  std::ostringstream ostr;
  ostr << outputPath << bfs::path(filenameL).stem().string() << ".sift";
  
  std::map<IndexT, std::unique_ptr<features::Regions> > regions_perImage;

  // Run the SIFT extraction
  image_describer->Describe(imageL, regions_perImage[0]);
  
  const openMVG::features::Regions* l = regions_perImage.at(0).get();
  const SIFT_Regions* regionsL = (const SIFT_Regions*)l;
  
  std::cout << " regionsL->Descriptors().size() to write " << regionsL->Features().size() << std::endl;
 
  // Format w.r.t. (vgg) *.sift file
  std::ofstream des_str(ostr.str().c_str());
  des_str << 128 <<std::endl;
  des_str << regionsL->Features().size() <<std::endl;
  
  for(int i=0;i<regionsL->Features().size();++i)
  {
    // Export keypoints
    des_str << regionsL->Features()[i].x() << " ";
    des_str << regionsL->Features()[i].y() << " ";

    float affineCoef = 1.f/((regionsL->Features()[i].scale()*scaleRectif)*(regionsL->Features()[i].scale()*scaleRectif));
    des_str << affineCoef << " ";
    des_str << 0 << " "; 
    des_str << affineCoef << " ";
    
    // Export descriptors
    for(int k=0;k<regionsL->Descriptors()[i].size();++k)
      des_str << (int) regionsL->Descriptors()[i][k] << " ";

    des_str << std::endl;
  }
  
#ifdef SINGLE_IMG
  return 0;
#else
  image_describer->Describe(imageR, regions_perImage[1]);
  
  std::cerr << "Extraction in both images done" << std::endl;

  const PointFeatures featsL = regions_perImage.at(0)->GetRegionsPositions();
  const PointFeatures featsR = regions_perImage.at(1)->GetRegionsPositions();

  // Show both images side by side
  {
    Image<unsigned char> concat;
    ConcatH(imageL, imageR, concat);
    string out_filename = "00_images.jpg";
    WriteImage(out_filename.c_str(), concat);
  }
    
  const openMVG::features::Regions* l = regions_perImage.at(0).get();
  const openMVG::features::Regions* r = regions_perImage.at(1).get();
  
  const SIFT_Regions* regionsL = (const SIFT_Regions*)l;
  const SIFT_Regions* regionsR = (const SIFT_Regions*)r;

  std::cerr << __LINE__ << " " << (intptr_t) regionsL<< std::endl;
  
  //- Draw features on the two image (side by side)
  {
     Image<unsigned char> concat;
     ConcatH(imageL, imageR, concat);

    //- Draw features :
    for (size_t i=0; i < featsL.size(); ++i )  {
      const SIOPointFeature point = regionsL->Features()[i];
      DrawCircle(point.x(), point.y(), point.scale(), 255, &concat);
    }
    for (size_t i=0; i < featsR.size(); ++i )  {
      const SIOPointFeature point = regionsR->Features()[i];
      DrawCircle(point.x()+imageL.Width(), point.y(), point.scale(), 255, &concat);
    }
    const std::string out_filename = "01_features.jpg";
    WriteImage(out_filename.c_str(), concat);
  }

  //-- Perform matching -> find Nearest neighbor, filtered with Distance ratio
  std::vector<IndMatch> vec_PutativeMatches;
  {
    // Find corresponding points
    matching::DistanceRatioMatch(
      0.8, matching::BRUTE_FORCE_L2,
       *regions_perImage.at(0).get(),
       *regions_perImage.at(1).get(),
       vec_PutativeMatches);

  //
  // SVG include xref does not work, so this is pointless
  //

    // Draw correspondences after Nearest Neighbor ratio filter
    {
      svgDrawer svgStream( imageL.Width() + imageR.Width(), max(imageL.Height(), imageR.Height()));
      svgStream.drawImage(filenameL, imageL.Width(), imageL.Height());
      svgStream.drawImage(filenameR, imageR.Width(), imageR.Height(), imageL.Width());
      for (size_t i = 0; i < vec_PutativeMatches.size(); ++i) {
      //Get back linked feature, draw a circle and link them by a line
        const SIOPointFeature L = regionsL->Features()[vec_PutativeMatches[i]._i];
        const SIOPointFeature R = regionsR->Features()[vec_PutativeMatches[i]._j];
        svgStream.drawLine(L.x(), L.y(), R.x()+imageL.Width(), R.y(), svgStyle().stroke("green", 2.0));
        svgStream.drawCircle(L.x(), L.y(), L.scale(), svgStyle().stroke("yellow", 2.0));
        svgStream.drawCircle(R.x()+imageL.Width(), R.y(), R.scale(),svgStyle().stroke("yellow", 2.0));
    }
    string out_filename = "02_siftMatches.svg";
    ofstream svgFile( out_filename.c_str() );
    svgFile << svgStream.closeSvgFile().str();
    svgFile.close();
    }
  }

  // Display some statistics
  {
    std::cout << featsL.size() << " Features on image A" << std::endl
    << featsR.size() << " Features on image B" << std::endl
    << vec_PutativeMatches.size() << " matches after matching with Distance Ratio filter" << std::endl;
  }

  return EXIT_SUCCESS;
#endif
}
