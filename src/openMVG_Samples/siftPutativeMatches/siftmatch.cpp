
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/image/image.hpp"
#include "openMVG/features/features.hpp"
#include "openMVG/matching/regions_matcher.hpp"

#include "nonFree/sift/SIFT_describer.hpp"
#include "nonFree/sift/SIFT_popSIFT_describer.hpp"
#include "nonFree/sift/SIFT_OPENCV_Image_describer.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include "third_party/vectorGraphics/svgDrawer.hpp"

#include <string>
#include <iostream>

using namespace openMVG;
using namespace openMVG::image;
using namespace openMVG::matching;
using namespace svg;
using namespace std;

enum SiftMode
{
    UNKNOWN    = 0,
    OPENCVSIFT = 1,
    VLFEAT     = 2,
    POPSIFT    = 3,
};

static int parseargs( int argc, char** argv, string& l, string& r )
{
    extern char *optarg;
    extern int optind, opterr, optopt;
    int opt;
    int choice = UNKNOWN;
    while(( opt = getopt(argc, argv, "hcopvl:r:")) != -1 ) {
        switch(opt) {
        case 'h' :
            std::cerr << "Usage: " << argv[0] << " [-h] [-copv] [-l <file> -r <file>]" << std::endl
                      << "       -h : print usage and quit" << std::endl
                      << "       -o : use OpenCV's SIFT" << std::endl
                      << "       -v : use VLFeat SIFT" << std::endl
                      << "       -p : use PopSift (GPU)" << std::endl
                      << "       -l <file> absolute path to \"left\"  file to compare" << std::endl
                      << "       -r <file> absolute path to \"right\" file to compare" << std::endl
                      << std::endl
                      << "       All parameters are optional. The default left and right files are" << std::endl
                      << "         " << l << " and" << std::endl
                      << "         " << r << std::endl;
            exit(0);
            break;
        case 'o' :
            choice = OPENCVSIFT;
            break;
        case 'p' :
            choice = POPSIFT;
            break;
        case 'v' :
            choice = VLFEAT;
            break;
        case 'l' :
            l = optarg;
            break;
        case 'r' :
            r = optarg;
            break;
        default :
            choice = UNKNOWN;
            break;
        }
    }
    return choice;
}

int main( int argc, char** argv )
{
  string jpg_filenameL = stlplus::folder_up(string(THIS_SOURCE_DIR))
    + "/imageData/StanfordMobileVisualSearch/Ace_0.png";
  string jpg_filenameR = stlplus::folder_up(string(THIS_SOURCE_DIR))
    + "/imageData/StanfordMobileVisualSearch/Ace_1.png";

  int choice = parseargs( argc, argv, jpg_filenameL, jpg_filenameR );

  Image<RGBColor> image;

  Image<unsigned char> imageL, imageR;
  ReadImage(jpg_filenameL.c_str(), &imageL);
  ReadImage(jpg_filenameR.c_str(), &imageR);

  //--
  // Detect regions thanks to an image_describer
  //--
  using namespace openMVG::features;
  std::unique_ptr<Image_describer> image_describer;
  switch( choice ) {
  case OPENCVSIFT :
    image_describer.reset( new SIFT_OPENCV_Image_describer );
    break;
#ifdef HAVE_POPSIFT
  case POPSIFT :
    image_describer.reset( new SIFT_popSIFT_describer );
    break;
#endif
  case VLFEAT :
  default :
    image_describer.reset( new SIFT_Image_describer );
    break;
  }

  std::map<IndexT, std::unique_ptr<features::Regions> > regions_perImage;
  image_describer->Describe(imageL, regions_perImage[0]);
  image_describer->Describe(imageR, regions_perImage[1]);

  const PointFeatures
    featsL = regions_perImage.at(0)->GetRegionsPositions(),
    featsR = regions_perImage.at(1)->GetRegionsPositions();

  // Show both images side by side
  {
    Image<unsigned char> concat;
    ConcatH(imageL, imageR, concat);
    string out_filename = "00_images.jpg";
    WriteImage(out_filename.c_str(), concat);
  }

  const SIFT_Regions* regionsL = dynamic_cast<SIFT_Regions*>(regions_perImage.at(0).get());
  const SIFT_Regions* regionsR = dynamic_cast<SIFT_Regions*>(regions_perImage.at(1).get());

  //- Draw features on the two image (side by side)
  {
    Image<unsigned char> concat;
    ConcatH(imageL, imageR, concat);

    //-- Draw features :
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

    // Draw correspondences after Nearest Neighbor ratio filter
    svgDrawer svgStream( imageL.Width() + imageR.Width(), max(imageL.Height(), imageR.Height()));
    svgStream.drawImage(jpg_filenameL, imageL.Width(), imageL.Height());
    svgStream.drawImage(jpg_filenameR, imageR.Width(), imageR.Height(), imageL.Width());
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

  // Display some statistics
  std::cout << featsL.size() << " Features on image A" << std::endl
   << featsR.size() << " Features on image B" << std::endl
   << vec_PutativeMatches.size() << " matches after matching with Distance Ratio filter" << std::endl;

  return EXIT_SUCCESS;
}
