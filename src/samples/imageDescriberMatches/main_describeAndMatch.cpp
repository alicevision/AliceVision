// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/image/image.hpp"
#include "aliceVision/feature/feature.hpp"
#include "aliceVision/feature/sift/ImageDescriber_SIFT.hpp"
#include "aliceVision/feature/akaze/ImageDescriber_AKAZE.hpp"
#include "aliceVision/matching/matching_filters.hpp"
#include "aliceVision/matching/regions_matcher.hpp"

#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"
#include "dependencies/vectorGraphics/svgDrawer.hpp"
#include "dependencies/cmdLine/cmdLine.h"

#include <string>
#include <iostream>

using namespace aliceVision;
using namespace aliceVision::image;
using namespace svg;
using namespace std;


int main(int argc, char **argv) {

  // Add options to choose the desired ImageDescriber
  std::string sImageDescriber_type = "SIFT";
  std::string jpg_filenameL = stlplus::folder_up(string(THIS_SOURCE_DIR))
    + "/imageData/StanfordMobileVisualSearch/Ace_0.png";
  std::string jpg_filenameR = stlplus::folder_up(string(THIS_SOURCE_DIR))
    + "/imageData/StanfordMobileVisualSearch/Ace_1.png";
  
  std::string sFeaturePreset = "";

  CmdLine cmd;
  cmd.add( make_option('t', sImageDescriber_type, "type") );
  cmd.add( make_option('l', jpg_filenameL, "left") );
  cmd.add( make_option('r', jpg_filenameR, "right") );
  cmd.add( make_option('p', sFeaturePreset, "describerPreset") );

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const std::string& s) {
      std::cerr << "Usage: " << argv[0] << '\n'
      << "\n[Optional]\n"
      << "[-l|--left] the left image (default imageData/StanfordMobileVisualSearch/Ace_0.png)"
      << "[-r|--right] the right image (default imageData/StanfordMobileVisualSearch/Ace_1.png)"
      << "[-t|--type\n"
      << "  (choose an image_describer interface):\n"
      << "   SIFT: SIFT keypoint & descriptor,\n"
      << "   AKAZE: AKAZE keypoint & floating point descriptor]"
      << "[-p|--describerPreset]\n"
      << "  (used to control the ImageDescriber configuration):\n"
      << "   LOW,\n"
      << "   MEDIUM,\n"
      << "   NORMAL (default),\n"
      << "   HIGH,\n"
      << "   ULTRA: !!Can take long time!!\n"
      << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  Image<RGBColor> image;

  Image<unsigned char> imageL, imageR;
  ReadImage(jpg_filenameL.c_str(), &imageL);
  ReadImage(jpg_filenameR.c_str(), &imageR);

  // Call Keypoint extractor
  using namespace aliceVision::feature;
  std::shared_ptr<ImageDescriber> image_describer;
  if (sImageDescriber_type == "SIFT")
    image_describer = std::make_shared<ImageDescriber_SIFT>(SiftParams());
  else if (sImageDescriber_type == "AKAZE")
    image_describer = std::make_shared<ImageDescriber_AKAZE>(AKAZEParams(AKAZEConfig(), AKAZE_MSURF));
  else if (sImageDescriber_type == "AKAZE_MLDB")
    image_describer = std::make_shared<ImageDescriber_AKAZE>(AKAZEParams(AKAZEConfig(), AKAZE_MLDB));

  if (image_describer.use_count()==0)
  {
    std::cerr << "Invalid ImageDescriber type" << std::endl;
    return EXIT_FAILURE;
  }
  if (!sFeaturePreset.empty())
  {
    if (!image_describer->Set_configuration_preset(sFeaturePreset))
    {
      std::cerr << "Preset configuration failed." << std::endl;
      return EXIT_FAILURE;
    }
  }

  //--
  // Detect regions thanks to the image_describer
  //--
  std::map<IndexT, std::unique_ptr<feature::Regions> > regions_perImage;
  image_describer->Describe(imageL, regions_perImage[0]);
  image_describer->Describe(imageR, regions_perImage[1]);

  const std::vector<PointFeature>
    featsL = regions_perImage.at(0)->GetRegionsPositions(),
    featsR = regions_perImage.at(1)->GetRegionsPositions();

  //--
  // Display used images & Features
  //--

  {
    //- Show images side by side
    Image<unsigned char> concat;
    ConcatH(imageL, imageR, concat);
    const string out_filename = "00_images.jpg";
    WriteImage(out_filename.c_str(), concat);
  }

  {
    //- Draw features on the images (side by side)
    Image<unsigned char> concat;
    ConcatH(imageL, imageR, concat);

    //-- Draw features :
    for (size_t i=0; i < featsL.size(); ++i )  {
      const PointFeature & imaA = featsL[i];
      DrawCircle(imaA.x(), imaA.y(), 3.0f, 255, &concat);
    }
    for (size_t i=0; i < featsR.size(); ++i )  {
      const PointFeature & imaB = featsR[i];
      DrawCircle(imaB.x()+imageL.Width(), imaB.y(), 3.0f, 255, &concat);
    }
    const string out_filename = "01_features.jpg";
    WriteImage(out_filename.c_str(), concat);
  }

  //--
  // Compute corresponding points
  //--
  //-- Perform matching -> find Nearest neighbor, filtered with Distance ratio
  matching::IndMatches vec_PutativeMatches;
  matching::DistanceRatioMatch(
    0.8, matching::BRUTE_FORCE_L2,
    *regions_perImage[0].get(),
    *regions_perImage[1].get(),
    vec_PutativeMatches);

  // Draw correspondences after Nearest Neighbor ratio filter
  {
    svgDrawer svgStream( imageL.Width() + imageR.Width(), max(imageL.Height(), imageR.Height()));
    svgStream.drawImage(jpg_filenameL, imageL.Width(), imageL.Height());
    svgStream.drawImage(jpg_filenameR, imageR.Width(), imageR.Height(), imageL.Width());
    for (size_t i = 0; i < vec_PutativeMatches.size(); ++i) {
      //Get back linked feature, draw a circle and link them by a line
      const PointFeature & L = featsL[vec_PutativeMatches[i]._i];
      const PointFeature & R = featsR[vec_PutativeMatches[i]._j];
      svgStream.drawLine(L.x(), L.y(), R.x()+imageL.Width(), R.y(), svgStyle().stroke("green", 2.0));
      svgStream.drawCircle(L.x(), L.y(), 3.0f, svgStyle().stroke("yellow", 2.0));
      svgStream.drawCircle(R.x()+imageL.Width(), R.y(), 3.0f,svgStyle().stroke("yellow", 2.0));
    }
    const string out_filename = "02_Matches.svg";
    ofstream svgFile( out_filename.c_str() );
    svgFile << svgStream.closeSvgFile().str();
    svgFile.close();
  }

  // Display some statistics
  std::cout
    << regions_perImage.at(0)->RegionCount() << " #Features on image A" << std::endl
    << regions_perImage.at(1)->RegionCount() << " #Features on image B" << std::endl
    << vec_PutativeMatches.size() << " #matches with Distance Ratio filter" << std::endl;

  return EXIT_SUCCESS;
}
