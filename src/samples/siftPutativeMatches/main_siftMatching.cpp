// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/image/all.hpp"
#include "aliceVision/feature/feature.hpp"
#include "aliceVision/matching/RegionsMatcher.hpp"
#include "aliceVision/feature/sift/ImageDescriber_SIFT.hpp"

#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"
#include "dependencies/vectorGraphics/svgDrawer.hpp"

#include <string>
#include <iostream>

using namespace aliceVision;
using namespace aliceVision::image;
using namespace aliceVision::matching;
using namespace svg;
using namespace std;

int main() {

  Image<RGBColor> image;
  string jpg_filenameL = stlplus::folder_up(string(THIS_SOURCE_DIR))
    + "/imageData/StanfordMobileVisualSearch/Ace_0.png";
  string jpg_filenameR = stlplus::folder_up(string(THIS_SOURCE_DIR))
    + "/imageData/StanfordMobileVisualSearch/Ace_1.png";

  Image<unsigned char> imageL, imageR;
  readImage(jpg_filenameL, imageL);
  readImage(jpg_filenameR, imageR);

  //--
  // Detect regions thanks to an image_describer
  //--
  using namespace aliceVision::feature;
  std::unique_ptr<ImageDescriber> image_describer(new ImageDescriber_SIFT());
  std::map<IndexT, std::unique_ptr<feature::Regions> > regions_perImage;
  image_describer->describe(imageL, regions_perImage[0]);
  image_describer->describe(imageR, regions_perImage[1]);

  const PointFeatures
    featsL = regions_perImage.at(0)->GetRegionsPositions(),
    featsR = regions_perImage.at(1)->GetRegionsPositions();

  // Show both images side by side
  {
    Image<unsigned char> concat;
    ConcatH(imageL, imageR, concat);
    string out_filename = "00_images.jpg";
    writeImage(out_filename, concat);
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
    writeImage(out_filename, concat);
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
