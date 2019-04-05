// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Timer.hpp>
#include <aliceVision/image/image.hpp>
#include <aliceVision/feature/akaze/AKAZE.hpp>

#include <dependencies/vectorGraphics/svgDrawer.hpp>
#include <dependencies/cmdLine/cmdLine.h>

#include <boost/filesystem.hpp>

#include <iostream>
#include <sstream>

using namespace aliceVision;
using namespace svg;
namespace fs = boost::filesystem;

void usage( const std::string & appName )
{
  std::cerr << "usage : " << std::endl ;
  std::cerr << "  " << appName << std::endl ;
  std::cerr << "[-i|--input imageFileName]" << std::endl ;
  std::cerr << "[-o|--output outputFileName]" << std::endl ;
  std::cerr << "[Optional]" << std::endl ;
  std::cerr << "[-p|--nb-octave 4]" << std::endl ;
  std::cerr << "[-q|--nb-slice 4]" << std::endl ;

  exit( EXIT_FAILURE ) ;
}

int main( int argc , char ** argv )
{
  CmdLine cmd;

  std::string sInputImage ;
  std::string sOuputFile ;

  int iNbOctave = 4 ;
  int iNbSlicePerOctave = 4 ;

  cmd.add( make_option('i', sInputImage , "input") );
  cmd.add( make_option('o', sOuputFile , "output") );
  cmd.add( make_option('p', iNbOctave , "nb-octave" ) ) ;
  cmd.add( make_option('q', iNbSlicePerOctave , "nb-slice" ) ) ;

  if( argc == 1 )
  {
    std::cerr << "Error : No option given" << std::endl ;
    usage( argv[0] ) ;
  }

  cmd.process(argc, argv);


  if( sInputImage.empty() )
  {
    std::cerr << "Error : input file name empty" << std::endl ;
    usage( argv[0] ) ;
  }

  if( sOuputFile.empty() )
  {
    std::cerr << "Error : output file name empty" << std::endl ;
    usage( argv[0] ) ;
  }

  // Compute base output filename
  const std::string outputBaseName = fs::path(sOuputFile).stem().string();

  image::Image<unsigned char> src;
  readImageadImage(sInputImage, src);

  Timer t;
  t.reset();

  AKAZEOptions options;
  options.fDesc_factor = 10.f * sqrt(2.f) ;
  AKAZE akaze(src, options);
  akaze.Compute_AKAZEScaleSpace();
  std::vector<AKAZEKeypoint> kpts;
  kpts.reserve(5000);
  akaze.Feature_Detection(kpts);
  
  std::cout << "in "
    << t.elapsedMs() << " msec." << std::endl 
    << t.elapsed() << " sec." << std::endl;

  akaze.Do_Subpixel_Refinement(kpts);

  for (size_t i = 0; i < kpts.size(); ++i)
  {
    AKAZEKeypoint & pt = kpts[i];
    akaze.Compute_Main_Orientation(pt,
      akaze.getSlices()[pt.class_id].Lx,
      akaze.getSlices()[pt.class_id].Ly);
  }
  std::cout << "Found " << kpts.size() << " keypoints" << std::endl;

  for (size_t i = 0; i < kpts.size(); ++i)
  {
    const AKAZEKeypoint & kp = kpts[i];
    float ratio = pow(2.f,kp.octave);
    DrawCircle(kp.x, kp.y, kp.size*2.5, 255, &src);
  }

  writeImage(outputBaseName + std::string("_feat.png"), src, image::EImageColorSpace::AUTO);

  svgDrawer svgStream( src.Width(), src.Height());
  svgStream.drawImage(sInputImage, src.Width(), src.Height());

  //-- Draw features
  for (size_t i=0; i< kpts.size(); ++i)  {
    const AKAZEKeypoint & kp = kpts[i];
    float ratio = pow(2.f,kp.octave);
    svgStream.drawCircle(kp.x, kp.y, kp.size*2.50,
        svgStyle().stroke("yellow", 1.0));

    svgStream.drawLine(
      kp.x, kp.y,
      kp.x + cos(kp.angle)*kp.size*2.5, kp.y + sin(kp.angle)*kp.size*2.5,
      svgStyle().stroke("blue", 1.0));
  }

  // Write the SVG file
  std::ofstream svgFile( (outputBaseName + std::string("_feat.svg")).c_str() );
  svgFile << svgStream.closeSvgFile().str();
  svgFile.close();

  return EXIT_SUCCESS ;
}
