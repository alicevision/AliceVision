// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2013 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "colorHarmonizeEngineGlobal.hpp"
#include "software/utils/sfmHelper/sfmIOHelper.hpp"

#include <aliceVision/system/Timer.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/graph/graph.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/image/all.hpp>
//load features per view
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
// feature matches
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/matching/io.hpp>
#include <aliceVision/stl/stl.hpp>
// selection Methods
#include <aliceVision/colorHarmonization/CommonDataByPair_fullFrame.hpp>
#include <aliceVision/colorHarmonization/CommonDataByPair_matchedPoints.hpp>
#include <aliceVision/colorHarmonization/CommonDataByPair_vldSegment.hpp>
// color harmonization solver
#include <aliceVision/colorHarmonization/GainOffsetConstraintBuilder.hpp>

#include <dependencies/vectorGraphics/svgDrawer.hpp>

#include <boost/progress.hpp>

#include <numeric>
#include <iomanip>
#include <iterator>
#include <algorithm>
#include <functional>
#include <sstream>

namespace fs = boost::filesystem;

namespace aliceVision {

using namespace lemon;
using namespace aliceVision::image;
using namespace aliceVision::matching;
using namespace aliceVision::lInfinity;
using namespace aliceVision::sfm;
using namespace aliceVision::sfmData;

typedef feature::PointFeature FeatureT;
typedef vector<FeatureT> featsT;

ColorHarmonizationEngineGlobal::ColorHarmonizationEngineGlobal(
    const string& sfmDataFilename,
    const std::vector<std::string>& featuresFolders,
    const std::vector<std::string>& matchesFolders,
    const string& outputDirectory,
    const std::vector<feature::EImageDescriberType>& descTypes,
    int selectionMethod,
    int imgRef)
  : _sfmDataFilename(sfmDataFilename)
  , _featuresFolders(featuresFolders)
  , _matchesFolders(matchesFolders)
  , _outputDirectory(outputDirectory)
  , _descTypes(descTypes)
{
  if(!fs::exists(outputDirectory))
    fs::create_directory(outputDirectory);

  // choose image reference
  while(imgRef < 0 || imgRef >= _fileNames.size())
  {
      cout << "Choose your reference image:\n";
      for( int i = 0; i < _fileNames.size(); ++i )
      {
        cout << "id: " << i << "\t" << _fileNames[ i ] << endl;
      }
      cin >> imgRef;
  }
  _imgRef = imgRef;

  // choose selection method
  while(selectionMethod < 0 || selectionMethod > 2)
  {
    cout << "Choose your selection method:\n"
      << "- FullFrame: 0\n"
      << "- Matched Points: 1\n"
      << "- VLD Segment: 2\n";
    cin >> selectionMethod;
  }
  _selectionMethod = static_cast<EHistogramSelectionMethod>(selectionMethod);

}

ColorHarmonizationEngineGlobal::~ColorHarmonizationEngineGlobal()
{}

inline void pauseProcess()
{
  unsigned char i;
  cout << "\nPause : type key and press enter: ";
  std::cin >> i;
}


bool ColorHarmonizationEngineGlobal::Process()
{
  const std::string vec_selectionMethod[ 3 ] = { "fullFrame", "matchedPoints", "KVLD" };
  const std::string vec_harmonizeMethod[ 1 ] = { "quantifiedGainCompensation" };
  const int harmonizeMethod = 0;

  //-------------------
  // Load data
  //-------------------

  if( !ReadInputData() )
    return false;
  if( _pairwiseMatches.empty() )
  {
    cout << endl << "Matches file is empty" << endl;
    return false;
  }

  //-- Remove EG with poor support:
  for (matching::PairwiseMatches::iterator matchesPerViewIt = _pairwiseMatches.begin();
       matchesPerViewIt != _pairwiseMatches.end();
       )
  {
    if (matchesPerViewIt->second.getNbAllMatches() < 120)
    {
      matchesPerViewIt = _pairwiseMatches.erase(matchesPerViewIt);
    }
    else
    {
      ++matchesPerViewIt;
    }
  }

  {
    graph::indexedGraph putativeGraph(matching::getImagePairs(_pairwiseMatches));

    // Save the graph before cleaning:
    graph::exportToGraphvizData(
      (fs::path(_outputDirectory) / "input_graph_poor_supportRemoved").string(),
      putativeGraph.g);
  }

  //-------------------
  // Keep the largest CC in the image graph
  //-------------------
  if (!CleanGraph())
  {
    std::cout << std::endl << "There is no largest CC in the graph" << std::endl;
    return false;
  }

  //-------------------
  // Compute remaining camera node Id
  //-------------------

  std::map<size_t, size_t> map_cameraNodeToCameraIndex; // graph node Id to 0->Ncam
  std::map<size_t, size_t> map_cameraIndexTocameraNode; // 0->Ncam correspondance to graph node Id
  std::set<size_t> set_indeximage;

  for (size_t i = 0; i < _pairwiseMatches.size(); ++i)
  {
    matching::PairwiseMatches::const_iterator iter = _pairwiseMatches.begin();
    std::advance(iter, i);

    const size_t I = iter->first.first;
    const size_t J = iter->first.second;
    set_indeximage.insert(I);
    set_indeximage.insert(J);
  }

  for (std::set<size_t>::const_iterator iterSet = set_indeximage.begin();
    iterSet != set_indeximage.end(); ++iterSet)
  {
    map_cameraIndexTocameraNode[std::distance(set_indeximage.begin(), iterSet)] = *iterSet;
    map_cameraNodeToCameraIndex[*iterSet] = std::distance(set_indeximage.begin(), iterSet);
  }

  std::cout << "\n Remaining cameras after CC filter : \n"
    << map_cameraIndexTocameraNode.size() << " from a total of " << _fileNames.size() << std::endl;

  size_t bin      = 256;
  double minvalue = 0.0;
  double maxvalue = 255.0;

  // For each edge computes the selection masks and histograms (for the RGB channels)
  std::vector<relativeColorHistogramEdge> map_relativeHistograms[3];
  map_relativeHistograms[0].resize(_pairwiseMatches.size());
  map_relativeHistograms[1].resize(_pairwiseMatches.size());
  map_relativeHistograms[2].resize(_pairwiseMatches.size());

  for (size_t i = 0; i < _pairwiseMatches.size(); ++i)
  {
    matching::PairwiseMatches::const_iterator iter = _pairwiseMatches.begin();
    std::advance(iter, i);

    const size_t viewI = iter->first.first;
    const size_t viewJ = iter->first.second;

    //
    const MatchesPerDescType& matchesPerDesc = iter->second;

    //-- Edges names:
    std::pair< std::string, std::string > p_imaNames;
    p_imaNames = make_pair( _fileNames[ viewI ], _fileNames[ viewJ ] );
    std::cout << "Current edge : "
      << fs::path(p_imaNames.first).filename().string() << "\t"
      << fs::path(p_imaNames.second).filename().string() << std::endl;

    //-- Compute the masks from the data selection:
    Image< unsigned char > maskI ( _imageSize[ viewI ].first, _imageSize[ viewI ].second );
    Image< unsigned char > maskJ ( _imageSize[ viewJ ].first, _imageSize[ viewJ ].second );

    switch(_selectionMethod)
    {
      case eHistogramHarmonizeFullFrame:
      {
        colorHarmonization::CommonDataByPair_fullFrame  dataSelector(
          p_imaNames.first,
          p_imaNames.second);
        dataSelector.computeMask( maskI, maskJ );
      }
      break;
      case eHistogramHarmonizeMatchedPoints:
      {
        int circleSize = 10;
        colorHarmonization::CommonDataByPair_matchedPoints dataSelector(
          p_imaNames.first,
          p_imaNames.second,
          matchesPerDesc,
          _regionsPerView.getRegionsPerDesc(viewI),
          _regionsPerView.getRegionsPerDesc(viewJ),
          circleSize);
        dataSelector.computeMask( maskI, maskJ );
      }
      break;
      case eHistogramHarmonizeVLDSegment:
      {
        maskI.fill(0);
        maskJ.fill(0);

        for(const auto& matchesIt: matchesPerDesc)
        {
          const feature::EImageDescriberType descType = matchesIt.first;
          const IndMatches& matches = matchesIt.second;
          colorHarmonization::CommonDataByPair_vldSegment dataSelector(
            p_imaNames.first,
            p_imaNames.second,
            matches,
            _regionsPerView.getRegions(viewI, descType).Features(),
            _regionsPerView.getRegions(viewJ, descType).Features());

          dataSelector.computeMask( maskI, maskJ );
        }
      }
      break;
      default:
        std::cout << "Selection method unsupported" << std::endl;
        return false;
    }

    //-- Export the masks
    bool bExportMask = false;
    if (bExportMask)
    {
      string sEdge = _fileNames[ viewI ] + "_" + _fileNames[ viewJ ];
      sEdge = (fs::path(_outputDirectory) / sEdge ).string();

      if( !fs::exists(sEdge) )
        fs::create_directory(sEdge);

      string out_filename_I = "00_mask_I.png";
      out_filename_I = (fs::path(sEdge) / out_filename_I).string();

      string out_filename_J = "00_mask_J.png";
      out_filename_J = (fs::path(sEdge) / out_filename_J).string();
      writeImage(out_filename_I, maskI, image::EImageColorSpace::AUTO);
      writeImage(out_filename_J, maskJ, image::EImageColorSpace::AUTO);
    }

    //-- Compute the histograms
    Image< RGBColor > imageI, imageJ;
    readImage(p_imaNames.first, imageI, image::EImageColorSpace::LINEAR);
    readImage(p_imaNames.second, imageJ, image::EImageColorSpace::LINEAR);

    utils::Histogram< double > histoI( minvalue, maxvalue, bin);
    utils::Histogram< double > histoJ( minvalue, maxvalue, bin);

    int channelIndex = 0; // RED channel
    colorHarmonization::CommonDataByPair::computeHisto( histoI, maskI, channelIndex, imageI );
    colorHarmonization::CommonDataByPair::computeHisto( histoJ, maskJ, channelIndex, imageJ );
    relativeColorHistogramEdge & edgeR = map_relativeHistograms[channelIndex][i];
    edgeR = relativeColorHistogramEdge(map_cameraNodeToCameraIndex[viewI], map_cameraNodeToCameraIndex[viewJ],
      histoI.GetHist(), histoJ.GetHist());

    histoI = histoJ = utils::Histogram<double>(minvalue, maxvalue, bin);
    channelIndex = 1; // GREEN channel
    colorHarmonization::CommonDataByPair::computeHisto( histoI, maskI, channelIndex, imageI );
    colorHarmonization::CommonDataByPair::computeHisto( histoJ, maskJ, channelIndex, imageJ );
    relativeColorHistogramEdge & edgeG = map_relativeHistograms[channelIndex][i];
    edgeG = relativeColorHistogramEdge(map_cameraNodeToCameraIndex[viewI], map_cameraNodeToCameraIndex[viewJ],
      histoI.GetHist(), histoJ.GetHist());

    histoI = histoJ = utils::Histogram<double>(minvalue, maxvalue, bin);
    channelIndex = 2; // BLUE channel
    colorHarmonization::CommonDataByPair::computeHisto( histoI, maskI, channelIndex, imageI );
    colorHarmonization::CommonDataByPair::computeHisto( histoJ, maskJ, channelIndex, imageJ );
    relativeColorHistogramEdge & edgeB = map_relativeHistograms[channelIndex][i];
    edgeB = relativeColorHistogramEdge(map_cameraNodeToCameraIndex[viewI], map_cameraNodeToCameraIndex[viewJ],
      histoI.GetHist(), histoJ.GetHist());
  }

  std::cout << "\n -- \n SOLVE for color consistency with linear programming\n --" << std::endl;
  //-- Solve for the gains and offsets:
  std::vector<size_t> vec_indexToFix;
  vec_indexToFix.push_back(map_cameraNodeToCameraIndex[_imgRef]);

  using namespace aliceVision::linearProgramming;

  std::vector<double> vec_solution_r(_fileNames.size() * 2 + 1);
  std::vector<double> vec_solution_g(_fileNames.size() * 2 + 1);
  std::vector<double> vec_solution_b(_fileNames.size() * 2 + 1);

  aliceVision::system::Timer timer;

  #if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
  typedef MOSEKSolver SOLVER_LP_T;
  #else
  typedef OSI_CISolverWrapper SOLVER_LP_T;
  #endif
  // Red channel
  {
    SOLVER_LP_T lpSolver(vec_solution_r.size());

    GainOffsetConstraintBuilder cstBuilder(map_relativeHistograms[0], vec_indexToFix);
    LPConstraintsSparse constraint;
    cstBuilder.Build(constraint);
    lpSolver.setup(constraint);
    lpSolver.solve();
    lpSolver.getSolution(vec_solution_r);
  }
  // Green channel
  {
    SOLVER_LP_T lpSolver(vec_solution_g.size());

    GainOffsetConstraintBuilder cstBuilder(map_relativeHistograms[1], vec_indexToFix);
    LPConstraintsSparse constraint;
    cstBuilder.Build(constraint);
    lpSolver.setup(constraint);
    lpSolver.solve();
    lpSolver.getSolution(vec_solution_g);
  }
  // Blue channel
  {
    SOLVER_LP_T lpSolver(vec_solution_b.size());

    GainOffsetConstraintBuilder cstBuilder(map_relativeHistograms[2], vec_indexToFix);
    LPConstraintsSparse constraint;
    cstBuilder.Build(constraint);
    lpSolver.setup(constraint);
    lpSolver.solve();
    lpSolver.getSolution(vec_solution_b);
  }

  std::cout << std::endl
    << " ColorHarmonization solving on a graph with: " << _pairwiseMatches.size() << " edges took (s): "
    << timer.elapsed() << std::endl
    << "LInfinity fitting error: \n"
    << "- for the red channel is: " << vec_solution_r.back() << " gray level(s)" <<std::endl
    << "- for the green channel is: " << vec_solution_g.back() << " gray level(s)" << std::endl
    << "- for the blue channel is: " << vec_solution_b.back() << " gray level(s)" << std::endl;

  std::cout << "\n\nFound solution_r:\n";
  std::copy(vec_solution_r.begin(), vec_solution_r.end(), std::ostream_iterator<double>(std::cout, " "));

  std::cout << "\n\nFound solution_g:\n";
  std::copy(vec_solution_g.begin(), vec_solution_g.end(), std::ostream_iterator<double>(std::cout, " "));

  std::cout << "\n\nFound solution_b:\n";
  std::copy(vec_solution_b.begin(), vec_solution_b.end(), std::ostream_iterator<double>(std::cout, " "));
  std::cout << std::endl;

  std::cout << "\n\nThere is :\n" << set_indeximage.size() << " images to transform." << std::endl;

  //-> convert solution to gain offset and creation of the LUT per image
  boost::progress_display my_progress_bar( set_indeximage.size() );
  for (std::set<size_t>::const_iterator iterSet = set_indeximage.begin();
    iterSet != set_indeximage.end(); ++iterSet, ++my_progress_bar)
  {
    const size_t imaNum = *iterSet;
    typedef Eigen::Matrix<double, 256, 1> Vec256;
    std::vector< Vec256 > vec_map_lut(3);

    const size_t nodeIndex = std::distance(set_indeximage.begin(), iterSet);

    const  double g_r = vec_solution_r[nodeIndex*2];
    const  double offset_r = vec_solution_r[nodeIndex*2+1];
    const  double g_g = vec_solution_g[nodeIndex*2];
    const  double offset_g = vec_solution_g[nodeIndex*2+1];
    const  double g_b = vec_solution_b[nodeIndex*2];
    const double offset_b = vec_solution_b[nodeIndex*2+1];

    for( size_t k = 0; k < 256; ++k)
    {
      vec_map_lut[0][k] = clamp( k * g_r + offset_r, 0., 255. );
      vec_map_lut[1][k] = clamp( k * g_g + offset_g, 0., 255. );
      vec_map_lut[2][k] = clamp( k * g_b + offset_b, 0., 255. );
    }

    Image< RGBColor > image_c;
    readImage( _fileNames[ imaNum ], image_c , image::EImageColorSpace::LINEAR);

    #pragma omp parallel for
    for( int j = 0; j < image_c.Height(); ++j )
    {
      for( int i = 0; i < image_c.Width(); ++i )
      {
        image_c(j, i)[0] = clamp(vec_map_lut[0][image_c(j, i)[0]], 0., 255.);
        image_c(j, i)[1] = clamp(vec_map_lut[1][image_c(j, i)[1]], 0., 255.);
        image_c(j, i)[2] = clamp(vec_map_lut[2][image_c(j, i)[2]], 0., 255.);
      }
    }

    const std::string out_folder = (fs::path(_outputDirectory) / (vec_selectionMethod[ _selectionMethod ] + "_" + vec_harmonizeMethod[ harmonizeMethod ])).string();
    if(!fs::exists(out_folder))
      fs::create_directory(out_folder);
    const std::string out_filename = (fs::path(out_folder) / fs::path(_fileNames[ imaNum ]).filename() ).string();

    writeImage(out_filename, image_c , image::EImageColorSpace::AUTO);
  }
  return true;
}

bool ColorHarmonizationEngineGlobal::ReadInputData()
{
  if(!fs::is_directory( _outputDirectory))
  {
    std::cerr << "The output folder is not a valid folder" << std::endl;
    return false;
  }

  if(!fs::is_regular_file(_sfmDataFilename ))
  {
    std::cerr << "Invalid input sfm_data file: " << _sfmDataFilename << std::endl;
    return false;
  }

  // a. Read input scenes views
  SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, _sfmDataFilename, sfmDataIO::ESfMData::VIEWS))
  {
    std::cerr << "The input file \""<< _sfmDataFilename << "\" cannot be read" << std::endl;
    return false;
  }

  // Read images names
  for(Views::const_iterator iter = sfmData.getViews().begin();
    iter != sfmData.getViews().end(); ++iter)
  {
    const View* v = iter->second.get();
    _fileNames.push_back(v->getImagePath());
    _imageSize.push_back( std::make_pair( v->getWidth(), v->getHeight() ));
  }

  // b. Read matches
  if(!sfm::loadPairwiseMatches(_pairwiseMatches, sfmData, _matchesFolders, _descTypes))
  {
    std::cerr << "Can't load matches files" << std::endl;
    return false;
  }

  // Read features:
  if(!sfm::loadRegionsPerView(_regionsPerView, sfmData, _featuresFolders, _descTypes))
  {
    std::cerr << "Can't load feature files" << std::endl;
    return false;
  }

  graph::indexedGraph putativeGraph(getImagePairs(_pairwiseMatches));

  // Save the graph before cleaning:
  graph::exportToGraphvizData((fs::path(_outputDirectory) / "initialGraph" ).string(),putativeGraph.g );

  return true;
}

bool ColorHarmonizationEngineGlobal::CleanGraph()
{
  // Create a graph from pairwise correspondences:
  // - keep the largest connected component.

  graph::indexedGraph putativeGraph(getImagePairs(_pairwiseMatches));

  // Save the graph before cleaning:
  graph::exportToGraphvizData((fs::path(_outputDirectory) / "initialGraph").string(), putativeGraph.g);

  const int connectedComponentCount = lemon::countConnectedComponents(putativeGraph.g);
  std::cout << "\n"
    << "ColorHarmonizationEngineGlobal::CleanGraph() :: => connected Component cardinal: "
    << connectedComponentCount << std::endl;

  if (connectedComponentCount > 1)  // If more than one CC, keep the largest
  {
    // Search the largest CC index
    const std::map<IndexT, std::set<lemon::ListGraph::Node> > map_subgraphs =
      aliceVision::graph::exportGraphToMapSubgraphs<lemon::ListGraph, IndexT>(putativeGraph.g);
    size_t count = std::numeric_limits<size_t>::min();
    std::map<IndexT, std::set<lemon::ListGraph::Node> >::const_iterator iterLargestCC = map_subgraphs.end();
    for(std::map<IndexT, std::set<lemon::ListGraph::Node> >::const_iterator iter = map_subgraphs.begin();
        iter != map_subgraphs.end(); ++iter)
    {
      if (iter->second.size() > count)  {
        count = iter->second.size();
        iterLargestCC = iter;
      }
      std::cout << "Connected component of size : " << iter->second.size() << std::endl;
    }

    //-- Remove all nodes that are not listed in the largest CC
    for(std::map<IndexT, std::set<lemon::ListGraph::Node> >::const_iterator iter = map_subgraphs.begin();
        iter != map_subgraphs.end(); ++iter)
    {
      if (iter == iterLargestCC) // Skip this CC since it's the one we want to keep
        continue;

      const std::set<lemon::ListGraph::Node> & ccSet = iter->second;
      for (std::set<lemon::ListGraph::Node>::const_iterator iter2 = ccSet.begin();
        iter2 != ccSet.end(); ++iter2)
      {
        // Remove all outgoing edges
        for (lemon::ListGraph::OutArcIt e(putativeGraph.g, *iter2); e!=INVALID; ++e)
        {
          putativeGraph.g.erase(e);
          const IndexT Idu = (*putativeGraph.map_nodeMapIndex)[putativeGraph.g.target(e)];
          const IndexT Idv = (*putativeGraph.map_nodeMapIndex)[putativeGraph.g.source(e)];
          matching::PairwiseMatches::iterator iterM = _pairwiseMatches.find(std::make_pair(Idu,Idv));
          if( iterM != _pairwiseMatches.end())
          {
            _pairwiseMatches.erase(iterM);
          }
          else // Try to find the opposite directed edge
          {
            iterM = _pairwiseMatches.find(std::make_pair(Idv,Idu));
            if( iterM != _pairwiseMatches.end())
              _pairwiseMatches.erase(iterM);
          }
        }
      }
    }
  }

  // Save the graph after cleaning:
  graph::exportToGraphvizData((fs::path(_outputDirectory) / "cleanedGraph").string(), putativeGraph.g);

  std::cout << "\n"
    << "Cardinal of nodes: " << lemon::countNodes(putativeGraph.g) << "\n"
    << "Cardinal of edges: " << lemon::countEdges(putativeGraph.g) << std::endl
    << std::endl;

  return true;
}

} // namespace aliceVision
