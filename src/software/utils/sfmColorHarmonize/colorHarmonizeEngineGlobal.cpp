// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2013 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "colorHarmonizeEngineGlobal.hpp"
#include "software/utils/sfmHelper/sfmIOHelper.hpp"

#include <aliceVision/system/ProgressDisplay.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/utils/filesIO.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/graph/graph.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/image/all.hpp>
// load features per view
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

#include <numeric>
#include <iomanip>
#include <iterator>
#include <algorithm>
#include <functional>
#include <filesystem>
#include <sstream>

namespace fs = std::filesystem;

namespace aliceVision {

using namespace lemon;
using namespace aliceVision::image;
using namespace aliceVision::matching;
using namespace aliceVision::lInfinity;
using namespace aliceVision::sfm;
using namespace aliceVision::sfmData;

typedef feature::PointFeature FeatureT;
typedef std::vector<FeatureT> featsT;

EHistogramSelectionMethod EHistogramSelectionMethod_stringToEnum(const std::string& histogramSelectionMethod)
{
    if (histogramSelectionMethod == "full_frame")
        return EHistogramSelectionMethod::eHistogramHarmonizeFullFrame;
    if (histogramSelectionMethod == "matched_points")
        return EHistogramSelectionMethod::eHistogramHarmonizeMatchedPoints;
    if (histogramSelectionMethod == "VLD_segments")
        return EHistogramSelectionMethod::eHistogramHarmonizeVLDSegment;

    throw std::invalid_argument("Invalid selection method: " + histogramSelectionMethod);
}

std::string EHistogramSelectionMethod_enumToString(const EHistogramSelectionMethod histogramSelectionMethod)
{
    if (histogramSelectionMethod == EHistogramSelectionMethod::eHistogramHarmonizeFullFrame)
        return "full_frame";
    if (histogramSelectionMethod == EHistogramSelectionMethod::eHistogramHarmonizeMatchedPoints)
        return "matched_points";
    if (histogramSelectionMethod == EHistogramSelectionMethod::eHistogramHarmonizeVLDSegment)
        return "VLD_segments";

    throw std::invalid_argument("Unrecognized EHistogramSelectionMethod: " + std::to_string(int(histogramSelectionMethod)));
}

std::ostream& operator<<(std::ostream& os, EHistogramSelectionMethod p) { return os << EHistogramSelectionMethod_enumToString(p); }

std::istream& operator>>(std::istream& in, EHistogramSelectionMethod& p)
{
    std::string token(std::istreambuf_iterator<char>(in), {});
    p = EHistogramSelectionMethod_stringToEnum(token);
    return in;
}

ColorHarmonizationEngineGlobal::ColorHarmonizationEngineGlobal(const std::string& sfmDataFilename,
                                                               const std::vector<std::string>& featuresFolders,
                                                               const std::vector<std::string>& matchesFolders,
                                                               const std::string& outputDirectory,
                                                               const std::vector<feature::EImageDescriberType>& descTypes,
                                                               EHistogramSelectionMethod selectionMethod,
                                                               int imgRef)
  : _sfmDataFilename(sfmDataFilename),
    _featuresFolders(featuresFolders),
    _matchesFolders(matchesFolders),
    _outputDirectory(outputDirectory),
    _descTypes(descTypes),
    _selectionMethod(selectionMethod),
    _imgRef(imgRef)
{
    if (!utils::exists(outputDirectory))
        fs::create_directory(outputDirectory);
}

ColorHarmonizationEngineGlobal::~ColorHarmonizationEngineGlobal() {}

inline void pauseProcess()
{
    unsigned char i;
    std::cout << "\nPause : type key and press enter: ";
    std::cin >> i;
}

bool ColorHarmonizationEngineGlobal::process()
{
    const std::string vecHarmonizeMethod[1] = {"quantifiedGainCompensation"};
    const int harmonizeMethod = 0;
    std::error_code ec;

    //-------------------
    // Load data
    //-------------------

    if (!readInputData())
        return false;
    if (_pairwiseMatches.empty())
    {
        std::cout << std::endl << "Matches file is empty" << std::endl;
        return false;
    }

    //-- Remove EG with poor support:
    for (matching::PairwiseMatches::iterator matchesPerViewIt = _pairwiseMatches.begin(); matchesPerViewIt != _pairwiseMatches.end();)
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
        graph::exportToGraphvizData((fs::path(_outputDirectory) / "input_graph_poor_supportRemoved").string(), putativeGraph.g);
    }

    //-------------------
    // Keep the largest CC in the image graph
    //-------------------
    if (!cleanGraph())
    {
        std::cout << std::endl << "There is no largest CC in the graph" << std::endl;
        return false;
    }

    //-------------------
    // Compute remaining camera node Id
    //-------------------

    std::map<size_t, size_t> mapCameraNodeToCameraIndex;  // graph node Id to 0->Ncam
    std::map<size_t, size_t> mapCameraIndexTocameraNode;  // 0->Ncam correspondance to graph node Id
    std::set<size_t> setIndexImage;

    for (size_t i = 0; i < _pairwiseMatches.size(); ++i)
    {
        matching::PairwiseMatches::const_iterator iter = _pairwiseMatches.begin();
        std::advance(iter, i);

        const size_t I = iter->first.first;
        const size_t J = iter->first.second;
        setIndexImage.insert(I);
        setIndexImage.insert(J);
    }

    for (std::set<size_t>::const_iterator iterSet = setIndexImage.begin(); iterSet != setIndexImage.end(); ++iterSet)
    {
        mapCameraIndexTocameraNode[std::distance(setIndexImage.begin(), iterSet)] = *iterSet;
        mapCameraNodeToCameraIndex[*iterSet] = std::distance(setIndexImage.begin(), iterSet);
    }

    std::cout << "\nRemaining cameras after CC filter : \n"
              << mapCameraIndexTocameraNode.size() << " from a total of " << _fileNames.size() << std::endl;

    size_t bin = 256;
    double minvalue = 0.0;
    double maxvalue = 255.0;

    // For each edge computes the selection masks and histograms (for the RGB channels)
    std::vector<relativeColorHistogramEdge> mapRelativeHistograms[3];
    mapRelativeHistograms[0].resize(_pairwiseMatches.size());
    mapRelativeHistograms[1].resize(_pairwiseMatches.size());
    mapRelativeHistograms[2].resize(_pairwiseMatches.size());

    for (size_t i = 0; i < _pairwiseMatches.size(); ++i)
    {
        matching::PairwiseMatches::const_iterator iter = _pairwiseMatches.begin();
        std::advance(iter, i);

        const size_t viewI = iter->first.first;
        const size_t viewJ = iter->first.second;

        const MatchesPerDescType& matchesPerDesc = iter->second;

        //-- Edges names:
        std::pair<std::string, std::string> p_imaNames;
        p_imaNames = make_pair(_fileNames[viewI], _fileNames[viewJ]);
        std::cout << "Current edge : " << fs::path(p_imaNames.first).filename().string() << "\t" << fs::path(p_imaNames.second).filename().string()
                  << std::endl;

        //-- Compute the masks from the data selection:
        Image<unsigned char> maskI(_imageSize[viewI].first, _imageSize[viewI].second);
        Image<unsigned char> maskJ(_imageSize[viewJ].first, _imageSize[viewJ].second);

        switch (_selectionMethod)
        {
            case EHistogramSelectionMethod::eHistogramHarmonizeFullFrame:
            {
                colorHarmonization::CommonDataByPair_fullFrame dataSelector(p_imaNames.first, p_imaNames.second);
                dataSelector.computeMask(maskI, maskJ);
            }
            break;
            case EHistogramSelectionMethod::eHistogramHarmonizeMatchedPoints:
            {
                int circleSize = 10;
                colorHarmonization::CommonDataByPair_matchedPoints dataSelector(p_imaNames.first,
                                                                                p_imaNames.second,
                                                                                matchesPerDesc,
                                                                                _regionsPerView.getRegionsPerDesc(viewI),
                                                                                _regionsPerView.getRegionsPerDesc(viewJ),
                                                                                circleSize);
                dataSelector.computeMask(maskI, maskJ);
            }
            break;
            case EHistogramSelectionMethod::eHistogramHarmonizeVLDSegment:
            {
                maskI.fill(0);
                maskJ.fill(0);

                for (const auto& matchesIt : matchesPerDesc)
                {
                    const feature::EImageDescriberType descType = matchesIt.first;
                    const IndMatches& matches = matchesIt.second;
                    colorHarmonization::CommonDataByPair_vldSegment dataSelector(p_imaNames.first,
                                                                                 p_imaNames.second,
                                                                                 matches,
                                                                                 _regionsPerView.getRegions(viewI, descType).Features(),
                                                                                 _regionsPerView.getRegions(viewJ, descType).Features());

                    dataSelector.computeMask(maskI, maskJ);
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
            std::string sEdge = _fileNames[viewI] + "_" + _fileNames[viewJ];
            sEdge = (fs::path(_outputDirectory) / sEdge).string();

            if (!utils::exists(sEdge))
                fs::create_directory(sEdge);

            std::string outFilenameI = "00_mask_I.png";
            outFilenameI = (fs::path(sEdge) / outFilenameI).string();

            std::string outFilenameJ = "00_mask_J.png";
            outFilenameJ = (fs::path(sEdge) / outFilenameJ).string();
            writeImage(outFilenameI, maskI, image::ImageWriteOptions());
            writeImage(outFilenameJ, maskJ, image::ImageWriteOptions());
        }

        //-- Compute the histograms
        Image<RGBColor> imageI, imageJ;
        readImage(p_imaNames.first, imageI, image::EImageColorSpace::LINEAR);
        readImage(p_imaNames.second, imageJ, image::EImageColorSpace::LINEAR);

        utils::Histogram<double> histoI(minvalue, maxvalue, bin);
        utils::Histogram<double> histoJ(minvalue, maxvalue, bin);

        int channelIndex = 0;  // RED channel
        colorHarmonization::CommonDataByPair::computeHisto(histoI, maskI, channelIndex, imageI);
        colorHarmonization::CommonDataByPair::computeHisto(histoJ, maskJ, channelIndex, imageJ);
        relativeColorHistogramEdge& edgeR = mapRelativeHistograms[channelIndex][i];
        edgeR = relativeColorHistogramEdge(mapCameraNodeToCameraIndex[viewI], mapCameraNodeToCameraIndex[viewJ], histoI.GetHist(), histoJ.GetHist());

        histoI = histoJ = utils::Histogram<double>(minvalue, maxvalue, bin);
        channelIndex = 1;  // GREEN channel
        colorHarmonization::CommonDataByPair::computeHisto(histoI, maskI, channelIndex, imageI);
        colorHarmonization::CommonDataByPair::computeHisto(histoJ, maskJ, channelIndex, imageJ);
        relativeColorHistogramEdge& edgeG = mapRelativeHistograms[channelIndex][i];
        edgeG = relativeColorHistogramEdge(mapCameraNodeToCameraIndex[viewI], mapCameraNodeToCameraIndex[viewJ], histoI.GetHist(), histoJ.GetHist());

        histoI = histoJ = utils::Histogram<double>(minvalue, maxvalue, bin);
        channelIndex = 2;  // BLUE channel
        colorHarmonization::CommonDataByPair::computeHisto(histoI, maskI, channelIndex, imageI);
        colorHarmonization::CommonDataByPair::computeHisto(histoJ, maskJ, channelIndex, imageJ);
        relativeColorHistogramEdge& edgeB = mapRelativeHistograms[channelIndex][i];
        edgeB = relativeColorHistogramEdge(mapCameraNodeToCameraIndex[viewI], mapCameraNodeToCameraIndex[viewJ], histoI.GetHist(), histoJ.GetHist());
    }

    std::cout << "\n -- \n SOLVE for color consistency with linear programming\n --" << std::endl;
    //-- Solve for the gains and offsets:
    std::vector<size_t> vecIndexToFix;
    vecIndexToFix.push_back(mapCameraNodeToCameraIndex[_imgRef]);

    using namespace aliceVision::linearProgramming;

    std::vector<double> vecSolutionR(_fileNames.size() * 2 + 1);
    std::vector<double> vecSolutionG(_fileNames.size() * 2 + 1);
    std::vector<double> vecSolutionB(_fileNames.size() * 2 + 1);

    aliceVision::system::Timer timer;

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
    typedef MOSEKSolver SOLVER_LP_T;
#else
    typedef OSI_CISolverWrapper SOLVER_LP_T;
#endif
    // Red channel
    {
        SOLVER_LP_T lpSolver(vecSolutionR.size());

        GainOffsetConstraintBuilder cstBuilder(mapRelativeHistograms[0], vecIndexToFix);
        LPConstraintsSparse constraint;
        cstBuilder.Build(constraint);
        lpSolver.setup(constraint);
        lpSolver.solve();
        lpSolver.getSolution(vecSolutionR);
    }
    // Green channel
    {
        SOLVER_LP_T lpSolver(vecSolutionG.size());

        GainOffsetConstraintBuilder cstBuilder(mapRelativeHistograms[1], vecIndexToFix);
        LPConstraintsSparse constraint;
        cstBuilder.Build(constraint);
        lpSolver.setup(constraint);
        lpSolver.solve();
        lpSolver.getSolution(vecSolutionG);
    }
    // Blue channel
    {
        SOLVER_LP_T lpSolver(vecSolutionB.size());

        GainOffsetConstraintBuilder cstBuilder(mapRelativeHistograms[2], vecIndexToFix);
        LPConstraintsSparse constraint;
        cstBuilder.Build(constraint);
        lpSolver.setup(constraint);
        lpSolver.solve();
        lpSolver.getSolution(vecSolutionB);
    }

    std::cout << std::endl
              << " ColorHarmonization solving on a graph with: " << _pairwiseMatches.size() << " edges took (s): " << timer.elapsed() << std::endl
              << "LInfinity fitting error: \n"
              << "- for the red channel is: " << vecSolutionR.back() << " gray level(s)" << std::endl
              << "- for the green channel is: " << vecSolutionG.back() << " gray level(s)" << std::endl
              << "- for the blue channel is: " << vecSolutionB.back() << " gray level(s)" << std::endl;

    std::cout << "\n\nFound solution_r:\n";
    std::copy(vecSolutionR.begin(), vecSolutionR.end(), std::ostream_iterator<double>(std::cout, " "));

    std::cout << "\n\nFound solution_g:\n";
    std::copy(vecSolutionG.begin(), vecSolutionG.end(), std::ostream_iterator<double>(std::cout, " "));

    std::cout << "\n\nFound solution_b:\n";
    std::copy(vecSolutionB.begin(), vecSolutionB.end(), std::ostream_iterator<double>(std::cout, " "));
    std::cout << std::endl;

    std::cout << "\n\nThere is :\n" << setIndexImage.size() << " images to transform." << std::endl;

    //-> convert solution to gain offset and creation of the LUT per image
    auto progressDisplay = system::createConsoleProgressDisplay(setIndexImage.size(), std::cout);
    for (std::set<size_t>::const_iterator iterSet = setIndexImage.begin(); iterSet != setIndexImage.end(); ++iterSet, ++progressDisplay)
    {
        const size_t imaNum = *iterSet;
        typedef Eigen::Matrix<double, 256, 1> Vec256;
        std::vector<Vec256> vecMapLut(3);

        const size_t nodeIndex = std::distance(setIndexImage.begin(), iterSet);

        const double gR = vecSolutionR[nodeIndex * 2];
        const double offsetR = vecSolutionR[nodeIndex * 2 + 1];
        const double gG = vecSolutionG[nodeIndex * 2];
        const double offsetG = vecSolutionG[nodeIndex * 2 + 1];
        const double gB = vecSolutionB[nodeIndex * 2];
        const double offsetB = vecSolutionB[nodeIndex * 2 + 1];

        for (size_t k = 0; k < 256; ++k)
        {
            vecMapLut[0][k] = clamp(k * gR + offsetR, 0., 255.);
            vecMapLut[1][k] = clamp(k * gG + offsetG, 0., 255.);
            vecMapLut[2][k] = clamp(k * gB + offsetB, 0., 255.);
        }

        Image<RGBColor> imageC;
        readImage(_fileNames[imaNum], imageC, image::EImageColorSpace::LINEAR);

#pragma omp parallel for
        for (int j = 0; j < imageC.height(); ++j)
        {
            for (int i = 0; i < imageC.width(); ++i)
            {
                imageC(j, i)[0] = clamp(vecMapLut[0][imageC(j, i)[0]], 0., 255.);
                imageC(j, i)[1] = clamp(vecMapLut[1][imageC(j, i)[1]], 0., 255.);
                imageC(j, i)[2] = clamp(vecMapLut[2][imageC(j, i)[2]], 0., 255.);
            }
        }

        const std::string outFolder =
          (fs::path(_outputDirectory) / (EHistogramSelectionMethod_enumToString(_selectionMethod) + "_" + vecHarmonizeMethod[harmonizeMethod]))
            .string();
        if (!utils::exists(outFolder))
            fs::create_directory(outFolder);
        const std::string outFilename = (fs::path(outFolder) / fs::path(_fileNames[imaNum]).filename()).string();

        writeImage(outFilename, imageC, image::ImageWriteOptions());
    }
    return true;
}

bool ColorHarmonizationEngineGlobal::readInputData()
{
    if (!fs::is_directory(_outputDirectory))
    {
        std::cerr << "The output folder is not a valid folder" << std::endl;
        return false;
    }

    if (!fs::is_regular_file(_sfmDataFilename))
    {
        std::cerr << "Invalid input sfm_data file: " << _sfmDataFilename << std::endl;
        return false;
    }

    // a. Read input scenes views
    SfMData sfmData;
    if (!sfmDataIO::load(sfmData, _sfmDataFilename, sfmDataIO::ESfMData::VIEWS))
    {
        std::cerr << "The input file \"" << _sfmDataFilename << "\" cannot be read" << std::endl;
        return false;
    }

    // Read images names
    for (Views::const_iterator iter = sfmData.getViews().begin(); iter != sfmData.getViews().end(); ++iter)
    {
        const View* v = iter->second.get();
        _fileNames.push_back(v->getImage().getImagePath());
        _imageSize.push_back(std::make_pair(v->getImage().getWidth(), v->getImage().getHeight()));
    }

    // b. Read matches
    if (!sfm::loadPairwiseMatches(_pairwiseMatches, sfmData, _matchesFolders, _descTypes))
    {
        std::cerr << "Can't load matches files" << std::endl;
        return false;
    }

    // Read features:
    if (!sfm::loadRegionsPerView(_regionsPerView, sfmData, _featuresFolders, _descTypes))
    {
        std::cerr << "Can't load feature files" << std::endl;
        return false;
    }

    graph::indexedGraph putativeGraph(getImagePairs(_pairwiseMatches));

    // Save the graph before cleaning:
    graph::exportToGraphvizData((fs::path(_outputDirectory) / "initialGraph").string(), putativeGraph.g);

    return true;
}

bool ColorHarmonizationEngineGlobal::cleanGraph()
{
    // Create a graph from pairwise correspondences:
    // - keep the largest connected component.

    graph::indexedGraph putativeGraph(getImagePairs(_pairwiseMatches));

    // Save the graph before cleaning:
    graph::exportToGraphvizData((fs::path(_outputDirectory) / "initialGraph").string(), putativeGraph.g);

    const int connectedComponentCount = lemon::countConnectedComponents(putativeGraph.g);
    std::cout << "\n"
              << "ColorHarmonizationEngineGlobal::CleanGraph() :: => connected Component cardinal: " << connectedComponentCount << std::endl;

    if (connectedComponentCount > 1)  // If more than one CC, keep the largest
    {
        // Search the largest CC index
        const std::map<IndexT, std::set<lemon::ListGraph::Node>> map_subgraphs =
          aliceVision::graph::exportGraphToMapSubgraphs<lemon::ListGraph, IndexT>(putativeGraph.g);
        size_t count = std::numeric_limits<size_t>::min();
        std::map<IndexT, std::set<lemon::ListGraph::Node>>::const_iterator iterLargestCC = map_subgraphs.end();
        for (std::map<IndexT, std::set<lemon::ListGraph::Node>>::const_iterator iter = map_subgraphs.begin(); iter != map_subgraphs.end(); ++iter)
        {
            if (iter->second.size() > count)
            {
                count = iter->second.size();
                iterLargestCC = iter;
            }
            std::cout << "Connected component of size : " << iter->second.size() << std::endl;
        }

        //-- Remove all nodes that are not listed in the largest CC
        for (std::map<IndexT, std::set<lemon::ListGraph::Node>>::const_iterator iter = map_subgraphs.begin(); iter != map_subgraphs.end(); ++iter)
        {
            if (iter == iterLargestCC)  // Skip this CC since it's the one we want to keep
                continue;

            const std::set<lemon::ListGraph::Node>& ccSet = iter->second;
            for (std::set<lemon::ListGraph::Node>::const_iterator iter2 = ccSet.begin(); iter2 != ccSet.end(); ++iter2)
            {
                // Remove all outgoing edges
                for (lemon::ListGraph::OutArcIt e(putativeGraph.g, *iter2); e != INVALID; ++e)
                {
                    putativeGraph.g.erase(e);
                    const IndexT Idu = (*putativeGraph.map_nodeMapIndex)[putativeGraph.g.target(e)];
                    const IndexT Idv = (*putativeGraph.map_nodeMapIndex)[putativeGraph.g.source(e)];
                    matching::PairwiseMatches::iterator iterM = _pairwiseMatches.find(std::make_pair(Idu, Idv));
                    if (iterM != _pairwiseMatches.end())
                    {
                        _pairwiseMatches.erase(iterM);
                    }
                    else  // Try to find the opposite directed edge
                    {
                        iterM = _pairwiseMatches.find(std::make_pair(Idv, Idu));
                        if (iterM != _pairwiseMatches.end())
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

}  // namespace aliceVision
