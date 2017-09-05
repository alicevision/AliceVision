// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_PAIRWISE_ADJACENCY_DISPLAY_H
#define OPENMVG_PAIRWISE_ADJACENCY_DISPLAY_H

#include "third_party/vectorGraphics/svgDrawer.hpp"
#include "openMVG/matching/indMatch.hpp"

namespace openMVG  {
namespace matching {

/// Display pair wises matches as an Adjacency matrix in svg format
inline void PairwiseMatchingToAdjacencyMatrixSVG(const size_t NbImages,
  const matching::PairwiseMatches & map_Matches,
  const std::string & sOutName)
{
  if ( !map_Matches.empty())
  {
    float scaleFactor = 5.0f;
    svg::svgDrawer svgStream((NbImages+3)*5, (NbImages+3)*5);
    // Go along all possible pair
    for (size_t I = 0; I < NbImages; ++I) {
      for (size_t J = 0; J < NbImages; ++J) {
        // If the pair have matches display a blue boxes at I,J position.
        matching::PairwiseMatches::const_iterator iterSearch = map_Matches.find(std::make_pair(I,J));
        if (iterSearch != map_Matches.end() && !iterSearch->second.empty())
        {
          // Display as a tooltip: (IndexI, IndexJ NbMatches)
          std::ostringstream os;
          os << "(" << J << "," << I << " " << iterSearch->second.getNbAllMatches() <<")";
          svgStream.drawSquare(J*scaleFactor, I*scaleFactor, scaleFactor/2.0f,
            svg::svgStyle().fill("blue").noStroke());
        } // HINT : THINK ABOUT OPACITY [0.4 -> 1.0] TO EXPRESS MATCH COUNT
      }
    }
    // Display axes with 0 -> NbImages annotation : _|
    std::ostringstream osNbImages;
    osNbImages << NbImages;
    svgStream.drawText((NbImages+1)*scaleFactor, scaleFactor, scaleFactor, "0", "black");
    svgStream.drawText((NbImages+1)*scaleFactor,
      (NbImages)*scaleFactor - scaleFactor, scaleFactor, osNbImages.str(), "black");
    svgStream.drawLine((NbImages+1)*scaleFactor, 2*scaleFactor,
      (NbImages+1)*scaleFactor, (NbImages)*scaleFactor - 2*scaleFactor,
      svg::svgStyle().stroke("black", 1.0));

    svgStream.drawText(scaleFactor, (NbImages+1)*scaleFactor, scaleFactor, "0", "black");
    svgStream.drawText((NbImages)*scaleFactor - scaleFactor,
      (NbImages+1)*scaleFactor, scaleFactor, osNbImages.str(), "black");
    svgStream.drawLine(2*scaleFactor, (NbImages+1)*scaleFactor,
      (NbImages)*scaleFactor - 2*scaleFactor, (NbImages+1)*scaleFactor,
      svg::svgStyle().stroke("black", 1.0));

    std::ofstream svgFileStream( sOutName.c_str());
    svgFileStream << svgStream.closeSvgFile().str();
  }
}

} // namespace matching
} // namespace openMVG

#endif // OPENMVG_PAIRWISE_ADJACENCY_DISPLAY_H
