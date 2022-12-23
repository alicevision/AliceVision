#include "brackets.hpp"

#include <fstream>

#include <aliceVision/numeric/numeric.hpp>

#include <boost/filesystem.hpp>

namespace aliceVision {
namespace hdr {

bool estimateBracketsFromSfmData(std::vector<std::vector<std::shared_ptr<sfmData::View>>>& groups, const sfmData::SfMData& sfmData, size_t countBrackets)
{
    size_t countImages = sfmData.getViews().size();
    if(countImages == 0)
    {
        return false;
    }

    if ((countBrackets > 0) && ((countImages % countBrackets) != 0))
    {
        return false;
    }

    const sfmData::Views & views = sfmData.getViews();

    // Order views by their image names (without path and extension to make sure we handle rotated images)
    std::vector<std::shared_ptr<sfmData::View>> viewsOrderedByName;
    for(auto& viewIt : sfmData.getViews())
    {
        viewsOrderedByName.push_back(viewIt.second);
    }
    std::sort(viewsOrderedByName.begin(), viewsOrderedByName.end(),
        [](const std::shared_ptr<sfmData::View>& a, const std::shared_ptr<sfmData::View>& b) -> bool {
            if(a == nullptr || b == nullptr)
                return true;

            boost::filesystem::path path_a(a->getImagePath());
            boost::filesystem::path path_b(b->getImagePath());

            return (path_a.stem().string() < path_b.stem().string());
        });

    // Print a warning if the aperture changes.
    std::set<float> fnumbers;
    for(auto& view : viewsOrderedByName) {
        fnumbers.insert(view->getMetadataFNumber());
    }
    
    if(fnumbers.size() != 1) {
        ALICEVISION_LOG_WARNING("Different apertures amongst the dataset. For correct HDR, you should only change "
                                "the shutter speed (and eventually the ISO).");
        ALICEVISION_LOG_WARNING("Used f-numbers:");
        for(auto f : fnumbers) {
            ALICEVISION_LOG_WARNING(" * " << f);
        }
    }
    
    std::vector<std::shared_ptr<sfmData::View>> group;
    std::vector<double> exposures;
    for(auto& view : viewsOrderedByName)
    {
        if (countBrackets > 0)
        {
            group.push_back(view);
            if(group.size() == countBrackets)
            {
                groups.push_back(group);
                group.clear();
            }
        }
        else
        {
            // Automatically determines the number of brackets
            double exp = view->getCameraExposureSetting().getExposure();
            if(!exposures.empty() && exp != exposures.back() && exp == exposures.front())
            {
                groups.push_back(group);
                group.clear();
                exposures.clear();
            }
            exposures.push_back(exp);
            group.push_back(view);
        }
    }
    
    if (!group.empty())
    {
        groups.push_back(group);
    }

    for(auto & group : groups)
    {
        // Sort all images by exposure time
        std::sort(group.begin(), group.end(),
                  [](const std::shared_ptr<sfmData::View>& a, const std::shared_ptr<sfmData::View>& b) -> bool {
                      if(a == nullptr || b == nullptr)
                          return true;
                      return (a->getCameraExposureSetting().getExposure() < b->getCameraExposureSetting().getExposure());
                  });
    }

    return true;
}

void selectTargetViews(std::vector<std::shared_ptr<sfmData::View>> & out_targetViews, const std::vector<std::vector<std::shared_ptr<sfmData::View>>> & groups, int offsetRefBracketIndex, const std::string& targetIndexesFilename)
{
    // If targetIndexesFilename cannot be opened or is not valid target views are derived from the middle exposed views
    // For odd number, there is no ambiguity on the middle image.
    // For even number, we arbitrarily choose the more exposed view (as we usually have more under-exposed images than over-exposed).
    const int viewNumberPerGroup = groups[0].size();
    const int middleIndex = viewNumberPerGroup / 2;
    int targetIndex = middleIndex + offsetRefBracketIndex;

    out_targetViews.clear();

    if ((targetIndex >= 0) && (targetIndex < viewNumberPerGroup))
    {
        ALICEVISION_LOG_INFO("Use offsetRefBracketIndex parameter");
    }
    else // try to use the index in the file
    {
        ALICEVISION_LOG_INFO("offsetRefBracketIndex parameter out of range, read file containing the target index");
        std::ifstream file(targetIndexesFilename);
        if (!file)
            ALICEVISION_THROW_ERROR("Failed to open file: " << targetIndexesFilename);
        std::string line;
        if (!std::getline(file, line))
            ALICEVISION_THROW_ERROR("Failed to read from file: " << targetIndexesFilename);
        targetIndex = atoi(line.c_str());
    }

        //Set the ldr ancestors id
        for (auto v : group)
        {
            group[targetIndex]->addAncestor(v->getViewId());
        }

        for (int i = 0; i < groups.size(); ++i)
        {
            out_targetViews.push_back(groups[i][targetIndexes[i]]);
        }
    }
    return;
}

}
}
