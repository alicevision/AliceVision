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

void selectTargetViews(std::vector<std::shared_ptr<sfmData::View>> & out_targetViews, const std::vector<std::vector<std::shared_ptr<sfmData::View>>> & groups, int offsetRefBracketIndex, const std::string& lumaStatFilepath, const double meanTargetedLuma)
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
    else // try to use the luminance statistics of the LDR images stored in the file
    {
        ALICEVISION_LOG_INFO("offsetRefBracketIndex parameter out of range, read file containing luminance statistics to compute an estimation");
        std::ifstream file(lumaStatFilepath);
        if (!file)
            ALICEVISION_THROW_ERROR("Failed to open file: " << lumaStatFilepath);
        std::vector<std::string> lines;
        std::string line;
        while (std::getline(file, line))
        {
            lines.push_back(line);
        }
        if ((lines.size() < 3) || (atoi(lines[0].c_str()) != groups.size()) || (atoi(lines[1].c_str()) < groups[0].size()))
        {
            ALICEVISION_THROW_ERROR("File: " << lumaStatFilepath << " is not a valid file");
        }
        int nbGroup = atoi(lines[0].c_str());
        int nbExp = atoi(lines[1].c_str());

        std::vector<double> v_lumaMeanMean;

        for (int i = 0; i < nbExp; ++i)
        {
            double lumaMeanMean = 0.0;
            for (int j = 0; j < nbGroup; ++j)
            {
                std::istringstream iss(lines[3 + j * nbExp + i]);
                aliceVision::IndexT srcId;
                int exposure, nbItem;
                double lumaMean, lumaMin, lumaMax;
                if (!(iss >> srcId >> exposure >> nbItem >> lumaMean >> lumaMin >> lumaMax))
                {
                    ALICEVISION_THROW_ERROR("File: " << lumaStatFilepath << " is not a valid file");
                }
                lumaMeanMean += lumaMean;
            }
            v_lumaMeanMean.push_back(lumaMeanMean / nbGroup);
        }

        // Makes sure the luminance curve is monotonic
        int firstvalid = -1;
        int lastvalid = 0;
        for (std::size_t k = 1; k < v_lumaMeanMean.size(); ++k)
        {
            bool valid = false;

            if (v_lumaMeanMean[k] > v_lumaMeanMean[k - 1])
            {
                valid = true;
            }

            if (valid)
            {
                if (firstvalid < 0)
                {
                    firstvalid = int(k) - 1;
                }
                lastvalid = int(k);
            }
            else
            {
                if (lastvalid != 0)
                {
                    break;
                }
            }
        }

        if (lastvalid >= firstvalid && firstvalid >= 0)
        {
            double minDiffWithLumaTarget = 1000.0;
            targetIndex = 0;

            for (int k = firstvalid; k <= lastvalid; ++k)
            {
                const double diffWithLumaTarget = fabs(v_lumaMeanMean[k] - meanTargetedLuma);
                if (diffWithLumaTarget < minDiffWithLumaTarget)
                {
                    minDiffWithLumaTarget = diffWithLumaTarget;
                    targetIndex = k;
                }
            }
            ALICEVISION_LOG_INFO("offsetRefBracketIndex parameter automaticaly set to " << targetIndex);
        }
        else
        {
            targetIndex = middleIndex;
            ALICEVISION_LOG_WARNING("Non valid luminance statistics file, offsetRefBracketIndex parameter set to medium exposure " << targetIndex);
        }
    }

    for (auto& group : groups)
    {
        //Set the ldr ancestors id
        for (auto v : group)
        {
            group[targetIndex]->addAncestor(v->getViewId());
        }

        out_targetViews.push_back(group[targetIndex]);
    }
    return;
}

}
}
