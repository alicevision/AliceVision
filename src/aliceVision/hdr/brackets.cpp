#include "brackets.hpp"

#include <fstream>

#include <aliceVision/numeric/numeric.hpp>
#include <cmath>

#include <filesystem>

namespace aliceVision {
namespace hdr {

bool estimateBracketsFromSfmData(std::vector<std::vector<std::shared_ptr<sfmData::View>>>& groups,
                                 const sfmData::SfMData& sfmData,
                                 size_t countBrackets)
{
    groups.clear();

    size_t countImages = sfmData.getViews().size();
    if (countImages == 0)
    {
        return false;
    }

    const sfmData::Views& views = sfmData.getViews();

    std::set<float> fnumbers;
    std::vector<LuminanceInfo> luminances;
    for (auto& viewIt : sfmData.getViews())
    {
        IndexT id = viewIt.first;
        auto view = viewIt.second; 
        if (view ==nullptr)
        {
            continue;
        }

        fnumbers.insert(view->getImage().getMetadataFNumber());
        double exp = view->getImage().getCameraExposureSetting().getExposure();
        std::string path = view->getImage().getImagePath();

        LuminanceInfo li(viewIt.first, path, exp);
        luminances.push_back(li);
    }

    if (fnumbers.size() != 1)
    {
        ALICEVISION_LOG_WARNING("Different apertures amongst the dataset. For correct HDR, you should only change "
                                "the shutter speed (and eventually the ISO).");
        ALICEVISION_LOG_WARNING("Used f-numbers:");
        for (auto f : fnumbers)
        {
            ALICEVISION_LOG_WARNING(" * " << f);
        }
    }

    std::vector<std::vector<IndexT>> groupsids;
    
    if (countBrackets == 0)
    {
        groupsids = estimateGroups(luminances);
    } 
    else 
    {
        groupsids = divideGroups(luminances, countBrackets);
    }

    for (const auto & group : groupsids)
    {
        std::vector<std::shared_ptr<sfmData::View>> gview;
        for (const auto & id : group)
        {
            gview.push_back(sfmData.getViews().at(id));
        }
        groups.push_back(gview);
    }

    return true;
}

int selectTargetViews(std::vector<std::shared_ptr<sfmData::View>>& targetViews,
                      std::vector<std::vector<std::shared_ptr<sfmData::View>>>& groups,
                      const int offsetRefBracketIndex,
                      const std::string& lumaStatFilepath,
                      const double meanTargetedLuma)
{
    // If targetIndexesFilename cannot be opened or is not valid, an error is thrown.
    // For odd number, there is no ambiguity on the middle image.
    // For even number, we arbitrarily choose the more exposed view (as we usually have more under-exposed images than over-exposed).
    const int viewNumberPerGroup = groups[0].size();
    const int middleIndex = viewNumberPerGroup / 2;
    int targetIndex = middleIndex + offsetRefBracketIndex;

    targetViews.clear();

    if ((targetIndex >= 0) && (targetIndex < viewNumberPerGroup))
    {
        ALICEVISION_LOG_INFO("Use offsetRefBracketIndex parameter");
    }
    else  // try to use the luminance statistics of the LDR images stored in the file
    {
        ALICEVISION_LOG_INFO("offsetRefBracketIndex parameter out of range, "
                             << "read file containing luminance statistics to compute an estimation");
        std::ifstream file(lumaStatFilepath);
        if (!file)
        {
            ALICEVISION_THROW_ERROR("Failed to open file: " << lumaStatFilepath);
        }
        std::vector<std::string> lines;
        std::string line;
        while (std::getline(file, line))
        {
            lines.push_back(line);
        }
        if ((lines.size() < 3) || (std::stoi(lines[0]) != groups.size()) || (std::stoi(lines[1]) < groups[0].size()) ||
            (lines.size() < 3 + std::stoi(lines[0]) * std::stoi(lines[1])))
        {
            ALICEVISION_THROW_ERROR("File '" << lumaStatFilepath << "' is not a valid file");
        }
        int nbGroup = std::stoi(lines[0]);
        int nbExp = std::stoi(lines[1]);

        std::vector<double> v_lumaMeanMean;

        for (int i = 0; i < nbExp; ++i)
        {
            double lumaMeanMean = 0.0;
            double nbValidViews = 0;
            for (int j = 0; j < nbGroup; ++j)
            {
                std::istringstream iss(lines[3 + j * nbExp + i]);
                aliceVision::IndexT srcId;
                int nbItem;
                double exposure, lumaMean, lumaMin, lumaMax;
                if (!(iss >> srcId >> exposure >> nbItem >> lumaMean >> lumaMin >> lumaMax))
                {
                    ALICEVISION_THROW_ERROR("File '" << lumaStatFilepath << "' is not a valid file");
                }

                // Discard dummy luminance info (with exposure set to -1.0) added at calibration stage if samples are
                // missing for a view
                if (exposure > 0.0)
                {
                    lumaMeanMean += lumaMean;
                    ++nbValidViews;
                }
            }
            v_lumaMeanMean.push_back(lumaMeanMean / nbValidViews);
        }

        // Adjust last index to avoid non increasing luminance curve due to saturation in highlights
        int lastIdx = v_lumaMeanMean.size() - 1;
        while ((lastIdx > 1) && ((v_lumaMeanMean[lastIdx] < v_lumaMeanMean[lastIdx - 1]) || (v_lumaMeanMean[lastIdx] < v_lumaMeanMean[lastIdx - 2])))
        {
            lastIdx--;
        }

        double minDiffWithLumaTarget = 1000.0;
        targetIndex = 0;

        for (int k = 0; k < lastIdx; ++k)
        {
            const double diffWithLumaTarget =
              (v_lumaMeanMean[k] > meanTargetedLuma) ? (v_lumaMeanMean[k] - meanTargetedLuma) : (meanTargetedLuma - v_lumaMeanMean[k]);
            if (diffWithLumaTarget < minDiffWithLumaTarget)
            {
                minDiffWithLumaTarget = diffWithLumaTarget;
                targetIndex = k;
            }
        }
        ALICEVISION_LOG_INFO("offsetRefBracketIndex parameter automatically set to " << targetIndex - middleIndex);
    }

    for (auto& group : groups)
    {
        // Set the ldr ancestors
        for (auto v : group)
        {
            group[targetIndex]->addAncestor(v->getViewId());
        }

        targetViews.push_back(group[targetIndex]);
    }
    return targetIndex;
}

std::vector<std::vector<LuminanceInfo>> splitBasedir(const std::vector<LuminanceInfo> & luminanceInfos)
{
    std::vector<std::vector<LuminanceInfo>> splitted;

    if (luminanceInfos.size() == 0)
    {
        return splitted;
    }

    //Ignore non existing files
    //Remove relative paths
    //Remove symlinks
    //This will enable correct path comparison
    std::vector<LuminanceInfo> correctedPaths;
    for (const auto item : luminanceInfos)
    {
        if (!std::filesystem::exists(item.mpath))
        {
            continue;
        }

        LuminanceInfo corrected = item;
        corrected.mpath = std::filesystem::canonical(item.mpath);

        correctedPaths.push_back(corrected);
    }

    //Sort luminanceinfos by names
    std::sort(correctedPaths.begin(),
              correctedPaths.end(),
              [](const LuminanceInfo& a, const LuminanceInfo& b) -> bool {
                return (a.mpath < b.mpath);
              });

    //Split items per base directory
    std::vector<LuminanceInfo> current;
    for (int index = 0; index < correctedPaths.size(); index++)
    {
        if (index == 0)
        {
            current.push_back(correctedPaths[index]);
            continue;
        }

        std::filesystem::path pathCur(correctedPaths[index].mpath);
        std::filesystem::path pathPrev(correctedPaths[index - 1].mpath);

        if (pathCur.parent_path() != pathPrev.parent_path())
        {
            splitted.push_back(current);
            current.clear();
        }
        
        current.push_back(correctedPaths[index]);
    }
    splitted.push_back(current);

    return splitted;
}

std::vector<std::vector<LuminanceInfo>> splitMonotonics(const std::vector<LuminanceInfo> & luminanceInfos)
{
    std::vector<std::vector<LuminanceInfo>> splitted;

    if (luminanceInfos.size() == 0)
    {
        return splitted;
    }

    //Split the luminanceInfos into groups which have monotonic values 
    //(either increasing or decreasing)
    std::vector<LuminanceInfo> current;
    current.push_back(luminanceInfos[0]);
    for (int index = 1; index < luminanceInfos.size(); index++)
    {   
        float val = luminanceInfos[index].mexposure;
        float prev = luminanceInfos[index - 1].mexposure;


        if (val == prev)
        {
            splitted.push_back(current);
            
            current.clear();
            current.push_back(luminanceInfos[index]);
            continue;
        }

        if (index + 1 == luminanceInfos.size())
        {
            current.push_back(luminanceInfos[index]);
            continue;
        }

        float next = luminanceInfos[index + 1].mexposure;

        //If sign is negative, then the function is not locally monotonic
        float sign = (next - val) * (val - prev);

        if (sign < 0)
        {
            current.push_back(luminanceInfos[index]);
            splitted.push_back(current);
            current.clear();
            //Extremity is added on both groups
            current.push_back(luminanceInfos[index]);
        }
        else
        {
            current.push_back(luminanceInfos[index]);
        }
    }

    //Close the last group
    splitted.push_back(current);

    return splitted;
}

/**
* @brief assume ref is smaller than larger
* Try to find a subpart of larger which has the same set of exposures that smaller
* @param smaller the set to compare
* @param larger the set where the subpart should be
* @return the index of the subpart or -1 if not found
*/
int extractIndex(const std::vector<LuminanceInfo> & smaller, const std::vector<LuminanceInfo> & larger)
{
    int largerSize = larger.size();
    int smallerSize = smaller.size();
    int diff = largerSize - smallerSize;

    //For all continuous subparts of the erased sequence
    for (int indexStart = 0; indexStart < diff; indexStart++)
    {
        //Check that the subpart is the same set of exposures
        bool allCorrect = true;
        for (int pos = 0; pos < smallerSize; pos++)
        {
            if (smaller[pos].mexposure != larger[indexStart + pos].mexposure)
            {
                allCorrect = false;
            }
        }

        if (allCorrect)
        {
            return indexStart;
        }
    }

    return -1;
}


std::vector<std::vector<IndexT>> estimateGroups(const std::vector<LuminanceInfo> & luminanceInfos)
{
    std::vector<std::vector<IndexT>> groups;
    if (luminanceInfos.size() == 0)
    {
        return groups;
    }

    //Split and order the items using path
    std::vector<std::vector<LuminanceInfo>> splitted = splitBasedir(luminanceInfos);
    //Create monotonic groups
    std::vector<std::vector<LuminanceInfo>> monotonics;
    for (const auto & luminanceInfoOneDir : splitted)
    {
        std::vector<std::vector<LuminanceInfo>> lmonotonics = splitMonotonics(luminanceInfoOneDir);
        monotonics.insert(monotonics.end(), lmonotonics.begin(), lmonotonics.end());
    }

    //Sort the voters groups by exposure increasing
    for (auto & group : monotonics)
    {
        std::sort(group.begin(), 
                  group.end(), 
                  [](const LuminanceInfo& a, const LuminanceInfo& b) {
                    return (a.mexposure < b.mexposure);
                  });
    }

    // Vote for the best bracket count
    std::map<size_t, int> counters;
    for (const auto& group : monotonics)
    {
        size_t bracketCount = group.size();
        if (counters.find(bracketCount) != counters.end())
        {
            counters[bracketCount]++;
        }
        else
        {
            counters[bracketCount] = 1;
        }
    }

    int maxSize = 0;
    int bestBracketCount = 0;
    for (const auto& item : counters)
    {
        if (item.second > maxSize)
        {
            maxSize = item.second;
            bestBracketCount = item.first;
        }
        else if (item.second == maxSize && item.first > bestBracketCount)
        {
            // If two brackets size have the same vote number,
            // keep the larger one (this avoids keeping only the outlier)
            bestBracketCount = item.first;
        }
    }

    // Only keep voters with the majority bracket size
    auto groupIt = monotonics.begin();
    std::vector<std::vector<LuminanceInfo>> eraseds;
    while (groupIt != monotonics.end())
    {
        if (groupIt->size() != bestBracketCount)
        {
            eraseds.push_back(*groupIt);
            groupIt = monotonics.erase(groupIt);
        }
        else
        {
            groupIt++;
        }
    }

    //Try to push back erased
    for (const auto & erased: eraseds)
    {
        //If erased is larger than the most voted, then 
        //Maybe it contains outliers. Try to find a correct subpart
        int diff = int(erased.size()) - bestBracketCount;
        if (diff < 0)
        {
            continue;
        }
        

        //Compare with all valid monotonics
        int offset = -1;
        for (const auto& monotonic : monotonics)
        {
            offset = extractIndex(monotonic, erased);
            if (offset >= 0)
            {
                break;
            }
        }

        //If something found, put it back on list of monotonics
        if (offset >= 0)
        {
            std::vector<LuminanceInfo> subpart;
            for (int index = 0; index < bestBracketCount; index++)
            {
                subpart.push_back(erased[offset + index]);
            }
            monotonics.push_back(subpart);
        }
    }

    //check coherency
    bool coherency = true;
    for (int idref = 1; idref < monotonics.size(); ++idref)
    {
        const int idprev = idref - 1;
        for (int idExposure = 0; idExposure < monotonics[idref].size(); ++idExposure)
        {
            if (!(monotonics[idref][idExposure].mexposure == monotonics[idprev][idExposure].mexposure))
            {
                ALICEVISION_LOG_WARNING("Non consistant exposures between poses have been detected.\
                Most likely the dataset has been captured with an automatic exposure mode enabled.\
                Final result can be impacted.");
                coherency = false;
                
                break;
            }
        }

        if (!coherency)
        {
            break;
        }
    }

    for (const auto & monotonic : monotonics)
    {
        std::vector<IndexT> group;
        for (const auto & li : monotonic)
        {
            group.push_back(li.mviewId);
        }
        groups.push_back(group);
    }

    ALICEVISION_LOG_INFO("Groups found : "  << monotonics.size());

    return groups;
}

std::vector<std::vector<aliceVision::IndexT>> divideGroups(const std::vector<LuminanceInfo> & luminanceInfos, unsigned bracketSize)
{
    std::vector<std::vector<IndexT>> groups;
    if (luminanceInfos.size() == 0)
    {
        return groups;
    }

    //Split and order the items using path
    std::vector<std::vector<LuminanceInfo>> splitted = splitBasedir(luminanceInfos);

    std::vector<std::vector<LuminanceInfo>> divided;

    for (const auto & item : splitted)
    {
        if (item.size() % bracketSize != 0)
        {
            ALICEVISION_LOG_ERROR("Input bracket size is not compatible with the number of images");
            return groups;
        }

        //For each group of bracketSize items
        for (int index = 0; index < item.size(); index += bracketSize)
        {   
            //Create a new set
            std::vector<LuminanceInfo> part;
            for (int bracket = 0; bracket < bracketSize; bracket++)
            {
                part.push_back(item[index + bracket]);
            }

            divided.push_back(part);
        }
    }

    //check coherency
    bool coherency = true;
    for (int idref = 1; idref < divided.size(); ++idref)
    {
        const int idprev = idref - 1;
        for (int idExposure = 0; idExposure < divided[idref].size(); ++idExposure)
        {
            if (!(divided[idref][idExposure].mexposure == divided[idprev][idExposure].mexposure))
            {
                ALICEVISION_LOG_WARNING("Non consistant exposures between poses have been detected.\
                Most likely the dataset has been captured with an automatic exposure mode enabled.\
                Final result can be impacted.");
                coherency = false;
                
                break;
            }
        }

        if (!coherency)
        {
            break;
        }
    }

    for (const auto & item : divided)
    {
        std::vector<IndexT> group;
        for (const auto & li : item)
        {
            group.push_back(li.mviewId);
        }
        groups.push_back(group);
    }

    return groups;
}

}  // namespace hdr
}  // namespace aliceVision