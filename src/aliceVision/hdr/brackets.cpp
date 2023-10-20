#include "brackets.hpp"

#include <fstream>

#include <aliceVision/numeric/numeric.hpp>

#include <boost/filesystem.hpp>

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

    // Order views by their image names (without path and extension to make sure we handle rotated images)
    std::vector<std::shared_ptr<sfmData::View>> viewsOrderedByName;
    for (auto& viewIt : sfmData.getViews())
    {
        viewsOrderedByName.push_back(viewIt.second);
    }

    std::sort(viewsOrderedByName.begin(),
              viewsOrderedByName.end(),
              [](const std::shared_ptr<sfmData::View>& a, const std::shared_ptr<sfmData::View>& b) -> bool {
                  if (a == nullptr || b == nullptr)
                      return true;

                  boost::filesystem::path path_a(a->getImage().getImagePath());
                  boost::filesystem::path path_b(b->getImage().getImagePath());

                  return (path_a.stem().string() < path_b.stem().string());
              });

    // Print a warning if the aperture changes.
    std::set<float> fnumbers;
    for (auto& view : viewsOrderedByName)
    {
        fnumbers.insert(view->getImage().getMetadataFNumber());
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

    std::vector<std::shared_ptr<sfmData::View>> group;
    double lastExposure = std::numeric_limits<double>::min();
    for (auto& view : viewsOrderedByName)
    {
        if (countBrackets > 0)
        {
            group.push_back(view);
            if (group.size() == countBrackets)
            {
                groups.push_back(group);
                group.clear();
            }
        }
        else
        {
            // Automatically determines the number of brackets
            double exp = view->getImage().getCameraExposureSetting().getExposure();
            if (exp < lastExposure)
            {
                groups.push_back(group);
                group.clear();
            }

            lastExposure = exp;
            group.push_back(view);
        }
    }

    if (!group.empty())
    {
        groups.push_back(group);
    }

    // Vote for the best bracket count
    std::map<size_t, int> counters;
    for (const auto& group : groups)
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

    // Only keep groups with the majority bracket size
    auto groupIt = groups.begin();
    while (groupIt != groups.end())
    {
        if (groupIt->size() != bestBracketCount)
        {
            groupIt = groups.erase(groupIt);
        }
        else
        {
            groupIt++;
        }
    }

    std::vector<std::vector<sfmData::ExposureSetting>> v_exposuresSetting;
    for (auto& group : groups)
    {
        // Sort all images by exposure time
        std::sort(group.begin(), group.end(), [](const std::shared_ptr<sfmData::View>& a, const std::shared_ptr<sfmData::View>& b) -> bool {
            if (a == nullptr || b == nullptr)
                return true;
            return (a->getImage().getCameraExposureSetting().getExposure() < b->getImage().getCameraExposureSetting().getExposure());
        });

        std::vector<sfmData::ExposureSetting> exposuresSetting;
        for (auto& v : group)
        {
            exposuresSetting.push_back(v->getImage().getCameraExposureSetting());
        }
        v_exposuresSetting.push_back(exposuresSetting);
    }

    // Check exposure consistency between group
    if (v_exposuresSetting.size() > 1)
    {
        for (int g = 1; g < v_exposuresSetting.size(); ++g)
        {
            for (int e = 0; e < v_exposuresSetting[g].size(); ++e)
            {
                if (!(v_exposuresSetting[g][e] == v_exposuresSetting[g - 1][e]))
                {
                    ALICEVISION_LOG_WARNING("Non consistant exposures between poses have been detected. Most likely the dataset has been captured "
                                            "with an automatic exposure mode enabled. Final result can be impacted.");
                    g = v_exposuresSetting.size();
                    break;
                }
            }
        }
    }

    return true;
}

int selectTargetViews(std::vector<std::shared_ptr<sfmData::View>>& out_targetViews,
                      std::vector<std::vector<std::shared_ptr<sfmData::View>>>& groups,
                      int offsetRefBracketIndex,
                      const std::string& lumaStatFilepath,
                      const double meanTargetedLuma)
{
    // If targetIndexesFilename cannot be opened or is not valid, an error is thrown.
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
            group[targetIndex]->addAncestorImage(v->getImageInfo());
        }

        out_targetViews.push_back(group[targetIndex]);
    }
    return targetIndex;
}

}  // namespace hdr
}  // namespace aliceVision
