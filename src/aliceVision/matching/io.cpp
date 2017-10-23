// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "io.hpp"

#include "aliceVision/matching/IndMatch.hpp"
#include <aliceVision/config.hpp>
#include <aliceVision/system/Logger.hpp>

#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>

#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"

#include <map>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace aliceVision {
namespace matching {

bool LoadMatchFile(
  PairwiseMatches & matches,
  const std::string & folder,
  const std::string & filename)
{
  if(!stlplus::is_file(stlplus::create_filespec(folder, filename)))
    return false;

  const std::string ext = stlplus::extension_part(filename);
  const std::string filepath = folder + "/" + filename;

  if (ext == "txt")
  {
    std::ifstream stream(filepath.c_str());
    if (!stream.is_open())
      return false;

    // Read from the text file
    // I J
    // nbDescType
    // descType matchesCount
    // idx idx
    // ...
    // descType matchesCount
    // idx idx
    // ...
    std::size_t I = 0;
    std::size_t J = 0;
    std::size_t nbDescType = 0;
    while(stream >> I >> J >> nbDescType)
    {
      for(std::size_t i = 0; i < nbDescType; ++i)
      {
        std::string descTypeStr;
        std::size_t nbMatches = 0;
        // Read descType and number of matches
        stream >> descTypeStr >> nbMatches;

        feature::EImageDescriberType descType = feature::EImageDescriberType_stringToEnum(descTypeStr);
        std::vector<IndMatch> matchesPerDesc(nbMatches);
        // Read all matches
        for (std::size_t i = 0; i < nbMatches; ++i)
        {
          stream >> matchesPerDesc[i];
        }
        matches[std::make_pair(I,J)][descType] = std::move(matchesPerDesc);
      }
    }
    stream.close();
    return true;
  }
  else if (ext == "bin")
  {
    std::ifstream stream (filepath.c_str(), std::ios::in | std::ios::binary);
    if (!stream.is_open())
      return false;

    cereal::PortableBinaryInputArchive archive(stream);
    PairwiseMatches loadMatches;
    archive(loadMatches);
    stream.close();
    if(matches.empty())
    {
      matches.swap(loadMatches);
    }
    else
    {
      // merge the loaded matches into the output
      for(const auto& v: loadMatches)
      {
        matches[v.first] = v.second;
      }
    }
    return true;
  }
  else
  {
    ALICEVISION_LOG_WARNING("Unknown matching file format: " << ext);
  }
  return false;
}


void filterMatchesByViews(
  PairwiseMatches & matches,
  const std::set<IndexT> & viewsKeys)
{
  matching::PairwiseMatches filteredMatches;
  for (matching::PairwiseMatches::const_iterator iter = matches.begin();
    iter != matches.end();
    ++iter)
  {
    if(viewsKeys.find(iter->first.first) != viewsKeys.end() &&
       viewsKeys.find(iter->first.second) != viewsKeys.end())
    {
      filteredMatches.insert(*iter);
    }
  }
  matches.swap(filteredMatches);
}


void filterMatchesByDesc(
  PairwiseMatches & allMatches,
  const std::vector<feature::EImageDescriberType>& descTypesFilter)
{
  matching::PairwiseMatches filteredMatches;
  for(const auto& matchesPerDesc: allMatches)
  {
    for(const auto& matches: matchesPerDesc.second)
    {
      const IndMatches& m = matches.second;
      // if current descType in descTypesFilter
      if(std::find(descTypesFilter.begin(), descTypesFilter.end(), matches.first) != descTypesFilter.end())
        filteredMatches[matchesPerDesc.first][matches.first] = m;
    }
  }
  allMatches.swap(filteredMatches);
}


bool LoadMatchFilePerImage(
  PairwiseMatches & matches,
  const std::set<IndexT> & viewsKeys,
  const std::string & folder,
  const std::string & basename)
{
  int nbLoadedMatchFiles = 0;
  // Load one match file per image
  #pragma omp parallel for num_threads(3)
  for(ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(viewsKeys.size()); ++i)
  {
    std::set<IndexT>::const_iterator it = viewsKeys.begin();
    std::advance(it, i);
    const IndexT idView = *it;
    const std::string matchFilename = std::to_string(idView) + "." + basename;
    PairwiseMatches fileMatches;
    if(!LoadMatchFile(fileMatches, folder, matchFilename))
    {
      #pragma omp critical
      {
        ALICEVISION_LOG_WARNING("Unable to load match file: " << folder << "/" << matchFilename);
      }
      continue;
    }
    #pragma omp critical
    {
      ++nbLoadedMatchFiles;
      // merge the loaded matches into the output
      for(const auto& v: fileMatches)
      {
        matches[v.first] = v.second;
      }
    }
  }
  if( nbLoadedMatchFiles == 0 )
  {
    ALICEVISION_LOG_WARNING("No matches file loaded in: " << folder);
    return false;
  }
  ALICEVISION_LOG_TRACE("Matches per image pair");
  for(const auto& imagePairIt: matches)
  {
    std::stringstream ss;
    ss << " * " << imagePairIt.first.first << "-" << imagePairIt.first.second << ": " << imagePairIt.second.getNbAllMatches() << "    ";
    for(const auto& matchesPerDeskIt: imagePairIt.second)
    {
       ss << " [" << feature::EImageDescriberType_enumToString(matchesPerDeskIt.first) << ": " << matchesPerDeskIt.second.size() << "]";
    }
    ALICEVISION_LOG_TRACE(ss.str());
  }
  return true;
}

bool Load(
  PairwiseMatches & matches,
  const std::set<IndexT> & viewsKeysFilter,
  const std::string & folder,
  const std::vector<feature::EImageDescriberType>& descTypesFilter,
  const std::string & mode)
{
  bool res = false;
  const std::string basename = "matches." + mode;
  if(stlplus::is_file(stlplus::create_filespec(folder, basename + ".txt")))
  {
    res = LoadMatchFile(matches, folder, basename + ".txt");
  }
  else if(stlplus::is_file(stlplus::create_filespec(folder, basename + ".bin")))
  {
    res = LoadMatchFile(matches, folder, basename + ".bin");
  }
  else if(!stlplus::folder_wildcard(folder, "*."+basename+".txt", false, true).empty())
  {
    res = LoadMatchFilePerImage(matches, viewsKeysFilter, folder, basename + ".txt");
  }
  else if(!stlplus::folder_wildcard(folder, "*."+basename+".bin", false, true).empty())
  {
    res = LoadMatchFilePerImage(matches, viewsKeysFilter, folder, basename + ".bin");
  }
  if(!res)
    return res;

  if(!viewsKeysFilter.empty())
    filterMatchesByViews(matches, viewsKeysFilter);

  if(!descTypesFilter.empty())
    filterMatchesByDesc(matches, descTypesFilter);

  ALICEVISION_LOG_TRACE("Matches per image pair");
  for(const auto& imagePairIt: matches)
  {
    std::stringstream ss;
    ss << " * " << imagePairIt.first.first << "-" << imagePairIt.first.second << ": " << imagePairIt.second.getNbAllMatches() << "    ";
    for(const auto& matchesPerDeskIt: imagePairIt.second)
    {
       ss << " [" << feature::EImageDescriberType_enumToString(matchesPerDeskIt.first) << ": " << matchesPerDeskIt.second.size() << "]";
    }
    ALICEVISION_LOG_TRACE(ss.str());
  }

  return res;
}


class MatchExporter
{
private:
  void saveTxt(
    const std::string & filepath,
    const PairwiseMatches::const_iterator& matchBegin,
    const PairwiseMatches::const_iterator& matchEnd)
  {
    std::ofstream stream(filepath.c_str(), std::ios::out);
    for(PairwiseMatches::const_iterator match = matchBegin;
      match != matchEnd;
      ++match)
    {
      const std::size_t I = match->first.first;
      const std::size_t J = match->first.second;
      const MatchesPerDescType & matchesPerDesc = match->second;
      stream << I << " " << J << '\n'
             << matchesPerDesc.size() << '\n';
      for(const auto& m: matchesPerDesc)
      {
        stream << feature::EImageDescriberType_enumToString(m.first) << " " << m.second.size() << '\n';
        copy(m.second.begin(), m.second.end(),
             std::ostream_iterator<IndMatch>(stream, "\n"));
      }
    }
  }

  void saveBinary(
    const std::string & filepath,
    const PairwiseMatches::const_iterator& matchBegin,
    const PairwiseMatches::const_iterator& matchEnd)
  {
    std::ofstream stream(filepath.c_str(), std::ios::out | std::ios::binary);
    cereal::PortableBinaryOutputArchive archive(stream);
    const PairwiseMatches matchesToExport(matchBegin, matchEnd);
    archive(matchesToExport);
    stream.close();
  }

public:
  MatchExporter(
    const PairwiseMatches& matches,
    const std::string& folder,
    const std::string& filename)
    : m_matches(matches)
    , m_directory(folder)
    , m_filename(filename)
    , m_ext(stlplus::extension_part(filename))
  {
  }

  ~MatchExporter()
  {
  }
  
  void saveGlobalFile()
  {
    const std::string filepath = m_directory + "/" + m_filename;
    if(m_ext == "txt")
    {
      saveTxt(filepath, m_matches.begin(), m_matches.end());
    }
    else if(m_ext == "bin")
    {
      saveBinary(filepath, m_matches.begin(), m_matches.end());
    }
    else
    {
      throw std::runtime_error(std::string("Unknown matching file format: ") + m_ext);
    }
  }

  /// Export matches into separate files, one for each image.
  void saveOneFilePerImage()
  {
    std::set<IndexT> keys;
    std::transform(
        m_matches.begin(), m_matches.end(),
        std::inserter(keys, keys.begin()),
        [](const PairwiseMatches::value_type &v) { return v.first.first; });

    PairwiseMatches::const_iterator matchBegin = m_matches.begin();
    PairwiseMatches::const_iterator matchEnd = m_matches.end();
    for(IndexT key: keys)
    {
      PairwiseMatches::const_iterator match = matchBegin;
      while(match->first.first == key && match != matchEnd)
      {
        ++match;
      }
      const std::string filepath = m_directory + "/" + std::to_string(key) + "." + m_filename;
      ALICEVISION_LOG_DEBUG("Export Matches in: " << filepath);
      
      if(m_ext == "txt")
      {
        saveTxt(filepath, matchBegin, match);
      }
      else if(m_ext == "bin")
      {
        saveBinary(filepath, matchBegin, match);
      }
      else
      {
        throw std::runtime_error(std::string("Unknown matching file format: ") + m_ext);
      }
      matchBegin = match;
    }
  }

public:
  const PairwiseMatches& m_matches;
  const std::string m_ext;
  std::string m_directory;
  std::string m_filename;
};


bool Save(
  const PairwiseMatches & matches,
  const std::string & folder,
  const std::string & mode,
  const std::string & extension,
  bool matchFilePerImage)
{
  const std::string filename = "matches." + mode + "." + extension;
  MatchExporter exporter(matches, folder, filename);
  if(matchFilePerImage)
    exporter.saveOneFilePerImage();
  else
    exporter.saveGlobalFile();
  return true;
}

}  // namespace matching
}  // namespace aliceVision
