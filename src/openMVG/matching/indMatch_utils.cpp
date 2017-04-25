#include "indMatch_utils.hpp"

#include "openMVG/matching/indMatch.hpp"

#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>

#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <map>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace openMVG {
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
    size_t I = 0;
    size_t J = 0;
    size_t nbDescType = 0;
    while(stream >> I >> J >> nbDescType)
    {
      for(int i =0; i < nbDescType; ++i)
      {
        std::string descTypeStr;
        size_t nbMatches = 0;
        // Read descType and number of matches
        stream >> descTypeStr >> nbMatches;

        features::EImageDescriberType descType = features::EImageDescriberType_stringToEnum(descTypeStr);
        std::vector<IndMatch> matchesPerDesc(nbMatches);
        // Read all matches
        for (size_t i = 0; i < nbMatches; ++i)
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
    OPENMVG_LOG_WARNING("Unknown matching file format: " << ext);
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
  const std::vector<features::EImageDescriberType>& descTypesFilter)
{
  std::cout << "filter load nb describer : " << descTypesFilter.size() << std::endl;
  matching::PairwiseMatches filteredMatches;
  for(const auto& matchesPerDesc: allMatches)
  {
    std::cout << "nb describers for image pair: " << matchesPerDesc.second.size() << std::endl;
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
#ifdef OPENMVG_USE_OPENMP
    #pragma omp parallel for num_threads(3)
#endif
  for(ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(viewsKeys.size()); ++i)
  {
    std::set<IndexT>::const_iterator it = viewsKeys.begin();
    std::advance(it, i);
    const IndexT idView = *it;
    const std::string matchFilename = std::to_string(idView) + "." + basename;
    PairwiseMatches fileMatches;
    if(!LoadMatchFile(fileMatches, folder, matchFilename))
    {
#ifdef OPENMVG_USE_OPENMP
      #pragma omp critical
#endif
      {
        OPENMVG_LOG_WARNING("Unable to load match file: " << folder << "/" << matchFilename);
      }
      continue;
    }
#ifdef OPENMVG_USE_OPENMP
      #pragma omp critical
#endif
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
    OPENMVG_LOG_WARNING("No matches file loaded in: " << folder);
    return false;
  }
  return true;
}

bool Load(
  PairwiseMatches & matches,
  const std::set<IndexT> & viewsKeysFilter,
  const std::string & folder,
  const std::vector<features::EImageDescriberType>& descTypesFilter,
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
      const size_t I = match->first.first;
      const size_t J = match->first.second;
      const MatchesPerDescType & matchesPerDesc = match->second;
      stream << I << " " << J << '\n'
             << matchesPerDesc.size() << '\n';
      for(const auto& m: matchesPerDesc)
      {
        stream << features::EImageDescriberType_enumToString(m.first) << " " << m.second.size() << '\n';
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
      OPENMVG_LOG_DEBUG("Export Matches in: " << filepath);
      
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
}  // namespace openMVG
