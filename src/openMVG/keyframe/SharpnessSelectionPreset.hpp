#pragma once

#include <string>
#include <iostream>
#include <algorithm>
#include <stdexcept>

namespace openMVG {
namespace keyframe {

/**
 * @brief Sharpness selection preset enum
 */
enum class ESharpnessSelectionPreset : std::uint8_t
{
  ULTRA
  , HIGH
  , NORMAL
  , LOW
  , VERY_LOW
  , NONE
};

/**
 * @brief convert an enum ESharpnessSelectionPreset to its corresponding string
 * @param ESharpnessSelectionPreset
 * @return String
 */
inline std::string ESharpnessSelectionPreset_enumToString(ESharpnessSelectionPreset sharpnessPreset)
{
  switch(sharpnessPreset)
  {
    case ESharpnessSelectionPreset::ULTRA:    return "ultra";
    case ESharpnessSelectionPreset::HIGH:     return "high";
    case ESharpnessSelectionPreset::NORMAL:   return "normal";
    case ESharpnessSelectionPreset::LOW:      return "low";
    case ESharpnessSelectionPreset::VERY_LOW: return "very_low";
    case ESharpnessSelectionPreset::NONE:     return "none";
  }
  throw std::out_of_range("Invalid sharpnessPreset enum");
}

/**
 * @brief convert a string sharpnessPreset to its corresponding enum ESharpnessSelectionPreset
 * @param String
 * @return ESharpnessSelectionPreset
 */
inline ESharpnessSelectionPreset ESharpnessSelectionPreset_stringToEnum(const std::string& sharpnessPreset)
{
  std::string preset = sharpnessPreset;
  std::transform(preset.begin(), preset.end(), preset.begin(), ::tolower); //tolower

  if(preset == "ultra")    return ESharpnessSelectionPreset::ULTRA;
  if(preset == "high")     return ESharpnessSelectionPreset::HIGH;
  if(preset == "normal")   return ESharpnessSelectionPreset::NORMAL;
  if(preset == "low")      return ESharpnessSelectionPreset::LOW;
  if(preset == "very_low") return ESharpnessSelectionPreset::VERY_LOW;
  if(preset == "none")     return ESharpnessSelectionPreset::NONE;

  throw std::out_of_range("Invalid sharpnessPreset : " + sharpnessPreset);
}

inline std::ostream& operator<<(std::ostream& os, const ESharpnessSelectionPreset sharpnessPreset)
{
  os << ESharpnessSelectionPreset_enumToString(sharpnessPreset);
  return os;
}

inline std::istream& operator>>(std::istream& in, ESharpnessSelectionPreset &sharpnessPreset)
{
  std::string token;
  in >> token;
  sharpnessPreset = ESharpnessSelectionPreset_stringToEnum(token);
  return in;
}

} // namespace keyframe
} // namespace openMVG
