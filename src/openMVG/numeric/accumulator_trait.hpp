// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef _OPENMVG_NUMERIC_ACCUMULATOR_TRAIT_HPP_
#define _OPENMVG_NUMERIC_ACCUMULATOR_TRAIT_HPP_

/// Accumulator trait to perform safe summation over a specified type
namespace aliceVision {

template<typename T>
struct Accumulator { typedef T Type; };
template<>
struct Accumulator<unsigned char>  { typedef float Type; };
template<>
struct Accumulator<unsigned short> { typedef float Type; };
template<>
struct Accumulator<unsigned int> { typedef float Type; };
template<>
struct Accumulator<char>   { typedef float Type; };
template<>
struct Accumulator<short>  { typedef float Type; };
template<>
struct Accumulator<int> { typedef float Type; };
template<>
struct Accumulator<bool>  { typedef unsigned int Type; };

} // namespace aliceVision

#endif //_OPENMVG_NUMERIC_ACCUMULATOR_TRAIT_HPP_