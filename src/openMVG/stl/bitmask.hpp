// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef ALICEVISION_STL_BITMASK_H
#define ALICEVISION_STL_BITMASK_H

// Taken from boost
#define ALICEVISION_BITMASK(Bitmask)                                            \
                                                                          \
  inline Bitmask operator| (Bitmask x , Bitmask y )                       \
  { return static_cast<Bitmask>( static_cast<std::int32_t>(x)     \
      | static_cast<std::int32_t>(y)); }                          \
                                                                          \
  inline Bitmask operator& (Bitmask x , Bitmask y )                       \
  { return static_cast<Bitmask>( static_cast<std::int32_t>(x)     \
      & static_cast<std::int32_t>(y)); }                          \
                                                                          \
  inline Bitmask operator^ (Bitmask x , Bitmask y )                       \
  { return static_cast<Bitmask>( static_cast<std::int32_t>(x)     \
      ^ static_cast<std::int32_t>(y)); }                          \
                                                                          \
  inline Bitmask operator~ (Bitmask x )                                   \
  { return static_cast<Bitmask>(~static_cast<std::int32_t>(x)); } \
                                                                          \
  inline Bitmask & operator&=(Bitmask & x , Bitmask y)                    \
  { x = x & y ; return x ; }                                              \
                                                                          \
  inline Bitmask & operator|=(Bitmask & x , Bitmask y)                    \
  { x = x | y ; return x ; }                                              \
                                                                          \
  inline Bitmask & operator^=(Bitmask & x , Bitmask y)                    \
  { x = x ^ y ; return x ; }                                              


#endif  // ALICEVISION_STL_HASH_H
