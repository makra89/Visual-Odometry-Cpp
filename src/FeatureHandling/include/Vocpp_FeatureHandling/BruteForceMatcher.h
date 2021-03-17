/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_BRUTE_FORCE_BINARY_MATCHER_H
#define VOCPP_BRUTE_FORCE_BINARY_MATCHER_H

#include <opencv2/core/core.hpp>
#include <Vocpp_FeatureHandling/Common.h>
#include <Vocpp_FeatureHandling/BriefDescriptor.h>

namespace VOCPP
{
namespace FeatureHandling
{

/**
  * /brief Brute force binary feature matcher class
  */
class BruteForceMatcher
{
public:
    
    /**
      * \brief Constructor
      */
    BruteForceMatcher(const uint32_t& in_maxDistance = 50U);

    /**
      * \brief Compares binary feature descriptions and return matches.
      *
      * \param[in] in_descFirst first set of feature descriptions
      * \param[in] in_descSecond second set of feature descriptions
      * \param[out] out_matches matches between description sets
      * \return True if at least one match has been found, false otherwise
      */
    bool MatchDesriptions(const std::vector<BinaryFeatureDescription>& in_descFirst, const std::vector<BinaryFeatureDescription>& in_descSecond,
        std::vector<BinaryDescriptionMatch>& out_matches);

private:
    // It is not allowed to copy the matcher directly
    BruteForceMatcher& operator=(const BruteForceMatcher&);
    BruteForceMatcher(const BruteForceMatcher&);

    uint32_t m_maxDistance; ///< maximum Hamming distance between binary descriptions for reporting a match
};

} //namespace FeatureHandling
} //namespace VOCPP

#endif /* VOCPP_BRUTE_FORCE_BINARY_MATCHER_H */
