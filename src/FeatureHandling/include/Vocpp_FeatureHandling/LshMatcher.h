/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_LSH_BINARY_MATCHER_H
#define VOCPP_LSH_BINARY_MATCHER_H

#include <opencv2/core/core.hpp>
#include <Vocpp_FeatureHandling/Common.h>
#include <Vocpp_FeatureHandling/BriefDescriptor.h>
#include <unordered_map>

namespace VOCPP
{
namespace FeatureHandling
{

/**
  * /brief LSH binary feature matcher class
  */
class LshMatcher
{
public:

    static const uint32_t s_lengthHashFunc = 2U;
    
    /**
      * \brief Constructor
      */
    LshMatcher(const uint32_t& in_maxDistance = 40U, const uint32_t& in_numHashFuncs = 16U, 
        /** sanity check parameter **/
        const double& in_maxPixelDist = 200.);

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
    LshMatcher& operator=(const LshMatcher&);
    LshMatcher(const LshMatcher&);

    void GenerateHashFuncs();

    void IndexDescriptions(const std::vector<BinaryFeatureDescription>& in_desc, std::unordered_map<std::bitset<s_lengthHashFunc*8U>, std::vector<uint32_t>>& out_bucketTable);

    uint32_t m_maxDistance; ///< maximum Hamming distance between binary descriptions for reporting a match
    uint32_t m_numHashFuncs;
    double m_maxPixelDist;
    std::vector<std::vector<uint32_t>> m_hashFuncs;
};

} //namespace FeatureHandling
} //namespace VOCPP

#endif /* VOCPP_LSH_BINARY_MATCHER_H */
