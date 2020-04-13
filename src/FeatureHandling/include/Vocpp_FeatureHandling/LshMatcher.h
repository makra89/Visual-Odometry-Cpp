/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#ifndef VOCPP_BRUTE_FORCE_BINARY_MATCHER_H
#define VOCPP_BRUTE_FORCE_BINARY_MATCHER_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <Vocpp_FeatureHandling/Common.h>
#include <Vocpp_FeatureHandling/BriefDescriptor.h>
#include <unordered_map>

namespace VOCPP
{
namespace FeatureHandling
{

/**
  * /brief Brute force binary feature matcher class
  */
class LshMatcher
{
public:
    
    /**
      * \brief Constructor
      */
    LshMatcher(const int& in_maxDistance = 50, const int& in_numHashFuncs = 30, const int& in_lengthHashFunc = 15,
        const int& in_lengthDescription = 256);

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

    void IndexDescriptions(const std::vector<BinaryFeatureDescription>& in_desc, std::unordered_map<int, std::vector<int>>& out_bucketTable);
    /**
      * \brief Compute Hamming Distance for two BRIEF descriptions
      *
      * \param[ín] in_first first (binary) description
      * \param[ín] in_second second (binary) description
      * \return True distance of descriptions
      */
    int ComputeHammingDistance(const BinaryFeatureDescription& in_first, const BinaryFeatureDescription& in_second) const;

    int m_maxDistance; ///< maximum Hamming distance between binary descriptions for reporting a match
    int m_numHashFuncs;
    int m_lengthHashFunc;
    int m_lengthDescription;
    std::vector<std::vector<int>> m_hashFuncs;
};

} //namespace FeatureHandling
} //namespace VOCPP

#endif /* VOCPP_BRUTE_FORCE_BINARY_MATCHER_H */
