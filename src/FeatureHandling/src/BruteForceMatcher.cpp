/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include<Vocpp_FeatureHandling/BruteForceMatcher.h>
#include<Vocpp_Utils/NumericalUtilities.h>
#include<Vocpp_Utils/TracingImpl.h>

namespace VOCPP
{
namespace FeatureHandling
{

BruteForceMatcher::BruteForceMatcher(const uint32_t& in_maxDistance) : m_maxDistance(in_maxDistance)
{
}

bool BruteForceMatcher::MatchDesriptions(const std::vector<BinaryFeatureDescription>& in_descFirst,
    const std::vector<BinaryFeatureDescription>& in_descSecond, std::vector<BinaryDescriptionMatch>& out_matches)
{    
    bool ret = true;

    if (in_descFirst.size() == 0 || in_descSecond.size() == 0)
    {
        VOCPP_TRACE_WARNING("[BruteForceMatcher]: No descriptions found in one or both of the provided frames")
        return false;
    }

    // Loop over all descriptions 
    for (uint32_t idx = 0U; idx < in_descFirst.size(); idx++)
    {
        uint32_t smallestDist = UINT_MAX;
        uint32_t smallestIdx2 = 0;
        
        for (uint32_t index2 = 0U; index2 < in_descSecond.size(); index2++)
        {
            uint32_t distance = BinaryFeatureDescription::ComputeHammingDistance(in_descFirst[idx], in_descSecond[index2], m_maxDistance + 1U);
            if (distance < smallestDist)
            {
                smallestDist = distance;
                smallestIdx2 = index2;
            }
        }

        // Check if distance is smaller than threshold
        if (smallestDist <= m_maxDistance)
        {
            BinaryDescriptionMatch match(in_descFirst[idx], in_descSecond[smallestIdx2], static_cast<double>(smallestDist));
            out_matches.push_back(match);
        }
    }
    
    return ret;
}

} //namespace FeatureHandling
} //namespace VOCPP
