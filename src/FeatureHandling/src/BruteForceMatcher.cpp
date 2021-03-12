/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include<Vocpp_FeatureHandling/BruteForceMatcher.h>
#include<Vocpp_Utils/NumericalUtilities.h>
#include <iostream>

namespace VOCPP
{
namespace FeatureHandling
{

BruteForceMatcher::BruteForceMatcher(const unsigned int& in_maxDistance) : m_maxDistance(in_maxDistance)
{
}

bool BruteForceMatcher::MatchDesriptions(const std::vector<BinaryFeatureDescription>& in_descFirst,
    const std::vector<BinaryFeatureDescription>& in_descSecond, std::vector<BinaryDescriptionMatch>& out_matches)
{    
    bool ret = true;

    if (in_descFirst.size() == 0 || in_descSecond.size() == 0)
    {
        std::cout << "[BruteForceMatcher]: No descriptions found in one or both of the provided frames" << std::endl;
        return false;
    }

    // Loop over all descriptions 
    for (auto desc1 : in_descFirst)
    {
        unsigned int smallestDist = UINT_MAX;
        unsigned int smallestIdx2 = 0;
        
        for (int index2 = 0; index2 < in_descSecond.size(); index2++)
        {
            unsigned int distance = BinaryFeatureDescription::ComputeHammingDistance(desc1, in_descSecond[index2], m_maxDistance + 1U);
            if (distance < smallestDist)
            {
                smallestDist = distance;
                smallestIdx2 = index2;
            }
        }

        // Check if distance is smaller than threshold
        if (smallestDist <= m_maxDistance)
        {
            out_matches.push_back(BinaryDescriptionMatch{ desc1,in_descSecond[smallestIdx2], static_cast<double>(smallestDist)});
        }
    }
    
    return ret;
}

} //namespace FeatureHandling
} //namespace VOCPP
