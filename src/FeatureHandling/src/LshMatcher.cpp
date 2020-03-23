/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include<Vocpp_FeatureHandling/LshMatcher.h>
#include<Vocpp_Utils/NumericalUtilities.h>
#include <iostream>

namespace VOCPP
{
namespace FeatureHandling
{

LshMatcher::LshMatcher(const int& in_maxDistance, const int& in_numHashFuncs, 
    const int& in_lengthHashFunc, const int& in_lengthDescription) :
    m_maxDistance(in_maxDistance),
    m_numHashFuncs(in_numHashFuncs),
    m_lengthHashFunc(in_lengthHashFunc),
    m_lengthDescription(in_lengthDescription),
    m_hashFuncs()
{
    GenerateHashFuncs();
}

bool LshMatcher::MatchDesriptions(const std::vector<BinaryFeatureDescription>& in_descFirst,
    const std::vector<BinaryFeatureDescription>& in_descSecond, std::vector<BinaryDescriptionMatch>& out_matches)
{    
    bool ret = true;

    if (in_descFirst.size() == 0 || in_descSecond.size() == 0)
    {
        std::cout << "[BruteForceMatcher]: No descriptions found in one or both of the provided frames" << std::endl;
        return false;
    }
    std::vector<BinaryDescriptionMatch> testMatches;
    std::map<int, std::vector<BinaryFeatureDescription>> bucketTable;
    IndexDescriptions(in_descSecond, bucketTable);
    
    for (auto descFirst : in_descFirst)
    {
        // Search for match candidates
        std::vector<BinaryFeatureDescription> candidates;
        for (auto hashFunc : m_hashFuncs)
        {
            int bucketId = 0;
            for (int k = 0; k < m_lengthHashFunc; k++)
            {
                bucketId += descFirst.GetDescription()[hashFunc[k]] * std::pow(2U, k);
            }

            if (bucketTable.count(bucketId) > 0)
            {
                for (auto candidate : bucketTable[bucketId])
                {
                    candidates.push_back(candidate);
                }
            }

            if (candidates.size() >= m_numHashFuncs)
            {
                break;
            }
        }

        int smallestDist = INT_MAX;
        int smallestIdx2 = 0;
        for (int index2 = 0; index2 < candidates.size(); index2++)
        {
            int distance = ComputeHammingDistance(descFirst, candidates[index2]);
            
            if (distance < smallestDist)
            {
                smallestDist = distance;
                smallestIdx2 = index2;
            }
        }

        // Check if distance is smaller than threshold
        if (smallestDist <= m_maxDistance)
        {
            out_matches.push_back(BinaryDescriptionMatch{ descFirst, candidates[smallestIdx2] , static_cast<float>(smallestDist) });
        }
    }

    /* Brute Force Approach - Keep for now */
    /* Loop over all descriptions
    for (auto desc1 : in_descFirst)
    {
        int smallestDist = INT_MAX;
        int smallestIdx2 = 0;
        
        for (int index2 = 0; index2 < in_descSecond.size(); index2++)
        {
            int distance = ComputeHammingDistance(desc1, in_descSecond[index2]);
            if (distance < smallestDist)
            {
                smallestDist = distance;
                smallestIdx2 = index2;
            }
        }

        // Check if distance is smaller than threshold
        if (smallestDist <= m_maxDistance)
        {
            //out_matches.push_back(BinaryDescriptionMatch{ desc1,in_descSecond[smallestIdx2], static_cast<float>(smallestDist)});
        }
    }*/
    
    return ret;
}

void LshMatcher::GenerateHashFuncs()
{
    for (int l = 0; l < m_numHashFuncs; l++)
    {
        std::vector<int> hashFunc;
        hashFunc.reserve(m_lengthHashFunc);
        for (int k = 0; k < m_lengthHashFunc; k++)
        {
            hashFunc.push_back(Utils::DrawIntInRange(0, m_lengthDescription));
        }

        m_hashFuncs.push_back(hashFunc);
    }
}

void LshMatcher::IndexDescriptions(const std::vector<BinaryFeatureDescription>& in_desc, std::map<int, std::vector<BinaryFeatureDescription>>& out_bucketTable)
{
    for (auto description : in_desc)
    {
        for (auto hashFunc : m_hashFuncs)
        {
            int bucketId = 0;
            for (int k = 0; k < m_lengthHashFunc; k++)
            {
                bucketId += description.GetDescription()[hashFunc[k]] * std::pow(2U, k);
            }
            
            if (out_bucketTable.count(bucketId) > 0)
            {
                out_bucketTable[bucketId].push_back(description);
            }
            else
            {
                std::vector<BinaryFeatureDescription> table;
                table.push_back(description);
                out_bucketTable[bucketId] = table;
            }
            
        }
    }
}

int LshMatcher::ComputeHammingDistance(const BinaryFeatureDescription& in_first, const BinaryFeatureDescription& in_second) const
{
    int distance = 0U;
    
    for (int i = 0; i < in_first.GetDescription().size(); i++)
    {
        if (in_first.GetDescription()[i] != in_second.GetDescription()[i])
        {
            distance++;
        }
    }

    return distance;
}


} //namespace FeatureHandling
} //namespace VOCPP
