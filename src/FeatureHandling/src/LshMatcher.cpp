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

LshMatcher::LshMatcher(const unsigned int& in_maxDistance, const unsigned int& in_numHashFuncs) :
    m_maxDistance(in_maxDistance),
    m_numHashFuncs(in_numHashFuncs),
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
        std::cout << "[LshMatcher]: No descriptions found in one or both of the provided frames" << std::endl;
        return false;
    }

    std::unordered_map<std::bitset<s_lengthHashFunc * 8U>, std::vector<unsigned int>> bucketTable;
    IndexDescriptions(in_descSecond, bucketTable);
    
    for (auto descFirst : in_descFirst)
    {
        // Search for match candidates
        std::vector<int> candidateIds;
        candidateIds.reserve(m_numHashFuncs);
        for (auto hashFunc : m_hashFuncs)
        {
           std::bitset<s_lengthHashFunc * 8U> bucketId;
            for (unsigned int k = 0U; k < s_lengthHashFunc; k++)
            {
                std::bitset<8U> bits(descFirst.GetDescription()[hashFunc[k]]);
                for (unsigned int l = 0U; l < 8U; l++)
                {
                    bucketId[k * 8U + l] = bits[l];
                }
            }
            if (bucketTable.count(bucketId) > 0)
            {
                for (auto candidate : bucketTable[bucketId])
                {
                    std::vector<int>::iterator it = std::find(candidateIds.begin(), candidateIds.end(), candidate);
                    if (it == candidateIds.end())
                    {
                        candidateIds.push_back(candidate);
                    }
                }
            }

            if (candidateIds.size() >= m_numHashFuncs)
            {
                break;
            }
        }

        unsigned int smallestDist = UINT32_MAX;
        unsigned int smallestIdx2 = 0U;
        for (unsigned int index2 = 0U; index2 < candidateIds.size(); index2++)
        {
            unsigned int distance = BinaryFeatureDescription::ComputeHammingDistance(descFirst, in_descSecond[candidateIds[index2]], m_maxDistance + 1U);
            
            if (distance < smallestDist)
            {
                smallestDist = distance;
                smallestIdx2 = index2;
            }
        }

        // Check if distance is smaller than threshold
        if (smallestDist <= m_maxDistance)
        {
            out_matches.push_back(BinaryDescriptionMatch{ descFirst,in_descSecond[candidateIds[smallestIdx2]] , static_cast<double>(smallestDist) });
        }
    }
    
    return ret;
}

void LshMatcher::GenerateHashFuncs()
{
    for (unsigned int l = 0U; l < m_numHashFuncs; l++)
    {
        std::vector<unsigned int> hashFunc;
        hashFunc.reserve(s_lengthHashFunc);
        for (unsigned int k = 0U; k < s_lengthHashFunc; k++)
        {
            hashFunc.push_back(static_cast<unsigned int>(Utils::DrawIntInRange(0, BinaryFeatureDescription::GetSizeInBytes() - 1)));
        }

        m_hashFuncs.push_back(hashFunc);
    }
}

void LshMatcher::IndexDescriptions(const std::vector<BinaryFeatureDescription>& in_desc, std::unordered_map<std::bitset<s_lengthHashFunc * 8U>, std::vector<unsigned int>>& out_bucketTable)
{
    unsigned int descId = 0U;
    for (auto description : in_desc)
    {
        for (auto hashFunc : m_hashFuncs)
        {
            std::bitset<s_lengthHashFunc * 8U> bucketId;
            for (unsigned int k = 0U; k < s_lengthHashFunc; k++)
            {
                std::bitset<8U> bits(description.GetDescription()[hashFunc[k]]);
                for (unsigned int l = 0U; l < 8U; l++)
                {
                    bucketId[k * 8U + l] = bits[l];
                }
            }

            if (out_bucketTable.count(bucketId) > 0)
            {
                out_bucketTable[bucketId].push_back(descId);
            }
            else
            {
                std::vector<unsigned int> table;
                table.push_back(descId);
                out_bucketTable[bucketId] = table;
            }
            
        }
        descId++;
    }
}

} //namespace FeatureHandling
} //namespace VOCPP
