/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include<Vocpp_FeatureHandling/LshMatcher.h>
#include<Vocpp_Utils/NumericalUtilities.h>
#include<Vocpp_Utils/TracingImpl.h>

namespace VOCPP
{
namespace FeatureHandling
{

LshMatcher::LshMatcher(const uint32_t& in_maxDistance, const uint32_t& in_numHashFuncs) :
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
        VOCPP_TRACE_WARNING("[LshMatcher]: No descriptions found in one or both of the provided frames")
        return false;
    }

    std::unordered_map<std::bitset<s_lengthHashFunc * 8U>, std::vector<uint32_t>> bucketTable;
    IndexDescriptions(in_descSecond, bucketTable);
    
    for (uint32_t descIdx = 0U; descIdx < in_descFirst.size(); descIdx++)
    {
        // Search for match candidates
        std::vector<uint32_t> candidateIds;
        candidateIds.reserve(m_numHashFuncs);
        
        for (uint32_t hashIdx = 0U; hashIdx < m_hashFuncs.size(); hashIdx++)
        {
           std::bitset<s_lengthHashFunc * 8U> bucketId;
            for (uint32_t k = 0U; k < s_lengthHashFunc; k++)
            {
                std::bitset<8U> bits(in_descFirst[descIdx].GetDescription()[m_hashFuncs[hashIdx][k]]);
                for (uint32_t l = 0U; l < 8U; l++)
                {
                    bucketId[k * 8U + l] = bits[l];
                }
            }
            if (bucketTable.count(bucketId) > 0)
            {
                for (uint32_t bucketIdx = 0U; bucketIdx < bucketTable[bucketId].size(); bucketIdx++)
                {
                    std::vector<uint32_t>::iterator it = std::find(candidateIds.begin(), candidateIds.end(), bucketTable[bucketId][bucketIdx]);
                    if (it == candidateIds.end())
                    {
                        candidateIds.push_back(bucketTable[bucketId][bucketIdx]);
                    }
                }
            }

            if (candidateIds.size() >= m_numHashFuncs)
            {
                break;
            }
        }

        uint32_t smallestDist = UINT32_MAX;
        uint32_t smallestIdx2 = 0U;
        for (uint32_t index2 = 0U; index2 < candidateIds.size(); index2++)
        {
            uint32_t distance = BinaryFeatureDescription::ComputeHammingDistance(in_descFirst[descIdx], in_descSecond[candidateIds[index2]], m_maxDistance + 1U);
            
            if (distance < smallestDist)
            {
                smallestDist = distance;
                smallestIdx2 = index2;
            }
        }

        // Check if distance is smaller than threshold
        if (smallestDist <= m_maxDistance)
        {
            BinaryDescriptionMatch match(in_descFirst[descIdx], in_descSecond[candidateIds[smallestIdx2]] , static_cast<double>(smallestDist));
            out_matches.push_back(match);
        }
    }
    
    return ret && (out_matches.size() > 0U);
}

void LshMatcher::GenerateHashFuncs()
{
    for (uint32_t l = 0U; l < m_numHashFuncs; l++)
    {
        std::vector<uint32_t> hashFunc;
        hashFunc.reserve(s_lengthHashFunc);
        for (uint32_t k = 0U; k < s_lengthHashFunc; k++)
        {
            hashFunc.push_back(static_cast<uint32_t>(Utils::DrawIntInRange(0, 32U - 1)));
        }

        m_hashFuncs.push_back(hashFunc);
    }
}

void LshMatcher::IndexDescriptions(const std::vector<BinaryFeatureDescription>& in_desc, std::unordered_map<std::bitset<s_lengthHashFunc * 8U>, std::vector<uint32_t>>& out_bucketTable)
{
    uint32_t descId = 0U;
    for (uint32_t descIdx = 0U; descIdx < in_desc.size(); descIdx++)
    {
        for (uint32_t hashIdx = 0U; hashIdx < m_hashFuncs.size(); hashIdx++)
        {
            std::bitset<s_lengthHashFunc * 8U> bucketId;
            for (uint32_t k = 0U; k < s_lengthHashFunc; k++)
            {
                std::bitset<8U> bits(in_desc[descIdx].GetDescription()[m_hashFuncs[hashIdx][k]]);
                for (uint32_t l = 0U; l < 8U; l++)
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
                std::vector<uint32_t> table;
                table.push_back(descId);
                out_bucketTable[bucketId] = table;
            }
            
        }
        descId++;
    }
}

} //namespace FeatureHandling
} //namespace VOCPP
