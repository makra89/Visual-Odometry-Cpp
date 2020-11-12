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

LshMatcher::LshMatcher(const unsigned int& in_maxDistance, const unsigned int& in_numHashFuncs,
    const unsigned int& in_lengthHashFunc, const unsigned int& in_lengthDescription) :
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
        std::cout << "[LshMatcher]: No descriptions found in one or both of the provided frames" << std::endl;
        return false;
    }

    std::unordered_map<unsigned int, std::vector<unsigned int>> bucketTable;
    IndexDescriptions(in_descSecond, bucketTable);
    
    for (auto descFirst : in_descFirst)
    {
        // Search for match candidates
        std::vector<int> candidateIds;
        candidateIds.reserve(m_numHashFuncs);
        for (auto hashFunc : m_hashFuncs)
        {
            unsigned int bucketId = 0U;
            for (unsigned int k = 0U; k < m_lengthHashFunc; k++)
            {
                bucketId += descFirst.GetDescription()[hashFunc[k]] * static_cast<unsigned int>(std::pow(2U, k));
            }
            if (bucketTable.count(bucketId) > 0)
            {
                for (auto candidate : bucketTable[bucketId])
                {
                    candidateIds.push_back(candidate);
                }
            }

            if (candidateIds.size() >= m_numHashFuncs)
            {
                break;
            }
        }

        unsigned int smallestDist = UINT_MAX;
        unsigned int smallestIdx2 = 0U;
        for (unsigned int index2 = 0U; index2 < candidateIds.size(); index2++)
        {
            unsigned int distance = ComputeHammingDistance(descFirst, in_descSecond[candidateIds[index2]]);
            
            if (distance < smallestDist)
            {
                smallestDist = distance;
                smallestIdx2 = index2;
            }
        }

        // Check if distance is smaller than threshold
        if (smallestDist <= m_maxDistance)
        {
            out_matches.push_back(BinaryDescriptionMatch{ descFirst,in_descSecond[candidateIds[smallestIdx2]] , static_cast<float>(smallestDist) });
        }
    }
    
    return ret;
}

void LshMatcher::GenerateHashFuncs()
{
    for (unsigned int l = 0U; l < m_numHashFuncs; l++)
    {
        std::vector<unsigned int> hashFunc;
        hashFunc.reserve(m_lengthHashFunc);
        for (unsigned int k = 0U; k < m_lengthHashFunc; k++)
        {
            hashFunc.push_back(static_cast<unsigned int>(Utils::DrawIntInRange(0, m_lengthDescription)));
        }

        m_hashFuncs.push_back(hashFunc);
    }
}

void LshMatcher::IndexDescriptions(const std::vector<BinaryFeatureDescription>& in_desc, std::unordered_map<unsigned int, std::vector<unsigned int>>& out_bucketTable)
{
    unsigned int descId = 0U;
    for (auto description : in_desc)
    {
        for (auto hashFunc : m_hashFuncs)
        {
            unsigned int bucketId = 0U;
            for (unsigned int k = 0U; k < m_lengthHashFunc; k++)
            {
                bucketId += description.GetDescription()[hashFunc[k]] * static_cast<unsigned int>(std::pow(2U, k));
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

unsigned int LshMatcher::ComputeHammingDistance(const BinaryFeatureDescription& in_first, const BinaryFeatureDescription& in_second) const
{
    unsigned int distance = 0U;
    
    for (unsigned int i = 0U; i < in_first.GetDescription().size(); i++)
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
