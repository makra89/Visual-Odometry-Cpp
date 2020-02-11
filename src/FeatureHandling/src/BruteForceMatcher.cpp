/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include<BruteForceMatcher.h>

#include <iostream>

namespace VOCPP
{
namespace FeatureHandling
{

BruteForceMatcher::BruteForceMatcher()
{
}


BruteForceMatcher* BruteForceMatcher::CreateInstance(const int in_maxDistance)
{
    BruteForceMatcher* matcher = new BruteForceMatcher();

    matcher->m_maxDistance = in_maxDistance;

    return matcher;
}


bool BruteForceMatcher::MatchDesriptions(const std::vector<cv::Mat>& in_descriptions1, const std::vector<cv::Mat>& in_descriptions2,
    const int& in_secondFrameId, std::vector<cv::DMatch>& out_matches)
{    
    bool ret = true;

    if (in_descriptions1.size() == 0 || in_descriptions2.size() == 0)
    {
        std::cout << "[BruteForceMatcher]: No descriptions found in one or both of the provided frames" << std::endl;
        return false;
    }

    // Loop over all descriptions
    // Look for first-frame features in the second frame
    int idX1 = 0;
    for (auto desc1 : in_descriptions1)
    {
        
        int smallestDist = INT_MAX;
        int smallestIdx2 = 0;
        
        int idX2 = 0;
        for (auto desc2 : in_descriptions2)
        {
            int distance = ComputeHammingDistance(desc1, desc2);
            if (distance < smallestDist)
            {
                smallestDist = distance;
                smallestIdx2 = idX2;
            }
           
            idX2++;
        }

        // Check if distance is smaller than threshold
        if (smallestDist <= m_maxDistance)
        {
            out_matches.push_back(cv::DMatch(idX1, smallestIdx2, in_secondFrameId, static_cast<float>(smallestDist)));
        }

        idX1++;
    }

    return ret;
}

int BruteForceMatcher::ComputeHammingDistance(const cv::Mat& left, const cv::Mat& right)
{
    int distance = 0U;
    
    if (!(left.size == right.size))
    {
        std::cout << "[ComputeHammingDistance]: size of left and right mat do not match" << std::endl;
    }
    else if (left.size[0] != 1 || right.size[0] != 1)
    {
        std::cout << "[ComputeHammingDistance]: Column vector expected, got something else" << std::endl;
    }
    else
    {
        for (int i = 0; i < left.size[1]; i++)
        {
            if (left.at<uint8_t>(0, i) != right.at<uint8_t>(0, i))
            {
                distance++;
            }
        }
    }

    return distance;
}


} //namespace FeatureHandling
} //namespace VOCPP
