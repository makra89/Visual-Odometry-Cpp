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


bool BruteForceMatcher::matchDesriptions(Utils::Frame& inout_frame1, Utils::Frame& inout_frame2)
{    
    bool ret = true;

    if (inout_frame1.GetKeypoints().size() == 0 || inout_frame2.GetKeypoints().size() == 0)
    {
        std::cout << "[BruteForceMatcher]: No keypoints found in one or both of the provided frames" << std::endl;
        return false;
    }

    if (inout_frame1.GetDescriptions().size() == 0 || inout_frame2.GetDescriptions().size() == 0)
    {
        std::cout << "[BruteForceMatcher]: No descriptions found in one or both of the provided frames" << std::endl;
        return false;
    }

    std::vector<cv::DMatch> matchesFrame1;
    std::vector<cv::DMatch> matchesFrame2;
    matchesFrame1 = inout_frame1.GetMatches();
    matchesFrame2 = inout_frame2.GetMatches();

    // Loop over all descriptions
    // Look for first-frame features in the second frame
    int idX1 = 0;
    for (auto desc1 : inout_frame1.GetDescriptions())
    {
        
        int smallestDist = INT_MAX;
        int smallestIdx2 = 0;
        
        int idX2 = 0;
        for (auto desc2 : inout_frame2.GetDescriptions())
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
            matchesFrame1.push_back(cv::DMatch(idX1, smallestIdx2, inout_frame2.GetId(), static_cast<float>(smallestDist)));
            matchesFrame2.push_back(cv::DMatch(smallestIdx2, idX1, inout_frame1.GetId(), static_cast<float>(smallestDist)));
        }

        idX1++;
    }

    // Append features to first frame
    inout_frame1.SetMatches(std::move(matchesFrame1));
    inout_frame2.SetMatches(std::move(matchesFrame2));

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
