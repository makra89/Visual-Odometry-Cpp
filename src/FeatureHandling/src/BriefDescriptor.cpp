/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include<BriefDescriptor.h>
#include<Vocpp_Utils/NumericalUtilities.h>

#include <iostream>
#include<random>

namespace VOCPP
{
namespace FeatureHandling
{

BriefDescriptor::BriefDescriptor()
{
}


BriefDescriptor* BriefDescriptor::CreateInstance(const int in_randomPairDrawRadius, const int in_numRandomPairs)
{
    BriefDescriptor* descriptor = new BriefDescriptor();

    descriptor->m_randomPairDrawRadius = in_randomPairDrawRadius;
    descriptor->m_numRandomPairs = in_numRandomPairs;

    descriptor->DrawPairs();

    return descriptor;
}


bool BriefDescriptor::ComputeDescriptions(const Frame& in_frame, const std::vector<cv::KeyPoint>& in_keypoints,
    std::vector<cv::Mat>& out_descriptions, std::vector<cv::KeyPoint>& out_validKeypoints)
{
    bool ret = false;

    if (in_keypoints.size() == 0)
    {    
        std::cout << "[BriefDescriptor]: No keypoints found in provided frame" << std::endl;
        return false;
    }

    if (out_descriptions.size() != 0)
    {
        std::cout << "[BriefDescriptor]: Frame with non-empty description vector provided, will be erased" << std::endl;
        return false;
    }

    // Loop over keypoints
    for (auto key : in_keypoints)
    {
        cv::Mat descriptions = cv::Mat::zeros(1, m_numRandomPairs, CV_8U);
        int numSucessfulPairs = 0U;

        // Loop over pairs
        for (auto pair : m_pairs)
        {
            int indFirstX = static_cast<int>(pair.at<float>(0, 0) + key.pt.x);
            int indFirstY = static_cast<int>(pair.at<float>(0, 1) + key.pt.y);
            int indSecX = static_cast<int>(pair.at<float>(0, 2) + key.pt.x);
            int indSecY = static_cast<int>(pair.at<float>(0, 3) + key.pt.y);
           
            // Check whether all indices are within range
            const bool inRangeFirstX = (indFirstX >= 0) && (indFirstX < in_frame.GetImage().size[1]);
            const bool inRangeFirstY = (indFirstY >= 0) && (indFirstY < in_frame.GetImage().size[0]);
            const bool inRangeSecX = (indSecX >= 0) && (indSecX < in_frame.GetImage().size[1]);
            const bool inRangeSecY = (indSecY >= 0) && (indSecY < in_frame.GetImage().size[0]);

            if (inRangeFirstX && inRangeFirstY && inRangeSecX && inRangeSecY)
            {
                // Compare intensities and append bin to description
                uint8_t bin = in_frame.GetImage().at<float>(indFirstY, indFirstX) > in_frame.GetImage().at<float>(indSecY, indSecX) ? 1U : 0U;
                descriptions.at<uint8_t>(0, numSucessfulPairs) = bin;
                numSucessfulPairs++;
            }
            else
            {
                break;
            }
        }

        if (numSucessfulPairs == m_numRandomPairs)
        {
            out_descriptions.push_back(descriptions);
            out_validKeypoints.push_back(key);
            ret = true;
        }
    }

    return ret;
}


void BriefDescriptor::DrawPairs()
{
    
    for (int i = 0U; i < m_numRandomPairs; i++)
    {
        // Draw index in range [-radius, radius]
        float index1 = Utils::DrawFloatInRange(-static_cast<float>(m_randomPairDrawRadius), static_cast<float>(m_randomPairDrawRadius));
        float index2 = Utils::DrawFloatInRange(-static_cast<float>(m_randomPairDrawRadius), static_cast<float>(m_randomPairDrawRadius));
        float index3 = Utils::DrawFloatInRange(-static_cast<float>(m_randomPairDrawRadius), static_cast<float>(m_randomPairDrawRadius));
        float index4 = Utils::DrawFloatInRange(-static_cast<float>(m_randomPairDrawRadius), static_cast<float>(m_randomPairDrawRadius));

        cv::Mat pairs = cv::Mat::zeros(1, 4, CV_32F);

        pairs.at<float>(0, 0) = index1;
        pairs.at<float>(0, 1) = index2;
        pairs.at<float>(0, 2) = index3;
        pairs.at<float>(0, 3) = index4;

        m_pairs.push_back(pairs);
    }
}

} //namespace FeatureHandling
} //namespace VOCPP