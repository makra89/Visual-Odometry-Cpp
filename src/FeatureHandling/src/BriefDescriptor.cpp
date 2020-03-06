/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_FeatureHandling/BriefDescriptor.h>
#include<Vocpp_Utils/NumericalUtilities.h>

#include <iostream>
#include<random>

namespace VOCPP
{
namespace FeatureHandling
{

BriefDescriptor::BriefDescriptor(const int in_randomPairDrawRadius)
{
    m_randomPairDrawRadius = in_randomPairDrawRadius;
    DrawPairs();
}


bool BriefDescriptor::ComputeDescriptions(const Frame& in_frame, const std::vector<Feature>& in_features,
    std::vector<BinaryFeatureDescription>& out_descriptions)
{
    bool ret = false;

    if (in_features.size() == 0)
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
    for (auto feature : in_features)
    {
        std::vector<bool> description;
        description.reserve(s_numRandomPairs);
        int numSucessfulPairs = 0U;

        // Loop over pairs
        for (auto pair : m_pairs)
        {
            int indFirstX = static_cast<int>(pair.x1 + feature.imageCoordX);
            int indFirstY = static_cast<int>(pair.y1 + feature.imageCoordY);
            int indSecX = static_cast<int>(pair.x2 + feature.imageCoordX);
            int indSecY = static_cast<int>(pair.y2 + feature.imageCoordY);
           
            // Check whether all indices are within range
            const bool inRangeFirstX = (indFirstX >= 0) && (indFirstX < in_frame.GetImage().size[1]);
            const bool inRangeFirstY = (indFirstY >= 0) && (indFirstY < in_frame.GetImage().size[0]);
            const bool inRangeSecX = (indSecX >= 0) && (indSecX < in_frame.GetImage().size[1]);
            const bool inRangeSecY = (indSecY >= 0) && (indSecY < in_frame.GetImage().size[0]);

            if (inRangeFirstX && inRangeFirstY && inRangeSecX && inRangeSecY)
            {
                // Compare intensities and append bin to description
                bool bin = in_frame.GetImage()(indFirstY, indFirstX) > in_frame.GetImage()(indSecY, indSecX) ? true : false;
                description.push_back(bin);
                numSucessfulPairs++;
            }
            else
            {
                break;
            }
        }

        if (numSucessfulPairs == s_numRandomPairs)
        {
            out_descriptions.push_back(BinaryFeatureDescription(feature, description));
            ret = true;
        }
    }

    return ret;
}


void BriefDescriptor::DrawPairs()
{
    
    for (int i = 0U; i < s_numRandomPairs; i++)
    {
        // Draw index in range [-radius, radius]
        float index1 = Utils::DrawFloatInRange(-static_cast<float>(m_randomPairDrawRadius), static_cast<float>(m_randomPairDrawRadius));
        float index2 = Utils::DrawFloatInRange(-static_cast<float>(m_randomPairDrawRadius), static_cast<float>(m_randomPairDrawRadius));
        float index3 = Utils::DrawFloatInRange(-static_cast<float>(m_randomPairDrawRadius), static_cast<float>(m_randomPairDrawRadius));
        float index4 = Utils::DrawFloatInRange(-static_cast<float>(m_randomPairDrawRadius), static_cast<float>(m_randomPairDrawRadius));

        m_pairs.push_back(PointPair{ index1, index2, index3, index4});
    }
}

} //namespace FeatureHandling
} //namespace VOCPP