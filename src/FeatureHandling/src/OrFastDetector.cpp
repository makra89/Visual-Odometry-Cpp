/* This file is part of the Visual-Odometry-Cpp project.
* It is subject to the license terms in the LICENSE file
* found in the top-level directory of this distribution.
*
* Copyright (C) 2020 Manuel Kraus
*/

#include <Vocpp_FeatureHandling/OrFastDetector.h>
#include <Vocpp_Utils/ImageProcessingUtils.h>
#include<opencv2/imgproc.hpp>

namespace VOCPP
{
namespace FeatureHandling
{

// The version of the FAST feature detector used here uses a ring of 16 pixels around the center pixel (radius = 3 --> diameter 7)
const int OrientedFastDetector::s_featureSize = 7;

OrientedFastDetector::OrientedFastDetector(const float& in_relTresh, const int& in_numPixelsAboveThresh,
    const int& in_harrisBlockSize) :
    m_relTresh(in_relTresh),
    m_numPixelsAboveThresh(in_numPixelsAboveThresh),
    m_harrisBlockSize(in_harrisBlockSize),
    m_harrisDetector()
{
}


bool OrientedFastDetector::ExtractFeatures(const Frame& in_frame, const int& in_maxNumFeatures, std::vector<Feature>& out_features)
{
    bool ret = true;

    if (in_frame.GetImage().dims != 2)
    {
        ret = false;
        std::cout << "[OrientedFastDetector]: Non-grayscale image has been provided" << std::endl;
    }
    else
    {
        cv::Mat1f fastScore = cv::Mat1f::zeros(in_frame.GetImage().rows, in_frame.GetImage().cols);
        for (int i = 0; i < in_frame.GetImage().rows; i++)
        {
            for (int j = 0; j < in_frame.GetImage().cols; j++)
            {
                int score = CheckIntensities(in_frame.GetImage(), j, i, in_frame.GetImage().cols, in_frame.GetImage().rows);
                if (score >= m_numPixelsAboveThresh)
                {
                    fastScore(i, j) = static_cast<float>(score);
                }
            }
        }

        // Thin out neighbouring features, we use the same radius as the one used for calculating the angle
        std::vector<cv::Point2f> localMax;
        std::vector<float> localMaxVal;
        Utils::ExtractLocalMaxima(fastScore, (s_featureSize - 1) / 2, localMax, localMaxVal, false);
        
        // Calculate Harris response for all surviving features
        int featureId = 0;
        for (auto max : localMax)
        {
            float response = m_harrisDetector.ComputeScore(in_frame.GetImage(), static_cast<int>(max.x)
                , static_cast<int>(max.y), m_harrisBlockSize);
            
            cv::Mat1f patch = cv::Mat1f::zeros(s_featureSize, s_featureSize);
            bool patchExtractSuccess = Utils::ExtractImagePatchAroundPixelPos(in_frame.GetImage(), patch, (s_featureSize - 1) / 2,
                static_cast<int>(max.x), static_cast<int>(max.y));

            // Calculate angle if successful and create feature
            if (patchExtractSuccess)
            {
                cv::Moments patchMoments = cv::moments(patch);
                float angle = static_cast<float>(std::atan2(patchMoments.m01, patchMoments.m10));

                out_features.push_back(Feature{ featureId, in_frame.GetId(), max.x, max.y, response, angle, static_cast<float>(s_featureSize)});
                featureId++;
            }
        }

        // Sort according to feature response and throw away weakest features if we have more than we need
        std::sort(out_features.begin(), out_features.end(), std::greater<Feature>());
        if (out_features.size() > in_maxNumFeatures)
        {
            out_features.resize(in_maxNumFeatures);
        }
    }

    return ret;
}

int OrientedFastDetector::CheckIntensities(const cv::Mat1f& in_image, const int& in_coordX, const int& in_coordY, 
    const int& in_imgWidth, const int& in_imgHeight)
{
    int ret = 0;
    
    bool inRangeX = (in_coordX - 3) > 0 && (in_coordX + 3 < in_imgWidth);
    bool inRangeY = (in_coordY - 3) > 0 && (in_coordY + 3 < in_imgHeight);

    if (inRangeX && inRangeY)
    {
        const float nucleusInt = in_image(in_coordY, in_coordX);
        // Pixel 1
        int passLower = (in_image(in_coordY - 3, in_coordX) + m_relTresh * nucleusInt) < nucleusInt ? 1 : 0;
        // Pixel 5
        if ((in_image(in_coordY, in_coordX + 3) + m_relTresh * nucleusInt) < nucleusInt) passLower++;
        // Pixel 9
        if ((in_image(in_coordY + 3, in_coordX) + m_relTresh * nucleusInt) < nucleusInt) passLower++;
        // Pixel 13
        if((in_image(in_coordY, in_coordX - 3) + m_relTresh * nucleusInt) < nucleusInt) passLower++;

        if (passLower < 3)
        {
            int passHigher = (in_image(in_coordY - 3, in_coordX) - m_relTresh * nucleusInt) > nucleusInt ? 1 : 0;
            // Pixel 5
            if ((in_image(in_coordY, in_coordX + 3) - m_relTresh * nucleusInt) > nucleusInt) passHigher++;
            // Pixel 9
            if ((in_image(in_coordY + 3, in_coordX) - m_relTresh * nucleusInt) > nucleusInt) passHigher++;
            // Pixel 13
            if ((in_image(in_coordY, in_coordX - 3) - m_relTresh * nucleusInt) > nucleusInt) passHigher++;
            
            if (passHigher >= 3)
            {
                ret = CheckAll(in_image, in_coordX, in_coordY);
            }
        }
        else
        {
            ret = CheckAll(in_image, in_coordX, in_coordY);
        }
    }

    return ret;
}

int OrientedFastDetector::CheckAll(const cv::Mat1f& in_image, const int& in_coordX, const int& in_coordY)
{
    int ret = 0;

    const float nucleusInt = in_image(in_coordY, in_coordX);
    // Pixel 1
    int passLower = (in_image(in_coordY - 3, in_coordX) + m_relTresh * nucleusInt) < nucleusInt ? 1 : 0;
    // Pixel 2
    if ((in_image(in_coordY - 3, in_coordX + 1) + m_relTresh * nucleusInt) < nucleusInt) passLower++;
    // Pixel 3
    if ((in_image(in_coordY - 2, in_coordX + 2) + m_relTresh * nucleusInt) < nucleusInt) passLower++;
    // Pixel 4
    if ((in_image(in_coordY - 1, in_coordX + 3) + m_relTresh * nucleusInt) < nucleusInt) passLower++;
    // Pixel 5
    if ((in_image(in_coordY, in_coordX + 3) + m_relTresh * nucleusInt) < nucleusInt) passLower++;
    // Pixel 6
    if ((in_image(in_coordY + 1, in_coordX + 3) + m_relTresh * nucleusInt) < nucleusInt) passLower++;
    // Pixel 7
    if ((in_image(in_coordY + 2, in_coordX + 2) + m_relTresh * nucleusInt) < nucleusInt) passLower++;
    // Pixel 8
    if ((in_image(in_coordY + 3, in_coordX + 1) + m_relTresh * nucleusInt) < nucleusInt) passLower++;
    // Pixel 9
    if ((in_image(in_coordY + 3, in_coordX) + m_relTresh * nucleusInt) < nucleusInt) passLower++;
    // Pixel 10
    if ((in_image(in_coordY + 3, in_coordX - 1) + m_relTresh * nucleusInt) < nucleusInt) passLower++;
    // Pixel 11
    if ((in_image(in_coordY + 2, in_coordX - 2) + m_relTresh * nucleusInt) < nucleusInt) passLower++;
    // Pixel 12
    if ((in_image(in_coordY + 1, in_coordX - 3) + m_relTresh * nucleusInt) < nucleusInt) passLower++;
    // Pixel 13
    if ((in_image(in_coordY, in_coordX - 3) + m_relTresh * nucleusInt) < nucleusInt) passLower++;
    // Pixel 14
    if ((in_image(in_coordY - 1, in_coordX - 3) + m_relTresh * nucleusInt) < nucleusInt) passLower++;
    // Pixel 15
    if ((in_image(in_coordY - 2, in_coordX - 2) + m_relTresh * nucleusInt) < nucleusInt) passLower++;
    // Pixel 16
    if ((in_image(in_coordY -3, in_coordX - 1) + m_relTresh * nucleusInt) < nucleusInt) passLower++;

    if (passLower < 12)
    {
        int passHigher = (in_image(in_coordY - 3, in_coordX) - m_relTresh * nucleusInt) > nucleusInt ? 1 : 0;
        // Pixel 2
        if ((in_image(in_coordY - 3, in_coordX + 1) + m_relTresh * nucleusInt) < nucleusInt) passHigher++;
        // Pixel 3
        if ((in_image(in_coordY - 2, in_coordX + 2) + m_relTresh * nucleusInt) < nucleusInt) passHigher++;
        // Pixel 4
        if ((in_image(in_coordY - 1, in_coordX + 3) + m_relTresh * nucleusInt) < nucleusInt) passHigher++;
        // Pixel 5
        if ((in_image(in_coordY, in_coordX + 3) + m_relTresh * nucleusInt) < nucleusInt) passHigher++;
        // Pixel 6
        if ((in_image(in_coordY + 1, in_coordX + 3) + m_relTresh * nucleusInt) < nucleusInt) passHigher++;
        // Pixel 7
        if ((in_image(in_coordY + 2, in_coordX + 2) + m_relTresh * nucleusInt) < nucleusInt) passHigher++;
        // Pixel 8
        if ((in_image(in_coordY + 3, in_coordX + 1) + m_relTresh * nucleusInt) < nucleusInt) passHigher++;
        // Pixel 9
        if ((in_image(in_coordY + 3, in_coordX) + m_relTresh * nucleusInt) < nucleusInt) passHigher++;
        // Pixel 10
        if ((in_image(in_coordY + 3, in_coordX - 1) + m_relTresh * nucleusInt) < nucleusInt) passHigher++;
        // Pixel 11
        if ((in_image(in_coordY + 2, in_coordX - 2) + m_relTresh * nucleusInt) < nucleusInt) passHigher++;
        // Pixel 12
        if ((in_image(in_coordY + 1, in_coordX - 3) + m_relTresh * nucleusInt) < nucleusInt) passHigher++;
        // Pixel 13
        if ((in_image(in_coordY, in_coordX - 3) + m_relTresh * nucleusInt) < nucleusInt) passLower++;
        // Pixel 14
        if ((in_image(in_coordY - 1, in_coordX - 3) + m_relTresh * nucleusInt) < nucleusInt) passHigher++;
        // Pixel 15
        if ((in_image(in_coordY - 2, in_coordX - 2) + m_relTresh * nucleusInt) < nucleusInt) passHigher++;
        // Pixel 16
        if ((in_image(in_coordY - 3, in_coordX - 1) + m_relTresh * nucleusInt) < nucleusInt) passHigher++;
            
        if (passHigher >= 12)
        {
            ret = passHigher;
        }
    }
    else
    {
        ret = passLower;
    }

    return ret;
}

} //namespace FeatureHandling
} //namespace VOCPP

